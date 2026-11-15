#include "mtc_task_executor.hpp"
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/planning_scene/planning_scene.hpp>

namespace pick_place
{
namespace mtc = moveit::task_constructor;
using ExecuteTask = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace
{
const std::vector<std::string> ARM = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
double secondsSince(Clock::time_point time)
{
    return std::chrono::duration<double>(Clock::now() - time).count();
}
}

MtcTaskExecutor::MtcTaskExecutor(rclcpp::Node &node, std::atomic<bool> &stopping,
    double planning_time, double velocity, double acceleration, Scene scene,
    ExecuteSolution execute_solution, JointsReached joints_reached)
    : node_(node), stopping_(stopping), planning_time_(planning_time), velocity_(velocity),
      acceleration_(acceleration), scene_(std::move(scene)),
      execute_solution_(std::move(execute_solution)), joints_reached_(std::move(joints_reached))
{
}

void MtcTaskExecutor::preempt()
{
    std::lock_guard<std::mutex> lock(planning_mutex_);
    if (planning_task_)
    {
        planning_task_->preempt();
    }
}

/**
 * @brief MTC Task의 로봇 모델과 현재 Planning Scene을 설정하고 시작 Stage를 추가한다.
 * @details Task의 group, eef, ik_frame 속성을 초기화한 뒤 현재 장면을 담은 FixedState를 추가한다.
 *          계획이나 로봇 실행은 하지 않는다.
 * @param task 설정하고 시작 Stage를 추가할 MTC 작업.
 * @param label Task에 표시할 이름.
 * @return task에 소유권이 전달된 FixedState Stage의 포인터.
 */
mtc::Stage *MtcTaskExecutor::configureTask(mtc::Task &task, const std::string &label)
{
    task.stages()->setName(label);
    task.loadRobotModel(node_.shared_from_this());
    task.setProperty("group", "ur5");
    task.setProperty("eef", "ur5_rg2_eef");
    task.setProperty("ik_frame", "rg2_hand");
    auto current = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
    current->setPlanningSceneMsg(scene_());
    auto fixed = std::make_unique<mtc::stages::FixedState>("measured current scene");
    fixed->setState(current);
    auto *start = fixed.get();
    task.add(std::move(fixed));
    return start;
}

/**
 * @brief 지정한 관절 목표로 이동하는 MoveTo Stage를 MTC Task에 추가한다.
 * @param task 이동 Stage를 추가할 MTC 작업.
 * @param label Stage에 표시할 이름.
 * @param group 이동할 MoveIt 관절 그룹 이름.
 * @param names 목표를 지정할 관절 이름 목록.
 * @param values names와 같은 순서의 목표 관절 위치(rad) 목록.
 * @return task에 소유권이 전달된 MoveTo Stage의 포인터.
 * @details OMPL 경로 계획기를 설정하고 Stage를 추가할 뿐, 계획·실행은 하지 않는다.
 * @throws std::out_of_range values의 원소가 names보다 적은 경우.
 */
mtc::Stage *MtcTaskExecutor::addJointMove(mtc::Task &task, const std::string &label, const std::string &group,
                         const std::vector<std::string> &names, const std::vector<double> &values)
{
    auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_.shared_from_this());
    planner->setPlannerId("ompl", "RRTConnectkConfigDefault");
    planner->setMaxVelocityScalingFactor(velocity_);
    planner->setMaxAccelerationScalingFactor(acceleration_);
    auto stage = std::make_unique<mtc::stages::MoveTo>(label, planner);
    stage->setGroup(group);
    stage->setTimeout(planning_time_);
    std::map<std::string, double> targets;
    for (size_t i = 0; i < names.size(); ++i)
    {
        targets[names[i]] = values.at(i);
    }
    stage->setGoal(targets);
    auto *result = stage.get();
    task.add(std::move(stage));
    return result;
}

/**
 * @brief 지정한 손 pose에 도달하기 위한 IK와 경로 계획 단계를 task에 추가한다.
 * @details 이 함수는 단계를 구성할 뿐 계획 실행이나 로봇 이동을 직접 수행하지 않는다.
 *          실제 계획과 실행은 호출자가 executeTask(task)를 호출할 때 수행된다.
 * @param task 이동 Stage를 추가할 MTC 작업.
 * @param label Stage 이름에 사용할 설명 문자열.
 * @param pose rg2_hand이 도달할 목표 위치와 방향.
 * @param start 목표 pose 생성 시 참조할 선행 Stage.
 */
void MtcTaskExecutor::addPoseMove(mtc::Task &task, const std::string &label,
                 const geometry_msgs::msg::PoseStamped &pose, mtc::Stage *start)
{
    auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_.shared_from_this());
    // 현재 노드를 사용하는 MoveIt 경로 계획기를 생성한다.
    planner->setPlannerId("ompl", "RRTConnectkConfigDefault");
    // OMPL의 RRTConnect 경로 계획 알고리즘을 선택한다.
    planner->setMaxVelocityScalingFactor(velocity_);
    // 관절 최대 속도의 사용 비율을 설정한다.
    planner->setMaxAccelerationScalingFactor(acceleration_);
    // 관절 최대 가속도의 사용 비율을 설정한다.
    auto generated = std::make_unique<mtc::stages::GeneratePose>(label + " pose");
    // 목표 위치와 방향을 생성하는 단계를 만든다.
    generated->setPose(pose);
    // 손이 도달할 목표 위치와 방향을 지정한다.
    generated->setMonitoredStage(start);
    // 목표 생성에 참고할 앞 단계를 지정한다.
    auto ik = std::make_unique<mtc::stages::ComputeIK>(label + " IK", std::move(generated));
    // 목표 생성 단계를 감싸는 IK 계산 단계를 만든다.
    ik->setGroup("ur5");
    // IK를 계산할 관절 그룹을 ur5로 지정한다.
    ik->setIKFrame("rg2_hand");
    // 목표 위치와 방향에 맞출 손 좌표계를 지정한다.
    ik->setMaxIKSolutions(32);
    // 찾을 IK 해의 최대 개수를 32개로 설정한다.
    ik->setMinSolutionDistance(.1);
    // 중복에 가까운 IK 해를 제외할 최소 관절 공간 거리를 설정한다.
    ik->setTimeout(planning_time_);
    // IK 단계의 계산 제한 시간을 설정한다.
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    // 인터페이스의 target_pose 속성을 IK 목표로 받아 사용한다.
    auto connect = std::make_unique<mtc::stages::Connect>("connect to " + label,
                                                          mtc::stages::Connect::GroupPlannerVector{{"ur5", planner}});
    connect->setTimeout(planning_time_);
    task.add(std::move(connect));
    task.add(std::move(ik));
}

/**
 * @brief MTC Task를 초기화·계획하고 첫 번째 해를 실행한 뒤 실제 관절 도달을 검증한다.
 * @param task Stage 구성이 완료된 MTC 작업.
 * @details 최대 3개 해를 요청하고 첫 번째 해의 궤적에 팔·그리퍼 컨트롤러를 지정한다.
 *          실행 대기가 실패하면 취소와 최종 결과 확인을 시도한 뒤 원래 예외를 전달한다.
 * @throws std::runtime_error 종료 요청, 계획 실패, 실행 거부·실패 또는 최종 관절 검증 실패 시.
 * @note MTC 초기화·계획 API 자체의 예외도 전달된다. 대기 처리는 작업 스레드에서 수행한다.
 */
void MtcTaskExecutor::executeTask(mtc::Task &task)
{
    if (stopping_)
    {
        throw std::runtime_error("Shutdown before planning");
    }
    task.setTimeout(planning_time_);
    task.init();
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        planning_task_ = &task;
    }
    bool planned = false;
    try
    {
        planned = bool(task.plan(3));
    }
    catch (...)
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        planning_task_ = nullptr;
        throw;
    }
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        planning_task_ = nullptr;
    }
    if (stopping_)
    {
        throw std::runtime_error("Shutdown during planning");
    }
    if (!planned || task.solutions().empty())
    {
        throw std::runtime_error("MTC could not find a solution");
    }
    ExecuteTask::Goal goal;
    task.solutions().front()->toMsg(goal.solution);
    std::map<std::string, double> targets;
    for (auto &sub : goal.solution.sub_trajectory)
    {
        const auto &trajectory = sub.trajectory.joint_trajectory;
        if (trajectory.points.empty())
        {
            continue;
        }
        sub.execution_info.controller_names.clear();
        for (const auto &name : trajectory.joint_names)
        {
            const std::string controller = std::find(ARM.begin(), ARM.end(), name) != ARM.end() ? "arm_trajectory_controller" : "gripper_trajectory_controller";
            if (std::find(sub.execution_info.controller_names.begin(),
                          sub.execution_info.controller_names.end(), controller) == sub.execution_info.controller_names.end())
            {
                sub.execution_info.controller_names.push_back(controller);
            }
        }
        const auto &point = trajectory.points.back();
        for (size_t i = 0; i < std::min(trajectory.joint_names.size(), point.positions.size()); ++i)
        {
            targets[trajectory.joint_names[i]] = point.positions[i];
        }
    }
    execute_solution_(goal);
    if (targets.empty())
    {
        throw std::runtime_error("MTC solution contains no joint trajectory");
    }
    verifyJoints(targets);
}

/**
 * @brief 실행 결과의 목표 관절값에 실제 관절이 도달했는지 최대 5초 동안 확인한다.
 * @param targets 확인할 관절 이름과 최종 목표 위치(rad)의 맵.
 * @details 각 관절 관측이 최신이고 목표 오차가 0.025 rad 이하여야 도달한 것으로 판단한다.
 * @throws std::runtime_error 종료가 요청되거나 제한 시간 안에 목표 도달을 확인하지 못한 경우.
 */
void MtcTaskExecutor::verifyJoints(const std::map<std::string, double> &targets)
{
    const auto begin = Clock::now();
    while (true)
    {
        const bool reached = joints_reached_(targets);
        if (reached)
        {
            return;
        }
        if (stopping_ || secondsSince(begin) > 5.)
        {
            throw std::runtime_error("Final joint state does not match executed trajectory");
        }
        std::this_thread::sleep_for(20ms);
    }
}
}  // namespace pick_place
