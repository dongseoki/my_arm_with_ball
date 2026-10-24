#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <atomic>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "place_stability.hpp"

using namespace std::chrono_literals;
namespace mtc = moveit::task_constructor;
using MoveGroup = moveit_msgs::action::MoveGroup;
using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
using ExecuteTask = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
using Clock = std::chrono::steady_clock;
static const std::vector<std::string> ARM = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
static const std::vector<std::string> FINGERS = {"rg2_finger_joint1", "rg2_finger_joint2"};
static const std::vector<std::string> TOUCH = {"rg2_hand", "rg2_leftfinger", "rg2_rightfinger"};

class PickAndPlace : public rclcpp::Node
{
public:
    PickAndPlace() : Node("pick_and_place_test", rclcpp::NodeOptions()
                                                     .automatically_declare_parameters_from_overrides(true))
    {
        ready_timeout_ = parameter("ready_timeout", 120.0);
        operation_timeout_ = parameter("operation_timeout", 90.0);
        max_age_ = parameter("state_max_age", 2.0);
        place_timeout_ = parameter("place_timeout", 10.0);
        stable_speed_ = parameter("stable_speed", 0.02);
        stable_duration_ = parameter("stable_duration", 1.0);
        planning_time_ = parameter("planning_time", 30.0);
        velocity_ = parameter("velocity_scaling", 0.1);
        acceleration_ = parameter("acceleration_scaling", 0.1);
        start_ = parameter("start_joints", std::vector<double>{});
        pregrasp_ = parameter("pregrasp_joints", std::vector<double>{
                                                     -0.6082914162445299, 0.774475422756991, 0.7405722030642679,
                                                     -3.0858439511376563, 1.570796327804393, 0.962504912344154});
        grasp_ = poseParameter("grasp_pose", {0.55, -0.25, 0.23, 0., 0.70710678, 0., 0.70710678});
        lift_ = poseParameter("lift_pose", {0.55, -0.25, 0.40, 0., 0.70710678, 0., 0.70710678});
        place_ = poseParameter("place_pose", {0.75, 0.25, 0.4, 0., 0.70710678, 0., 0.70710678});
        retreat_ = poseParameter("retreat_pose", {0.75, 0.25, 0.55, 0., 0.70710678, 0., 0.70710678});
        auto bounds = parameter("basket_bounds", std::vector<double>{.71, .79, .244, .256, .033, .11});
        if (bounds.size() != 6 || pregrasp_.size() != 6 ||
            (!start_.empty() && start_.size() != 6))
        {
            throw std::invalid_argument("Invalid parameter size");
        }
        for (size_t i = 0; i < 3; ++i)
        {
            if (!std::isfinite(bounds[2 * i]) || !std::isfinite(bounds[2 * i + 1]) ||
                bounds[2 * i] >= bounds[2 * i + 1])
            {
                throw std::invalid_argument("Invalid basket bounds");
            }
        }
        for (double value : {ready_timeout_, operation_timeout_, max_age_, place_timeout_,
                             stable_speed_, stable_duration_, planning_time_, velocity_, acceleration_})
        {
            if (!std::isfinite(value) || value <= 0.)
            {
                throw std::invalid_argument("Invalid positive parameter");
            }
        }
        for (const auto &values : {start_, pregrasp_})
        {
            if (!std::all_of(values.begin(), values.end(), [](double x)
                             { return std::isfinite(x); }))
            {
                throw std::invalid_argument("Nonfinite joint target");
            }
        }
        if (velocity_ > 1. || acceleration_ > 1.)
        {
            throw std::invalid_argument("Scaling must be <= 1");
        }
        std::array<double, 6> limits;
        std::copy(bounds.begin(), bounds.end(), limits.begin());
        stability_ = std::make_unique<pick_place::PlaceStability>(limits, stable_speed_, stable_duration_);
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                for (size_t i = 0; i < std::min(msg->name.size(), msg->position.size()); ++i) {
                    if (std::isfinite(msg->position[i])) {
                        joints_[msg->name[i]] = msg->position[i]; joint_stamps_[msg->name[i]] = Clock::now();
                    }
                }
                joint_received_ = Clock::now();
            });
        contact_sub_ = create_subscription<std_msgs::msg::Bool>("/grasp/contact", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                contact_ = msg->data; contact_received_ = Clock::now();
            });
        attached_sub_ = create_subscription<std_msgs::msg::Bool>("/grasp/attached", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                attached_ = msg->data; attached_received_ = Clock::now();
            });
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/grasp/ball_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                const auto & p = msg->pose;
                const double norm = std::sqrt(p.orientation.x*p.orientation.x + p.orientation.y*p.orientation.y +
                    p.orientation.z*p.orientation.z + p.orientation.w*p.orientation.w);
                if (msg->header.frame_id != "world" || !std::isfinite(p.position.x) ||
                    !std::isfinite(p.position.y) || !std::isfinite(p.position.z) ||
                    !std::isfinite(norm) || norm < 1e-9) {
                    stability_->reset(); stable_ = false; return;
                }
                if (pose_received_ && secondsSince(*pose_received_) > max_age_) {stability_->reset();}
                pose_ = *msg; pose_received_ = Clock::now();
                if (state_ == State::VERIFY_PLACE) {
                    stable_ = stability_->update({msg->pose.position.x, msg->pose.position.y,
                        msg->pose.position.z}, rclcpp::Time(msg->header.stamp).seconds());
                }
            });
        controllers_ = create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
        get_scene_ = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        apply_scene_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
        attach_ = create_client<std_srvs::srv::Trigger>("/grasp/attach");
        detach_ = create_client<std_srvs::srv::Trigger>("/grasp/detach");
        move_ = rclcpp_action::create_client<MoveGroup>(this, "/move_action");
        arm_controller_ = rclcpp_action::create_client<FollowTrajectory>(this, "/arm_trajectory_controller/follow_joint_trajectory");
        gripper_controller_ = rclcpp_action::create_client<FollowTrajectory>(this, "/gripper_trajectory_controller/follow_joint_trajectory");
        execute_ = rclcpp_action::create_client<ExecuteTask>(this, "/execute_task_solution");
        started_ = Clock::now();
        timer_ = create_wall_timer(100ms, std::bind(&PickAndPlace::tick, this));
        RCLCPP_INFO(get_logger(), "WAIT_READY: waiting for controllers, MoveIt and fresh simulation state");
    }

    ~PickAndPlace() override
    {
        stopping_ = true;
        {
            std::lock_guard<std::mutex> lock(planning_mutex_);
            if (planning_task_)
            {
                planning_task_->preempt();
            }
        }
        // Worker awaits check stopping every 50ms; MTC plans have an explicit
        // timeout and receive preempt above. Join before destroying clients/data.
        if (work_.valid())
        {
            work_.wait();
        } // 시작한 작업의 결과를 아직 가져오지 않았는지 확인한다.
    }

private:
    /// @brief 자동 1회 Pick-and-Place의 실행 상태. tick()이 작업 결과에 따라 전환한다.
    enum class State
    {
        WAIT_READY,        ///< 로봇 제어기, MoveIt, 센서 정보와 작업 장면이 준비되기를 기다리는 상태를 의미한다.
        APPROACH,          ///< 그리퍼를 열고 준비 자세를 거쳐 공을 잡을 위치로 이동하는 상태를 의미한다.
        CLOSE,             ///< 공에 닿을 때까지 그리퍼를 닫고, 잡은 공을 Gazebo에서 손에 부착하는 상태를 의미한다.
        ATTACH,            ///< Gazebo에서 잡은 공을 MoveIt도 손에 붙은 물체로 인식하도록 반영하고 확인하는 상태를 의미한다.
        TRANSPORT,         ///< 잡은 공을 위로 들어 올린 뒤 바구니 위까지 옮기는 상태를 의미한다.
        RELEASE,           ///< 그리퍼를 열어 공을 손에서 분리하고, MoveIt에도 분리 결과를 반영하는 상태를 의미한다.
        VERIFY_PLACE,      ///< 놓은 공이 바구니 안에 있고 일정 시간 거의 움직이지 않는지 확인하며 기다리는 상태를 의미한다.
        REFRESH_SCENE,     ///< 안정된 공의 최신 위치를 MoveIt 작업 장면에 다시 반영하는 상태를 의미한다.
        RETURN,            ///< 공을 놓은 위치에서 물러나 저장된 시작 관절 자세로 돌아가는 상태를 의미한다.
        SUCCEEDED_WAITING, ///< 작업을 성공적으로 마친 뒤 재실행 없이 노드와 시뮬레이션을 유지하며 기다리는 상태를 의미한다.
        FAILED_WAITING     ///< 작업 실패 원인을 기록한 뒤 자동 재시도나 복귀 없이 노드와 시뮬레이션을 유지하며 기다리는 상태를 의미한다.
    };
    template <class T>
    T parameter(const std::string &name, const T &fallback)
    {
        if (!has_parameter(name))
        {
            declare_parameter<T>(name, fallback);
        }
        return get_parameter(name).get_value<T>();
    }
    geometry_msgs::msg::PoseStamped poseParameter(const std::string &name, const std::vector<double> &defaults)
    {
        const auto v = parameter(name, defaults);
        if (v.size() != 7 || !std::all_of(v.begin(), v.end(), [](double x)
                                          { return std::isfinite(x); }))
        {
            throw std::invalid_argument("Invalid pose: " + name);
        }
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "world";
        p.pose.position.x = v[0];
        p.pose.position.y = v[1];
        p.pose.position.z = v[2];
        const double norm = std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6]);
        if (norm < 1e-9)
        {
            throw std::invalid_argument("Zero quaternion: " + name);
        }
        p.pose.orientation.x = v[3] / norm;
        p.pose.orientation.y = v[4] / norm;
        p.pose.orientation.z = v[5] / norm;
        p.pose.orientation.w = v[6] / norm;
        return p;
    }
    static double secondsSince(Clock::time_point t)
    {
        return std::chrono::duration<double>(Clock::now() - t).count();
    }
    bool fresh(const std::optional<Clock::time_point> &t) const
    {
        return t && secondsSince(*t) < max_age_;
    }
    std::map<std::string, double> measured()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &names : {ARM, FINGERS})
        {
            for (const auto &name : names)
            {
                if (!joint_stamps_.count(name) || secondsSince(joint_stamps_.at(name)) > max_age_)
                {
                    throw std::runtime_error("Stale joint observation: " + name);
                }
            }
        }
        return joints_;
    }
    geometry_msgs::msg::PoseStamped ball()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!fresh(pose_received_) || pose_.header.frame_id != "world")
        {
            throw std::runtime_error("Ball pose stale or not world-frame");
        }
        return pose_;
    }
    template <class Future>
    auto await(Future &&future, double timeout, const std::string &label)
    {
        const auto begin = Clock::now();
        while (future.wait_for(50ms) != std::future_status::ready)
        {
            if (stopping_ || !rclcpp::ok() || secondsSince(begin) > timeout)
            {
                throw std::runtime_error(label + " timeout/shutdown");
            }
        }
        return future.get();
    }
    template <class ClientPtr, class Goal>
    auto sendGoal(const ClientPtr &client, const Goal &goal, double timeout, const std::string &label)
    {
        using Client = typename std::decay_t<ClientPtr>::element_type;
        using GoalHandle = typename Client::GoalHandle;
        struct ResponseState
        {
            std::mutex mutex;
            bool abandoned{false};
            typename GoalHandle::SharedPtr handle;
        };
        auto state = std::make_shared<ResponseState>();
        typename Client::SendGoalOptions options;
        options.goal_response_callback = [this, client, state, label](typename GoalHandle::SharedPtr handle)
        {
            bool cancel = false;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->handle = handle;
                cancel = state->abandoned && static_cast<bool>(handle);
            }
            if (cancel)
            {
                cancelLateGoal(client, handle, label);
            }
        };
        auto future = client->async_send_goal(goal, options);
        try
        {
            return await(future, timeout, label);
        }
        catch (...)
        {
            typename GoalHandle::SharedPtr handle;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->abandoned = true;
                handle = state->handle;
            }
            if (handle)
            {
                cancelLateGoal(client, handle, label);
            }
            throw;
        }
    }
    template <class ClientPtr, class GoalHandlePtr>
    void cancelLateGoal(const ClientPtr &client, const GoalHandlePtr &handle, const std::string &label)
    {
        const auto logger = get_logger();
        RCLCPP_ERROR(logger, "%s goal response arrived after timeout; cancelling the late goal", label.c_str());
        try
        {
            client->async_cancel_goal(handle, [logger, label](const auto &response)
                                      {
                if (response->goals_canceling.empty()) {
                    RCLCPP_ERROR(logger, "%s late-goal cancel request was not accepted", label.c_str());
                } });
            client->async_get_result(handle, [logger, label](const auto &result)
                                     {
                if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_WARN(logger, "%s late goal cancellation confirmed", label.c_str());
                } else {
                    RCLCPP_ERROR(logger, "%s late goal terminal result was not cancellation (code=%d)",
                        label.c_str(), static_cast<int>(result.code));
                } });
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(logger, "%s late goal cancellation could not be confirmed: %s", label.c_str(), ex.what());
        }
    }
    template <class Service>
    auto call(typename rclcpp::Client<Service>::SharedPtr client,
              const std::shared_ptr<typename Service::Request> &request)
    {
        if (!client->service_is_ready())
        {
            throw std::runtime_error("Service unavailable: " + std::string(client->get_service_name()));
        }
        auto pending = client->async_send_request(request);
        try
        {
            return await(pending, 5., client->get_service_name());
        }
        catch (...)
        {
            client->remove_pending_request(pending);
            throw;
        }
    }
    moveit_msgs::msg::PlanningScene scene()
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = 1023;
        // Complete live scene, including robot state and transforms.
        return call<moveit_msgs::srv::GetPlanningScene>(get_scene_, request)->scene;
    }
    std::string readiness()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!fresh(joint_received_) || !fresh(contact_received_) || !fresh(attached_received_) || !fresh(pose_received_))
            {
                return "fresh joint/contact/attachment/ball observations";
            }
            for (const auto &names : {ARM, FINGERS})
            {
                for (const auto &name : names)
                {
                    if (!joints_.count(name) || !joint_stamps_.count(name) ||
                        secondsSince(joint_stamps_.at(name)) > max_age_)
                    {
                        return "fresh joint " + name;
                    }
                }
            }
            if (attached_.value_or(false))
            {
                return "ball unexpectedly attached before start";
            }
        }
        if (!move_->action_server_is_ready() || !execute_->action_server_is_ready() ||
            !arm_controller_->action_server_is_ready() || !gripper_controller_->action_server_is_ready())
        {
            return "MoveIt action interfaces";
        }
        if (!attach_->service_is_ready() || !detach_->service_is_ready() || !apply_scene_->service_is_ready())
        {
            return "grasp/scene services";
        }
        auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        auto response = call<controller_manager_msgs::srv::ListControllers>(controllers_, request);
        for (const auto &name : {"joint_state_broadcaster", "arm_trajectory_controller", "gripper_trajectory_controller"})
        {
            const auto found = std::find_if(response->controller.begin(), response->controller.end(),
                                            [name](const auto &c)
                                            {
                                                return c.name == name && c.state == "active";
                                            });
            if (found == response->controller.end())
            {
                return std::string("active controller ") + name;
            }
        }
        auto current = scene();
        for (const auto &id : {"apple", "basket"})
        {
            if (std::none_of(current.world.collision_objects.begin(), current.world.collision_objects.end(),
                             [id](const auto &obj)
                             {
                                 return obj.id == id;
                             }))
            {
                return std::string("scene object ") + id;
            }
        }
        return "";
    }
    void apply(moveit_msgs::msg::PlanningScene diff)
    {
        diff.is_diff = true;
        diff.robot_state.is_diff = true;
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene = std::move(diff);
        if (!call<moveit_msgs::srv::ApplyPlanningScene>(apply_scene_, request)->success)
        {
            throw std::runtime_error("Planning Scene rejected diff");
        }
    }
    moveit_msgs::msg::CollisionObject sphere(const geometry_msgs::msg::Pose &pose, const std::string &frame)
    {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = frame;
        object.id = "apple";
        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape.SPHERE;
        shape.dimensions = {.0325};
        object.primitives.push_back(shape);
        object.primitive_poses.push_back(pose);
        return object;
    }
    void touchAllowed(bool allowed)
    {
        auto matrix = scene().allowed_collision_matrix;
        auto names = TOUCH;
        names.push_back("apple");
        names.push_back("pick_area_floor");
        for (const auto &name : names)
        {
            if (std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name) == matrix.entry_names.end())
            {
                matrix.entry_names.push_back(name);
                for (auto &entry : matrix.entry_values)
                {
                    entry.enabled.push_back(false);
                }
                moveit_msgs::msg::AllowedCollisionEntry entry;
                entry.enabled.resize(matrix.entry_names.size(), false);
                matrix.entry_values.push_back(entry);
            }
        }
        auto index = [&matrix](const std::string &name)
        {
            return std::distance(matrix.entry_names.begin(), std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name));
        };
        const auto apple = index("apple");
        for (const auto &name : TOUCH)
        {
            matrix.entry_values[apple].enabled[index(name)] = allowed;
            matrix.entry_values[index(name)].enabled[apple] = allowed;
        }
        matrix.entry_values[apple].enabled[index("pick_area_floor")] = true;
        matrix.entry_values[index("pick_area_floor")].enabled[apple] = true;
        moveit_msgs::msg::PlanningScene diff;
        diff.allowed_collision_matrix = matrix;
        apply(diff);
    }
    /**
     * @brief Planning Scene의 공이 기대한 소속·형상·좌표계·위치로 등록됐는지 검증한다.
     * @param attached true이면 rg2_hand에 부착된 공 하나만, false이면 world 물체 목록의
     *                 공 하나만 존재해야 한다.
     * @param expected 비교 기준 공. header.frame_id와 primitive_poses[0]의 중심 위치를 사용한다.
     * @details 장면 조회 → 공 수집 → 소속·개수 확인 → 형상·좌표계 확인 → 중심 위치 계산
     *          → 위치 오차 확인의 6단계로 처리한다. 반지름은 0.0325 m를 기준으로 검사한다.
     *          장면을 변경하거나 Gazebo의 실제 파지 상태를 확인하지는 않는다.
     *          구의 중심 위치만 비교하며 공의 방향 일치 여부는 검사하지 않는다.
     * @throws std::runtime_error 장면 조회에 실패하거나 공의 소속·개수·형상·좌표계가
     *                            맞지 않거나 중심 위치 오차가 0.005 m를 초과한 경우.
     */
    void verifyScene(bool attached, const moveit_msgs::msg::CollisionObject &expected)
    {
        // 1. MoveIt에서 현재 Planning Scene을 조회한다.
        const auto current = scene();
        // 2. 양쪽 목록의 apple 개수를 세고, 기대한 소속과 부착 링크에 맞는 공을 수집한다.
        std::vector<moveit_msgs::msg::CollisionObject> objects;
        size_t world_count = 0, attached_count = 0;
        for (const auto &obj : current.world.collision_objects)
        {
            if (obj.id == "apple")
            {
                ++world_count;
                if (!attached)
                {
                    objects.push_back(obj);
                }
            }
        }
        for (const auto &obj : current.robot_state.attached_collision_objects)
        {
            if (obj.object.id == "apple")
            {
                ++attached_count;
                if (attached && obj.link_name == "rg2_hand")
                {
                    objects.push_back(obj.object);
                }
            }
        }
        // 3. 원하는 목록에 공이 정확히 하나 있고 반대쪽에는 없으며, 검증 후보도 하나인지 확인한다.
        if (world_count != (attached ? 0u : 1u) || attached_count != (attached ? 1u : 0u) || objects.size() != 1)
        {
            throw std::runtime_error("Planning Scene world/attached membership mismatch");
        }
        // 4. 단일 구 형상, 반지름 0.0325 m(허용 오차 1e-6 m), 기대한 좌표계인지 확인한다.
        const auto &obj = objects.front();
        if (obj.primitives.size() != 1 || obj.primitive_poses.size() != 1 ||
            obj.primitives[0].type != shape_msgs::msg::SolidPrimitive::SPHERE ||
            obj.primitives[0].dimensions.size() != 1 || std::abs(obj.primitives[0].dimensions[0] - .0325) > 1e-6 ||
            obj.header.frame_id != expected.header.frame_id)
        {
            throw std::runtime_error("Planning Scene sphere geometry mismatch");
        }
        // 5. 물체 원점의 위치·회전과 구의 로컬 중심을 합성해 header.frame_id 기준 중심을 계산한다.
        // MoveIt이 primitive_poses를 물체 원점 쪽으로 재배치할 수 있어 로컬 위치만 비교하지 않는다.
        const auto &origin = obj.pose;
        Eigen::Quaterniond rotation(origin.orientation.w, origin.orientation.x,
                                    origin.orientation.y, origin.orientation.z);
        if (rotation.norm() < 1e-9)
        {
            rotation = Eigen::Quaterniond::Identity();
        }
        else
        {
            rotation.normalize();
        }
        const auto &local = obj.primitive_poses[0].position;
        const Eigen::Vector3d actual = Eigen::Vector3d(origin.position.x, origin.position.y,
                                                       origin.position.z) +
                                       rotation * Eigen::Vector3d(local.x, local.y, local.z);
        // 6. 기대한 중심과의 거리 오차가 0.005 m(5 mm)를 초과하면 검증 실패로 처리한다.
        const auto &b = expected.primitive_poses[0].position;
        if ((actual - Eigen::Vector3d(b.x, b.y, b.z)).norm() > .005)
        {
            throw std::runtime_error("Planning Scene ball pose mismatch: actual=" +
                                     std::to_string(actual.x()) + "," + std::to_string(actual.y()) + "," +
                                     std::to_string(actual.z()) + " expected=" + std::to_string(b.x) + "," +
                                     std::to_string(b.y) + "," + std::to_string(b.z));
        }
    }
    /**
     * @brief Gazebo에서 측정한 공의 위치·자세를 MoveIt Planning Scene에 반영하고 검증한다.
     * @param attached true이면 손에 부착된 물체로, false이면 world의 독립 물체로 등록한다.
     * @param refresh attached가 false일 때 true이면 기존 부착 정보 제거를 생략하고
     *                world 물체를 갱신한다. attached가 true이면 사용하지 않는다.
     * @details 측정 → 부착 여부에 따른 변경 내용 구성 → 적용 → 검증의 4단계로 처리한다.
     *          Gazebo의 실제 부착·분리는 수행하지 않고 MoveIt의 충돌 물체 정보만 동기화한다.
     * @throws std::runtime_error 측정값 또는 공의 자세가 유효하지 않거나,
     *                            Planning Scene 적용·검증에 실패한 경우.
     */
    void syncScene(bool attached, bool refresh = false)
    {
        // 1. 공의 현재 위치·자세를 읽고 변경 메시지와 검증 기준을 준비한다.
        const auto measured_ball = ball();
        moveit_msgs::msg::PlanningScene diff;
        moveit_msgs::msg::CollisionObject expected;
        // 2. 부착 여부에 맞게 Planning Scene에 반영할 변경 내용을 구성한다.
        if (attached)
        {
            // 현재 관절값으로 손의 world 기준 위치·자세를 계산한다.
            mtc::Task model_task;
            model_task.loadRobotModel(shared_from_this());
            moveit::core::RobotState state(model_task.getRobotModel());
            state.setToDefaultValues();
            state.setVariablePositions(measured());
            state.update();
            const auto &hand = state.getGlobalLinkTransform("rg2_hand");
            const auto &p = measured_ball.pose;
            Eigen::Quaterniond orientation(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
            if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
            {
                throw std::runtime_error("Invalid ball quaternion");
            }
            Eigen::Isometry3d world_ball = Eigen::Isometry3d::Identity();
            world_ball.linear() = orientation.normalized().toRotationMatrix();
            world_ball.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
            // world 기준 공의 위치·자세를 rg2_hand 기준으로 변환한다.
            const auto relative = hand.inverse() * world_ball;
            geometry_msgs::msg::Pose local;
            local.position.x = relative.translation().x();
            local.position.y = relative.translation().y();
            local.position.z = relative.translation().z();
            Eigen::Quaterniond q(relative.rotation());
            local.orientation.x = q.x();
            local.orientation.y = q.y();
            local.orientation.z = q.z();
            local.orientation.w = q.w();
            // 변환한 공을 손에 부착된 충돌 물체로 등록한다.
            expected = sphere(local, "rg2_hand");
            moveit_msgs::msg::AttachedCollisionObject object;
            object.link_name = "rg2_hand";
            object.touch_links = TOUCH;
            object.object = expected;
            diff.robot_state.attached_collision_objects.push_back(object);
        }
        else
        {
            // 일반 분리에서는 부착 정보를 제거하고, 단순 갱신에서는 제거를 생략한다.
            if (!refresh)
            {
                moveit_msgs::msg::AttachedCollisionObject object;
                object.link_name = "rg2_hand";
                object.object.id = "apple";
                object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
                diff.robot_state.attached_collision_objects.push_back(object);
            }
            // 측정한 공을 world 기준의 독립 충돌 물체로 등록한다.
            expected = sphere(measured_ball.pose, "world");
            diff.world.collision_objects.push_back(expected);
        }
        // 3. 준비한 변경 내용을 MoveIt Planning Scene에 적용한다.
        apply(diff);
        // 4. 공의 world/attached 소속·개수, 구 형상·반지름, 좌표계·위치를 검증한다.
        verifyScene(attached, expected);
    }
    void requireAttachment(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!fresh(attached_received_) || attached_.value_or(!value) != value)
        {
            throw std::runtime_error("Gazebo attachment state mismatch/stale");
        }
    }
    void trigger(bool attach)
    {
        auto client = attach ? attach_ : detach_;
        auto response = call<std_srvs::srv::Trigger>(client, std::make_shared<std_srvs::srv::Trigger::Request>());
        if (!response->success)
        {
            throw std::runtime_error(response->message);
        }
        const auto begin = Clock::now();
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (fresh(attached_received_) && attached_.value_or(!attach) == attach)
                {
                    return;
                }
            }
            if (stopping_ || secondsSince(begin) > 3.)
            {
                throw std::runtime_error("Gazebo attachment confirmation timeout");
            }
            std::this_thread::sleep_for(20ms);
        }
    }
    void verifyJoints(const std::map<std::string, double> &targets)
    {
        const auto begin = Clock::now();
        while (true)
        {
            bool reached = true;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                reached = fresh(joint_received_);
                for (const auto &[name, position] : targets)
                {
                    if (!joints_.count(name) || !joint_stamps_.count(name) ||
                        secondsSince(joint_stamps_.at(name)) > max_age_ || std::abs(joints_.at(name) - position) > .025)
                    {
                        reached = false;
                    }
                }
            }
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
    void executeTask(mtc::Task &task)
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
        auto handle = sendGoal(execute_, goal, 10., "MTC execute goal");
        if (!handle)
        {
            throw std::runtime_error("MTC execution rejected");
        }
        try
        {
            auto result = await(execute_->async_get_result(handle), operation_timeout_, "MTC execution");
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code.val != 1)
            {
                throw std::runtime_error("MTC execution failed: " + std::to_string(result.result->error_code.val));
            }
        }
        catch (...)
        {
            auto cancel = execute_->async_cancel_goal(handle);
            try
            {
                await(cancel, 5., "MTC cancel");
                await(execute_->async_get_result(handle), 10., "MTC terminal result");
            }
            catch (const std::exception &cleanup)
            {
                RCLCPP_ERROR(get_logger(), "MTC goal terminal state UNCONFIRMED after cancellation: %s", cleanup.what());
            }
            throw;
        }
        if (targets.empty())
        {
            throw std::runtime_error("MTC solution contains no joint trajectory");
        }
        verifyJoints(targets);
    }
    mtc::Stage *configureTask(mtc::Task &task, const std::string &label)
    {
        task.stages()->setName(label);
        task.loadRobotModel(shared_from_this());
        task.setProperty("group", "ur5");
        task.setProperty("eef", "ur5_rg2_eef");
        task.setProperty("ik_frame", "rg2_hand");
        auto current = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
        current->setPlanningSceneMsg(scene());
        auto fixed = std::make_unique<mtc::stages::FixedState>("measured current scene");
        fixed->setState(current);
        auto *start = fixed.get();
        task.add(std::move(fixed));
        return start;
    }
    mtc::Stage *addJointMove(mtc::Task &task, const std::string &label, const std::string &group,
                             const std::vector<std::string> &names, const std::vector<double> &values)
    {
        auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
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
    void addPoseMove(mtc::Task &task, const std::string &label,
                     const geometry_msgs::msg::PoseStamped &pose, mtc::Stage *start)
    {
        auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
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
     * @brief 그리퍼를 열고 공을 잡기 직전의 자세까지 이동하는 MTC Task.
     * @verbatim
     * Task 생성 전: touchAllowed(true)로 손-공 접촉을 Planning Scene에서 허용
     *
     * ↕ measured current scene       [FixedState: 현재 Planning Scene]
     * ↓ open fingers                 [MoveTo: 그리퍼 관절 → {1.18, 1.18}]
     * ↓ start pose                   [MoveTo: 팔 관절 → start_]
     * ↓ pregrasp                     [MoveTo: 팔 관절 → pregrasp_]
     * ‖ connect to approach apple    [Connect: pregrasp에서 목표 IK 자세까지 연결]
     * ↕ approach apple IK            [ComputeIK: 손 목표 pose → 팔 관절 자세]
     *     ↕ approach apple pose      [GeneratePose: grasp_, pregrasp 단계 관측]
     * @endverbatim
     * 화살표는 생성(↕), 이동(↓), 연결(‖) 역할을 구분한 개요다.
     * addJointMove/addPoseMove는 단계만 추가하며, executeTask(task)가 계획·실행한다.
     * 접촉 닫기·Gazebo 부착은 이후 closeContact()에서, 장면 부착·운반은 후속 상태에서 수행한다.
     */
    void approach()
    {
        touchAllowed(true);
        mtc::Task task;
        configureTask(task, "stage5 approach");
        addJointMove(task, "open fingers", "gripper", FINGERS, {1.18, 1.18});
        addJointMove(task, "start pose", "ur5", ARM, start_);
        auto *pregrasp = addJointMove(task, "pregrasp", "ur5", ARM, pregrasp_);
        addPoseMove(task, "approach apple", grasp_, pregrasp);
        executeTask(task);
    }
    void closeContact()
    {
        // 1단계: 양쪽 손가락 관절을 닫기 목표로 설정하고 계획 조건을 구성한다.
        MoveGroup::Goal goal;
        auto &request = goal.request;
        request.group_name = "gripper";
        request.pipeline_id = "ompl";
        request.planner_id = "RRTConnectkConfigDefault";
        request.allowed_planning_time = planning_time_;
        request.num_planning_attempts = 5;
        request.max_velocity_scaling_factor = velocity_;
        request.max_acceleration_scaling_factor = acceleration_;
        request.start_state.is_diff = true;
        moveit_msgs::msg::Constraints constraints;
        for (const auto &name : FINGERS)
        {
            moveit_msgs::msg::JointConstraint joint;
            joint.joint_name = name;
            joint.position = .2;
            joint.tolerance_above = .005;
            joint.tolerance_below = .005;
            joint.weight = 1.;
            constraints.joint_constraints.push_back(joint);
        }
        request.goal_constraints.push_back(constraints);
        goal.planning_options.planning_scene_diff.is_diff = true;
        goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
        // 2단계: MoveGroup Action에 닫기 goal을 보내고 승인되지 않으면 실패 처리한다.
        auto handle = sendGoal(move_, goal, 10., "close acceptance");
        if (!handle)
        {
            throw std::runtime_error("Gripper close rejected");
        }
        auto result_future = move_->async_get_result(handle);
        try
        {
            // 3단계: 최신 접촉 신호 또는 닫기 Action 종료를 기다린다.
            const auto begin = Clock::now();
            bool contact = false;
            while (true)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    contact = fresh(contact_received_) && contact_.value_or(false);
                }
                if (contact || result_future.wait_for(20ms) == std::future_status::ready)
                {
                    break;
                }
                if (stopping_ || secondsSince(begin) > 30.)
                {
                    throw std::runtime_error("Finger contact timeout");
                }
            }
            if (!contact)
            {
                throw std::runtime_error("Gripper finished closing without contact");
            }
            // 4단계: 접촉 뒤에도 goal이 진행 중이면 취소하고 최종 결과를 확인한다.
            if (result_future.wait_for(0ms) != std::future_status::ready)
            {
                const auto cancel = await(move_->async_cancel_goal(handle), 5., "close cancellation");
                if (cancel->goals_canceling.empty())
                {
                    throw std::runtime_error("Close cancellation not accepted");
                }
            }
            const auto result = await(result_future, 10., "close terminal result");
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED && result.code != rclcpp_action::ResultCode::CANCELED)
            {
                throw std::runtime_error("Close did not terminate successfully/canceled");
            }
            // 5단계: 실제 손가락 관절값과 최소 개방량 조건을 확인한다.
            const auto q = measured();
            if (q.at(FINGERS[0]) < -.002 || q.at(FINGERS[1]) < -.002 ||
                q.at(FINGERS[0]) + q.at(FINGERS[1]) <= 5. * M_PI / 180.)
            {
                throw std::runtime_error("Contact close opening <= 5 degrees");
            }
            // Gazebo requires recent contact: request attachment immediately after close termination.
            // 6단계: Gazebo 부착을 요청하고 실제 부착 상태가 확인될 때까지 기다린다.
            trigger(true);
        }
        catch (...)
        {
            if (result_future.wait_for(0ms) != std::future_status::ready)
            {
                try
                {
                    await(move_->async_cancel_goal(handle), 5., "failed close cancel");
                    await(result_future, 10., "failed close terminal");
                }
                catch (const std::exception &cleanup)
                {
                    RCLCPP_ERROR(get_logger(), "Close goal terminal state UNCONFIRMED after cancellation: %s", cleanup.what());
                }
            }
            throw;
        }
    }
    /**
     * @brief 부착된 공을 수직으로 들어 올리고, 별도의 Task로 바구니 위까지 운반한다.
     * @verbatim
     * Task 1: stage5 lift
     * ↕ measured current scene       [FixedState: 현재 Planning Scene]
     * ↓ lift apple vertically        [MoveRelative + CartesianPath: world +Z 이동]
     *
     * executeTask(lift) 완료 → Gazebo 부착 확인 → 측정된 손-공 XY 오프셋으로 목표 보정
     *
     * Task 2: stage5 transport
     * ↕ measured current scene       [FixedState: lift 후 새로 읽은 Planning Scene]
     * ‖ connect to above basket      [Connect: 현재 자세에서 목표 IK 자세까지 경로 생성]
     * ↕ above basket IK              [ComputeIK: 보정된 목표 pose의 관절 자세 계산]
     *     ↕ above basket pose        [GeneratePose: 보정된 place_ 목표]
     * @endverbatim
     * 각 Task는 executeTask()에서 계획·실행한다. 두 Task를 한 번에 미리 계획하지 않는다.
     * 부착 상태는 lift 전후 및 운반 후에 확인한다. 여기서는 공을 놓지 않는다.
     * 중간의 model Task는 손 위치 계산에 필요한 로봇 모델만 읽으며 실행하지 않는다.
     */
    void transport()
    {
        requireAttachment(true);
        mtc::Task lift;
        configureTask(lift, "stage5 lift");
        moveit::core::RobotState lift_state(lift.getRobotModel());
        lift_state.setToDefaultValues();
        lift_state.setVariablePositions(measured());
        lift_state.update();
        const double lift_distance = lift_.pose.position.z -
                                     lift_state.getGlobalLinkTransform("rg2_hand").translation().z();
        if (!std::isfinite(lift_distance) || lift_distance < .01 || lift_distance > .5)
        {
            throw std::runtime_error("Invalid upward lift distance: " + std::to_string(lift_distance));
        }
        auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian->setStepSize(.005);
        cartesian->setMinFraction(1.0);
        cartesian->setMaxVelocityScalingFactor(velocity_);
        cartesian->setMaxAccelerationScalingFactor(acceleration_);
        auto lift_stage = std::make_unique<mtc::stages::MoveRelative>("lift apple vertically", cartesian);
        lift_stage->setGroup("ur5");
        lift_stage->setIKFrame("rg2_hand");
        lift_stage->setMinMaxDistance(lift_distance, lift_distance);
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "world";
        direction.vector.z = 1.0;
        lift_stage->setDirection(direction);
        lift.add(std::move(lift_stage));
        executeTask(lift);
        requireAttachment(true);
        // Keep the measured ball under the intended basket release XY despite hand-relative offset.
        mtc::Task model;
        model.loadRobotModel(shared_from_this());
        moveit::core::RobotState state(model.getRobotModel());
        state.setToDefaultValues();
        state.setVariablePositions(measured());
        state.update();
        const auto hand = state.getGlobalLinkTransform("rg2_hand").translation();
        const auto p = ball().pose.position;
        auto target = place_;
        target.pose.position.x -= p.x - hand.x();
        target.pose.position.y -= p.y - hand.y();
        RCLCPP_INFO(get_logger(), "Transport hand target=(%.6f, %.6f, %.6f), ball offset=(%.6f, %.6f)",
                    target.pose.position.x, target.pose.position.y, target.pose.position.z, p.x - hand.x(), p.y - hand.y());
        mtc::Task carry;
        auto *start = configureTask(carry, "stage5 transport");
        addPoseMove(carry, "above basket", target, start);
        executeTask(carry);
        requireAttachment(true);
    }
    /**
     * @brief 그리퍼를 연 뒤 Gazebo와 MoveIt에서 공을 분리한다.
     * @verbatim
     * Task 생성 전: Gazebo 부착 상태 확인
     *
     * ↕ measured current scene       [FixedState: 현재 Planning Scene]
     * ↓ open fingers                 [MoveTo: 그리퍼 관절 → {1.18, 1.18}]
     *
     * executeTask(task) 완료 → trigger(false) → syncScene(false)
     * @endverbatim
     * Gazebo 분리 요청·확인과 Planning Scene 분리는 MTC Stage가 아니라 Task 실행 후 처리다.
     * 공이 바구니 안에서 안정됐는지는 이후 VERIFY_PLACE 상태에서 확인한다.
     */
    void release()
    {
        requireAttachment(true);
        mtc::Task task;
        configureTask(task, "stage5 open for release");
        addJointMove(task, "open fingers", "gripper", FINGERS, {1.18, 1.18});
        executeTask(task);
        trigger(false);
        syncScene(false);
    }
    std::string stateName() const
    {
        switch (state_)
        {
        case State::WAIT_READY:
            return "WAIT_READY";
        case State::APPROACH:
            return "APPROACH";
        case State::CLOSE:
            return "CLOSE_CONTACT";
        case State::ATTACH:
            return "ATTACH_SCENE";
        case State::TRANSPORT:
            return "TRANSPORT";
        case State::RELEASE:
            return "RELEASE";
        case State::VERIFY_PLACE:
            return "VERIFY_PLACE";
        case State::REFRESH_SCENE:
            return "REFRESH_SCENE";
        case State::RETURN:
            return "RETURN";
        case State::SUCCEEDED_WAITING:
            return "SUCCEEDED_WAITING";
        default:
            return "FAILED_WAITING";
        }
    }
    void fail(const std::string &reason)
    {
        RCLCPP_ERROR(get_logger(), "STAGE5 FAILED stage=%s reason=%s; waiting without retry", stateName().c_str(), reason.c_str());
        state_ = State::FAILED_WAITING;
    }
    void advance(State next)
    {
        state_ = next;
        state_started_ = Clock::now();
        RCLCPP_INFO(get_logger(), "STAGE5 state=%s", stateName().c_str());
    }
    void tick()
    {
        // All shared observation/state accesses use this lock. Blocking calls only run in work_.
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::SUCCEEDED_WAITING || state_ == State::FAILED_WAITING)
        {
            return;
        }
        if (work_.valid())
        { // 시작한 작업의 결과를 아직 가져오지 않았는지 확인한다.
            if (work_.wait_for(0ms) != std::future_status::ready)
            {
                return;
            }
            try
            {
                const auto reason = work_.get();
                if (state_ == State::WAIT_READY)
                {
                    waiting_reason_ = reason;
                    if (!reason.empty())
                    {
                        if (secondsSince(started_) > ready_timeout_)
                        {
                            fail("Preparation timed out: " + reason);
                        }
                        return;
                    }
                    if (start_.empty())
                    {
                        for (const auto &name : ARM)
                        {
                            start_.push_back(joints_.at(name));
                        }
                    }
                    RCLCPP_INFO(get_logger(), "STAGE5 starting single automatic task");
                    advance(State::APPROACH);
                }
                else
                {
                    switch (state_)
                    {
                    case State::APPROACH:
                        advance(State::CLOSE);
                        break;
                    case State::CLOSE:
                        advance(State::ATTACH);
                        break;
                    case State::ATTACH:
                        advance(State::TRANSPORT);
                        break;
                    case State::TRANSPORT:
                        advance(State::RELEASE);
                        break;
                    case State::RELEASE:
                        stability_->reset();
                        stable_ = false;
                        advance(State::VERIFY_PLACE);
                        break;
                    case State::REFRESH_SCENE:
                        advance(State::RETURN);
                        break;
                    case State::RETURN:
                        advance(State::SUCCEEDED_WAITING);
                        RCLCPP_INFO(get_logger(), "STAGE5 PICK AND PLACE PASSED; waiting");
                        break;
                    default:
                        break;
                    }
                }
            }
            catch (const std::exception &ex)
            {
                fail(ex.what());
                return;
            }
        }
        if (state_ == State::WAIT_READY && secondsSince(started_) > ready_timeout_)
        {
            fail("Preparation timed out: " + waiting_reason_);
            return;
        }
        if (state_ == State::VERIFY_PLACE)
        {
            if (!fresh(pose_received_))
            {
                stability_->reset();
                stable_ = false;
            }
            if (stable_)
            {
                RCLCPP_INFO(get_logger(), "Place stable at (%.5f, %.5f, %.5f), speed %.5f m/s", pose_.pose.position.x,
                            pose_.pose.position.y, pose_.pose.position.z, stability_->last_speed());
                advance(State::REFRESH_SCENE);
            }
            else if (secondsSince(state_started_) > place_timeout_)
            {
                RCLCPP_ERROR(get_logger(),
                             "Place verification last observation=(%.5f, %.5f, %.5f), speed=%.5f m/s, fresh=%s",
                             pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z,
                             stability_->last_speed(), fresh(pose_received_) ? "true" : "false");
                fail("Ball outside basket or not continuously stable");
                return;
            }
            else
            {
                return;
            }
        }
        const State selected = state_;
        if (selected == State::SUCCEEDED_WAITING)
        {
            return;
        }
        work_ = std::async(std::launch::async, [this, selected]() -> std::string
                           {
            if (selected == State::WAIT_READY) {
                try {return readiness();
                } catch (const std::exception & ex) {return ex.what();
                }
            }
            switch (selected) {
                case State::APPROACH: approach();
                break;
                case State::CLOSE: closeContact();
                break;
                case State::ATTACH: requireAttachment(true);
                syncScene(true);
                break;
                case State::TRANSPORT: transport();
                break;
                case State::RELEASE: release();
                break;
                case State::REFRESH_SCENE: requireAttachment(false);
                syncScene(false, true);
                break;
                /**
                 * @brief 놓기 성공 후 손을 후퇴시키고 시작 관절 자세로 복귀하는 Task.
                 * @verbatim
                 * ↕ measured current scene       [FixedState: 최종 공 위치가 반영된 장면]
                 * ‖ connect to retreat           [Connect: 현재 자세에서 후퇴 IK 자세까지 경로 생성]
                 * ↕ retreat IK                   [ComputeIK: retreat_ 목표의 관절 자세 계산]
                 *     ↕ retreat pose             [GeneratePose: retreat_]
                 * ↓ return start                 [MoveTo: 팔 관절 → start_]
                 * @endverbatim
                 * executeTask(task)로 계획·실행한 뒤 touchAllowed(false)로 손-공 접촉 허용을 해제한다.
                 * 이 작업이 정상 반환하면 다음 tick()에서 SUCCEEDED_WAITING으로 전환한다.
                 */
                case State::RETURN: {
                    mtc::Task task;
                    auto * start = configureTask(task, "stage5 return");
                    addPoseMove(task, "retreat", retreat_, start);
                    addJointMove(task, "return start", "ur5", ARM, start_);
                    executeTask(task);
                    touchAllowed(false);
                    break;
                }
                default: break;
            }
            return ""; });
    }

    std::mutex mutex_, planning_mutex_;
    mtc::Task *planning_task_{nullptr};
    std::atomic<bool> stopping_{false};
    State state_{State::WAIT_READY};
    Clock::time_point started_, state_started_{Clock::now()};
    std::optional<Clock::time_point> joint_received_, contact_received_, attached_received_, pose_received_;
    std::map<std::string, double> joints_;
    std::map<std::string, Clock::time_point> joint_stamps_;
    std::optional<bool> contact_, attached_;
    geometry_msgs::msg::PoseStamped pose_, grasp_, lift_, place_, retreat_;
    std::vector<double> start_, pregrasp_;
    double ready_timeout_, operation_timeout_, max_age_, place_timeout_, stable_speed_, stable_duration_;
    double planning_time_, velocity_, acceleration_;
    bool stable_{false};
    std::string waiting_reason_{"required interfaces"};
    std::unique_ptr<pick_place::PlaceStability> stability_;
    std::future<std::string> work_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_, attached_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr attach_, detach_;
    rclcpp_action::Client<FollowTrajectory>::SharedPtr arm_controller_, gripper_controller_;
    rclcpp_action::Client<MoveGroup>::SharedPtr move_;
    rclcpp_action::Client<ExecuteTask>::SharedPtr execute_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<PickAndPlace>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pick_and_place_test"), "%s", ex.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
