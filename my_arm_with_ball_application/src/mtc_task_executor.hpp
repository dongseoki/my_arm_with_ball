#ifndef MY_ARM_WITH_BALL_APPLICATION_MTC_TASK_EXECUTOR_HPP_
#define MY_ARM_WITH_BALL_APPLICATION_MTC_TASK_EXECUTOR_HPP_

#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

namespace pick_place
{
/// @brief MTC 작업 구성·계획·실행 및 최종 관절 도달 확인. 노드는 작업 종료까지 살아 있어야 한다.
class MtcTaskExecutor
{
public:
    using Scene = std::function<moveit_msgs::msg::PlanningScene()>;
    using JointsReached = std::function<bool(const std::map<std::string, double> &)>;
    MtcTaskExecutor(rclcpp::Node &node, std::atomic<bool> &stopping,
        double planning_time, double velocity, double acceleration, double operation_timeout,
        Scene scene, JointsReached joints_reached);
    /// @brief 진행 중인 MTC 경로 계획을 중단해 달라고 요청한다.
    /// @note 중단 완료를 기다리지 않으며 로봇 비상정지나 실행 중 Action 취소가 아니다.
    void preempt();
    bool isExecutionReady() const;
    moveit::task_constructor::Stage *configureTask(moveit::task_constructor::Task &task, const std::string &label);
    moveit::task_constructor::Stage *addJointMove(moveit::task_constructor::Task &task,
        const std::string &label, const std::string &group,
        const std::vector<std::string> &names, const std::vector<double> &values);
    void addPoseMove(moveit::task_constructor::Task &task, const std::string &label,
        const geometry_msgs::msg::PoseStamped &pose, moveit::task_constructor::Stage *start);
    void executeTask(moveit::task_constructor::Task &task);
    void verifyJoints(const std::map<std::string, double> &targets);

private:
    using ExecuteTask = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
    void executeMtcSolution(const ExecuteTask::Goal &goal);
    rclcpp::Node &node_;
    std::atomic<bool> &stopping_;
    double planning_time_, velocity_, acceleration_, operation_timeout_;
    Scene scene_;
    rclcpp_action::Client<ExecuteTask>::SharedPtr execute_;
    JointsReached joints_reached_;
    std::mutex planning_mutex_;
    moveit::task_constructor::Task *planning_task_{nullptr};
};
}  // namespace pick_place
#endif  // MY_ARM_WITH_BALL_APPLICATION_MTC_TASK_EXECUTOR_HPP_
