#ifndef MY_ARM_WITH_BALL_APPLICATION_PLANNING_SCENE_MANAGER_HPP_
#define MY_ARM_WITH_BALL_APPLICATION_PLANNING_SCENE_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

namespace pick_place
{
/// @brief Gazebo 물리 상태와 별개로 MoveIt 장면의 조회·수정·검증을 담당한다.
/// @note 서비스 대기는 노드가 제공한 호출 함수를 사용하여 기존 종료·타임아웃 처리를 유지한다.
class PlanningSceneManager
{
public:
    enum class AttachmentState { Attached, Detached };
    enum class SceneUpdate { Attach, Detach, RefreshWorld };
    using GetScene = std::function<moveit_msgs::srv::GetPlanningScene::Response::SharedPtr(
        const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr &)>;
    using ApplyScene = std::function<moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr(
        const moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr &)>;

    PlanningSceneManager(GetScene get_scene, ApplyScene apply_scene);
    moveit_msgs::msg::PlanningScene scene();
    void setHandBallCollisionAllowed(bool allowed);
    /// @brief 관측한 공을 반영한다. 손 변환은 Attach에서만 필요할 때 계산한다.
    void syncScene(SceneUpdate update, const geometry_msgs::msg::PoseStamped &measured_ball,
                   const std::function<Eigen::Isometry3d()> &hand_transform);

private:
    void apply(moveit_msgs::msg::PlanningScene diff);
    moveit_msgs::msg::CollisionObject sphere(const geometry_msgs::msg::Pose &pose, const std::string &frame);
    void verifyScene(AttachmentState attachment, const moveit_msgs::msg::CollisionObject &expected);
    GetScene get_scene_;
    ApplyScene apply_scene_;
};
}  // namespace pick_place

#endif  // MY_ARM_WITH_BALL_APPLICATION_PLANNING_SCENE_MANAGER_HPP_
