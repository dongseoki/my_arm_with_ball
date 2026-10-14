#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("planning_scene_loader");
    auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
    while (rclcpp::ok() && !client->wait_for_service(1s)) {
        RCLCPP_INFO(node->get_logger(), "Waiting for move_group planning scene service");
    }
    if (!rclcpp::ok()) {
        return 0;
    }
    try {
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene.is_diff = true;
        request->scene.robot_state.is_diff = true;
        moveit_msgs::msg::CollisionObject ball;
        ball.header.frame_id = "world";
        ball.id = "apple";
        ball.operation = moveit_msgs::msg::CollisionObject::ADD;
        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions = {0.0325};
        geometry_msgs::msg::Pose ball_pose;
        ball_pose.position.x = 0.55;
        ball_pose.position.y = -0.25;
        ball_pose.position.z = 0.0325;
        ball_pose.orientation.w = 1.0;
        ball.primitives.push_back(sphere);
        ball.primitive_poses.push_back(ball_pose);
        request->scene.world.collision_objects.push_back(ball);

        // Gazebo has a ground plane. Exclude the fixed base footprint while
        // protecting the reachable pick area from below-ground IK branches.
        moveit_msgs::msg::CollisionObject floor;
        floor.header.frame_id = "world";
        floor.id = "pick_area_floor";
        floor.operation = moveit_msgs::msg::CollisionObject::ADD;
        shape_msgs::msg::SolidPrimitive slab;
        slab.type = shape_msgs::msg::SolidPrimitive::BOX;
        slab.dimensions = {1.25, 2.0, 0.05};
        geometry_msgs::msg::Pose slab_pose;
        slab_pose.position.x = 0.875;
        slab_pose.position.z = -0.025;
        slab_pose.orientation.w = 1.0;
        floor.primitives.push_back(slab);
        floor.primitive_poses.push_back(slab_pose);
        request->scene.world.collision_objects.push_back(floor);

        moveit_msgs::msg::CollisionObject basket;
        basket.header.frame_id = "world";
        basket.id = "basket";
        basket.operation = moveit_msgs::msg::CollisionObject::ADD;
        const auto path = ament_index_cpp::get_package_share_directory(
            "my_arm_with_ball_moveit_config") + "/meshes/basket.stl";
        std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource("file://" + path));
        if (!mesh) {
            throw std::runtime_error("Cannot load basket collision mesh: " + path);
        }
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(mesh.get(), mesh_msg);
        basket.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
        geometry_msgs::msg::Pose basket_pose;
        basket_pose.position.x = 0.75;
        basket_pose.position.y = 0.25;
        basket_pose.position.z = 0.000022;
        basket_pose.orientation.w = 1.0;
        basket.mesh_poses.push_back(basket_pose);
        request->scene.world.collision_objects.push_back(basket);
        for (const auto& object : request->scene.world.collision_objects) {
            moveit_msgs::msg::ObjectColor color;
            color.id = object.id;
            color.color.r = object.id == "apple" ? 0.8f : 0.6f;
            color.color.g = object.id == "apple" ? 0.1f : 0.4f;
            color.color.b = 0.1f;
            color.color.a = 1.0f;
            request->scene.object_colors.push_back(color);
        }
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future, 30s) !=
            rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
            throw std::runtime_error("move_group did not acknowledge initial planning scene");
        }
        RCLCPP_INFO(node->get_logger(), "Applied apple, pick-area floor, and basket in world frame");
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
