"""Start a MoveGroup with trajectory execution disabled."""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = MoveItConfigsBuilder(
        "ur5_rg2", package_name="my_arm_with_ball_moveit_config"
    ).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[config.to_dict(), {
                "allow_trajectory_execution": False,
                "disable_capabilities": "move_group/MoveGroupExecuteTrajectoryAction",
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            }],
        ),
    ])
