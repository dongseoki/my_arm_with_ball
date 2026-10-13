"""Start a MoveGroup with trajectory execution disabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = MoveItConfigsBuilder(
        "ur5_rg2", package_name="my_arm_with_ball_moveit_config"
    ).trajectory_execution(
        file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False
    ).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    return LaunchDescription([
        DeclareLaunchArgument("allow_trajectory_execution", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value="move_group/MoveGroupExecuteTrajectoryAction",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[config.to_dict(), {
                "allow_trajectory_execution": LaunchConfiguration(
                    "allow_trajectory_execution"
                ),
                "disable_capabilities": LaunchConfiguration("disable_capabilities"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "trajectory_execution.allowed_start_tolerance": 0.02,
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
