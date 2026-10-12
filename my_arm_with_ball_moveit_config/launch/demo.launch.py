"""Independent home joint states, TF, static scene and planning-only MoveIt."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = MoveItConfigsBuilder(
        "ur5_rg2", package_name="my_arm_with_ball_moveit_config"
    ).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    launch_dir = config.package_path / "launch"
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description], output="screen"),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             parameters=[config.robot_description, {"rate": 30}], output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(launch_dir / "move_group.launch.py"))),
        Node(package="my_arm_with_ball_moveit_config", executable="planning_scene_loader",
             output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(launch_dir / "moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ])
