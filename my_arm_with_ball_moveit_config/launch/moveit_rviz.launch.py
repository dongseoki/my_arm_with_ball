"""RViz planning display for the independent demo."""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = MoveItConfigsBuilder(
        "ur5_rg2", package_name="my_arm_with_ball_moveit_config"
    ).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", str(config.package_path / "config/moveit.rviz")],
            parameters=[config.robot_description, config.robot_description_semantic,
                        config.robot_description_kinematics, config.planning_pipelines,
                        config.joint_limits],
        ),
    ])
