import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory("my_arm_with_ball_bringup")
    pkg_description = get_package_share_directory("my_arm_with_ball_description")
    pkg_gazebo = get_package_share_directory("my_arm_with_ball_gazebo")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(pkg_description, "urdf", "ur5_rg2.urdf.xacro"),
            ]
        ),
        value_type=str,
    )

    gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            "-r",
            os.path.join(pkg_gazebo, "worlds", "apple_pick_place.sdf"),
        ],
        cwd=pkg_bringup,
        condition=UnlessCondition(LaunchConfiguration("headless")),
        output="screen",
    )

    gazebo_headless = ExecuteProcess(
        cmd=[
            FindExecutable(name="gz"),
            "sim",
            "-s",
            "-r",
            os.path.join(pkg_gazebo, "worlds", "apple_pick_place.sdf"),
        ],
        cwd=pkg_bringup,
        condition=IfCondition(LaunchConfiguration("headless")),
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": os.path.join(pkg_bringup, "config", "bridge.yaml")}],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_bringup, "config", "ur5_rg2.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    arm_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    gripper_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    def start_arm_after_broadcaster(event, _context):
        if event.returncode == 0:
            return [arm_position_controller]
        return [LogInfo(msg="ERROR: joint_state_broadcaster failed; arm controller was not started")]

    def start_gripper_after_arm(event, _context):
        if event.returncode == 0:
            return [gripper_position_controller]
        return [LogInfo(msg="ERROR: arm controller failed; gripper controller was not started")]

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "rviz",
                default_value=IfElseSubstitution(LaunchConfiguration("headless"), "false", "true"),
                description="Start RViz2 with the UR5/RG2 robot model.",
            ),
            gazebo,
            gazebo_headless,
            bridge,
            robot_state_publisher,
            rviz,
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=start_arm_after_broadcaster,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_position_controller,
                    on_exit=start_gripper_after_arm,
                )
            ),
        ]
    )
