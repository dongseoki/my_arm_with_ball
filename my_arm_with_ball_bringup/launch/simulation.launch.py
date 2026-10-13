import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_bringup = get_package_share_directory("my_arm_with_ball_bringup")
    pkg_description = get_package_share_directory("my_arm_with_ball_description")
    pkg_gazebo = get_package_share_directory("my_arm_with_ball_gazebo")
    pkg_moveit = get_package_share_directory("my_arm_with_ball_moveit_config")

    moveit_config = MoveItConfigsBuilder(
        "ur5_rg2", package_name="my_arm_with_ball_moveit_config"
    ).trajectory_execution(
        file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False
    ).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()

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
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {
            "allow_trajectory_execution": True,
            "trajectory_execution.allowed_start_tolerance": 0.02,
            "use_sim_time": True,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        }],
    )

    planning_scene_loader = Node(
        package="my_arm_with_ball_moveit_config",
        executable="planning_scene_loader",
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

    arm_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_trajectory_controller",
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
            return [arm_trajectory_controller]
        return [
            LogInfo(
                msg=(
                    "오류: joint_state_broadcaster 시작 실패; 팔 컨트롤러를 시작하지 않습니다. / "
                    "Error: joint_state_broadcaster failed; arm controller will not start."
                )
            )
        ]

    def start_gripper_after_arm(event, _context):
        if event.returncode == 0:
            return [gripper_position_controller]
        return [
            LogInfo(
                msg=(
                    "오류: arm_trajectory_controller 시작 실패; 그리퍼 컨트롤러를 시작하지 않습니다. / "
                    "Error: arm_trajectory_controller failed; gripper controller will not start."
                )
            )
        ]

    def start_moveit_after_gripper(event, _context):
        if event.returncode == 0:
            return [move_group, planning_scene_loader, rviz]
        return [
            LogInfo(
                msg=(
                    "오류: gripper_position_controller 시작 실패; MoveIt 실행 기능을 시작하지 않습니다. / "
                    "Error: gripper position controller failed; MoveIt execution will not start."
                )
            )
        ]

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
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=start_arm_after_broadcaster,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_trajectory_controller,
                    on_exit=start_gripper_after_arm,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gripper_position_controller,
                    on_exit=start_moveit_after_gripper,
                )
            ),
        ]
    )
