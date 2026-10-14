"""Launch the existing simulation plus the bounded stage-4 grasp test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation = os.path.join(
        get_package_share_directory('my_arm_with_ball_bringup'),
        'launch', 'simulation.launch.py',
    )
    test = Node(
        package='my_arm_with_ball_application',
        executable='grasp_test',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    def on_test_exit(event, _context):
        if event.returncode != 0:
            raise RuntimeError(f'Stage-4 grasp test failed (exit {event.returncode})')
        return [LogInfo(msg='Stage-4 grasp test passed'), Shutdown(reason='stage-4 test passed')]

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
        test,
        RegisterEventHandler(OnProcessExit(target_action=test, on_exit=on_test_exit)),
    ])
