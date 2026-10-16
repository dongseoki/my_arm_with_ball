"""Start the simulation and run the one-shot pick-and-place task."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup = get_package_share_directory('my_arm_with_ball_bringup')
    application = get_package_share_directory('my_arm_with_ball_application')
    simulation = os.path.join(bringup, 'launch', 'simulation.launch.py')
    task_parameters = os.path.join(application, 'config', 'pick_and_place_test.yaml')
    moveit_config = MoveItConfigsBuilder(
        'ur5_rg2', package_name='my_arm_with_ball_moveit_config'
    ).trajectory_execution(
        file_path='config/moveit_controllers.yaml', moveit_manage_controllers=False
    ).planning_pipelines(pipelines=['ompl']).to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
                'move_group_capabilities': 'move_group/ExecuteTaskSolutionCapability',
            }.items(),
        ),
        Node(
            package='my_arm_with_ball_application',
            executable='pick_and_place_test',
            output='screen',
            parameters=[moveit_config.to_dict(), task_parameters, {'use_sim_time': True}],
        ),
    ])
