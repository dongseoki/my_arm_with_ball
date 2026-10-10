import copy
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prepare_simulation(model_path, world_path, controllers_path):
    """Add the control system to the model include and prepare its description."""
    model_root = ET.parse(model_path).getroot()
    model = model_root.find('model')
    if model is None or model.find('ros2_control') is None:
        raise ValueError('Robot SDF must contain a model-level ros2_control resource.')
    description_root = copy.deepcopy(model_root)
    description_model = description_root.find('model')
    for uri in description_model.findall('.//mesh/uri'):
        value = (uri.text or '').strip()
        if value and '://' not in value and not os.path.isabs(value):
            uri.text = (Path(model_path).resolve().parent / value).as_uri()

    # Strip model-level Gazebo systems, not the hardware plugin resource.
    for plugin in list(description_model.findall('plugin')):
        description_model.remove(plugin)
    for joint in list(description_model.findall('joint')):
        if (joint.get('type') == 'fixed'
                and joint.findtext('parent') == 'world'
                and joint.findtext('child') == 'base_link'):
            description_model.remove(joint)

    world_root = ET.parse(world_path).getroot()
    world = world_root.find('world')
    if world is None:
        raise ValueError('World SDF must contain a world.')
    includes = [item for item in world.findall('include')
                if (item.findtext('uri') or '').strip() == 'model://ur5_rg2']
    if len(includes) != 1:
        raise ValueError('World must include model://ur5_rg2 exactly once.')
    # SDF supports plugin overrides on <include>. This attaches the system to
    # the included model without copying or rewriting the physical model.
    plugin = ET.SubElement(includes[0], 'plugin', {
        'name': 'gz_ros2_control::GazeboSimROS2ControlPlugin',
        'filename': 'libgz_ros2_control-system.so',
    })
    ET.SubElement(plugin, 'parameters').text = str(Path(controllers_path).resolve())
    return (ET.tostring(world_root, encoding='unicode'),
            ET.tostring(description_root, encoding='unicode'))


def _on_spawner_exit(next_actions, controller):
    def on_exit(event, context):
        if event.returncode != 0:
            reason = f'{controller} spawner failed with exit code {event.returncode}'
            get_logger('launch.user').error(reason)
            return [EmitEvent(event=Shutdown(reason=reason))]
        return next_actions
    return on_exit


def _launch_simulation(context):
    pkg_bringup = get_package_share_directory('my_arm_with_ball_bringup')
    pkg_description = get_package_share_directory('my_arm_with_ball_description')
    pkg_gazebo = get_package_share_directory('my_arm_with_ball_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_xml, robot_description = prepare_simulation(
        os.path.join(pkg_description, 'models', 'ur5_rg2', 'model.sdf'),
        os.path.join(pkg_gazebo, 'worlds', 'apple_pick_place.sdf'),
        os.path.join(pkg_bringup, 'config', 'ros2_controllers.yaml'),
    )
    runtime_directory = tempfile.TemporaryDirectory(prefix='my_arm_ros2_control_')
    world_path = Path(runtime_directory.name) / 'apple_pick_place.sdf'
    world_path.write_text(world_xml, encoding='utf-8')

    def cleanup(event, context):
        runtime_directory.cleanup()
        return []

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {"-s " if headless else ""}{world_path}'}.items(),
    )
    description_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}], output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_bringup, 'config', 'bridge.yaml')}],
        output='screen',
    )
    state_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'], output='screen',
    )
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_position_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'], output='screen',
    )
    keyboard = Node(
        package='my_arm_with_ball_application', executable='keyboard_joint_teleop',
        condition=IfCondition(LaunchConfiguration('keyboard')), output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen',
    )
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        RegisterEventHandler(OnProcessExit(
            target_action=state_spawner,
            on_exit=_on_spawner_exit([arm_spawner], 'joint_state_broadcaster'))),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner,
            on_exit=_on_spawner_exit([keyboard], 'arm_position_controller'))),
        description_publisher, gazebo, bridge, state_spawner, rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2.'),
        DeclareLaunchArgument(
            'keyboard', default_value='false',
            description='Enable keyboard control when launch has an interactive terminal.'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run Gazebo server without GUI.'),
        OpaqueFunction(function=_launch_simulation),
    ])
