"""Regression contracts for the phase-two control/physical model boundary."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / 'my_arm_with_ball_description/models/ur5_rg2/model.sdf'
WORLD = ROOT / 'my_arm_with_ball_gazebo/worlds/apple_pick_place.sdf'
BRINGUP = ROOT / 'my_arm_with_ball_bringup'
CONTROLLERS = BRINGUP / 'config/ros2_controllers.yaml'
ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
GRIPPER = ['rg2_finger_joint1', 'rg2_finger_joint2']


def semantic_tree(element):
    """Ignore XML formatting, not model values or child ordering."""
    return [element.tag, sorted(element.attrib.items()),
            (element.text or '').strip(),
            [semantic_tree(child) for child in element]]


def physical_digest(model, tag):
    """Fingerprint immutable physical elements independent of formatting."""
    serialized = json.dumps([semantic_tree(e) for e in model.findall(tag)],
                            sort_keys=True).encode()
    return hashlib.sha256(serialized).hexdigest()


class ControlConfigurationTests(unittest.TestCase):
    """Check resource ownership and legacy physical invariants."""

    @classmethod
    def setUpClass(cls):
        """Load repository fixtures once for the test class."""
        cls.model = ET.parse(MODEL).getroot().find('model')
        cls.config = yaml.safe_load(CONTROLLERS.read_text())

    def test_physical_links_and_joints_preserve_pre_migration_model(self):
        """Physical links and joints preserve pre migration model."""
        # Baselines captured before phase two: catches changes to geometry,
        # inertia, collision, poses, axes, limits, dynamics and world fixing.
        self.assertEqual(physical_digest(self.model, 'link'),
                         '8e4af060fad19963b847936c309cf2b84b510104e177b194db062c8a8402b3e3')
        self.assertEqual(physical_digest(self.model, 'joint'),
                         '13842dc03c117b6c5cab2fd31a9f6d93153a5587789fa2c9269e24797451f431')

    def test_only_arm_joints_expose_position_commands(self):
        """Only arm joints expose position commands."""
        control = self.model.find('ros2_control')
        self.assertIsNotNone(control)
        self.assertEqual(control.attrib, {'name': 'GazeboSimSystem', 'type': 'system'})
        self.assertEqual(control.findtext('hardware/plugin'),
                         'gz_ros2_control/GazeboSimSystem')
        joints = control.findall('joint')
        self.assertEqual([j.get('name') for j in joints], ARM + GRIPPER)
        for joint in joints:
            with self.subTest(joint=joint.get('name')):
                self.assertEqual([i.get('name') for i in joint.findall('command_interface')],
                                 ['position'] if joint.get('name') in ARM else [])
                self.assertEqual([i.get('name') for i in joint.findall('state_interface')],
                                 ['position', 'velocity'])
                self.assertIsNotNone(self.model.find("joint[@name='%s']" % joint.get('name')))

    def test_legacy_controllers_only_drive_gripper_with_original_gains(self):
        """Legacy controllers only drive gripper with original gains."""
        plugins = self.model.findall('plugin')
        position = [p for p in plugins if 'JointPositionController' in p.get('name', '')]
        self.assertEqual([p.findtext('joint_name') for p in position], GRIPPER)
        for plugin in position:
            for parameter, expected in [('p_gain', '20'), ('d_gain', '1'), ('i_gain', '0'),
                                        ('cmd_min', '-10.6'), ('cmd_max', '10.6')]:
                self.assertEqual(plugin.findtext(parameter), expected)
        self.assertFalse(any('JointStatePublisher' in p.get('name', '') for p in plugins))

    def test_forward_controller_maps_six_targets_in_keyboard_order(self):
        """Forward controller maps six targets in keyboard order."""
        manager = self.config['controller_manager']['ros__parameters']
        self.assertEqual(manager['arm_position_controller']['type'],
                         'forward_command_controller/ForwardCommandController')
        self.assertEqual(manager['joint_state_broadcaster']['type'],
                         'joint_state_broadcaster/JointStateBroadcaster')
        arm = self.config['arm_position_controller']['ros__parameters']
        self.assertEqual(arm['joints'], ARM)
        self.assertEqual(arm['interface_name'], 'position')

    def test_broadcaster_uses_eight_measured_states_not_extra_zero_joints(self):
        """Broadcaster uses eight measured states not extra zero joints."""
        broadcaster = self.config['joint_state_broadcaster']['ros__parameters']
        self.assertEqual(broadcaster['joints'], ARM + GRIPPER)
        self.assertEqual(broadcaster['interfaces'], ['position', 'velocity'])
        self.assertFalse(broadcaster.get('extra_joints'))

    def test_bridge_preserves_gripper_and_clock_without_duplicate_joint_states(self):
        """Bridge preserves gripper and clock without duplicate joint states."""
        bridges = yaml.safe_load((BRINGUP / 'config/bridge.yaml').read_text())
        topics = {entry['ros_topic_name']: entry for entry in bridges}
        self.assertNotIn('/joint_states', topics)
        self.assertEqual(topics['/clock']['direction'], 'GZ_TO_ROS')
        for joint in GRIPPER:
            bridge = topics['/%s/cmd_pos' % joint]
            self.assertEqual(bridge['direction'], 'ROS_TO_GZ')
            self.assertEqual(bridge['ros_type_name'], 'std_msgs/msg/Float64')
            self.assertEqual(bridge['gz_topic_name'], '/model/ur5_rg2/joint/%s/0/cmd_pos' % joint)


class SimulationPreparationTests(unittest.TestCase):
    """Check generated worlds, descriptions and activation gating."""

    @classmethod
    def setUpClass(cls):
        """Load repository fixtures once for the test class."""
        spec = importlib.util.spec_from_file_location(
            'stage2_simulation_launch', BRINGUP / 'launch/simulation.launch.py')
        cls.launch_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.launch_module)

    def setUp(self):
        """Prepare fresh world and state-publisher description fixtures."""
        self.world_xml, self.description_xml = self.launch_module.prepare_simulation(
            str(MODEL), str(WORLD), str(CONTROLLERS))
        self.world = ET.fromstring(self.world_xml).find('world')
        self.include = next(item for item in self.world.findall('include')
                            if item.findtext('uri', '').strip() == 'model://ur5_rg2')
        self.physical = ET.parse(MODEL).getroot().find('model')
        self.description = ET.fromstring(self.description_xml).find('model')

    def test_physical_sdf_keeps_joint_graph_and_gripper_plugins(self):
        """Physical source model stays intact; plugin is attached by the include."""
        self.assertIsNotNone(self.physical.find('ros2_control'))
        self.assertEqual(self.physical.find('joint').get('name'), 'ur5_rg2_joint_world')
        self.assertEqual(len([p for p in self.physical.findall('plugin')
                              if 'JointPositionController' in p.get('name', '')]), 2)

    def test_description_keeps_resources_but_excludes_world_fixed_joint_and_plugins(self):
        """Description keeps resources but excludes world fixed joint and plugins."""
        self.assertIsNotNone(self.description.find('ros2_control'))
        source_control = ET.parse(MODEL).getroot().find('model/ros2_control')
        description_control = self.description.find('ros2_control')
        self.assertEqual(semantic_tree(description_control), semantic_tree(source_control))
        self.assertEqual(description_control.findtext('hardware/plugin'),
                         'gz_ros2_control/GazeboSimSystem')
        self.assertEqual(self.description.findall('plugin'), [])
        self.assertFalse(any(j.findtext('parent') == 'world'
                             for j in self.description.findall('joint')))
        source = ET.parse(MODEL).getroot().find('model')
        self.assertEqual([semantic_tree(j) for j in self.description.findall('joint')],
                         [semantic_tree(j) for j in source.findall('joint')
                          if j.findtext('parent') != 'world'])

    def test_only_robot_include_receives_plugin_overlay(self):
        """Only robot include receives plugin; model is not inlined."""
        original = ET.parse(WORLD).getroot().find('world')
        expected = [semantic_tree(e) for e in original
                    if e.tag != 'include' or
                    e.findtext('uri', '').strip() != 'model://ur5_rg2']
        actual = [semantic_tree(e) for e in self.world if e is not self.include]
        self.assertEqual(actual, expected)
        self.assertEqual(self.include.findtext('uri'), 'model://ur5_rg2')

    def test_description_keeps_links_with_absolute_mesh_uris(self):
        """Description links keep their geometry with resolved mesh URIs."""
        source = ET.parse(MODEL).getroot().find('model')
        for uri in source.findall('.//mesh/uri'):
            uri.text = (MODEL.parent / uri.text.strip()).resolve().as_uri()
        expected = [semantic_tree(link) for link in source.findall('link')]
        self.assertEqual([semantic_tree(link) for link in self.description.findall('link')],
                         expected)
        for uri in self.description.findall('.//mesh/uri'):
            self.assertTrue(uri.text.startswith('file:///'))
            self.assertTrue(Path(unquote(urlsplit(uri.text).path)).is_file())

    def test_include_plugin_has_resolved_configuration(self):
        """Include override configures gz_ros2_control with resolved YAML path."""
        plugins = [p for p in self.include.findall('plugin')
                   if 'gz_ros2_control' in p.get('filename', '')]
        self.assertEqual(len(plugins), 1)
        self.assertEqual(plugins[0].get('name'),
                         'gz_ros2_control::GazeboSimROS2ControlPlugin')
        self.assertEqual(Path(plugins[0].findtext('parameters')), CONTROLLERS.resolve())

    def test_preparation_is_relocatable_repeatable_and_never_mutates_inputs(self):
        """Generated world is relocatable, repeatable and does not mutate inputs."""
        with tempfile.TemporaryDirectory(prefix='control & test ') as directory:
            root = Path(directory)
            model = root / 'model/model.sdf'
            shutil.copytree(MODEL.parent, model.parent)
            world = root / 'world.sdf'
            config = root / 'controllers.yaml'
            shutil.copyfile(WORLD, world)
            shutil.copyfile(CONTROLLERS, config)
            paths = (model, world, config)
            before = [p.read_bytes() for p in paths]
            first = self.launch_module.prepare_simulation(*map(str, paths))
            second = self.launch_module.prepare_simulation(*map(str, paths))
            self.assertEqual(first, second)
            self.assertEqual([p.read_bytes() for p in paths], before)
            plugin = ET.fromstring(first[0]).find(
                "world/include[uri='model://ur5_rg2']/plugin")
            self.assertEqual(Path(plugin.findtext('parameters')), config.resolve())
            for uri in ET.fromstring(first[1]).findall('.//mesh/uri'):
                self.assertTrue(Path(unquote(urlsplit(uri.text).path)).is_relative_to(root))

    def test_spawner_success_starts_only_requested_next_actions(self):
        """Spawner success starts only requested next actions."""
        next_actions = [object()]
        callback = self.launch_module._on_spawner_exit(next_actions, 'arm_position_controller')
        self.assertEqual(callback(SimpleNamespace(returncode=0), None), next_actions)

    def test_spawner_failure_shuts_down_without_starting_downstream_actions(self):
        """Spawner failure shuts down without starting downstream actions."""
        from launch.actions import EmitEvent
        next_action = object()
        callback = self.launch_module._on_spawner_exit([next_action], 'joint_state_broadcaster')
        with patch.object(self.launch_module, 'get_logger'):
            actions = callback(SimpleNamespace(returncode=1), None)
        self.assertEqual(len(actions), 1)
        self.assertIsInstance(actions[0], EmitEvent)
        self.assertNotIn(next_action, actions)

    def test_invalid_model_without_control_resources_is_rejected(self):
        """Invalid model without control resources is rejected."""
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / 'model.sdf'
            model.write_text('<sdf version="1.10"><model name="ur5_rg2"/></sdf>')
            with self.assertRaisesRegex(ValueError, 'ros2_control'):
                self.launch_module.prepare_simulation(
                    str(model), str(WORLD), str(CONTROLLERS))

    def test_world_requires_exactly_one_robot_include(self):
        """World requires exactly one robot include."""
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / 'world.sdf'
            for count in (0, 2):
                with self.subTest(robot_includes=count):
                    includes = '<include><uri>model://ur5_rg2</uri></include>' * count
                    world.write_text('<sdf version="1.10"><world name="test">' + includes +
                                     '</world></sdf>')
                    with self.assertRaisesRegex(ValueError, 'exactly once'):
                        self.launch_module.prepare_simulation(
                            str(MODEL), str(world), str(CONTROLLERS))


if __name__ == '__main__':
    unittest.main()
