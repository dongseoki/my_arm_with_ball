import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from keyboard_joint_teleop import KeyboardJointTeleop  # noqa: E402


class KeyboardJointTeleopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = KeyboardJointTeleop()
        self.node.arm_publisher = Mock()
        self.node.gripper_publisher = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def arm_command(self):
        return self.node.arm_publisher.publish.call_args.args[0].data

    def gripper_command(self):
        message = self.node.gripper_publisher.publish.call_args.args[0]
        self.assertIsInstance(message, Float64MultiArray)
        return list(message.data)

    def mock_controllers(self, controllers):
        client = Mock()
        client.wait_for_service.return_value = True
        future = Mock()
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(controller=controllers)
        client.call_async.return_value = future
        self.node.create_client = Mock(return_value=client)
        return client, future

    def test_gripper_commands_have_two_equal_targets_and_respect_limits(self):
        for target, expected in ((1.18, 1.18), (0.0, 0.0), (2.0, 1.18), (-1.0, 0.0)):
            with self.subTest(target=target):
                self.node.set_gripper(target)
                self.assertEqual(self.gripper_command(), [expected, expected])

    def test_arm_commands_keep_six_targets_in_controller_order(self):
        self.node.selected_joint = 2
        self.node.change_selected_joint(0.05)
        self.assertEqual(list(self.arm_command()), [0.0, 0.0, 0.05, 0.0, 0.0, 0.0])

        self.node.selected_joint = 4
        self.node.change_selected_joint(-0.05)
        self.assertEqual(list(self.arm_command()), [0.0, 0.0, 0.05, 0.0, -0.05, 0.0])

        self.node.reset_joint(2)
        self.assertEqual(list(self.arm_command()), [0.0, 0.0, 0.0, 0.0, -0.05, 0.0])

    def test_arm_limit_and_full_reset_include_gripper(self):
        self.node.positions[0] = 6.28
        self.node.change_selected_joint(0.05)
        self.assertEqual(self.arm_command()[0], 6.28319)

        self.node.set_gripper(1.18)
        self.assertEqual(self.node.gripper_position, 1.18)
        self.node.reset_all()
        self.assertEqual(list(self.arm_command()), [0.0] * 6)
        self.assertEqual(self.gripper_command(), [0.0, 0.0])

    def test_wait_for_both_active_controllers_before_initial_commands(self):
        self.mock_controllers([
            SimpleNamespace(name="arm_position_controller", state="active"),
            SimpleNamespace(name="gripper_position_controller", state="active"),
        ])

        self.assertTrue(self.node.wait_for_controllers(timeout_sec=0.1))
        self.node.arm_publisher.publish.assert_not_called()
        self.node.gripper_publisher.publish.assert_not_called()
        self.node.publish_all()
        self.assertEqual(list(self.arm_command()), [0.0] * 6)
        self.assertEqual(self.gripper_command(), [0.0, 0.0])

    def test_wait_keeps_polling_until_gripper_is_active(self):
        client, future = self.mock_controllers([])
        future.result.side_effect = [
            SimpleNamespace(controller=[
                SimpleNamespace(name="arm_position_controller", state="active"),
                SimpleNamespace(name="gripper_position_controller", state="inactive"),
            ]),
            SimpleNamespace(controller=[
                SimpleNamespace(name="arm_position_controller", state="active"),
                SimpleNamespace(name="gripper_position_controller", state="active"),
            ]),
        ]
        with patch("keyboard_joint_teleop.time.sleep"):
            self.assertTrue(self.node.wait_for_controllers(timeout_sec=0.1))
        self.assertEqual(client.call_async.call_count, 2)
        self.node.arm_publisher.publish.assert_not_called()
        self.node.gripper_publisher.publish.assert_not_called()

    def test_wait_expires_if_either_controller_is_missing_or_inactive(self):
        for active_name, other_name in (
            ("arm_position_controller", "gripper_position_controller"),
            ("gripper_position_controller", "arm_position_controller"),
        ):
            for other_state in (None, "inactive"):
                with self.subTest(active=active_name, other_state=other_state):
                    controllers = [SimpleNamespace(name=active_name, state="active")]
                    if other_state is not None:
                        controllers.append(SimpleNamespace(name=other_name, state=other_state))
                    self.mock_controllers(controllers)
                    self.assertFalse(self.node.wait_for_controllers(timeout_sec=0.01))
                    self.node.arm_publisher.publish.assert_not_called()
                    self.node.gripper_publisher.publish.assert_not_called()

    def test_wait_expires_without_controller_service(self):
        client = Mock()
        client.wait_for_service.return_value = False
        self.node.create_client = Mock(return_value=client)

        self.assertFalse(self.node.wait_for_controllers(timeout_sec=0.01))
        self.node.arm_publisher.publish.assert_not_called()
        self.node.gripper_publisher.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
