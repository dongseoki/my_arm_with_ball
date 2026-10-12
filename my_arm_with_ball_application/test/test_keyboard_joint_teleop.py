import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import rclpy

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
        self.node.gripper_publishers = (Mock(), Mock())

    def tearDown(self):
        self.node.destroy_node()

    def arm_command(self):
        return self.node.arm_publisher.publish.call_args.args[0].data

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
        for publisher in self.node.gripper_publishers:
            self.assertEqual(publisher.publish.call_args.args[0].data, 0.0)

    def test_wait_for_active_controller_before_initial_arm_command(self):
        client = Mock()
        client.wait_for_service.return_value = True
        future = Mock()
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(
            controller=[SimpleNamespace(name="arm_position_controller", state="active")]
        )
        client.call_async.return_value = future
        self.node.create_client = Mock(return_value=client)

        self.assertTrue(self.node.wait_for_arm_controller(timeout_sec=0.1))
        self.node.arm_publisher.publish.assert_not_called()
        self.node.publish_arm()
        self.assertEqual(list(self.arm_command()), [0.0] * 6)

    def test_wait_expires_without_controller(self):
        client = Mock()
        client.wait_for_service.return_value = False
        self.node.create_client = Mock(return_value=client)

        self.assertFalse(self.node.wait_for_arm_controller(timeout_sec=0.01))
        self.node.arm_publisher.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
