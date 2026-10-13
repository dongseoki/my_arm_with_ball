import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from keyboard_joint_teleop import (  # noqa: E402
    ANGLE_STEP,
    ARM_CONTROLLER_NAME,
    CONTROLLER_NAMES,
    GOAL_POSITION_TOLERANCE,
    GRIPPER_CONTROLLER_NAME,
    INITIAL_POSITIONS,
    JOINT_NAMES,
    KeyboardJointTeleop,
)


class KeyboardJointTeleopTest(unittest.TestCase):
    """Verify keyboard target and FollowJointTrajectory handling."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = KeyboardJointTeleop()
        self.node.action_client = Mock()
        self.node.gripper_publisher = Mock()
        self.node.create_client = Mock()
        self.set_joint_state([0.0] * 6)

    def tearDown(self):
        self.node.destroy_node()

    def set_joint_state(self, positions):
        message = JointState()
        message.name = list(JOINT_NAMES)
        message.position = list(positions)
        self.node._joint_state_callback(message)

    def arm_goal(self):
        return self.node.action_client.send_goal_async.call_args.args[0]

    def test_relative_key_uses_latest_measured_state_and_sends_timed_goal(self):
        self.set_joint_state([0.1, 0.2, -0.1, 0.0, 0.0, 0.0])
        self.node.selected_joint = 2

        self.assertTrue(self.node.change_selected_joint(ANGLE_STEP))
        goal = self.arm_goal()
        self.assertEqual(goal.trajectory.joint_names, list(JOINT_NAMES))
        self.assertEqual(list(goal.trajectory.points[0].positions), [0.1, 0.2, -0.1, 0, 0, 0])
        self.assertAlmostEqual(goal.trajectory.points[1].positions[2], -0.05)
        self.assertAlmostEqual(
            goal.trajectory.points[1].time_from_start.sec
            + goal.trajectory.points[1].time_from_start.nanosec / 1e9,
            1.0,
        )
        self.assertEqual(len(goal.goal_tolerance), 6)
        self.assertTrue(all(t.position == GOAL_POSITION_TOLERANCE for t in goal.goal_tolerance))

    def test_stale_state_blocks_goal(self):
        self.node.joint_state_received_at = time.monotonic() - 1.0
        self.assertFalse(self.node.change_selected_joint(ANGLE_STEP))
        self.node.action_client.send_goal_async.assert_not_called()

    def test_joint_limit_violation_is_rejected_not_clamped(self):
        self.set_joint_state([6.28, 0, 0, 0, 0, 0])
        self.node.selected_joint = 0
        self.assertFalse(self.node.change_selected_joint(ANGLE_STEP))
        self.node.action_client.send_goal_async.assert_not_called()

    def test_reset_selected_and_reset_all_targets_use_measured_start(self):
        self.set_joint_state([0.2, -0.3, 0.4, -0.5, 0.6, -0.7])
        self.node.reset_joint(2)
        self.assertEqual(list(self.arm_goal().trajectory.points[-1].positions),
                         [0.2, -0.3, 0.0, -0.5, 0.6, -0.7])
        self.node.action_in_progress = False
        self.node.reset_arm()
        self.assertEqual(
            list(self.arm_goal().trajectory.points[-1].positions), list(INITIAL_POSITIONS)
        )

    def test_arm_input_is_ignored_while_action_in_progress(self):
        self.node.action_in_progress = True
        self.assertFalse(self.node.change_selected_joint(ANGLE_STEP))
        self.node.action_client.send_goal_async.assert_not_called()

    def test_gripper_commands_keep_existing_topic_and_x_has_no_gripper_side_effect(self):
        self.node.set_gripper(1.18)
        message = self.node.gripper_publisher.publish.call_args.args[0]
        self.assertEqual(list(message.data), [1.18, 1.18])
        self.node.gripper_publisher.reset_mock()
        self.node.reset_arm()
        self.node.gripper_publisher.publish.assert_not_called()
        self.assertEqual(GRIPPER_CONTROLLER_NAME, "gripper_position_controller")

    def test_wait_requires_only_configured_arm_and_gripper_controllers(self):
        client = Mock()
        client.wait_for_service.return_value = True
        future = Mock()
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(controller=[
            SimpleNamespace(name=ARM_CONTROLLER_NAME, state="active"),
            SimpleNamespace(name=GRIPPER_CONTROLLER_NAME, state="active"),
        ])
        client.call_async.return_value = future
        self.node.create_client.return_value = client

        self.assertEqual(CONTROLLER_NAMES, (ARM_CONTROLLER_NAME, GRIPPER_CONTROLLER_NAME))
        self.assertTrue(self.node.wait_for_controllers(timeout_sec=0.1))

    def test_rejected_goal_logs_and_releases_input_lock(self):
        self.node.action_in_progress = True
        future = Mock()
        future.result.return_value = SimpleNamespace(accepted=False)
        self.node._goal_response_callback(future)
        self.assertFalse(self.node.action_in_progress)

    def test_failed_action_result_logs_and_releases_input_lock(self):
        self.node.action_in_progress = True
        self.node.goal_target = tuple(INITIAL_POSITIONS)
        future = Mock()
        future.result.return_value = SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED,
            result=FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string="test failure",
            ),
        )
        self.node._result_callback(future)
        self.assertFalse(self.node.action_in_progress)

    def test_timeout_requests_cancel_and_only_clears_after_terminal_result(self):
        self.node.action_in_progress = True
        self.node.goal_timed_out = False
        self.node.goal_deadline = time.monotonic() - 1.0
        self.node.goal_target = tuple(INITIAL_POSITIONS)
        handle = Mock()
        self.node.goal_handle = handle
        handle.cancel_goal_async.return_value.add_done_callback = Mock()

        self.node.check_action_progress()

        handle.cancel_goal_async.assert_called_once()
        self.assertTrue(self.node.action_in_progress)
        self.assertTrue(self.node.goal_timed_out)

        future = Mock()
        future.result.return_value = SimpleNamespace(
            status=GoalStatus.STATUS_CANCELED,
            result=FollowJointTrajectory.Result(),
        )
        self.node._result_callback(future)
        self.assertFalse(self.node.action_in_progress)


if __name__ == "__main__":
    unittest.main()
