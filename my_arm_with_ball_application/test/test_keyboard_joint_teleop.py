import subprocess
import sys
import textwrap
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

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
    GRIPPER_JOINT_NAMES,
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
        self.node.gripper_action_client = Mock()
        self.node.create_client = Mock()
        self.set_joint_state([0.0] * 6)
        self.set_gripper_state([0.0, 0.0])

    def tearDown(self):
        self.node.destroy_node()

    def set_joint_state(self, positions):
        message = JointState()
        message.name = list(JOINT_NAMES)
        message.position = list(positions)
        self.node._joint_state_callback(message)

    def set_gripper_state(self, positions):
        self.node._joint_state_callback(JointState(
            name=list(GRIPPER_JOINT_NAMES), position=list(positions)
        ))

    def gripper_goal(self):
        return self.node.gripper_action_client.send_goal_async.call_args.args[0]

    def successful_result(self):
        future = Mock()
        future.result.return_value = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL
            ),
        )
        return future

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

    def test_gripper_open_close_uses_measured_start_and_existing_targets(self):
        self.set_gripper_state([0.2, 0.3])
        self.assertTrue(self.node.set_gripper(1.18))
        goal = self.gripper_goal()
        self.assertEqual(goal.trajectory.joint_names, list(GRIPPER_JOINT_NAMES))
        self.assertEqual(list(goal.trajectory.points[0].positions), [0.2, 0.3])
        self.assertEqual(list(goal.trajectory.points[-1].positions), [1.18, 1.18])
        self.assertEqual(len(goal.goal_tolerance), 2)
        self.node._clear_action()
        self.set_gripper_state([1.18, 1.18])
        self.assertTrue(self.node.set_gripper(0.0))
        self.assertEqual(list(self.gripper_goal().trajectory.points[-1].positions), [0.0, 0.0])
        self.assertEqual(GRIPPER_CONTROLLER_NAME, "gripper_trajectory_controller")

    def test_arm_reset_has_no_gripper_side_effect(self):
        self.node.reset_arm()
        self.node.gripper_action_client.send_goal_async.assert_not_called()

    def test_busy_lock_blocks_both_directions_without_queueing(self):
        self.node.reset_arm()
        self.assertFalse(self.node.set_gripper(1.18))
        self.node.gripper_action_client.send_goal_async.assert_not_called()
        self.node._clear_action()
        self.node.action_client.reset_mock()
        self.node.set_gripper(1.18)
        self.assertFalse(self.node.change_selected_joint(ANGLE_STEP))
        self.assertFalse(self.node.reset_joint(0))
        self.assertFalse(self.node.reset_arm())
        self.assertFalse(self.node.set_gripper(0.0))
        self.node.action_client.send_goal_async.assert_not_called()
        self.assertEqual(self.node.gripper_action_client.send_goal_async.call_count, 1)

    def test_gripper_missing_stale_invalid_or_out_of_range_blocks_goal(self):
        self.node.gripper_state_received_at = None
        self.assertFalse(self.node.set_gripper(1.18))
        self.set_gripper_state([0.0, 0.0])
        self.node.gripper_state_received_at -= 1.0
        self.assertFalse(self.node.set_gripper(1.18))
        self.set_gripper_state([float("nan"), 0.0])
        self.assertFalse(self.node.set_gripper(1.18))
        self.set_gripper_state([0.0, 0.0])
        for value in (-0.1, 1.19, float("nan")):
            self.assertFalse(self.node.set_gripper(value))
        self.node.gripper_action_client.send_goal_async.assert_not_called()

    def test_partial_joint_messages_do_not_refresh_other_group(self):
        self.node.joint_state_received_at -= 1.0
        self.set_gripper_state([0.1, 0.2])
        self.assertFalse(self.node._state_is_fresh())
        self.assertTrue(self.node._state_is_fresh(GRIPPER_JOINT_NAMES))
        self.node.gripper_state_received_at -= 1.0
        self.set_joint_state([0.0] * 6)
        self.assertFalse(self.node._state_is_fresh(GRIPPER_JOINT_NAMES))

    def test_gripper_success_requires_both_actual_joints_at_target(self):
        self.node.set_gripper(1.18)
        self.node._result_callback(self.successful_result())
        self.set_gripper_state([1.18, 1.0])
        self.node.check_action_progress()
        self.assertTrue(self.node.action_in_progress)
        self.set_gripper_state([1.18, 1.18])
        self.node.check_action_progress()
        self.assertFalse(self.node.action_in_progress)

    def test_gripper_success_without_measured_target_reports_failure(self):
        self.node.set_gripper(1.18)
        self.node._result_callback(self.successful_result())
        self.node.awaiting_state_deadline = time.monotonic() - 1.0
        with patch.object(self.node, "_log_error") as error:
            self.node.check_action_progress()
        self.assertIn("did not reach", error.call_args.args[1])
        self.assertFalse(self.node.action_in_progress)

    def test_gripper_send_exception_and_rejection_release_lock(self):
        self.node.gripper_action_client.send_goal_async.side_effect = RuntimeError("failed")
        self.assertFalse(self.node.set_gripper(1.18))
        self.assertFalse(self.node.action_in_progress)
        self.node.gripper_action_client.send_goal_async.side_effect = None
        self.node.set_gripper(1.18)
        future = Mock()
        future.result.return_value = SimpleNamespace(accepted=False)
        self.node._goal_response_callback(future)
        self.assertFalse(self.node.action_in_progress)

    def test_gripper_aborted_and_canceled_results_release_lock(self):
        for status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self.node.set_gripper(1.18)
            future = self.successful_result()
            future.result.return_value.status = status
            self.node._result_callback(future)
            self.assertFalse(self.node.action_in_progress)

    def test_gripper_timeout_before_acceptance_waits_and_cancels_late_handle(self):
        self.node.set_gripper(1.18)
        self.node.goal_deadline = time.monotonic() - 1.0
        self.node.check_action_progress()
        self.assertTrue(self.node.action_in_progress)
        handle = Mock(accepted=True)
        future = Mock()
        future.result.return_value = handle
        self.node._goal_response_callback(future)
        handle.cancel_goal_async.assert_called_once()
        response = Mock()
        response.result.return_value = SimpleNamespace(goals_canceling=[object()])
        self.node._cancel_callback(response)
        self.assertTrue(self.node.action_in_progress)
        self.assertFalse(self.node.set_gripper(0.0))
        terminal = self.successful_result()
        terminal.result.return_value.status = GoalStatus.STATUS_CANCELED
        self.node._result_callback(terminal)
        self.assertFalse(self.node.action_in_progress)

    def test_failed_cancel_confirmation_keeps_all_inputs_locked(self):
        self.node.set_gripper(1.18)
        future = Mock()
        future.result.return_value = SimpleNamespace(goals_canceling=[])
        self.node._cancel_callback(future)
        self.assertFalse(self.node.reset_arm())
        self.assertFalse(self.node.set_gripper(0.0))

    def test_result_exception_cancels_without_unlocking_unknown_execution(self):
        self.node.set_gripper(1.18)
        self.node.goal_handle = Mock()
        future = Mock()
        future.result.side_effect = RuntimeError("lost result")
        self.node._result_callback(future)
        self.node.goal_handle.cancel_goal_async.assert_called_once()
        self.assertTrue(self.node.action_in_progress)
        self.assertFalse(self.node.reset_arm())

    def test_cancel_request_exception_preserves_busy_lock(self):
        self.node.set_gripper(1.18)
        self.node.goal_handle = Mock()
        self.node.goal_handle.cancel_goal_async.side_effect = RuntimeError("lost connection")
        self.node.goal_deadline = time.monotonic() - 1.0
        self.node.check_action_progress()
        self.assertTrue(self.node.action_in_progress)
        self.assertFalse(self.node.set_gripper(0.0))

    def test_shutdown_requests_cancel_but_does_not_claim_unconfirmed_completion(self):
        self.node.set_gripper(1.18)
        self.node.goal_handle = Mock()
        self.assertFalse(self.node.cancel_active_goal(timeout_sec=0.0))
        self.node.goal_handle.cancel_goal_async.assert_called_once()
        self.assertTrue(self.node.action_in_progress)
        self.node._result_callback(self.successful_result())
        self.assertTrue(self.node.cancel_active_goal(timeout_sec=0.0))

    def test_startup_waits_for_both_action_servers_and_both_joint_groups(self):
        self.node.action_client.wait_for_server.return_value = True
        self.node.gripper_action_client.wait_for_server.return_value = True
        self.assertTrue(self.node.wait_for_action_server(timeout_sec=0.1))
        self.node.gripper_action_client.wait_for_server.assert_called_once()
        self.assertTrue(self.node.wait_for_fresh_joint_state(timeout_sec=0.1))
        self.node.gripper_state_received_at = None
        self.assertFalse(self.node.wait_for_fresh_joint_state(timeout_sec=0.0))
        self.assertFalse(self.node.wait_for_action_server(timeout_sec=0.0))

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


class KeyboardSignalShutdownTest(unittest.TestCase):
    """Send real OS signals in isolation with a live ROS context and active goal."""

    def test_sigint_and_sigterm_cancel_before_ros_shutdown(self):
        script = textwrap.dedent("""
            import os
            import signal
            import sys
            from types import SimpleNamespace
            from unittest.mock import Mock, patch
            import rclpy
            import keyboard_joint_teleop as teleop

            original_class = teleop.KeyboardJointTeleop
            events = []

            def make_node():
                node = original_class()
                node.wait_for_controllers = Mock(return_value=True)
                node.wait_for_action_server = Mock(return_value=True)
                node.wait_for_fresh_joint_state = Mock(return_value=True)
                node.action_in_progress = True
                node.goal_joint_names = teleop.GRIPPER_JOINT_NAMES
                node.goal_deadline = None
                node.goal_handle = Mock()

                def cancel():
                    assert rclpy.ok(), "ROS context died before cancellation"
                    events.append("cancel")
                    # A repeated interrupt during cleanup must not interrupt it.
                    os.kill(os.getpid(), int(sys.argv[1]))
                    result = Mock()
                    result.result.return_value = SimpleNamespace(
                        status=teleop.GoalStatus.STATUS_CANCELED,
                        result=teleop.FollowJointTrajectory.Result(),
                    )
                    node._result_callback(result)
                    assert not node.action_in_progress
                    response = Mock()
                    response.result.return_value = SimpleNamespace(goals_canceling=[1])
                    return response

                node.goal_handle.cancel_goal_async.side_effect = cancel
                original_destroy = node.destroy_node

                def destroy():
                    assert rclpy.ok(), "ROS context died before node destruction"
                    assert events == ["cancel"]
                    events.append("destroy")
                    original_destroy()

                node.destroy_node = destroy
                return node

            def interrupt(*args):
                os.kill(os.getpid(), int(sys.argv[1]))
                raise AssertionError("signal was not handled")

            original_handler = signal.getsignal(int(sys.argv[1]))
            with patch.object(teleop.sys.stdin, "isatty", return_value=True), \
                    patch.object(teleop.termios, "tcgetattr", return_value=[]), \
                    patch.object(teleop.termios, "tcsetattr"), \
                    patch.object(teleop.tty, "setcbreak"), \
                    patch.object(teleop, "KeyboardJointTeleop", side_effect=make_node), \
                    patch.object(teleop.select, "select", side_effect=interrupt):
                assert teleop.run() == 0
            assert events == ["cancel", "destroy"]
            assert not rclpy.ok()
            assert signal.getsignal(int(sys.argv[1])) == original_handler
            print("signal cancellation verified")
        """)
        script_directory = str(Path(__file__).resolve().parents[1] / "scripts")
        import signal
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=signum):
                result = subprocess.run(
                    [sys.executable, "-c", script, str(int(signum))],
                    cwd=script_directory, capture_output=True, text=True, timeout=15,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("signal cancellation verified", result.stdout)


if __name__ == "__main__":
    unittest.main()
