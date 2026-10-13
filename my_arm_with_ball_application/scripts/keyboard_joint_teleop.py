#!/usr/bin/env python3

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_LIMITS = ((-6.28319, 6.28319),) * 6
INITIAL_POSITIONS = (0.0,) * 6
GRIPPER_JOINT_NAMES = ("rg2_finger_joint1", "rg2_finger_joint2")
GRIPPER_LIMITS = (0.0, 1.18)
ANGLE_STEP = 0.05
ARM_CONTROLLER_NAME = "arm_trajectory_controller"
GRIPPER_CONTROLLER_NAME = "gripper_position_controller"
CONTROLLER_NAMES = (ARM_CONTROLLER_NAME, GRIPPER_CONTROLLER_NAME)
CONTROLLER_WAIT_TIMEOUT = 120.0
STATE_MAX_AGE = 0.5
GOAL_POSITION_TOLERANCE = 0.01
GOAL_TIME_TOLERANCE = 2.0
TRAJECTORY_TIMEOUT_MARGIN = 2.0
MIN_TRAJECTORY_DURATION = 1.0
CONSERVATIVE_JOINT_SPEED = 0.2


class KeyboardJointTeleop(Node):
    """Read keyboard input and execute guarded arm trajectories."""

    def __init__(self) -> None:
        super().__init__("keyboard_joint_teleop")
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{ARM_CONTROLLER_NAME}/follow_joint_trajectory",
        )
        self.gripper_publisher = self.create_publisher(
            Float64MultiArray, f"/{GRIPPER_CONTROLLER_NAME}/commands", 10
        )
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.joint_positions: dict[str, float] = {}
        self.joint_state_received_at: float | None = None
        self.selected_joint = 0
        self.gripper_position = GRIPPER_LIMITS[0]
        self.action_in_progress = False
        self.goal_timed_out = False
        self.goal_deadline: float | None = None
        self.goal_target: tuple[float, ...] | None = None
        self.awaiting_state_deadline: float | None = None
        self.goal_handle = None

    def _joint_state_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        state = dict(zip(message.name, message.position))
        if not all(name in state and math.isfinite(state[name]) for name in JOINT_NAMES):
            return
        self.joint_positions = {name: float(state[name]) for name in JOINT_NAMES}
        self.joint_state_received_at = time.monotonic()

    def _state_is_fresh(self) -> bool:
        return (
            self.joint_state_received_at is not None
            and time.monotonic() - self.joint_state_received_at <= STATE_MAX_AGE
            and all(name in self.joint_positions for name in JOINT_NAMES)
        )

    def _current_arm_positions(self) -> tuple[float, ...] | None:
        if not self._state_is_fresh():
            return None
        return tuple(self.joint_positions[name] for name in JOINT_NAMES)

    def _log_error(self, korean: str, english: str) -> None:
        self.get_logger().error(f"오류: {korean} / Error: {english}")

    def wait_for_controllers(self, timeout_sec: float = CONTROLLER_WAIT_TIMEOUT) -> bool:
        client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info(
            f"컨트롤러 활성화 대기 / Waiting for controllers: {', '.join(CONTROLLER_NAMES)}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if not client.wait_for_service(timeout_sec=min(0.5, remaining)):
                continue
            future = client.call_async(ListControllers.Request())
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))
            if not future.done():
                break
            try:
                controllers = future.result().controller
            except Exception as error:
                self.get_logger().warning(
                    f"컨트롤러 상태 조회 실패: {error} / Failed to query controller state: {error}"
                )
                continue
            active_names = {
                controller.name for controller in controllers if controller.state == "active"
            }
            if all(name in active_names for name in CONTROLLER_NAMES):
                return True
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))

        self._log_error(
            f"필요한 컨트롤러가 제한 시간 내 활성화되지 않았습니다: {', '.join(CONTROLLER_NAMES)}",
            "required controllers did not become active before timeout: "
            f"{', '.join(CONTROLLER_NAMES)}",
        )
        return False

    def wait_for_action_server(self, timeout_sec: float = CONTROLLER_WAIT_TIMEOUT) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.action_client.wait_for_server(timeout_sec=0.2):
                return True
            rclpy.spin_once(self, timeout_sec=0.0)
        self._log_error(
            "팔 trajectory 액션 서버를 찾을 수 없습니다",
            "arm trajectory action server was not available before timeout",
        )
        return False

    def wait_for_fresh_joint_state(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._current_arm_positions() is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self._log_error(
            "신선한 6축 /joint_states를 받지 못했습니다",
            "did not receive a fresh six-joint /joint_states message",
        )
        return False

    def _trajectory_duration(self, start: tuple[float, ...], target: tuple[float, ...]) -> float:
        max_delta = max(abs(goal - current) for current, goal in zip(start, target))
        return max(MIN_TRAJECTORY_DURATION, max_delta / CONSERVATIVE_JOINT_SPEED)

    def _make_goal(
        self, start: tuple[float, ...], target: tuple[float, ...], duration_sec: float
    ) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        start_point = JointTrajectoryPoint()
        start_point.positions = list(start)
        target_point = JointTrajectoryPoint()
        target_point.positions = list(target)
        target_point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1.0) * 1_000_000_000)
        )
        goal.trajectory.points = [start_point, target_point]
        goal.goal_time_tolerance = Duration(sec=int(GOAL_TIME_TOLERANCE))
        goal.goal_tolerance = [
            JointTolerance(name=name, position=GOAL_POSITION_TOLERANCE)
            for name in JOINT_NAMES
        ]
        return goal

    def send_arm_goal(self, target: tuple[float, ...]) -> bool:
        if self.action_in_progress:
            self.get_logger().warning(
                "팔 동작 중 입력을 무시했습니다 / Ignoring arm input while trajectory is active"
            )
            return False

        start = self._current_arm_positions()
        if start is None:
            self._log_error(
                "관절 상태가 없거나 0.5초보다 오래되어 명령하지 않습니다",
                "joint state is missing or older than 0.5 seconds; command was not sent",
            )
            return False
        if len(target) != len(JOINT_NAMES):
            self._log_error("목표 관절 수가 6개가 아닙니다", "target must contain all six arm joints")
            return False
        for name, position, limits in zip(JOINT_NAMES, target, JOINT_LIMITS):
            if not math.isfinite(position) or not limits[0] <= position <= limits[1]:
                self._log_error(
                    f"{name} 목표가 유효 범위를 벗어났습니다: {position:.4f} rad",
                    f"target for {name} is outside its valid range: {position:.4f} rad",
                )
                return False

        duration = self._trajectory_duration(start, target)
        goal = self._make_goal(start, target, duration)
        self.action_in_progress = True
        self.goal_timed_out = False
        self.goal_target = target
        self.goal_deadline = time.monotonic() + duration + TRAJECTORY_TIMEOUT_MARGIN
        self.awaiting_state_deadline = None
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._log_error(
                f"trajectory 액션 요청 실패: {error}",
                f"trajectory action request failed: {error}",
            )
            self._clear_action()
            return
        if not handle.accepted:
            self._log_error("trajectory 목표가 거부되었습니다", "trajectory goal was rejected")
            self._clear_action()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_callback)
        if self.goal_timed_out:
            self.goal_handle.cancel_goal_async().add_done_callback(self._cancel_callback)

    def _result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:
            self._log_error(
                f"trajectory 결과 수신 실패: {error}",
                f"failed to receive trajectory result: {error}",
            )
            self._clear_action()
            return

        if self.goal_timed_out:
            self._log_error(
                "trajectory 제한 시간이 초과되었습니다; 액션 종료를 확인했습니다",
                "trajectory timed out; terminal action result was received",
            )
            self._clear_action()
            return
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self._log_error(
                f"trajectory 실행 실패 (code={result.error_code}, {result.error_string})",
                "trajectory execution failed "
                f"(code={result.error_code}, {result.error_string})",
            )
            self._clear_action()
            return

        self.goal_deadline = None
        self.awaiting_state_deadline = time.monotonic() + TRAJECTORY_TIMEOUT_MARGIN

    def _clear_action(self) -> None:
        self.action_in_progress = False
        self.goal_timed_out = False
        self.goal_deadline = None
        self.goal_target = None
        self.awaiting_state_deadline = None
        self.goal_handle = None

    def check_action_progress(self) -> None:
        if not self.action_in_progress:
            return
        now = time.monotonic()
        if self.awaiting_state_deadline is not None:
            current = self._current_arm_positions()
            if current is not None and self.goal_target is not None:
                max_error = max(
                    abs(actual - target) for actual, target in zip(current, self.goal_target)
                )
                if max_error <= GOAL_POSITION_TOLERANCE:
                    self.get_logger().info(
                        f"목표 도달 확인 (최대 오차 {max_error:.4f} rad) / "
                        f"Target verified (max error {max_error:.4f} rad)"
                    )
                    self._clear_action()
                    return
            if now >= self.awaiting_state_deadline:
                self._log_error(
                    "액션은 성공했지만 /joint_states가 목표 허용 오차에 도달하지 않았습니다",
                    "action succeeded but /joint_states did not reach the goal tolerance",
                )
                self._clear_action()
                return

        deadline_passed = self.goal_deadline is not None and now >= self.goal_deadline
        if deadline_passed and not self.goal_timed_out:
            self.goal_timed_out = True
            self._log_error(
                "trajectory 실행 시간이 초과되어 취소를 요청합니다",
                "trajectory execution timed out; requesting cancellation",
            )
            if self.goal_handle is None:
                # Wait for the accepted goal handle before attempting cancellation.
                return
            self.goal_handle.cancel_goal_async().add_done_callback(self._cancel_callback)

    def _cancel_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._log_error(
                f"액션 취소 결과 확인 실패: {error}",
                f"failed to confirm action cancellation: {error}",
            )
            return
        if not response.goals_canceling:
            self.get_logger().error(
                "오류: 액션 취소가 확인되지 않았습니다; 팔 입력을 차단합니다 / "
                "Error: action cancellation was not confirmed; arm input remains blocked"
            )

    def change_selected_joint(self, delta: float) -> bool:
        current = self._current_arm_positions()
        if current is None:
            self._log_error(
                "신선한 관절 상태가 없어 상대 목표를 계산할 수 없습니다",
                "cannot calculate relative target without a fresh joint state",
            )
            return False
        target = list(current)
        target[self.selected_joint] += delta
        return self.send_arm_goal(tuple(target))

    def reset_joint(self, joint_index: int) -> bool:
        current = self._current_arm_positions()
        if current is None:
            self._log_error(
                "신선한 관절 상태가 없어 리셋 목표를 보낼 수 없습니다",
                "cannot send reset goal without a fresh joint state",
            )
            return False
        target = list(current)
        target[joint_index] = INITIAL_POSITIONS[joint_index]
        return self.send_arm_goal(tuple(target))

    def reset_arm(self) -> bool:
        return self.send_arm_goal(INITIAL_POSITIONS)

    def set_gripper(self, position: float) -> None:
        lower_limit, upper_limit = GRIPPER_LIMITS
        self.gripper_position = max(lower_limit, min(upper_limit, position))
        message = Float64MultiArray()
        message.data = [self.gripper_position] * len(GRIPPER_JOINT_NAMES)
        self.gripper_publisher.publish(message)

    def print_help(self) -> None:
        self.get_logger().info(
            "UR5 joints 1-6 | a: -0.05 rad | d: +0.05 rad | "
            "o: open RG2 | c: close RG2 | r: reset selected | x: reset arm | q: quit"
        )


def run() -> int:
    if not sys.stdin.isatty():
        print(
            "오류: 대화형 터미널이 필요합니다. / Error: an interactive terminal is required.",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = KeyboardJointTeleop()
    original_terminal_settings = termios.tcgetattr(sys.stdin)
    exit_code = 0

    try:
        if not node.wait_for_controllers() or not node.wait_for_action_server():
            exit_code = 1
            return exit_code
        if not node.wait_for_fresh_joint_state():
            exit_code = 1
            return exit_code

        tty.setcbreak(sys.stdin.fileno())
        node.print_help()
        node.get_logger().info(
            f"선택 관절: {JOINT_NAMES[node.selected_joint]} / "
            f"Selected joint: {JOINT_NAMES[node.selected_joint]}"
        )

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.check_action_progress()
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not readable:
                continue

            key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()
            if key == "q":
                break
            if key in "123456":
                if node.action_in_progress:
                    node.get_logger().warning(
                        "팔 동작 중 입력을 무시했습니다 / Ignoring arm input while trajectory is active"
                    )
                else:
                    node.selected_joint = int(key) - 1
                    node.get_logger().info(
                        f"선택 관절: {JOINT_NAMES[node.selected_joint]} / "
                        f"Selected joint: {JOINT_NAMES[node.selected_joint]}"
                    )
            elif key == "a":
                node.change_selected_joint(-ANGLE_STEP)
            elif key == "d":
                node.change_selected_joint(ANGLE_STEP)
            elif key == "r":
                node.reset_joint(node.selected_joint)
            elif key == "x":
                node.reset_arm()
            elif key == "o":
                node.set_gripper(GRIPPER_LIMITS[1])
            elif key == "c":
                node.set_gripper(GRIPPER_LIMITS[0])
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(run())
