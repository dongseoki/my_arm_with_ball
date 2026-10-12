#!/usr/bin/env python3

import os
import select
import sys
import termios
import time
import tty

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_LIMITS = (
    (-6.28319, 6.28319),
    (-6.28319, 6.28319),
    (-6.28319, 6.28319),
    (-6.28319, 6.28319),
    (-6.28319, 6.28319),
    (-6.28319, 6.28319),
)
INITIAL_POSITIONS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
GRIPPER_JOINT_NAMES = ("rg2_finger_joint1", "rg2_finger_joint2")
GRIPPER_LIMITS = (0.0, 1.18)
INITIAL_GRIPPER_POSITION = 0.0
ANGLE_STEP = 0.05
ARM_CONTROLLER_NAME = "arm_position_controller"
GRIPPER_CONTROLLER_NAME = "gripper_position_controller"
CONTROLLER_NAMES = (ARM_CONTROLLER_NAME, GRIPPER_CONTROLLER_NAME)
CONTROLLER_WAIT_TIMEOUT = 120.0


class KeyboardJointTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_joint_teleop")
        self.arm_publisher = self.create_publisher(
            Float64MultiArray, f"/{ARM_CONTROLLER_NAME}/commands", 10
        )
        self.gripper_publisher = self.create_publisher(
            Float64MultiArray, f"/{GRIPPER_CONTROLLER_NAME}/commands", 10
        )
        self.positions = list(INITIAL_POSITIONS)
        self.gripper_position = INITIAL_GRIPPER_POSITION
        self.selected_joint = 0

    def wait_for_controllers(self, timeout_sec: float = CONTROLLER_WAIT_TIMEOUT) -> bool:
        client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info(f"Waiting for {', '.join(CONTROLLER_NAMES)} to become active")

        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if not client.wait_for_service(timeout_sec=min(0.5, remaining)):
                continue

            future = client.call_async(ListControllers.Request())
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=min(0.2, deadline - time.monotonic()))

            if not future.done():
                break
            try:
                controllers = future.result().controller
            except Exception as error:
                self.get_logger().warn(f"Could not query controller state: {error}")
                continue
            active_names = {
                controller.name for controller in controllers if controller.state == "active"
            }
            if all(name in active_names for name in CONTROLLER_NAMES):
                return True

            time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))

        self.get_logger().error(f"{', '.join(CONTROLLER_NAMES)} did not become active")
        return False

    def publish_arm(self) -> None:
        message = Float64MultiArray()
        message.data = list(self.positions)
        self.arm_publisher.publish(message)

    def publish_all(self) -> None:
        self.publish_arm()
        self.publish_gripper()

    def publish_gripper(self) -> None:
        message = Float64MultiArray()
        message.data = [self.gripper_position] * len(GRIPPER_JOINT_NAMES)
        self.gripper_publisher.publish(message)

    def change_selected_joint(self, delta: float) -> None:
        lower_limit, upper_limit = JOINT_LIMITS[self.selected_joint]
        next_position = self.positions[self.selected_joint] + delta
        self.positions[self.selected_joint] = max(
            lower_limit, min(upper_limit, next_position)
        )
        self.publish_arm()

    def reset_joint(self, joint_index: int) -> None:
        self.positions[joint_index] = INITIAL_POSITIONS[joint_index]
        self.publish_arm()

    def reset_all(self) -> None:
        self.positions = list(INITIAL_POSITIONS)
        self.gripper_position = INITIAL_GRIPPER_POSITION
        self.publish_all()

    def set_gripper(self, position: float) -> None:
        lower_limit, upper_limit = GRIPPER_LIMITS
        self.gripper_position = max(lower_limit, min(upper_limit, position))
        self.publish_gripper()

    def print_help(self) -> None:
        self.get_logger().info(
            "Select UR5 joint with 1-6 | a: -0.05 rad | d: +0.05 rad | "
            "o: open RG2 | c: close RG2 | r: reset selected | "
            "x: reset all | q: quit"
        )


def run() -> int:
    if not sys.stdin.isatty():
        print("keyboard_joint_teleop requires an interactive terminal.", file=sys.stderr)
        return 1

    rclpy.init()
    node = KeyboardJointTeleop()
    original_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        if not node.wait_for_controllers():
            return 1

        tty.setcbreak(sys.stdin.fileno())
        node.publish_all()
        node.print_help()
        node.get_logger().info(
            f"Selected {JOINT_NAMES[node.selected_joint]} "
            f"(target={node.positions[node.selected_joint]:.4f} rad)"
        )

        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                rclpy.spin_once(node, timeout_sec=0.0)
                continue

            key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()
            if key in "123456":
                node.selected_joint = int(key) - 1
                node.get_logger().info(
                    f"Selected {JOINT_NAMES[node.selected_joint]} "
                    f"(target={node.positions[node.selected_joint]:.4f} rad)"
                )
            elif key == "a":
                node.change_selected_joint(-ANGLE_STEP)
            elif key == "d":
                node.change_selected_joint(ANGLE_STEP)
            elif key == "r":
                node.reset_joint(node.selected_joint)
            elif key == "o":
                node.set_gripper(GRIPPER_LIMITS[1])
            elif key == "c":
                node.set_gripper(GRIPPER_LIMITS[0])
            elif key == "x":
                node.reset_all()
            elif key == "q":
                break

            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(run())
