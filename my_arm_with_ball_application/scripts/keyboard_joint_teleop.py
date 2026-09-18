#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_NAMES = tuple(f"panda_joint{index}" for index in range(1, 8))
JOINT_LIMITS = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)
INITIAL_POSITIONS = (0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.0)
ANGLE_STEP = 0.05


class KeyboardJointTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_joint_teleop")
        self.publishers = tuple(
            self.create_publisher(Float64, f"/{joint_name}/cmd_pos", 10)
            for joint_name in JOINT_NAMES
        )
        self.positions = list(INITIAL_POSITIONS)
        self.selected_joint = 0

    def publish_joint(self, joint_index: int) -> None:
        message = Float64()
        message.data = self.positions[joint_index]
        self.publishers[joint_index].publish(message)

    def publish_all(self) -> None:
        for joint_index in range(len(JOINT_NAMES)):
            self.publish_joint(joint_index)

    def change_selected_joint(self, delta: float) -> None:
        lower_limit, upper_limit = JOINT_LIMITS[self.selected_joint]
        next_position = self.positions[self.selected_joint] + delta
        self.positions[self.selected_joint] = max(
            lower_limit, min(upper_limit, next_position)
        )
        self.publish_joint(self.selected_joint)

    def reset_joint(self, joint_index: int) -> None:
        self.positions[joint_index] = INITIAL_POSITIONS[joint_index]
        self.publish_joint(joint_index)

    def print_help(self) -> None:
        self.get_logger().info(
            "Select joint with 1-7 | a: -0.05 rad | d: +0.05 rad | "
            "r: reset selected | x: reset all | q: quit"
        )


def run() -> int:
    if not sys.stdin.isatty():
        print("keyboard_joint_teleop requires an interactive terminal.", file=sys.stderr)
        return 1

    rclpy.init()
    node = KeyboardJointTeleop()
    original_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
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

            key = sys.stdin.read(1).lower()
            if key in "1234567":
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
            elif key == "x":
                node.positions = list(INITIAL_POSITIONS)
                node.publish_all()
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
