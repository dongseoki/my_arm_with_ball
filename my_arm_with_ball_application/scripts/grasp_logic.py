"""Pure, testable gate for the agreed RG2 grasp condition."""

import math


MIN_OPENING_RAD = math.radians(5.0)


def opening_angle_rad(left_joint_rad, right_joint_rad):
    """Both modeled finger joints increase in the opening direction."""
    if not math.isfinite(left_joint_rad) or not math.isfinite(right_joint_rad):
        raise ValueError('Finger joint positions must be finite')
    if left_joint_rad < -0.002 or right_joint_rad < -0.002:
        raise ValueError('Finger joint position outside closed limit')
    return left_joint_rad + right_joint_rad


def may_attach(contact, left_joint_rad, right_joint_rad):
    """Check contact and opening after the close action has ended."""
    return bool(contact) and opening_angle_rad(left_joint_rad, right_joint_rad) > MIN_OPENING_RAD
