"""Threshold and empty-gripper negative cases for the stage-4 gate."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from grasp_logic import may_attach, MIN_OPENING_RAD, opening_angle_rad  # noqa: E402


class GraspLogicTest(unittest.TestCase):

    def test_total_opening_and_finger_direction(self):
        self.assertAlmostEqual(opening_angle_rad(0.12, 0.13), 0.25)
        self.assertAlmostEqual(opening_angle_rad(0.0, -0.001), -0.001)

    def test_strict_five_degree_boundary(self):
        half = MIN_OPENING_RAD / 2.0
        self.assertFalse(may_attach(True, half, half))
        self.assertFalse(may_attach(True, half - 0.001, half))
        self.assertTrue(may_attach(True, half + 0.001, half))

    def test_no_contact_never_attaches(self):
        self.assertFalse(may_attach(False, 0.5, 0.5))
        self.assertFalse(may_attach(None, 0.5, 0.5))

    def test_invalid_joint_state_fails_closed(self):
        for value in [math.nan, math.inf, -math.inf, -0.1]:
            with self.assertRaises(ValueError):
                opening_angle_rad(value, 0.2)


if __name__ == '__main__':
    unittest.main()
