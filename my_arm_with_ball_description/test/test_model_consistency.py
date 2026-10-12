"""Guard the shared kinematics of independently maintained SDF and Xacro models."""

import math
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET


def numbers(value):
    return tuple(float(item) for item in value.split())


class ModelConsistencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = ET.parse(sys.argv[1]).getroot().find("model")
        urdf_xml = subprocess.check_output(["xacro", sys.argv[2]], text=True)
        cls.urdf = ET.fromstring(urdf_xml)

    def test_links_and_meshes(self):
        sdf_links = {link.get("name"): link for link in self.sdf.findall("link")}
        urdf_links = {link.get("name"): link for link in self.urdf.findall("link")}
        self.assertEqual(set(urdf_links), set(sdf_links) | {"world"})
        for name, sdf_link in sdf_links.items():
            urdf_link = urdf_links[name]
            for kind in ("visual", "collision"):
                sdf_uri = sdf_link.findtext(f"{kind}/geometry/mesh/uri")
                urdf_mesh = urdf_link.find(f"{kind}/geometry/mesh")
                if sdf_uri is None:
                    self.assertIsNone(urdf_mesh, (name, kind))
                else:
                    self.assertIsNotNone(urdf_mesh, (name, kind))
                    self.assertTrue(
                        urdf_mesh.get("filename").endswith(sdf_uri),
                        (name, kind),
                    )
                    sdf_pose = numbers(sdf_link.findtext(f"{kind}/pose", "0 0 0 0 0 0"))
                    urdf_origin = urdf_link.find(f"{kind}/origin")
                    urdf_pose = numbers(urdf_origin.get("xyz")) + numbers(urdf_origin.get("rpy"))
                    for actual, expected in zip(urdf_pose, sdf_pose):
                        self.assertTrue(math.isclose(actual, expected, abs_tol=1e-8), (name, kind))
            sdf_inertial = sdf_link.find("inertial")
            if sdf_inertial is not None:
                urdf_inertial = urdf_link.find("inertial")
                self.assertIsNotNone(urdf_inertial, name)
                self.assertAlmostEqual(
                    float(urdf_inertial.find("mass").get("value")),
                    float(sdf_inertial.findtext("mass")),
                    msg=name,
                )
                for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
                    self.assertAlmostEqual(
                        float(urdf_inertial.find("inertia").get(key)),
                        float(sdf_inertial.findtext(f"inertia/{key}")),
                        msg=(name, key),
                    )

    def test_joint_kinematics_and_limits(self):
        sdf_joints = {joint.get("name"): joint for joint in self.sdf.findall("joint")}
        urdf_joints = {joint.get("name"): joint for joint in self.urdf.findall("joint")}
        self.assertEqual(set(urdf_joints), set(sdf_joints))
        for name, sdf_joint in sdf_joints.items():
            urdf_joint = urdf_joints[name]
            self.assertEqual(urdf_joint.get("type"), sdf_joint.get("type"), name)
            self.assertEqual(urdf_joint.find("parent").get("link"), sdf_joint.findtext("parent"), name)
            self.assertEqual(urdf_joint.find("child").get("link"), sdf_joint.findtext("child"), name)
            sdf_pose = numbers(sdf_joint.findtext("pose", "0 0 0 0 0 0"))
            urdf_origin = urdf_joint.find("origin")
            urdf_pose = numbers(urdf_origin.get("xyz")) + numbers(urdf_origin.get("rpy"))
            for actual, expected in zip(urdf_pose, sdf_pose):
                self.assertTrue(math.isclose(actual, expected, abs_tol=1e-8), name)
            sdf_axis = sdf_joint.find("axis")
            if sdf_axis is None:
                continue
            self.assertEqual(numbers(urdf_joint.find("axis").get("xyz")), numbers(sdf_axis.findtext("xyz")), name)
            urdf_limit = urdf_joint.find("limit")
            for key in ("lower", "upper", "effort", "velocity"):
                self.assertAlmostEqual(float(urdf_limit.get(key)), float(sdf_axis.findtext(f"limit/{key}")), msg=(name, key))

    def test_control_interfaces(self):
        control = self.urdf.find("ros2_control")
        self.assertEqual(control.findtext("hardware/plugin"), "gz_ros2_control/GazeboSimSystem")
        joints = control.findall("joint")
        self.assertEqual(len(joints), 8)
        self.assertEqual(
            {joint.get("name") for joint in joints},
            {joint.get("name") for joint in self.sdf.findall("joint[@type='revolute']")},
        )
        for joint in joints:
            name = joint.get("name")
            commands = [item.get("name") for item in joint.findall("command_interface")]
            states = {item.get("name") for item in joint.findall("state_interface")}
            self.assertEqual(commands, [] if name.startswith("rg2_") else ["position"], name)
            self.assertEqual(states, {"position", "velocity"}, name)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
