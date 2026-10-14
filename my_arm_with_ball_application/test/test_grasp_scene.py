"""Scene requests and service failures without a running MoveIt instance."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix, PlanningScene

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import grasp_scene  # noqa: E402, I100


def pose(x=0.0, y=0.0, z=0.0, angle=0.0):
    result = PoseStamped()
    result.header.frame_id = 'world'
    result.pose.position.x = x
    result.pose.position.y = y
    result.pose.position.z = z
    result.pose.orientation.z = math.sin(angle / 2)
    result.pose.orientation.w = math.cos(angle / 2)
    return result


class GraspSceneTest(unittest.TestCase):

    def test_relative_pose_inverse_rotation_and_translation(self):
        result = grasp_scene.relative_ball_pose(pose(1.0, 3.0, 4.0),
                                                pose(1.0, 2.0, 3.0, math.pi / 2))
        self.assertAlmostEqual(result.position.x, 1.0)
        self.assertAlmostEqual(result.position.y, 0.0)
        self.assertAlmostEqual(result.position.z, 1.0)
        self.assertAlmostEqual(result.orientation.z, -math.sqrt(0.5))
        self.assertAlmostEqual(result.orientation.w, math.sqrt(0.5))

    def test_reject_invalid_frame_position_and_quaternion(self):
        invalid = pose()
        invalid.header.frame_id = 'base_link'
        with self.assertRaises(ValueError):
            grasp_scene.attach_request(invalid, pose())
        invalid = pose(float('nan'))
        with self.assertRaises(ValueError):
            grasp_scene.detach_request(invalid)
        invalid = pose()
        invalid.pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            grasp_scene.detach_request(invalid)

    def test_attach_and_detach_are_atomic_diffs_with_correct_geometry(self):
        request = grasp_scene.attach_request(pose(0.5), pose(0.4))
        self.assertTrue(request.scene.is_diff)
        self.assertTrue(request.scene.robot_state.is_diff)
        self.assertEqual(len(request.scene.world.collision_objects), 0)
        attached = request.scene.robot_state.attached_collision_objects[0]
        self.assertEqual(attached.object.id, 'apple')
        self.assertEqual(attached.link_name, 'rg2_hand')
        self.assertEqual(attached.touch_links, grasp_scene.TOUCH_LINKS)
        self.assertEqual(list(attached.object.primitives[0].dimensions), [0.0325])
        self.assertAlmostEqual(attached.object.primitive_poses[0].position.x, 0.1)
        request = grasp_scene.detach_request(pose(0.7))
        self.assertEqual(request.scene.robot_state.attached_collision_objects[0].object.operation,
                         attached.object.REMOVE)
        obj = request.scene.world.collision_objects[0]
        self.assertEqual(obj.header.frame_id, 'world')
        self.assertEqual(obj.primitive_poses[0].position.x, 0.7)
        self.assertEqual(obj.operation, obj.ADD)

    def test_matrix_preserves_unrelated_entries_and_defaults(self):
        matrix = AllowedCollisionMatrix(
            entry_names=['arm', 'basket'],
            entry_values=[AllowedCollisionEntry(enabled=[True, True]),
                          AllowedCollisionEntry(enabled=[True, False])],
            default_entry_names=['basket'], default_entry_values=[True])
        allowed = grasp_scene.ball_touch_matrix(matrix, True)
        self.assertEqual(matrix.entry_names, ['arm', 'basket'])
        self.assertTrue(allowed.entry_values[0].enabled[1])
        self.assertFalse(allowed.entry_values[1].enabled[1])
        self.assertEqual(allowed.default_entry_names, ['basket'])
        apple = allowed.entry_names.index('apple')
        self.assertTrue(allowed.entry_values[apple].enabled[1])
        floor = allowed.entry_names.index('pick_area_floor')
        self.assertTrue(allowed.entry_values[apple].enabled[floor])
        reset = grasp_scene.ball_touch_matrix(allowed, False)
        for link in grasp_scene.TOUCH_LINKS:
            index = allowed.entry_names.index(link)
            self.assertTrue(allowed.entry_values[apple].enabled[index])
            self.assertTrue(allowed.entry_values[index].enabled[apple])
            self.assertFalse(reset.entry_values[apple].enabled[index])
            self.assertFalse(reset.entry_values[index].enabled[apple])
        self.assertTrue(reset.entry_values[0].enabled[1])
        self.assertTrue(reset.entry_values[apple].enabled[floor])

    def test_wait_initial_world_then_verify_attach_and_destroy_clients(self):
        node = Mock()
        scenes = [PlanningScene(), grasp_scene.detach_request(pose()).scene,
                  grasp_scene.attach_request(pose(), pose()).scene]
        # REMOVE operations are diffs, not snapshots. Convert to snapshot states.
        scenes[1].robot_state.attached_collision_objects = []
        scenes[2].world.collision_objects = []
        responses = [SimpleNamespace(scene=scenes[0]), SimpleNamespace(scene=scenes[1]),
                     SimpleNamespace(success=True), SimpleNamespace(scene=scenes[2])]
        with patch.object(grasp_scene, '_call', side_effect=responses) as call, \
                patch.object(grasp_scene.rclpy, 'ok', return_value=True):
            self.assertTrue(grasp_scene.attach_ball(node, pose(), pose()))
        self.assertEqual(call.call_count, 4)
        self.assertEqual(node.destroy_client.call_count, 2)

    def test_apply_rejection_is_not_success(self):
        node = Mock()
        with patch.object(grasp_scene, '_wait_state'), \
                patch.object(grasp_scene, '_call', return_value=SimpleNamespace(success=False)):
            with self.assertRaisesRegex(RuntimeError, 'rejected'):
                grasp_scene.detach_ball(node, pose())
        self.assertEqual(node.destroy_client.call_count, 2)

    def test_timeout_cancels_future(self):
        future = Mock()
        future.done.return_value = False
        client = Mock()
        client.call_async.return_value = future
        with patch.object(grasp_scene.rclpy, 'ok', return_value=True), \
                patch.object(grasp_scene.rclpy, 'spin_until_future_complete'):
            with self.assertRaises(TimeoutError):
                grasp_scene._call(Mock(), client, Mock(), grasp_scene.time.monotonic() + 1)
        future.cancel.assert_called_once()

    def test_post_apply_state_must_be_confirmed(self):
        node = Mock()
        with patch.object(grasp_scene, '_wait_state',
                          side_effect=[None, TimeoutError('verify')]), \
                patch.object(grasp_scene, '_call', return_value=SimpleNamespace(success=True)):
            with self.assertRaises(TimeoutError):
                grasp_scene.attach_ball(node, pose(), pose())
        self.assertEqual(node.destroy_client.call_count, 2)

    def test_scene_object_pose_and_geometry_are_checked(self):
        scene = grasp_scene.detach_request(pose(0.5, -0.2, 0.04)).scene
        expected = pose(0.5, -0.2, 0.04)
        self.assertTrue(grasp_scene._object_matches(scene, False, expected.pose))
        scene.world.collision_objects[0].primitive_poses[0].position.z = 0.2
        self.assertFalse(grasp_scene._object_matches(scene, False, expected.pose))
        scene.world.collision_objects[0].primitives[0].dimensions[0] = 0.0325
        scene.world.collision_objects[0].pose = expected.pose
        scene.world.collision_objects[0].primitive_poses[0] = pose().pose
        self.assertTrue(grasp_scene._object_matches(scene, False, expected.pose))
        scene.world.collision_objects[0].primitive_poses[0].position.z = 0.04
        scene.world.collision_objects[0].primitives[0].dimensions[0] = 0.1
        self.assertFalse(grasp_scene._object_matches(scene, False, expected.pose))


if __name__ == '__main__':
    unittest.main()
