"""Verified Planning Scene transitions for the stage-4 single-ball test.

The synchronous entry points spin the supplied node and must be called from its
main control loop, not from an executor callback. Gazebo transitions remain the
caller's responsibility; a service/verification failure raises instead of hiding
an inconsistent simulation/Planning Scene state.
"""

import math
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from shape_msgs.msg import SolidPrimitive

BALL_ID = 'apple'
BALL_RADIUS = 0.0325
HAND_LINK = 'rg2_hand'
TOUCH_LINKS = ['rg2_hand', 'rg2_leftfinger', 'rg2_rightfinger']
FLOOR_ID = 'pick_area_floor'


def _quaternion(pose):
    values = (pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Pose requires a finite, nonzero quaternion')
    return tuple(value / norm for value in values)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _validate_world(pose):
    if pose.header.frame_id != 'world':
        raise ValueError('Ball and hand poses must use the world frame')
    if not all(math.isfinite(value) for value in (
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)):
        raise ValueError('Pose position must be finite')
    _quaternion(pose.pose)


def relative_ball_pose(ball_pose_world, hand_pose_world):
    """Compute hand^-1 * ball, normalizing valid input quaternions."""
    _validate_world(ball_pose_world)
    _validate_world(hand_pose_world)
    ball, hand = ball_pose_world.pose, hand_pose_world.pose
    hx, hy, hz, hw = _quaternion(hand)
    inverse = (-hx, -hy, -hz, hw)
    offset = (ball.position.x - hand.position.x,
              ball.position.y - hand.position.y,
              ball.position.z - hand.position.z, 0.0)
    rotated = _multiply(_multiply(inverse, offset), (hx, hy, hz, hw))
    orientation = _multiply(inverse, _quaternion(ball))
    result = Pose()
    result.position.x, result.position.y, result.position.z = rotated[:3]
    (result.orientation.x, result.orientation.y,
     result.orientation.z, result.orientation.w) = orientation
    return result


def _ball_object(frame, pose=None):
    result = CollisionObject()
    result.id = BALL_ID
    result.header.frame_id = frame
    result.operation = CollisionObject.REMOVE if pose is None else CollisionObject.ADD
    if pose is not None:
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [BALL_RADIUS]
        result.primitives = [sphere]
        result.primitive_poses = [pose]
    return result


def attach_request(ball_pose_world, hand_pose_world):
    """Build an attached-object ADD; MoveIt removes same-ID world object."""
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    attached = AttachedCollisionObject()
    attached.link_name = HAND_LINK
    attached.touch_links = list(TOUCH_LINKS)
    attached.object = _ball_object(
        HAND_LINK, relative_ball_pose(ball_pose_world, hand_pose_world))
    request.scene.robot_state.attached_collision_objects = [attached]
    return request


def detach_request(ball_pose_world):
    """Build one atomic diff: remove attached ball and restore measured world pose."""
    _validate_world(ball_pose_world)
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    attached = AttachedCollisionObject()
    attached.link_name = HAND_LINK
    attached.object = _ball_object(HAND_LINK)
    request.scene.robot_state.attached_collision_objects = [attached]
    request.scene.world.collision_objects = [_ball_object('world', ball_pose_world.pose)]
    return request


def ball_state(scene):
    """Return apple world presence and all links holding apple."""
    world = any(obj.id == BALL_ID for obj in scene.world.collision_objects)
    links = [obj.link_name for obj in scene.robot_state.attached_collision_objects
             if obj.object.id == BALL_ID]
    return world, links


def _object_matches(scene, attached, expected_pose):
    objects = (scene.robot_state.attached_collision_objects if attached
               else scene.world.collision_objects)
    matches = [item.object if attached else item for item in objects
               if (item.object.id if attached else item.id) == BALL_ID]
    if len(matches) != 1:
        return False
    obj = matches[0]
    frame = HAND_LINK if attached else 'world'
    if obj.header.frame_id != frame or len(obj.primitives) != 1 or len(obj.primitive_poses) != 1:
        return False
    primitive = obj.primitives[0]
    if primitive.type != SolidPrimitive.SPHERE or len(primitive.dimensions) != 1:
        return False
    origin = obj.pose
    local = obj.primitive_poses[0]
    quaternion = (origin.orientation.x, origin.orientation.y,
                  origin.orientation.z, origin.orientation.w)
    rotation = ((0.0, 0.0, 0.0, 1.0) if quaternion == (0.0, 0.0, 0.0, 0.0)
                else _quaternion(origin))
    point = (local.position.x, local.position.y, local.position.z, 0.0)
    rotated = _multiply(_multiply(rotation, point),
                        (-rotation[0], -rotation[1], -rotation[2], rotation[3]))
    expected = expected_pose.position
    return (abs(primitive.dimensions[0] - BALL_RADIUS) < 1e-4
            and all(abs(getattr(origin.position, axis) + rotated[index]
                        - getattr(expected, axis)) < 0.02
                    for index, axis in enumerate(('x', 'y', 'z'))))


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0 or not rclpy.ok():
        raise TimeoutError('Planning Scene transition timed out or ROS shut down')
    return remaining


def _call(node, client, request, deadline):
    if not client.wait_for_service(timeout_sec=_remaining(deadline)):
        raise TimeoutError('Planning Scene service unavailable')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=_remaining(deadline))
    if not future.done():
        future.cancel()
        raise TimeoutError('Planning Scene service response timed out')
    result = future.result()
    if result is None:
        raise RuntimeError('Planning Scene service returned no response')
    return result


def _wait_state(node, client, expected, deadline, expected_pose=None):
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    reported = False
    while True:
        scene = _call(node, client, request, deadline).scene
        matches = expected_pose is None or _object_matches(
            scene, bool(expected[1]), expected_pose)
        if ball_state(scene) == expected and matches:
            return
        if expected_pose is not None and not reported and ball_state(scene) == expected:
            objects = (scene.robot_state.attached_collision_objects if expected[1]
                       else scene.world.collision_objects)
            obj = objects[0].object if expected[1] else objects[0]
            node.get_logger().warning(
                f'Planning Scene ball geometry mismatch: frame={obj.header.frame_id}, '
                f'primitives={len(obj.primitives)}, poses={len(obj.primitive_poses)}, '
                f'object_pose={obj.pose}, '
                f'actual={obj.primitive_poses[0] if obj.primitive_poses else None}, '
                f'expected={expected_pose}')
            reported = True
        time.sleep(min(0.05, _remaining(deadline)))


def _transition(node, request, attached, timeout_sec, refresh=False):
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('timeout_sec must be finite and positive')
    deadline = time.monotonic() + timeout_sec
    apply_client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    get_client = node.create_client(GetPlanningScene, 'get_planning_scene')
    try:
        # The initial loader applies apple and basket once. Waiting for apple
        # prevents its late startup request from overwriting an attached ball.
        initial = (True, []) if attached or refresh else (False, [HAND_LINK])
        _wait_state(node, get_client, initial, deadline)
        if not _call(node, apply_client, request, deadline).success:
            raise RuntimeError('MoveIt rejected the Planning Scene transition')
        expected = (False, [HAND_LINK]) if attached else (True, [])
        obj = (request.scene.robot_state.attached_collision_objects[0].object
               if attached else request.scene.world.collision_objects[0])
        _wait_state(node, get_client, expected, deadline, obj.primitive_poses[0])
        return True
    finally:
        node.destroy_client(apply_client)
        node.destroy_client(get_client)


def attach_ball(node, ball_pose_world, hand_pose_world, *, timeout_sec=30.0):
    """Attach apple and return True only after GetPlanningScene verifies it."""
    return _transition(node, attach_request(ball_pose_world, hand_pose_world),
                       True, timeout_sec)


def detach_ball(node, ball_pose_world, *, timeout_sec=30.0):
    """Restore apple to measured Gazebo world pose and verify world/attached state."""
    return _transition(node, detach_request(ball_pose_world), False, timeout_sec)


def refresh_ball(node, ball_pose_world, *, timeout_sec=30.0):
    """Replace the released ball's world pose with the measured post-fall pose."""
    _validate_world(ball_pose_world)
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    request.scene.world.collision_objects = [_ball_object('world', ball_pose_world.pose)]
    return _transition(node, request, False, timeout_sec, refresh=True)


def ball_touch_matrix(matrix, allowed):
    """Copy an ACM and change only symmetric apple/touch-link entries."""
    import copy

    result = copy.deepcopy(matrix)
    defaults = dict(zip(result.default_entry_names, result.default_entry_values))
    for name in [BALL_ID] + TOUCH_LINKS + [FLOOR_ID]:
        if name in result.entry_names:
            continue
        old_names = list(result.entry_names)
        result.entry_names.append(name)
        values = []
        for index, other in enumerate(old_names):
            applicable = [defaults[item] for item in (name, other) if item in defaults]
            value = all(applicable) if applicable else False
            result.entry_values[index].enabled.append(value)
            values.append(value)
        values.append(defaults.get(name, False))
        result.entry_values.append(AllowedCollisionEntry(enabled=values))
    ball_index = result.entry_names.index(BALL_ID)
    for link in TOUCH_LINKS:
        index = result.entry_names.index(link)
        result.entry_values[ball_index].enabled[index] = allowed
        result.entry_values[index].enabled[ball_index] = allowed
    # The ball starts and ends resting on the floor; that contact is expected
    # even while the ball is attached for the first part of a lift.
    floor_index = result.entry_names.index(FLOOR_ID)
    result.entry_values[ball_index].enabled[floor_index] = True
    result.entry_values[floor_index].enabled[ball_index] = True
    return result


def set_ball_touch_allowed(node, allowed, *, timeout_sec=30.0):
    """Allow/reset apple contact with hand/fingers, preserving unrelated ACM pairs."""
    if not isinstance(allowed, bool):
        raise ValueError('allowed must be a bool')
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('timeout_sec must be finite and positive')
    deadline = time.monotonic() + timeout_sec
    get_client = node.create_client(GetPlanningScene, 'get_planning_scene')
    apply_client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    try:
        get_request = GetPlanningScene.Request()
        get_request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        scene = _call(node, get_client, get_request, deadline).scene
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.allowed_collision_matrix = ball_touch_matrix(
            scene.allowed_collision_matrix, allowed)
        if not _call(node, apply_client, request, deadline).success:
            raise RuntimeError('MoveIt rejected the ball touch collision matrix')
        while True:
            matrix = _call(node, get_client, get_request, deadline).scene.allowed_collision_matrix
            names = matrix.entry_names
            if all(name in names for name in [BALL_ID] + TOUCH_LINKS + [FLOOR_ID]):
                ball = names.index(BALL_ID)
                if all(matrix.entry_values[ball].enabled[names.index(link)] == allowed
                       and matrix.entry_values[names.index(link)].enabled[ball] == allowed
                       for link in TOUCH_LINKS) and (
                        matrix.entry_values[ball].enabled[names.index(FLOOR_ID)]
                        and matrix.entry_values[names.index(FLOOR_ID)].enabled[ball]):
                    return True
            time.sleep(min(0.05, _remaining(deadline)))
    finally:
        node.destroy_client(apply_client)
        node.destroy_client(get_client)
