#!/usr/bin/env python3
"""Short Gazebo/MoveIt grasp-and-release acceptance test (not pick-and-place)."""

import math
import random
import sys
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from grasp_logic import may_attach
from grasp_scene import attach_ball, detach_ball, refresh_ball, set_ball_touch_allowed
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
FINGER_JOINTS = ['rg2_finger_joint1', 'rg2_finger_joint2']
PREGRASP_Q = [
    -0.6082914162445299, 0.774475422756991, 0.7405722030642679,
    -3.0858439511376563, 1.570796327804393, 0.962504912344154,
]
SUCCESS = 1


class GraspTest(Node):

    def __init__(self):
        super().__init__('stage4_grasp_test')
        self.joints = {}
        self.contact = None
        self.attached = None
        self.ball_pose = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Bool, '/grasp/contact', self.on_contact, 10)
        self.create_subscription(Bool, '/grasp/attached', self.on_attached, 10)
        self.create_subscription(PoseStamped, '/grasp/ball_pose', self.on_ball_pose, 10)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.arm_controller = ActionClient(
            self, FollowJointTrajectory, '/arm_trajectory_controller/follow_joint_trajectory')
        self.gripper_controller = ActionClient(
            self, FollowJointTrajectory, '/gripper_trajectory_controller/follow_joint_trajectory')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.attach_client = self.create_client(Trigger, '/grasp/attach')
        self.detach_client = self.create_client(Trigger, '/grasp/detach')

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def on_contact(self, msg):
        self.contact = msg.data

    def on_attached(self, msg):
        self.attached = msg.data

    def on_ball_pose(self, msg):
        self.ball_pose = msg

    def until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f'{label}: timeout after {timeout}s')

    def call(self, client, request, label, timeout=30.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{label}: service unavailable')
        future = client.call_async(request)
        self.until(future.done, timeout, label)
        if future.exception() is not None:
            raise RuntimeError(f'{label}: {future.exception()}')
        return future.result()

    def scene_has_ball(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        try:
            response = self.call(self.scene_client, req, 'planning scene', 5.0)
        except RuntimeError:
            return False
        return any(obj.id == 'apple' for obj in response.scene.world.collision_objects)

    def wait_ready(self):
        self.get_logger().info('Waiting for Gazebo, controllers, MoveIt and initial scene')
        self.until(lambda: len(self.joints.keys() & set(ARM_JOINTS + FINGER_JOINTS)) == 8
                   and self.contact is not None and self.attached is not None
                   and self.ball_pose is not None, 120.0, 'simulation state')
        if not self.move_group.wait_for_server(timeout_sec=60.0):
            raise RuntimeError('MoveGroup action unavailable')
        for client, label in [(self.arm_controller, 'arm controller'),
                              (self.gripper_controller, 'gripper controller')]:
            if not client.wait_for_server(timeout_sec=60.0):
                raise RuntimeError(f'{label} action unavailable')
        for client, label in [(self.ik_client, 'IK'), (self.fk_client, 'FK'),
                              (self.attach_client, 'attach'), (self.detach_client, 'detach')]:
            if not client.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f'{label} service unavailable')
        self.until(self.scene_has_ball, 60.0, 'initial apple collision object')
        # move_group owns separate action clients and may discover controllers
        # after this node's clients have become ready.
        ready_at = time.monotonic() + 2.0
        self.until(lambda: time.monotonic() >= ready_at, 3.0, 'controller discovery')

    def make_goal(self, group, target):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.allowed_planning_time = 8.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.start_state.is_diff = True
        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0
        constraints = Constraints()
        for name, position in target.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.005
            joint.tolerance_below = 0.005
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def move_joints(self, group, target):
        goal = self.make_goal(group, target)
        sent = self.move_group.send_goal_async(goal)
        self.until(sent.done, 15.0, f'{group} goal acceptance')
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError(f'{group}: MoveGroup goal rejected')
        result_future = handle.get_result_async()
        self.until(result_future.done, 90.0, f'{group} execution')
        result = result_future.result().result
        if result.error_code.val != SUCCESS:
            measured = {name: self.joints.get(name) for name in target}
            raise RuntimeError(f'{group}: MoveIt error {result.error_code.val}; '
                               f'target={target}, measured={measured}')
        self.until(lambda: all(abs(self.joints.get(name, 1e6) - value) < 0.025
                               for name, value in target.items()), 5.0,
                   f'{group} measured joint target')

    def close_until_attached(self):
        """Stop the MoveIt close action at contact, then request attachment."""
        goal = self.make_goal('gripper', dict.fromkeys(FINGER_JOINTS, 0.2))
        sent = self.move_group.send_goal_async(goal)
        self.until(sent.done, 15.0, 'gripper close goal acceptance')
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('Contact-aware gripper close goal rejected')
        result_future = handle.get_result_async()
        try:
            self.until(lambda: self.contact is True or result_future.done(),
                       30.0, 'finger-ball contact')
            if not self.contact:
                raise RuntimeError('Gripper close finished without contact')
            if not result_future.done():
                cancel = handle.cancel_goal_async()
                self.until(cancel.done, 5.0, 'gripper close cancellation')
                if not cancel.result().goals_canceling:
                    raise RuntimeError('MoveGroup did not accept close cancellation')
                self.until(result_future.done, 10.0, 'gripper close terminal result')
            terminal = result_future.result()
            if terminal.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                raise RuntimeError(f'Close action ended with status {terminal.status}')
            if not may_attach(True, *(self.joints[name] for name in FINGER_JOINTS)):
                raise RuntimeError('Contact-aware close ended at <=5° total opening')
            self.trigger(self.attach_client, 'attach after close')
            self.until(lambda: self.attached is True, 3.0, 'attached state')
            self.get_logger().info(
                f'Gripper close stopped on contact (status {terminal.status}); measured joints='
                f'{[self.joints[name] for name in FINGER_JOINTS]}')
        except Exception:
            if not result_future.done():
                cancel = handle.cancel_goal_async()
                self.until(cancel.done, 5.0, 'failed gripper close cancellation')
                self.until(result_future.done, 10.0, 'failed gripper close terminal result')
            raise

    def hand_pose(self):
        request = GetPositionFK.Request()
        request.header.frame_id = 'world'
        request.fk_link_names = ['rg2_hand']
        request.robot_state.joint_state.name = ARM_JOINTS + FINGER_JOINTS
        request.robot_state.joint_state.position = [
            self.joints[n] for n in ARM_JOINTS + FINGER_JOINTS]
        response = self.call(self.fk_client, request, 'FK')
        if response.error_code.val != SUCCESS or not response.pose_stamped:
            raise RuntimeError(f'FK failed: {response.error_code.val}')
        return response.pose_stamped[0]

    def move_hand(self, z):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose.position.x = 0.55
        pose.pose.position.y = -0.25
        pose.pose.position.z = z
        pose.pose.orientation.y = math.sqrt(0.5)
        pose.pose.orientation.w = math.sqrt(0.5)
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur5'
        request.ik_request.ik_link_name = 'rg2_hand'
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        request.ik_request.robot_state.joint_state.name = ARM_JOINTS + FINGER_JOINTS
        rng = random.Random(20261014)
        for attempt in range(20):
            seed = [self.joints[n] for n in ARM_JOINTS] if attempt == 0 else [
                rng.uniform(-math.pi, math.pi) for _ in ARM_JOINTS]
            request.ik_request.robot_state.joint_state.position = seed + [
                self.joints[n] for n in FINGER_JOINTS]
            response = self.call(self.ik_client, request, 'IK', 5.0)
            if response.error_code.val == SUCCESS:
                solution = dict(zip(response.solution.joint_state.name,
                                    response.solution.joint_state.position))
                for name in ARM_JOINTS:
                    solution[name] = (solution[name] + math.pi) % (2.0 * math.pi) - math.pi
                self.get_logger().info(f'Hand z={z} IK target: '
                                       f'{[solution[name] for name in ARM_JOINTS]}')
                self.move_joints('ur5', {name: solution[name] for name in ARM_JOINTS})
                return
        raise RuntimeError(f'No collision-free IK for hand z={z}')

    def trigger(self, client, label, expected=True):
        response = self.call(client, Trigger.Request(), label, 5.0)
        if response.success != expected:
            raise RuntimeError(f'{label}: expected success={expected}, got {response.success}: '
                               f'{response.message}')
        self.get_logger().info(f'{label}: {response.message}')

    def run(self):
        self.wait_ready()
        if self.attached:
            raise RuntimeError('Ball unexpectedly attached at start')
        self.get_logger().info('Negative control: close empty gripper')
        self.move_joints('gripper', dict.fromkeys(FINGER_JOINTS, 0.0))
        self.until(lambda: self.contact is False, 3.0, 'empty contact')
        self.trigger(self.attach_client, 'empty attach', expected=False)
        self.move_joints('gripper', dict.fromkeys(FINGER_JOINTS, 1.18))

        self.get_logger().info('Move above apple and permit intentional finger contact')
        self.move_joints('ur5', dict(zip(ARM_JOINTS, PREGRASP_Q)))
        set_ball_touch_allowed(self, True)
        completed = False
        try:
            # Keep the first contact approach above the floor and ball crown.
            self.move_hand(0.23)
            hand = self.hand_pose().pose.position
            ball = self.ball_pose.pose.position
            self.get_logger().info(f'At grasp: hand=({hand.x:.3f},{hand.y:.3f},{hand.z:.3f}), '
                                   f'ball=({ball.x:.3f},{ball.y:.3f},{ball.z:.3f})')
            self.get_logger().info('Close with MoveIt and stop when Gazebo confirms contact')
            self.close_until_attached()
            before = self.ball_pose.pose.position.z
            hand_before = self.hand_pose()
            relative_before = self.ball_pose.pose.position.z - hand_before.pose.position.z
            attach_ball(self, self.ball_pose, hand_before)
            self.move_hand(0.35)
            self.until(lambda: self.ball_pose.pose.position.z > before + 0.04,
                       5.0, 'ball follows lift')
            if not self.attached:
                raise RuntimeError('Ball detached during lift')
            relative_after = self.ball_pose.pose.position.z - self.hand_pose().pose.position.z
            if abs(relative_after - relative_before) > 0.03:
                raise RuntimeError('Ball did not preserve hand-relative height during lift')
            self.get_logger().info('Detach and confirm free fall')
            release_z = self.ball_pose.pose.position.z
            self.trigger(self.detach_client, 'detach')
            self.until(lambda: self.attached is False, 3.0, 'detached state')
            detach_ball(self, self.ball_pose)
            self.move_joints('gripper', dict.fromkeys(FINGER_JOINTS, 1.18))
            self.until(lambda: self.ball_pose.pose.position.z < release_z - 0.04,
                       8.0, 'ball free fall')
            self.until(lambda: self.ball_pose.pose.position.z < 0.04,
                       8.0, 'ball reaches floor')
            refresh_ball(self, self.ball_pose)
            self.get_logger().info('STAGE4 GRASP TEST PASSED')
            completed = True
        finally:
            if completed:
                set_ball_touch_allowed(self, False)


def main():
    rclpy.init()
    node = GraspTest()
    try:
        node.run()
        return 0
    except Exception as exc:
        node.get_logger().error(f'STAGE4 GRASP TEST FAILED: {exc}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
