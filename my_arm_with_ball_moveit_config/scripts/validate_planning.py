#!/usr/bin/env python3
"""Exercise live planning services without executing any trajectory."""
import json
import math
import random

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity


JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
          "rg2_finger_joint1", "rg2_finger_joint2"]


def state(values):
    result = RobotState()
    result.joint_state.name = JOINTS
    result.joint_state.position = values
    return result


class Validation(Node):
    def __init__(self):
        super().__init__("stage2_planning_validation")
        self.service_clients = {}

    def call(self, service_type, name, request):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(service_type, name)
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f"Unavailable service: {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)
        if not future.done():
            raise RuntimeError(f"Service timed out: {name}")
        return future.result()

    def valid(self, robot_state):
        request = GetStateValidity.Request()
        request.robot_state = robot_state
        request.group_name = "ur5"
        return self.call(GetStateValidity, "check_state_validity", request)

    def ik(self, xyz, avoid_collisions=True):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur5"
        request.ik_request.ik_link_name = "rg2_hand"
        request.ik_request.pose_stamped.header.frame_id = "world"
        pose = request.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = xyz
        # Local +X points down, local +Y is world +Y: fingers close horizontally.
        pose.orientation.y = math.sqrt(0.5)
        pose.orientation.w = math.sqrt(0.5)
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.timeout.sec = 1
        rng = random.Random(20261012)
        for attempt in range(20):
            values = [0.0] * 8 if attempt == 0 else [rng.uniform(-math.pi, math.pi) for _ in range(6)] + [0.0, 0.0]
            request.ik_request.robot_state = state(values)
            response = self.call(GetPositionIK, "compute_ik", request)
            if response.error_code.val == 1:
                solution = response.solution
                solution.joint_state.position = [
                    (v + math.pi) % (2.0 * math.pi) - math.pi if n in JOINTS[:6] else v
                    for n, v in zip(solution.joint_state.name, solution.joint_state.position)]
                return solution
        raise RuntimeError(f"IK failed for {xyz}: {response.error_code.val}")

    def plan(self, start, goal):
        request = GetMotionPlan.Request()
        mr = request.motion_plan_request
        mr.group_name = "ur5"
        mr.pipeline_id = "ompl"
        mr.planner_id = "RRTConnectkConfigDefault"
        mr.start_state = start
        mr.allowed_planning_time = 5.0
        mr.num_planning_attempts = 5
        mr.max_velocity_scaling_factor = 0.1
        mr.max_acceleration_scaling_factor = 0.1
        mr.workspace_parameters.header.frame_id = "world"
        mr.workspace_parameters.min_corner.x = -2.0
        mr.workspace_parameters.min_corner.y = -2.0
        mr.workspace_parameters.min_corner.z = -2.0
        mr.workspace_parameters.max_corner.x = 2.0
        mr.workspace_parameters.max_corner.y = 2.0
        mr.workspace_parameters.max_corner.z = 2.0
        constraints = Constraints()
        for name, position in zip(goal.joint_state.name, goal.joint_state.position):
            if name not in JOINTS[:6]:
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        mr.goal_constraints = [constraints]
        response = self.call(GetMotionPlan, "plan_kinematic_path", request).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f"Planning failed: {response.error_code.val}")
        trajectory = response.trajectory.joint_trajectory
        checked = 0
        previous = trajectory.points[0].positions
        for point in trajectory.points:
            count = max(1, math.ceil(max(abs(a-b) for a, b in zip(previous, point.positions)) / 0.01))
            for i in range(1, count + 1):
                positions = [a+(b-a)*i/count for a, b in zip(previous, point.positions)]
                values = dict(zip(trajectory.joint_names, positions))
                robot_state = state([values.get(n, 0.0) for n in JOINTS])
                if not self.valid(robot_state).valid:
                    raise RuntimeError("Trajectory collision at interpolated state")
                checked += 1
            previous = point.positions
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "world"
        fk_request.fk_link_names = ["rg2_hand"]
        final_values = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        fk_request.robot_state = state([final_values.get(n, 0.0) for n in JOINTS])
        fk = self.call(GetPositionFK, "compute_fk", fk_request)
        p = fk.pose_stamped[0].pose.position
        q = fk.pose_stamped[0].pose.orientation
        dot = abs((q.y + q.w) * math.sqrt(0.5))
        angle_error = 2.0 * math.acos(min(1.0, dot))
        if angle_error > 0.005:
            raise RuntimeError(f"Goal orientation error: {angle_error}")
        return {"planning_time": response.planning_time,
                "trajectory_points": len(trajectory.points), "checked_states": checked,
                "final_xyz": [p.x, p.y, p.z], "orientation_error_rad": angle_error}


def main():
    rclpy.init()
    node = Validation()
    try:
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = node.call(GetPlanningScene, "get_planning_scene", scene_request).scene
        print(json.dumps({"scene_objects": [{"id": o.id, "frame": o.header.frame_id,
                          "pose": [o.pose.position.x, o.pose.position.y, o.pose.position.z],
                          "meshes": len(o.meshes), "primitives": len(o.primitives)}
                         for o in scene.world.collision_objects]}), flush=True)
        start = state([0.0] * 8)
        if not node.valid(start).valid:
            raise RuntimeError("Home state invalid")
        print(json.dumps({"home_valid": True}), flush=True)
        for label, xyz in [("apple_approach", [0.55, -0.25, 0.3325]),
                           ("lift", [0.55, -0.25, 0.45]),
                           ("basket_above", [0.75, 0.25, 0.4])]:
            goal = node.ik(xyz)
            print(json.dumps({"ik_target": label, "positions": list(goal.joint_state.position)}), flush=True)
            result = node.plan(start, goal)
            if max(abs(a-b) for a, b in zip(xyz, result["final_xyz"])) > 0.005:
                raise RuntimeError(f"Goal position error: {result}")
            print(json.dumps({"target": label, "frame": "world", "xyz": xyz,
                              "quaternion_xyzw": [0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)],
                              "goal_joint_names": list(goal.joint_state.name),
                              "goal_joint_positions": list(goal.joint_state.position), **result}), flush=True)
            start = goal
        basket_goal = node.ik([0.64, 0.25, 0.14], avoid_collisions=False)
        basket_invalid = node.valid(basket_goal)
        basket_pairs = [[c.contact_body_1, c.contact_body_2] for c in basket_invalid.contacts]
        if basket_invalid.valid or not any("basket" in pair for pair in basket_pairs):
            raise RuntimeError(f"Basket collision negative control failed: {basket_pairs}")
        print(json.dumps({"negative_control": "hand_in_basket_wall", "xyz": [0.64, 0.25, 0.14],
                          "valid": basket_invalid.valid, "contacts": basket_pairs}), flush=True)
        # Put the hand body directly through the sphere. IK ignores collisions here
        # only to construct a deliberately invalid state for the negative control.
        collision_goal = node.ik([0.55, -0.25, 0.0325], avoid_collisions=False)
        invalid = node.valid(collision_goal)
        pairs = [[c.contact_body_1, c.contact_body_2] for c in invalid.contacts]
        if invalid.valid or not any("apple" in pair for pair in pairs):
            raise RuntimeError(f"Ball collision negative control failed: {pairs}")
        print(json.dumps({"negative_control": "hand_in_apple", "valid": invalid.valid,
                          "contacts": pairs}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
