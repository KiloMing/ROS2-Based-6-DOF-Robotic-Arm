#!/usr/bin/env python3
"""Same MoveGroup planning/execution pipeline as RViz; CAD domain only."""
import argparse
import os
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--open', action='store_true')
    group.add_argument('--close', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID') != '87':
        raise RuntimeError('Only CAD mock domain 87 supported')
    rclpy.init()
    node = rclpy.create_node('cad_gripper_move')
    def wait(future, timeout):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('MoveIt request timed out')
        return future.result()
    try:
        client = ActionClient(node, MoveGroup, '/move_action')
        if not client.wait_for_server(timeout_sec=20):
            raise RuntimeError('MoveGroup is unavailable')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'gripper'
        req.pipeline_id = 'ompl'
        req.allowed_planning_time = 10.
        req.num_planning_attempts = 3
        req.max_velocity_scaling_factor = .3
        req.max_acceleration_scaling_factor = .3
        req.start_state.is_diff = True
        constraint = Constraints()
        constraint.joint_constraints = [JointConstraint(
            joint_name='gripper_drive_joint', position=.58 if args.close else 0.,
            tolerance_above=.0001, tolerance_below=.0001, weight=1.)]
        req.goal_constraints = [constraint]
        goal.planning_options.plan_only = not args.execute
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = wait(client.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError('MoveGroup rejected goal')
        try:
            result = wait(handle.get_result_async(), 60)
        except Exception:
            wait(handle.cancel_goal_async(), 5)
            raise
        if result.status != 4 or result.result.error_code.val != 1:
            raise RuntimeError(f'MoveIt failed: {result.result.error_code.val}')
        trajectory = result.result.planned_trajectory.joint_trajectory
        print(f'MoveIt success: joints={trajectory.joint_names}, points={len(trajectory.points)}, execute={args.execute}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
