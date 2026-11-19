"""Position-only MoveIt IK and joint planning for a fixed approximate contact TCP.

Defaults to plan-only. No physical contact sensor or grasp confirmation is provided.
"""
import argparse
import copy
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Pose
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from contact_path import position, rotation, unit, angle_deg, check_straight_pose, reverse_trajectory, join_trajectories

NAMES = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_rotation_joint']
GROUP = 'contact_arm'
TCP = 'contact_tcp_approx'


WRIST = 'wrist_rotation_joint'
WRIST_TOLERANCE = 1e-4  # radians: numerical planning tolerance, about 0.006 degree


def locked_wrist_constraints(angle):
    result = Constraints()
    joint = JointConstraint(); joint.joint_name = WRIST; joint.position = float(angle)
    joint.tolerance_above = joint.tolerance_below = WRIST_TOLERANCE
    joint.weight = 1.
    result.joint_constraints = [joint]
    return result


def validate_locked_wrist(trajectory, angle):
    tr = trajectory.joint_trajectory
    index = list(tr.joint_names).index(WRIST)
    for point in tr.points:
        value = point.positions[index]
        if not math.isfinite(value) or abs(value - angle) > WRIST_TOLERANCE + 1e-12:
            raise ValueError('004 wrist rotation changed in planned trajectory')


def joint_bounds(xml):
    root = ET.fromstring(xml)
    bounds = {}
    for joint in root.findall('joint'):
        if joint.get('name') in NAMES:
            limit = joint.find('limit')
            bounds[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    if set(bounds) != set(NAMES):
        raise ValueError('Robot model does not contain all arm joint limits')
    if not all(math.isfinite(lo) and math.isfinite(hi) and lo < hi for lo, hi in bounds.values()):
        raise ValueError('Invalid model joint limits')
    return np.array([bounds[n][0] for n in NAMES]), np.array([bounds[n][1] for n in NAMES])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--standoff', type=float, default=0.0, help='metres before observed ball surface; default zero')
    parser.add_argument('--attempts', type=int, default=20)
    parser.add_argument('--approach-distance', type=float, default=.03, help='final straight approach length in metres')
    parser.add_argument('--approach-axis', type=float, nargs=3, default=None, help='physical forward axis expressed in TCP frame; default model gripper forward axis +Z')
    parser.add_argument('--table-z', type=float, default=0.0, help='table top in base_link metres; default follows current URDF table assumption')
    parser.add_argument('--report', default='/tmp/contact_moveit_report.json')
    args = parser.parse_args()
    if not 0 <= args.standoff <= .3 or not 1 <= args.attempts <= 100:
        parser.error('standoff must be 0..0.3 and attempts 1..100')
    if not .005 <= args.approach_distance <= .1 or not math.isfinite(args.table_z):
        parser.error("approach-distance must be 0.005..0.1 and table-z finite")
    if args.approach_axis is not None:
        try: unit(args.approach_axis)
        except ValueError as exc: parser.error(str(exc))
    rclpy.init()
    node = rclpy.create_node('contact_moveit')
    data = {}
    handle = None
    report = {'status': 'starting', 'tcp': TCP, 'group': GROUP, 'physical_contact_confirmed': False}
    subscriptions = [
        node.create_subscription(JointState, '/joint_states', lambda m: data.update(js=m), 10),
        node.create_subscription(PointStamped, '/vision/ball_surface_point_base', lambda m: data.update(ball=m), 10),
    ]

    def age(msg):
        return (node.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9

    def fresh(msg):
        return 0 <= age(msg) < .8

    def wait(future, timeout):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS request timed out')
        return future.result()

    def client(kind, name):
        c = node.create_client(kind, name)
        if not c.wait_for_service(timeout_sec=4):
            raise RuntimeError(f'Service unavailable: {name}')
        return c

    def controllers_active():
        answer = wait(control.call_async(ListControllers.Request()), 4)
        states = {c.name: c.state for c in answer.controller}
        report['controllers'] = states
        for name in ('arm_trajectory_controller', 'joint_state_broadcaster'):
            if states.get(name) != 'active':
                raise RuntimeError(f'{name} is {states.get(name, "missing")}; restore hardware feedback first')

    try:
        control = client(ListControllers, '/controller_manager/list_controllers')
        controllers_active()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(k in data and fresh(data[k]) for k in ('js', 'ball')):
                break
        else:
            ages = {k: round(age(data[k]), 3) if k in data else 'missing' for k in ('js', 'ball')}
            raise RuntimeError(f'No fresh inputs; ages in seconds: {ages}')
        js, ball = copy.deepcopy(data['js']), copy.deepcopy(data['ball'])
        if ball.header.frame_id != 'base_link':
            raise RuntimeError('Ball point must be expressed in base_link')
        q0 = np.array([dict(zip(js.name, js.position))[n] for n in NAMES])
        p = np.array([ball.point.x, ball.point.y, ball.point.z])
        wrist_angle = float(q0[NAMES.index(WRIST)])
        wrist_lock = locked_wrist_constraints(wrist_angle)
        report['locked_wrist_rad'] = wrist_angle
        if not np.all(np.isfinite(np.r_[q0, p])):
            raise RuntimeError('Nonfinite target or joint feedback')
        params = client(GetParameters, '/move_group/get_parameters')
        req = GetParameters.Request(); req.names = ['robot_description']
        model_xml = wait(params.call_async(req), 4).values[0].string_value
        lower, upper = joint_bounds(model_xml)
        tcp_joint = ET.fromstring(model_xml).find("joint[@name='contact_tcp_approx_joint']/origin")
        tcp_offset = np.array(list(map(float, tcp_joint.get('xyz').split())))
        if np.linalg.norm(tcp_offset[:2]) > 1e-9 or abs(tcp_offset[2] - .110) > 1e-9:
            raise RuntimeError('Loaded TCP does not match confirmed axial 110mm geometry; rebuild and restart MoveIt')
        axis = unit(args.approach_axis if args.approach_axis is not None else [0., 0., 1.])
        report['tcp_offset_m'] = tcp_offset.tolist()
        report['tcp_geometry_status'] = 'user-confirmed axial 110mm fingertip-plane center; internal shape/frame calibration provisional'
        report['approach_axis_tcp'] = axis.tolist()
        report['axis_source'] = 'explicit' if args.approach_axis is not None else 'model gripper forward +Z'
        scene_client = client(ApplyPlanningScene, '/apply_planning_scene')
        scene_req = ApplyPlanningScene.Request(); scene_req.scene.is_diff = True
        table = CollisionObject(); table.id = 'contact_workspace_table'; table.header.frame_id = 'base_link'
        primitive = SolidPrimitive(); primitive.type = SolidPrimitive.BOX; primitive.dimensions = [2., 2., .05]
        table_pose = Pose(); table_pose.orientation.w = 1.; table_pose.position.z = args.table_z - .025 - .0001
        table.primitives = [primitive]; table.primitive_poses = [table_pose]; table.operation = CollisionObject.ADD
        scene_req.scene.world.collision_objects = [table]
        if not wait(scene_client.call_async(scene_req), 4).success: raise RuntimeError('Could not add table to planning scene')
        report['table_z_m'] = args.table_z
        if np.any(q0 < lower) or np.any(q0 > upper):
            raise RuntimeError('Current feedback lies outside model joint limits')
        fk = client(GetPositionFK, '/compute_fk')
        ik = client(GetPositionIK, '/compute_ik')
        planner = client(GetMotionPlan, '/plan_kinematic_path')
        cartesian = client(GetCartesianPath, '/compute_cartesian_path')

        def state(q):
            result = copy.deepcopy(js)
            values = dict(zip(NAMES, q))
            result.position = [values.get(n, v) for n, v in zip(js.name, js.position)]
            result.velocity = []; result.effort = []
            return result

        def pose(q):
            req = GetPositionFK.Request(); req.header.frame_id = 'base_link'
            req.fk_link_names = [TCP]; req.robot_state.joint_state = state(q); req.robot_state.is_diff = True
            ans = wait(fk.call_async(req), 4)
            if ans.error_code.val != 1 or not ans.pose_stamped:
                raise RuntimeError(f'TCP FK failed ({ans.error_code.val}); restart MoveIt with the new model')
            return ans.pose_stamped[0].pose

        start_pose = pose(q0)
        start = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        distance = np.linalg.norm(p - start)
        if distance <= args.standoff + .001:
            raise RuntimeError('Already within requested target distance; no motion required')
        target = p - (p - start) / distance * args.standoff
        report.update(ball_base_m=p.tolist(), start_tcp_m=start.tolist(), target_tcp_m=target.tolist(),
                      initial_joints=q0.tolist(), joint_lower=lower.tolist(), joint_upper=upper.tolist(),
                      ball_stamp={'sec': ball.header.stamp.sec, 'nanosec': ball.header.stamp.nanosec})
        print(f'TCP {start.tolist()} -> target {target.tolist()}; distance {distance:.4f} m', flush=True)
        rng = np.random.default_rng(7)
        candidates = []
        codes = []
        rejected_orientation = 0
        approach_direction = unit(target - start)
        for attempt in range(args.attempts):
            seed = q0.copy() if attempt == 0 else rng.uniform(lower, upper)
            seed[NAMES.index(WRIST)] = wrist_angle
            req = GetPositionIK.Request(); request = req.ik_request
            request.group_name = GROUP; request.ik_link_name = TCP
            request.constraints = wrist_lock
            request.robot_state.joint_state = state(seed); request.robot_state.is_diff = True
            request.avoid_collisions = True; request.timeout.nanosec = 200000000
            request.pose_stamped.header.frame_id = 'base_link'
            request.pose_stamped.pose = copy.deepcopy(start_pose)
            request.pose_stamped.pose.position.x, request.pose_stamped.pose.position.y, request.pose_stamped.pose.position.z = map(float, target)
            ans = wait(ik.call_async(req), 4); codes.append(ans.error_code.val)
            if ans.error_code.val != 1:
                continue
            values = dict(zip(ans.solution.joint_state.name, ans.solution.joint_state.position))
            q = np.array([values[n] for n in NAMES])
            if not np.all(np.isfinite(q)) or np.any(q < lower) or np.any(q > upper):
                continue
            if abs(q[NAMES.index(WRIST)] - wrist_angle) > WRIST_TOLERANCE:
                continue
            result = pose(q); v = result.position
            error = float(np.linalg.norm(np.array([v.x, v.y, v.z]) - target))
            if error > .002 or any(np.linalg.norm(q - item[1]) < .01 for item in candidates):
                continue
            alignment = angle_deg(rotation(result).apply(axis), approach_direction)
            if alignment > 45.:
                rejected_orientation += 1
                continue
            candidates.append((float(np.linalg.norm(q - q0)), q, error))
        candidates.sort(key=lambda c: c[0])
        report['ik_codes'] = codes
        report['orientation_rejections'] = rejected_orientation
        report['candidate_count'] = len(candidates)
        print(f'Valid position IK candidates: {len(candidates)} / {args.attempts}', flush=True)
        if not candidates:
            raise RuntimeError('No collision-free position IK candidate; see report, target unchanged')
        trajectory = None
        report['cartesian_attempts'] = []
        for _, q, error in candidates:
            contact_pose = pose(q)
            prepoint = target - approach_direction * args.approach_distance
            retreat_pose = copy.deepcopy(contact_pose)
            retreat_pose.position.x, retreat_pose.position.y, retreat_pose.position.z = map(float, prepoint)
            # Position-only IK may change orientation. Supplying a single fixed
            # orientation waypoint makes Cartesian interpolation demand an
            # unsupported six-dimensional path. First obtain reachable poses at
            # dense position waypoints, then let MoveIt interpolate/check them.
            waypoints = []
            waypoint_q = q.copy()
            waypoint_error = None
            for fraction in np.linspace(0., 1., int(math.ceil(args.approach_distance / .002)) + 1)[1:]:
                point_target = target + fraction * (prepoint - target)
                point_req = GetPositionIK.Request(); ir = point_req.ik_request
                ir.group_name = GROUP; ir.ik_link_name = TCP; ir.avoid_collisions = True
                ir.robot_state.joint_state = state(waypoint_q); ir.robot_state.is_diff = True
                ir.constraints = wrist_lock
                ir.timeout.nanosec = 100000000
                ir.pose_stamped.header.frame_id = 'base_link'; ir.pose_stamped.pose = copy.deepcopy(contact_pose)
                ir.pose_stamped.pose.position.x, ir.pose_stamped.pose.position.y, ir.pose_stamped.pose.position.z = map(float, point_target)
                answer = wait(ik.call_async(point_req), 4)
                if answer.error_code.val != 1:
                    waypoint_error = f'Waypoint IK failed: {answer.error_code.val}'
                    break
                values = dict(zip(answer.solution.joint_state.name, answer.solution.joint_state.position))
                next_q = np.array([values[n] for n in NAMES])
                if not np.all(np.isfinite(next_q)) or np.max(np.abs(next_q - waypoint_q)) > .15:
                    waypoint_error = 'Waypoint IK has a discontinuous joint change'
                    break
                if abs(next_q[NAMES.index(WRIST)] - wrist_angle) > WRIST_TOLERANCE:
                    waypoint_error = 'Waypoint IK moved locked wrist'
                    break
                reachable_pose = pose(next_q)
                if np.linalg.norm(position(reachable_pose) - point_target) > .002:
                    waypoint_error = 'Waypoint IK position error exceeds 2 mm'
                    break
                waypoints.append(reachable_pose); waypoint_q = next_q
            if waypoint_error:
                report['cartesian_attempts'].append({'rejected': waypoint_error})
                continue
            request = GetCartesianPath.Request(); request.header.frame_id = 'base_link'
            request.start_state.joint_state = state(q); request.start_state.is_diff = True
            request.group_name = GROUP; request.link_name = TCP; request.waypoints = waypoints
            request.max_step = .002; request.avoid_collisions = True
            request.path_constraints = wrist_lock
            request.revolute_jump_threshold = .15
            request.max_velocity_scaling_factor = .1; request.max_acceleration_scaling_factor = .1
            cart = wait(cartesian.call_async(request), 10)
            entry = {'fraction': cart.fraction, 'code': cart.error_code.val}
            report['cartesian_attempts'].append(entry)
            if cart.error_code.val != 1 or cart.fraction < .999999:
                continue
            try:
                final_segment = reverse_trajectory(cart.solution)
                validate_locked_wrist(final_segment, wrist_angle)
                max_drift = 0.
                for point in final_segment.joint_trajectory.points:
                    values = dict(zip(final_segment.joint_trajectory.joint_names, point.positions))
                    actual = pose([values[n] for n in NAMES])
                    _, drift = check_straight_pose(actual, prepoint, target, rotation(contact_pose))
                    max_drift = max(max_drift, drift)
                    if angle_deg(rotation(actual).apply(axis), approach_direction) > 45.:
                        raise ValueError('Tool approach axis points away from selected approach direction')
                if np.linalg.norm(position(pose([dict(zip(final_segment.joint_trajectory.joint_names, final_segment.joint_trajectory.points[0].positions))[n] for n in NAMES])) - prepoint) > .002:
                    raise ValueError('Cartesian segment did not reach precontact point')
            except ValueError as exc:
                entry['rejected'] = str(exc)
                continue
            prevalues = dict(zip(final_segment.joint_trajectory.joint_names, final_segment.joint_trajectory.points[0].positions))
            pre_q = np.array([prevalues[n] for n in NAMES])
            req = GetMotionPlan.Request(); motion = req.motion_plan_request
            motion.group_name = GROUP; motion.start_state.joint_state = state(q0); motion.start_state.is_diff = True
            motion.path_constraints = wrist_lock
            motion.allowed_planning_time = 5.; motion.num_planning_attempts = 5
            motion.max_velocity_scaling_factor = .1; motion.max_acceleration_scaling_factor = .1
            goal = Constraints()
            for name, value in zip(NAMES, pre_q):
                joint = JointConstraint(); joint.joint_name = name; joint.position = float(value)
                joint.tolerance_above = joint.tolerance_below = .0001; joint.weight = 1.
                goal.joint_constraints.append(joint)
            motion.goal_constraints = [goal]
            ans = wait(planner.call_async(req), 10).motion_plan_response
            entry['transit_code'] = ans.error_code.val
            if ans.error_code.val == 1:
                try:
                    candidate_trajectory = join_trajectories(ans.trajectory, final_segment)
                    validate_locked_wrist(candidate_trajectory, wrist_angle)
                    trajectory = candidate_trajectory
                except ValueError as exc:
                    entry['rejected'] = str(exc)
                    continue
                report.update(precontact_tcp_m=prepoint.tolist(), approach_direction=approach_direction.tolist(), final_segment_points=len(final_segment.joint_trajectory.points), cartesian_fraction=cart.fraction, max_orientation_drift_deg=max_drift)
                break
        if trajectory is None:
            raise RuntimeError('IK candidates exist, but no complete straight approach plus transit path passed validation; inspect cartesian_attempts in report')
        validate_locked_wrist(trajectory, wrist_angle)
        tr = trajectory.joint_trajectory
        if set(tr.joint_names) != set(NAMES) or len(tr.points) < 2:
            raise RuntimeError('Invalid trajectory joint set or empty trajectory')
        last_time = -1.
        for point in tr.points:
            values = dict(zip(tr.joint_names, point.positions))
            q = np.array([values[n] for n in NAMES])
            t = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if not np.all(np.isfinite(q)) or np.any(q < lower - 1e-6) or np.any(q > upper + 1e-6) or t <= last_time:
                raise RuntimeError('Trajectory has invalid positions, limits or timestamps')
            last_time = t
        end_pose = pose(q); v = end_pose.position
        end_error = float(np.linalg.norm(np.array([v.x, v.y, v.z]) - target))
        if end_error > .002:
            raise RuntimeError(f'Planned endpoint error {end_error:.6f} m')
        report['endpoint_quaternion_xyzw'] = [end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w]
        report['start_quaternion_xyzw'] = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]
        report.update(status='planned', endpoint_error_m=end_error, trajectory_points=len(tr.points), duration_s=last_time,
                      final_joints=q.tolist(), trajectory=[{'positions': list(pt.positions), 'time_s': pt.time_from_start.sec + pt.time_from_start.nanosec/1e9} for pt in tr.points], trajectory_joint_names=list(tr.joint_names))
        print(f'Plan success: {len(tr.points)} points, endpoint error {end_error*1000:.3f} mm', flush=True)
        if not args.execute:
            print('Plan only; no motion sent. Approximate TCP/axis; approach direction and final straight segment validated against the model.')
            return
        controllers_active()
        previous_ball = data.get('ball'); previous_js = data.get('js')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(k in data and fresh(data[k]) for k in ('js', 'ball')) and data['ball'] is not previous_ball and data['js'] is not previous_js:
                break
        else:
            raise RuntimeError('Fresh pre-execution observations unavailable')
        now = data['ball'].point
        if data['ball'].header.frame_id != 'base_link' or not np.all(np.isfinite([now.x, now.y, now.z])):
            raise RuntimeError('Invalid pre-execution target frame or coordinates')
        if np.linalg.norm(np.array([now.x, now.y, now.z]) - p) > .01:
            raise RuntimeError('Ball moved more than 10 mm during planning; replan')
        current = dict(zip(data['js'].name, data['js'].position))
        if any(n not in current or not math.isfinite(current[n]) or abs(current[n] - q0[i]) > .005 for i, n in enumerate(NAMES)):
            raise RuntimeError('Robot moved during planning; replan')
        action = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        if not action.wait_for_server(timeout_sec=4):
            raise RuntimeError('Execution action unavailable')
        goal = ExecuteTrajectory.Goal(); goal.trajectory = trajectory
        handle = wait(action.send_goal_async(goal), 5)
        if not handle.accepted:
            handle = None
            raise RuntimeError('Execution rejected')
        answer = wait(handle.get_result_async(), max(30., last_time * 1.5 + 10))
        handle = None
        report['execution_code'] = answer.result.error_code.val
        if answer.result.error_code.val != 1:
            raise RuntimeError(f'Execution failed: {answer.result.error_code.val}')
        report['status'] = 'executed'
        print('Execution succeeded. Physical contact is NOT confirmed by software.')
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        if handle is not None:
            try:
                wait(handle.cancel_goal_async(), 3)
            except Exception:
                pass
        Path(args.report).write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print('Report:', args.report, flush=True)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
