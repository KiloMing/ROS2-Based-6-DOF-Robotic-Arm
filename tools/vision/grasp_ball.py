#!/usr/bin/env python3
"""Single stationary-ball grasp. Default: plan all stages without moving hardware.

Requires measured diameter, maximum opening and fingertip-to-pad-center depth.
A successful action is not evidence of object retention. No force sensing exists.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Pose
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from contact_moveit import NAMES, GROUP, TCP, WRIST, joint_bounds, locked_wrist_constraints, validate_locked_wrist
from scipy.spatial.transform import Rotation
from contact_path import position, rotation, angle_deg, seconds, set_seconds, check_straight_pose
from grasp_geometry import check_dimensions, finger_position, grasp_targets


class GraspSession:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('grasp_ball')
        self.data = {}; self.handle = None; self.clients = {}
        self.report = {'status':'starting', 'stage':'initialize', 'grasp_confirmed':False,
                       'configuration':vars(args), 'trajectories':[]}
        self.subs = [self.node.create_subscription(JointState,'/joint_states',lambda m:self.data.update(js=m),10),
                     self.node.create_subscription(PointStamped,'/vision/ball_surface_point_base',lambda m:self.data.update(ball=m),10)]
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer,self.node)

    def wait(self, future, timeout=5.):
        rclpy.spin_until_future_complete(self.node,future,timeout_sec=timeout)
        if not future.done(): raise RuntimeError('ROS request timed out')
        return future.result()

    def call(self, kind, name, request, timeout=5.):
        if name not in self.clients:
            client = self.node.create_client(kind,name)
            if not client.wait_for_service(timeout_sec=5): raise RuntimeError(f'Service missing: {name}')
            self.clients[name] = client
        return self.wait(self.clients[name].call_async(request),timeout)

    def fresh(self, msg):
        age = (self.node.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        return 0 <= age < .8

    def observe(self, ball=True):
        previous = dict(self.data); end = time.monotonic()+5
        keys = ['js','ball'] if ball else ['js']
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.1)
            if all(k in self.data and self.data[k] is not previous.get(k) and self.fresh(self.data[k]) for k in keys):
                js = copy.deepcopy(self.data['js'])
                values = dict(zip(js.name,js.position))
                if any(n not in values or not math.isfinite(values[n]) for n in NAMES+['left_finger_joint']):
                    raise RuntimeError('Invalid joint feedback')
                return js, copy.deepcopy(self.data.get('ball'))
        raise RuntimeError('Fresh observations unavailable')

    def ball_center(self):
        _,ball=self.observe()
        if ball.header.frame_id!='base_link': raise RuntimeError('Ball frame is not base_link')
        stamp=Time.from_msg(ball.header.stamp);deadline=time.monotonic()+2
        while True:
            try: tf=self.buffer.lookup_transform('base_link',self.args.camera_frame,stamp);break
            except Exception:
                if time.monotonic()>deadline: raise RuntimeError('Matching camera TF unavailable')
                rclpy.spin_once(self.node,timeout_sec=.05)
        t=tf.transform.translation;surface=[ball.point.x,ball.point.y,ball.point.z]
        center,_,_=grasp_targets(surface,[t.x,t.y,t.z],self.axis,self.args.ball_diameter,self.args.clearance,self.args.insertion)
        return center

    def controllers(self):
        ans = self.call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request())
        states = {x.name:x.state for x in ans.controller}
        if any(states.get(n)!='active' for n in ['arm_trajectory_controller','gripper_trajectory_controller','joint_state_broadcaster']):
            raise RuntimeError(f'Controllers inactive: {states}')

    def state(self, q, opening=None):
        js = copy.deepcopy(self.js); values = dict(zip(NAMES,q))
        if opening is not None: values.update(left_finger_joint=opening,right_finger_joint=-opening)
        js.position = [values.get(n,v) for n,v in zip(js.name,js.position)]
        js.velocity=[]; js.effort=[]
        return js

    def fk(self, q, opening=None):
        req=GetPositionFK.Request();req.header.frame_id='base_link';req.fk_link_names=[TCP]
        req.robot_state.joint_state=self.state(q,opening);req.robot_state.is_diff=True
        ans=self.call(GetPositionFK,'/compute_fk',req)
        if ans.error_code.val!=1: raise RuntimeError('FK failed')
        return ans.pose_stamped[0].pose

    def ik(self, target, seed, opening, orientation):
        req=GetPositionIK.Request();r=req.ik_request
        # The original arm group has full-pose IK; convert the axial TCP goal
        # to its tool0 tip, avoiding the position-only contact solver's drift.
        r.group_name='arm';r.ik_link_name='tool0';r.constraints=self.lock;r.avoid_collisions=True
        r.timeout.nanosec=200000000;r.robot_state.joint_state=self.state(seed,opening);r.robot_state.is_diff=True
        r.pose_stamped.header.frame_id='base_link';r.pose_stamped.pose=copy.deepcopy(self.orientation)
        tool_target=np.asarray(target)-rotation(self.orientation).apply(self.tip_from_tool)
        r.pose_stamped.pose.position.x,r.pose_stamped.pose.position.y,r.pose_stamped.pose.position.z=map(float,tool_target)
        ans=self.call(GetPositionIK,'/compute_ik',req)
        if ans.error_code.val!=1: raise ValueError(f'IK failed: {ans.error_code.val}')
        values=dict(zip(ans.solution.joint_state.name,ans.solution.joint_state.position));q=np.array([values[n] for n in NAMES])
        if not np.all(np.isfinite(q)) or np.any(q<self.lower) or np.any(q>self.upper) or abs(q[4]-self.wrist)>1e-4:
            raise ValueError('IK violates joint bounds or fixed 004')
        pose=self.fk(q,opening)
        if np.linalg.norm(position(pose)-target)>.002 or angle_deg(rotation(pose).apply([0,0,1]),self.axis)>5:
            raise ValueError('IK cannot align the gripper within 5 degrees')
        return q,pose

    def validate(self, trajectory, start):
        tr=trajectory.joint_trajectory
        if set(tr.joint_names)!=set(NAMES) or len(tr.points)<2: raise ValueError('Invalid trajectory')
        validate_locked_wrist(trajectory,self.wrist)
        last=-1.
        for p in tr.points:
            if len(p.positions)!=len(NAMES): raise ValueError('Incomplete trajectory point')
            vals=dict(zip(tr.joint_names,p.positions));q=np.array([vals[n] for n in NAMES]);t=seconds(p.time_from_start)
            if not np.all(np.isfinite(q)) or np.any(q<self.lower-1e-6) or np.any(q>self.upper+1e-6) or t<=last:
                raise ValueError('Invalid trajectory limits or times')
            last=t
        first=dict(zip(tr.joint_names,tr.points[0].positions))
        if max(abs(first[n]-start[i]) for i,n in enumerate(NAMES))>.005: raise ValueError('Trajectory start mismatch')
        for p in (tr.points[0],tr.points[-1]):
            if any(not math.isfinite(v) or abs(v)>1e-5+1e-12 for v in p.velocities): raise ValueError('Non-rest trajectory endpoint')
            p.velocities=[0. for _ in p.velocities]
        return trajectory

    def transit(self, start, target):
        orientation=self.fk(start,self.open_q);rng=np.random.default_rng(7);errors=[]
        for attempt in range(20):
            seed=start.copy() if attempt==0 else rng.uniform(self.lower,self.upper);seed[4]=self.wrist
            try:
                q,_=self.ik(target,seed,self.open_q,orientation)
                req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name=GROUP
                m.start_state.joint_state=self.state(start,self.open_q);m.start_state.is_diff=True;m.path_constraints=self.lock
                m.allowed_planning_time=5.;m.num_planning_attempts=3;m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1
                goal=Constraints()
                for n,v in zip(NAMES,q):
                    c=JointConstraint();c.joint_name=n;c.position=float(v);c.tolerance_above=c.tolerance_below=.0001;c.weight=1.;goal.joint_constraints.append(c)
                m.goal_constraints=[goal]
                ans=self.call(GetMotionPlan,'/plan_kinematic_path',req,10).motion_plan_response
                if ans.error_code.val!=1: raise ValueError(f'Transit failed: {ans.error_code.val}')
                return self.validate(ans.trajectory,start)
            except ValueError as exc: errors.append(str(exc))
        raise RuntimeError(f'No aligned pregrasp path: {errors}')

    def straight(self, start, target, opening):
        initial=self.fk(start,opening);origin=position(initial);seed=start.copy();waypoints=[]
        count=max(1,int(math.ceil(np.linalg.norm(target-origin)/.002)))
        for t in np.linspace(0,1,count+1)[1:]:
            q,pose=self.ik(origin+t*(target-origin),seed,opening,initial)
            if np.max(np.abs(q-seed))>.15: raise RuntimeError('Discontinuous straight-path IK')
            seed=q;waypoints.append(pose)
        req=GetCartesianPath.Request();req.header.frame_id='base_link';req.group_name='arm';req.link_name='tool0'
        for waypoint in waypoints:
            tool_point=position(waypoint)-rotation(waypoint).apply(self.tip_from_tool)
            waypoint.position.x,waypoint.position.y,waypoint.position.z=map(float,tool_point)
        req.start_state.joint_state=self.state(start,opening);req.start_state.is_diff=True;req.waypoints=waypoints
        req.path_constraints=self.lock;req.avoid_collisions=True;req.max_step=.002;req.revolute_jump_threshold=.15
        req.max_velocity_scaling_factor=.1;req.max_acceleration_scaling_factor=.1
        ans=self.call(GetCartesianPath,'/compute_cartesian_path',req,15)
        if ans.error_code.val!=1 or ans.fraction<.999999: raise RuntimeError(f'Incomplete straight path: {ans.fraction}')
        trajectory=self.validate(ans.solution,start)
        for pt in trajectory.joint_trajectory.points:
            vals=dict(zip(trajectory.joint_trajectory.joint_names,pt.positions));p=self.fk([vals[n] for n in NAMES],opening)
            check_straight_pose(p,origin,target,rotation(initial),max_rotation_deg=5.)
            if angle_deg(rotation(p).apply([0,0,1]),self.axis)>5: raise RuntimeError('Gripper axis drift')
        if np.linalg.norm(position(p)-target)>.002: raise RuntimeError('Straight endpoint mismatch')
        return trajectory

    def action(self, kind, name, goal, timeout):
        client=ActionClient(self.node,kind,name)
        if not client.wait_for_server(timeout_sec=5): raise RuntimeError(f'Action missing: {name}')
        self.handle=self.wait(client.send_goal_async(goal))
        if not self.handle.accepted: self.handle=None;raise RuntimeError(f'Action rejected: {name}')
        answer=self.wait(self.handle.get_result_async(),timeout);self.handle=None
        if answer.status!=4: raise RuntimeError(f'Action failed: status={answer.status}, result={answer.result}')
        return answer.result

    def gripper(self, opening):
        self.controllers();js,_=self.observe(ball=False);current=dict(zip(js.name,js.position))['left_finger_joint']
        goal=FollowJointTrajectory.Goal();tr=goal.trajectory;tr.joint_names=['left_finger_joint']
        duration=max(2.,abs(opening-current)/.003)
        for value,t in [(current,0.),(opening,duration)]:
            p=JointTrajectoryPoint();p.positions=[float(value)];p.velocities=[0.];set_seconds(p.time_from_start,t);tr.points.append(p)
        result=self.action(FollowJointTrajectory,'/gripper_trajectory_controller/follow_joint_trajectory',goal,duration+10)
        if result.error_code!=0: raise RuntimeError(f'Gripper rejected: {result.error_string}')
        self.js,_=self.observe(ball=False)
        actual=dict(zip(self.js.name,self.js.position))['left_finger_joint']
        self.report.setdefault('gripper_feedback',[]).append({'command':opening,'actual':actual})
        if abs(actual-opening)>.002: raise RuntimeError('Gripper did not reach expected opening; no automatic lift')

    def arm(self, trajectory):
        self.controllers();js,_=self.observe(ball=False);values=dict(zip(js.name,js.position))
        tr=trajectory.joint_trajectory
        if any(abs(values[n]-v)>.005 for n,v in zip(tr.joint_names,tr.points[0].positions)): raise RuntimeError('Arm moved since planning')
        goal=ExecuteTrajectory.Goal();goal.trajectory=trajectory
        result=self.action(ExecuteTrajectory,'/execute_trajectory',goal,max(30.,seconds(tr.points[-1].time_from_start)*1.5+10))
        if result.error_code.val!=1: raise RuntimeError(f'Arm execution failed: {result.error_code.val}')
        self.js,_=self.observe(ball=False);vals=dict(zip(self.js.name,self.js.position))
        expected=dict(zip(tr.joint_names,tr.points[-1].positions))
        q=np.array([vals[n] for n in NAMES]);end=np.array([expected[n] for n in NAMES])
        if abs(q[4]-self.wrist)>.01 or np.linalg.norm(position(self.fk(q))-position(self.fk(end)))>.005:
            raise RuntimeError('Arm feedback endpoint differs by more than 5 mm or wrist drift')
        return q

    @staticmethod
    def endpoint(trajectory):
        tr=trajectory.joint_trajectory
        values=dict(zip(tr.joint_names,tr.points[-1].positions))
        return np.array([values[n] for n in NAMES])

    def save_path(self, label, trajectory):
        tr=trajectory.joint_trajectory
        self.report['trajectories'].append({'stage':label,'names':list(tr.joint_names),'points':[{'positions':list(p.positions),'time_s':seconds(p.time_from_start)} for p in tr.points]})
        print(f'{label}: {len(tr.points)} points, {seconds(tr.points[-1].time_from_start):.1f}s',flush=True)
        return np.array([dict(zip(tr.joint_names,tr.points[-1].positions))[n] for n in NAMES])

    def run(self):
        a=self.args;self.controllers();self.js,ball=self.observe()
        req=GetParameters.Request();req.names=['robot_description']
        xml=self.call(GetParameters,'/move_group/get_parameters',req).values[0].string_value
        root=ET.fromstring(xml);self.lower,self.upper=joint_bounds(xml)
        offset=list(map(float,root.find("joint[@name='contact_tcp_approx_joint']/origin").get('xyz').split()))
        if not np.allclose(offset,[0,0,.110],atol=1e-9): raise RuntimeError('Restart MoveIt with the 110mm axial TCP model')
        tool_offset=np.array(list(map(float,root.find("joint[@name='tool0_joint']/origin").get('xyz').split())))
        self.tip_from_tool=np.array(offset)-tool_offset
        finger_length=float(root.find("link[@name='left_finger_link']/collision/geometry/box").get('size').split()[2])
        if a.insertion+a.ball_diameter/2>finger_length:
            raise RuntimeError('Ball would overlap the palm at requested insertion depth')
        halfmax=float(root.find("joint[@name='left_finger_joint']/limit").get('upper'))
        self.open_q=finger_position(a.max_opening,a.max_opening,halfmax)
        close_q=finger_position(a.ball_diameter-a.compression,a.max_opening,halfmax)
        vals=dict(zip(self.js.name,self.js.position));q=np.array([vals[n] for n in NAMES]);self.wrist=float(q[4]);self.lock=locked_wrist_constraints(self.wrist)
        self.axis=rotation(self.fk(q)).apply([0,0,1])
        # Require a cluster of new observations; use matching camera TF for each.
        centers=[self.ball_center() for _ in range(5)]
        center=np.median(centers,axis=0)
        if max(np.linalg.norm(x-center) for x in centers)>.005: raise RuntimeError('Ball observations are not stable within 5mm')
        self.report.update(ball_center_estimate_m=center.tolist(),locked_wrist_rad=self.wrist,fit_margin_m=a.max_opening-a.ball_diameter,
                           center_method='surface ROI plus radius along camera ray; approximate, not sphere fit')
        scene=ApplyPlanningScene.Request();scene.scene.is_diff=True
        table=CollisionObject();table.id='contact_workspace_table';table.header.frame_id='base_link';table.operation=CollisionObject.ADD
        box=SolidPrimitive();box.type=SolidPrimitive.BOX;box.dimensions=[2.,2.,.05]
        pose=Pose();pose.orientation.w=1.;pose.position.z=a.table_z-.025-.0001;table.primitives=[box];table.primitive_poses=[pose];scene.scene.world.collision_objects=[table]
        if not self.call(ApplyPlanningScene,'/apply_planning_scene',scene).success: raise RuntimeError('Table scene update failed')
        # Search a small set of approach pitches. For this base + three-pitch
        # mechanism, yaw must lie in the ball's radial plane. Roll stays fixed.
        pitches=[float(np.sum(q[1:4])),math.radians(100),math.radians(90),math.radians(110),math.radians(120)]
        self.report['pitch_attempts']=[]
        for pitch in pitches:
            self.orientation=self.fk(q)
            desired=Rotation.from_euler('z',math.atan2(center[1],center[0]))*Rotation.from_euler('y',pitch)*Rotation.from_euler('z',self.wrist)
            quat=desired.as_quat()
            self.orientation.orientation.x,self.orientation.orientation.y,self.orientation.orientation.z,self.orientation.orientation.w=map(float,quat)
            self.axis=desired.apply([0,0,1])
            pre=center-self.axis*(a.ball_diameter/2+a.clearance);tip=center+self.axis*a.insertion
            try:
                self.report['stage']='plan_pregrasp';transit=self.transit(q,pre)
                pre_q=self.endpoint(transit)
                self.report['stage']='plan_insertion';entry=self.straight(pre_q,tip,self.open_q)
                tip_q=self.endpoint(entry)
                self.report['stage']='plan_lift';lift_target=position(self.fk(tip_q,close_q))+[0,0,a.lift]
                lift=self.straight(tip_q,lift_target,close_q)
                self.report['pitch_attempts'].append({'pitch_deg':math.degrees(pitch),'accepted':True})
                self.report.update(approach_axis=self.axis.tolist(),pregrasp_tip_m=pre.tolist(),grasp_tip_m=tip.tolist())
                self.save_path('pregrasp',transit);self.save_path('insertion',entry);self.save_path('lift',lift)
                break
            except (ValueError,RuntimeError) as exc:
                self.report['pitch_attempts'].append({'pitch_deg':math.degrees(pitch),'rejected':str(exc)})
        else: raise RuntimeError('No complete pregrasp/insertion/lift plan; see pitch_attempts')
        if not a.execute:
            self.report['status']='planned';return
        if np.linalg.norm(self.ball_center()-center)>.01:
            raise RuntimeError('Ball moved during planning')
        self.report['stage']='open';self.gripper(self.open_q)
        self.report['stage']='pregrasp';actual=self.arm(transit)
        # Re-estimate the center with matching TF after moving the camera.
        if np.linalg.norm(self.ball_center()-center)>.01:
            raise RuntimeError('Ball center changed after pregrasp; rerun localization')
        self.report['stage']='insertion';entry=self.straight(actual,tip,self.open_q);actual=self.arm(entry)
        actual_pose=self.fk(actual,self.open_q)
        pad_center=position(actual_pose)-rotation(actual_pose).apply([0,0,1])*a.insertion
        if np.linalg.norm(pad_center-center)>.005: raise RuntimeError('Grasp center not aligned within 5mm')
        self.report['stage']='close';self.gripper(close_q)
        self.report['stage']='lift';vals=dict(zip(self.js.name,self.js.position));actual=np.array([vals[n] for n in NAMES])
        lift=self.straight(actual,position(self.fk(actual,close_q))+[0,0,a.lift],close_q);self.arm(lift)
        self.report.update(status='completed_unverified',stage='hold',grasp_confirmed=False)
        print('Motion sequence completed; holding. Object retention requires visual confirmation.',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--ball-diameter',type=float,required=True,help='measured metres')
    p.add_argument('--max-opening',type=float,required=True,help='measured maximum inner opening, metres')
    p.add_argument('--insertion',type=float,required=True,help='fingertip plane to desired pad center, metres')
    p.add_argument('--compression',type=float,default=.002,help='nominal closing below ball diameter, metres')
    p.add_argument('--clearance',type=float,default=.05)
    p.add_argument('--lift',type=float,default=.03)
    p.add_argument('--table-z',type=float,default=0.)
    p.add_argument('--camera-frame',default='camera_color_optical_frame')
    p.add_argument('--report',default='/tmp/grasp_ball_report.json')
    a=p.parse_args()
    try:
        margin=check_dimensions(a.ball_diameter,a.max_opening,a.insertion,a.compression)
        if not .01<=a.clearance<=.15 or not .005<=a.lift<=.05 or not math.isfinite(a.table_z): raise ValueError('Invalid clearance, lift or table height')
    except ValueError as exc:p.error(str(exc))
    print(f'Opening minus diameter: {margin*1000:.1f}mm. Opening calibration and retention are not force-verified.',flush=True)
    rclpy.init();session=GraspSession(a)
    try:session.run()
    except Exception as exc:
        session.report.update(status='failed',error=str(exc));raise
    finally:
        if session.handle is not None:
            try:session.wait(session.handle.cancel_goal_async(),3)
            except Exception:pass
        Path(a.report).write_text(json.dumps(session.report,indent=2,allow_nan=False)+'\n')
        print('Report:',a.report,flush=True);session.node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
