#!/usr/bin/env python3
"""CAD MoveIt target planning; physical execution requires --execute."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint, DisplayTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from rosidl_runtime_py.convert import message_to_ordereddict

ROOT=Path(__file__).resolve().parent
NAMES=['base_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_rotation_joint']
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true',help='Execute one freshly planned trajectory through MoveIt')
    ap.add_argument('--xyz',nargs=3,type=float,help='Explicit target in base_link, metres; skips camera')
    ap.add_argument('--calibration',type=Path,default=ROOT/'config/eye_to_hand_draft.json')
    ap.add_argument('--standoff',type=float,default=.05,help='Distance before measured ball surface, along current tip-to-surface line')
    ap.add_argument('--output',type=Path,default=ROOT/'analysis/latest_target_plan.json')
    args=ap.parse_args()
    if not np.isfinite(args.standoff) or args.standoff<0:ap.error('standoff must be finite and nonnegative')
    rclpy.init();node=rclpy.create_node('fixed_camera_target_preview');data={}
    node.create_subscription(JointState,'/joint_states',lambda m:data.update(js=m),10)
    node.create_subscription(PointStamped,'/vision/ball_surface_point',lambda m:data.update(ball=m),10)
    def fresh(m):return 0<=(node.get_clock().now()-Time.from_msg(m.header.stamp)).nanoseconds/1e9<.5
    def call(typ,name,req):
        c=node.create_client(typ,name)
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError(f'{name} unavailable')
        f=c.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=20)
        if not f.done() or f.result() is None:raise RuntimeError(f'{name} timed out')
        return f.result()
    try:
        end=time.monotonic()+15
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if 'js' in data and fresh(data['js']) and all(n in data['js'].name for n in NAMES):
                if args.xyz is not None or ('ball' in data and fresh(data['ball'])):break
        else:raise RuntimeError('No fresh complete joint feedback or ball detection; no plan generated')
        js=data['js'];fk=GetPositionFK.Request();fk.header.frame_id='base_link';fk.fk_link_names=['cad_tip_reference'];fk.robot_state.joint_state=js
        result=call(GetPositionFK,'/compute_fk',fk)
        if result.error_code.val!=1:raise RuntimeError(f'FK failed: {result.error_code.val}')
        pt=result.pose_stamped[0].pose.position;tip=np.array([pt.x,pt.y,pt.z]);surface=None
        if args.xyz is not None:target=np.array(args.xyz)
        else:
            cfg=json.loads(args.calibration.read_text());ball=data['ball']
            if cfg['base_frame']!='base_link' or ball.header.frame_id!=cfg['camera_frame']:raise RuntimeError('Calibration frame mismatch')
            t=np.array(cfg['T_base_camera']);p=np.array([ball.point.x,ball.point.y,ball.point.z,1.])
            if t.shape!=(4,4) or not np.isfinite(t).all() or not np.isfinite(p).all() or p[2]<=0:raise RuntimeError('Invalid camera transform/point')
            surface=(t@p)[:3];delta=surface-tip;dist=np.linalg.norm(delta)
            if dist<=args.standoff:raise RuntimeError('Tip already within requested standoff; choose a target after inspection')
            target=surface-args.standoff*delta/dist
            print('DRAFT calibration: positioning error remains; not validated for grasping',flush=True)
        if not np.isfinite(target).all():raise RuntimeError('Invalid target')
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.allowed_planning_time=8.;r.num_planning_attempts=5;r.max_velocity_scaling_factor=.1;r.max_acceleration_scaling_factor=.1;r.start_state.joint_state=js;r.start_state.is_diff=True
        pc=PositionConstraint();pc.header.frame_id='base_link';pc.link_name='cad_tip_reference';pc.weight=1.
        shape=SolidPrimitive();shape.type=SolidPrimitive.SPHERE;shape.dimensions=[.005]
        pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=map(float,target)
        pc.constraint_region.primitives=[shape];pc.constraint_region.primitive_poses=[pose];c=Constraints();c.position_constraints=[pc];r.goal_constraints=[c]
        print('Target base_link [m]:',target.tolist(),flush=True)
        result=call(GetMotionPlan,'/plan_kinematic_path',req).motion_plan_response
        report={'preview_only':not args.execute,'executed':False,'tip_link':'cad_tip_reference','orientation_constrained':False,'target_base_m':target.tolist(),'surface_base_m':None if surface is None else surface.tolist(),'error_code':result.error_code.val,'trajectory':message_to_ordereddict(result.trajectory),'start_state':message_to_ordereddict(result.trajectory_start)}
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2))
        if result.error_code.val!=1:raise RuntimeError(f'MoveIt planning failed: {result.error_code.val}; report: {args.output}')
        pub=node.create_publisher(DisplayTrajectory,'/display_planned_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        msg=DisplayTrajectory();msg.model_id='am1_step_cad';msg.trajectory_start=result.trajectory_start;msg.trajectory=[result.trajectory]
        until=time.monotonic()+3
        while time.monotonic()<until:pub.publish(msg);rclpy.spin_once(node,timeout_sec=.2)
        print(f'Plan successful: {len(result.trajectory.joint_trajectory.points)} points; {args.output}',flush=True)
        if args.execute:
            if not fresh(data['js']):raise RuntimeError('Joint feedback stale before execution')
            if surface is not None:
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.1)
                    b=data.get('ball')
                    if b is not None and fresh(b):break
                if b is None or not fresh(b) or b.header.frame_id!=cfg['camera_frame']:raise RuntimeError('Ball feedback stale before execution')
                latest=(t@np.array([b.point.x,b.point.y,b.point.z,1.]))[:3]
                if not np.isfinite(latest).all() or np.linalg.norm(latest-surface)>.01:raise RuntimeError('Ball moved more than 1 cm; replan required')
            client=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
            if not client.wait_for_server(timeout_sec=5):raise RuntimeError('MoveIt execution unavailable')
            goal=ExecuteTrajectory.Goal();goal.trajectory=result.trajectory
            f=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,f,timeout_sec=10)
            if not f.done():raise RuntimeError('Execution acceptance timed out; check MoveIt before retrying')
            handle=f.result()
            if not handle.accepted:raise RuntimeError('Execution rejected')
            report['execution_requested']=True;args.output.write_text(json.dumps(report,indent=2))
            print('Executing through MoveIt...',flush=True)
            f=handle.get_result_async()
            duration=result.trajectory.joint_trajectory.points[-1].time_from_start
            deadline=time.monotonic()+duration.sec+duration.nanosec/1e9+30
            try:
                while not f.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
                if not f.done():raise RuntimeError('Execution timed out')
                outcome=f.result();report['execution_code']=outcome.result.error_code.val;report['action_status']=outcome.status;report['executed']=outcome.status==4 and outcome.result.error_code.val==1
                args.output.write_text(json.dumps(report,indent=2))
                if not report['executed']:raise RuntimeError(f"Execution failed: {report['execution_code']}")
                print('Execution succeeded; no gripper command sent',flush=True)
            except BaseException:
                cancel=handle.cancel_goal_async();rclpy.spin_until_future_complete(node,cancel,timeout_sec=5)
                raise
        else:print('Preview only; no motion sent',flush=True)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
