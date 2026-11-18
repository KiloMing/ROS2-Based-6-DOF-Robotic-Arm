"""Replan to a fixed demonstrated arm pose; does not track a ball or close gripper."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import copy
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory

NAMES=['base_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_rotation_joint']

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--center-camera',action='store_true',help='Small camera-centering correction using fresh 3D ball point; no demonstration replay')
    parser.add_argument('--target',type=Path,default=Path(__file__).parent/'config/demonstrated_target.json')
    args=parser.parse_args()
    saved=json.loads(args.target.read_text())
    target=dict(zip(saved['joint_state']['name'],saved['joint_state']['position']))
    if not saved.get('user_confirmed_ideal_pose') or not all(math.isfinite(target[n]) for n in NAMES):
        raise ValueError('Invalid demonstrated target')
    rclpy.init();node=rclpy.create_node('approach_demonstrated_target');data={};handle=None;finished=False
    buffer=Buffer(); listener=TransformListener(buffer,node)
    ball_sub=node.create_subscription(PointStamped,'/vision/ball_surface_point',lambda msg:data.update(ball=msg),1)
    sub=node.create_subscription(JointState,'/joint_states',lambda m:data.update(js=m),10)
    def wait(f,seconds=10):
        rclpy.spin_until_future_complete(node,f,timeout_sec=seconds)
        if not f.done():raise RuntimeError('Service/action timeout')
        return f.result()
    def state():
        previous=data.get('js');end=time.monotonic()+4
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            m=data.get('js')
            if m is not None and m is not previous:
                age=(node.get_clock().now()-Time.from_msg(m.header.stamp)).nanoseconds/1e9
                q=dict(zip(m.name,m.position))
                if 0<=age<.5 and all(n in q and math.isfinite(q[n]) for n in NAMES):return m,q
        raise RuntimeError('No fresh complete joint feedback; no motion sent')
    try:
        controllers=node.create_client(ListControllers,'/controller_manager/list_controllers')
        if not controllers.wait_for_service(timeout_sec=4):raise RuntimeError('Controller manager unavailable')
        active={c.name:c.state for c in wait(controllers.call_async(ListControllers.Request())).controller}
        if any(active.get(n)!='active' for n in ('arm_trajectory_controller','joint_state_broadcaster')):
            raise RuntimeError(f'Controllers not active: {active}')
        js,start=state()
        ball_reference=None
        if args.center_camera:
            deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                ball=data.get('ball')
                if ball is None:continue
                age=(node.get_clock().now()-Time.from_msg(ball.header.stamp)).nanoseconds/1e9
                if not 0<=age<.5:continue
                try:
                    tf=buffer.lookup_transform('base_link',ball.header.frame_id,Time.from_msg(ball.header.stamp))
                    break
                except Exception:continue
            else:raise RuntimeError('Camera centering requires fresh reliable depth and matching TF; no motion sent')
            xyz=np.array([ball.point.x,ball.point.y,ball.point.z])
            if not np.isfinite(xyz).all() or not .10<xyz[2]<1.0:raise RuntimeError('Invalid centering depth')
            t=tf.transform.translation;r=tf.transform.rotation
            world=Rotation.from_quat([r.x,r.y,r.z,r.w]).apply(xyz)+[t.x,t.y,t.z]
            ball_reference=ball
            camera=json.loads((Path(__file__).resolve().parents[2]/'arm_ws/src/arm_moveit_config/config/camera_handeye.json').read_text())
            cr=Rotation.from_quat(camera['quaternion_xyzw']);ct=np.array(camera['translation_m'])
            fk=node.create_client(GetPositionFK,'/compute_fk')
            if not fk.wait_for_service(timeout_sec=3):raise RuntimeError('FK unavailable')
            def camera_point(q):
                req=GetPositionFK.Request();req.header.frame_id='base_link';req.fk_link_names=[camera['parent_frame']]
                req.robot_state.joint_state=copy.deepcopy(js)
                values=dict(zip(NAMES,q))
                req.robot_state.joint_state.position=[values.get(n,v) for n,v in zip(js.name,js.position)]
                answer=wait(fk.call_async(req),3)
                if answer.error_code.val!=1:raise RuntimeError('Camera FK failed')
                pose=answer.pose_stamped[0].pose;rot=pose.orientation
                R=Rotation.from_quat([rot.x,rot.y,rot.z,rot.w]);pos=pose.position
                origin=np.array([pos.x,pos.y,pos.z])+R.apply(ct)
                return (R*cr).inv().apply(world-origin)
            initial=np.array([start[n] for n in NAMES]);observed=camera_point(initial)
            if np.linalg.norm(observed-xyz)>.005:raise RuntimeError('Camera FK and observed TF disagree')
            before=np.linalg.norm(xyz[:2]/xyz[2])
            if before<.02:print('Ball already near optical center; no motion sent');return
            def residual(q):
                point=camera_point(q)
                return np.r_[point[:2]/max(point[2],.05),5*(point[2]-xyz[2]),.02*(q-initial)]
            solution=least_squares(residual,initial,bounds=(initial-.08,initial+.08),diff_step=1e-3,max_nfev=50)
            predicted=camera_point(solution.x)
            after=np.linalg.norm(predicted[:2]/predicted[2])
            if predicted[2]<=.10 or abs(predicted[2]-xyz[2])>.015 or after>before*.9:
                raise RuntimeError('No useful bounded centering correction; no motion sent')
            target=dict(zip(NAMES,solution.x))
            print(f'Predicted normalized center error {before:.3f} -> {after:.3f}; single bounded correction',flush=True)
        print('Target degrees:',{n:round(math.degrees(target[n]),2) for n in NAMES},flush=True)
        client=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        if not client.wait_for_service(timeout_sec=4):raise RuntimeError('MoveIt unavailable')
        request=GetMotionPlan.Request();m=request.motion_plan_request
        m.group_name='arm';m.start_state.joint_state=js;m.start_state.is_diff=True
        m.allowed_planning_time=5.;m.num_planning_attempts=5
        m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1
        constraints=Constraints()
        for name in NAMES:
            j=JointConstraint();j.joint_name=name;j.position=target[name]
            j.tolerance_above=.0001;j.tolerance_below=.0001;j.weight=1.
            constraints.joint_constraints.append(j)
        m.goal_constraints=[constraints]
        response=wait(client.call_async(request),15).motion_plan_response
        if response.error_code.val!=1:raise RuntimeError(f'Planning failed: {response.error_code.val}')
        tr=response.trajectory.joint_trajectory
        if set(tr.joint_names)!=set(NAMES) or len(tr.points)<2:raise RuntimeError('Invalid arm trajectory')
        previous=-1.
        for p in tr.points:
            t=p.time_from_start.sec+p.time_from_start.nanosec/1e9
            if t<=previous or len(p.positions)!=5 or not all(math.isfinite(x) for x in p.positions):raise RuntimeError('Invalid trajectory points')
            if args.center_camera and any(abs(v-start[n])>.085 for n,v in zip(tr.joint_names,p.positions)):
                raise RuntimeError('Centering trajectory exceeds local joint envelope')
            previous=t
        end=dict(zip(tr.joint_names,tr.points[-1].positions))
        if any(abs(end[n]-target[n])>.002 for n in NAMES):raise RuntimeError('Endpoint does not match demonstration')
        print(f'Plan passed: {len(tr.points)} points, duration {previous:.1f}s',flush=True)
        if not args.execute:print('Plan only; no motion sent');return
        _,current=state()
        if any(abs(current[n]-start[n])>.005 for n in NAMES):raise RuntimeError('Robot moved since planning')
        if args.center_camera:
            ball=data.get('ball')
            if ball is None or ball is ball_reference or not 0<=(node.get_clock().now()-Time.from_msg(ball.header.stamp)).nanoseconds/1e9<.5:
                raise RuntimeError('No fresh pre-execution ball observation')
            new=np.array([ball.point.x,ball.point.y,ball.point.z])
            if np.linalg.norm(new-xyz)>.01:raise RuntimeError('Ball observation changed; replan required')
        action=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
        if not action.wait_for_server(timeout_sec=4):raise RuntimeError('Execution unavailable')
        goal=ExecuteTrajectory.Goal();goal.trajectory=response.trajectory
        handle=wait(action.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('Goal rejected')
        result=wait(handle.get_result_async(),max(30,previous*1.5+10))
        finished=True
        print('Execution result:',result.result.error_code.val,flush=True)
        if result.result.error_code.val!=1:raise RuntimeError('Execution failed; no retry')
        _,actual=state()
        print('Measured endpoint errors (degrees):',{n:round(math.degrees(actual[n]-target[n]),3) for n in NAMES},flush=True)
    finally:
        if handle is not None and handle.accepted and not finished and rclpy.ok():
            try:wait(handle.cancel_goal_async(),3)
            except Exception:print('Cancellation not confirmed; inspect controller')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
