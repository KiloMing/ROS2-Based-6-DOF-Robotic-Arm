"""One bounded 10 mm approach; requires fresh vision, TF and feedback."""
import argparse,time,json
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped,Pose
from tf2_ros import Buffer,TransformListener
from moveit_msgs.srv import GetPositionFK, GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from pathlib import Path
import copy
from moveit_msgs.action import ExecuteTrajectory

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    rclpy.init();n=rclpy.create_node('approach_ball_once');b=Buffer();listener=TransformListener(b,n)
    data={}
    execution_handle=None
    subs=[n.create_subscription(JointState,'/joint_states',lambda m:data.update(js=m),1),n.create_subscription(PointStamped,'/vision/ball_surface_point_base',lambda m:data.update(ball=m),1)]
    def fresh(m):return 0 <= (n.get_clock().now()-Time.from_msg(m.header.stamp)).nanoseconds/1e9 < .8
    def wait(f,timeout=10):
        rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
        if not f.done():raise RuntimeError('Operation timeout')
        return f.result()
    try:
        end=time.monotonic()+10
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if all(k in data and fresh(data[k]) for k in ('js','ball')):
                try:
                    tip=b.lookup_transform('base_link','fingertip_approx',Time());tool=b.lookup_transform('base_link','tool0',Time())
                    if fresh(tip) and fresh(tool):break
                except Exception:pass
        else:raise RuntimeError('No fresh ball/feedback/TF; no motion sent')
        js=data['js'];ball=data['ball'];p=np.array([ball.point.x,ball.point.y,ball.point.z])
        t=tip.transform.translation;start=np.array([t.x,t.y,t.z]);delta=p-start;dist=float(np.linalg.norm(delta))
        if not np.isfinite(dist) or not .15<dist<.6:raise RuntimeError(f'Distance outside first-test bounds: {dist}')
        if start[2]<.12:raise RuntimeError('Approximate fingertip too low for first test')
        desired=start+delta/dist*.01
        origin=dict(zip(js.name,js.position))
        names=['base_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_rotation_joint']
        initial=np.array([origin[name] for name in names])
        config=json.loads((Path(__file__).resolve().parents[2]/'arm_ws/src/arm_moveit_config/config/fingertip_approx.json').read_text())
        offset=np.array(config['translation_m'])
        fk_client=n.create_client(GetPositionFK,'/compute_fk')
        plan_client=n.create_client(GetMotionPlan,'/plan_kinematic_path')
        if not fk_client.wait_for_service(timeout_sec=3) or not plan_client.wait_for_service(timeout_sec=3):
            raise RuntimeError('MoveIt FK/planning unavailable')
        def fk(q):
            request=GetPositionFK.Request();request.header.frame_id='base_link'
            request.fk_link_names=['wrist_rotation_link'];request.robot_state.joint_state=copy.deepcopy(js)
            positions=dict(zip(names,q))
            request.robot_state.joint_state.position=[positions.get(name,value) for name,value in zip(js.name,js.position)]
            answer=wait(fk_client.call_async(request),3)
            if answer.error_code.val!=1:raise RuntimeError('FK failed')
            pose=answer.pose_stamped[0].pose;o=pose.orientation
            rotation=Rotation.from_quat([o.x,o.y,o.z,o.w])
            pos=pose.position
            return np.array([pos.x,pos.y,pos.z])+rotation.apply(offset),rotation
        computed_start,initial_rotation=fk(initial)
        if np.linalg.norm(computed_start-start)>.003:
            raise RuntimeError('MoveIt FK differs from live fingertip TF by >3mm')
        # Solve fingertip position with small joint/rotation regularization instead of
        # imposing an unreachable fixed six-dimensional tool pose on a 5-DOF arm.
        def residual(q):
            tip,rotation=fk(q)
            return np.r_[tip-desired,.001*(q-initial),.001*(initial_rotation.inv()*rotation).as_rotvec()]
        solution=least_squares(residual,initial,bounds=(initial-.08,initial+.08),
                               diff_step=1e-3,max_nfev=35,gtol=1e-9,ftol=1e-8,xtol=1e-8)
        candidate,rotation=fk(solution.x)
        error=float(np.linalg.norm(candidate-desired))
        angle=float((initial_rotation.inv()*rotation).magnitude())
        print(f'Position solve error={error*1000:.2f}mm, orientation change={np.rad2deg(angle):.2f}deg',flush=True)
        if error>.001 or angle>np.deg2rad(5):raise RuntimeError('No acceptable nearby fingertip solution')
        req=GetMotionPlan.Request();motion=req.motion_plan_request
        motion.group_name='arm';motion.start_state.joint_state=js;motion.start_state.is_diff=True
        motion.allowed_planning_time=3.;motion.num_planning_attempts=3
        motion.max_velocity_scaling_factor=.1;motion.max_acceleration_scaling_factor=.1
        constraint=Constraints()
        for name,value in zip(names,solution.x):
            joint=JointConstraint();joint.joint_name=name;joint.position=float(value)
            joint.tolerance_above=.0001;joint.tolerance_below=.0001;joint.weight=1.
            constraint.joint_constraints.append(joint)
        motion.goal_constraints=[constraint]
        result=wait(plan_client.call_async(req),8).motion_plan_response
        print(f'MoveIt plan error={result.error_code.val}',flush=True)
        if result.error_code.val!=1:raise RuntimeError('Collision-checked joint planning failed')
        trajectory=result.trajectory;tr=trajectory.joint_trajectory
        if len(tr.points)<2:raise RuntimeError('Empty path')
        for pt in tr.points:
            qmap=dict(zip(tr.joint_names,pt.positions))
            if any(abs(qmap[name]-origin[name])>.10 for name in names):raise RuntimeError('Joint excursion exceeds 0.1 rad')
            point,orientation=fk([qmap[name] for name in names])
            if np.linalg.norm(point-start)>.015 or point[2]<.10 or np.linalg.norm(point-p)>dist+.002:
                raise RuntimeError('Trajectory leaves local fingertip approach envelope')
            if (initial_rotation.inv()*orientation).magnitude()>np.deg2rad(5):
                raise RuntimeError('Trajectory orientation change exceeds 5 degrees')
        endpoint,_=fk([dict(zip(tr.joint_names,tr.points[-1].positions))[name] for name in names])
        if np.linalg.norm(endpoint-desired)>.002:raise RuntimeError('Planned fingertip endpoint differs from requested step')
        print(f'Checked {len(tr.points)} trajectory points; fingertip distance {dist:.3f} -> {np.linalg.norm(endpoint-p):.3f}m',flush=True)
        if not args.execute:print('Plan only; no motion sent');return
        # Require newly received observations before dispatch, not merely the original sample.
        end=time.monotonic()+3
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if data['ball'] is not ball and fresh(data['ball']) and fresh(data['js']):break
        else:raise RuntimeError('No fresh pre-execution observations')
        nowball=data['ball'].point
        if np.linalg.norm(np.array([nowball.x,nowball.y,nowball.z])-p)>.01:raise RuntimeError('Target moved >10mm')
        nowq=dict(zip(data['js'].name,data['js'].position))
        if any(abs(nowq[name]-origin[name])>.005 for name in tr.joint_names):raise RuntimeError('Robot moved since planning')
        action=ActionClient(n,ExecuteTrajectory,'/execute_trajectory')
        if not action.wait_for_server(timeout_sec=3):raise RuntimeError('Execution server unavailable')
        goal=ExecuteTrajectory.Goal();goal.trajectory=trajectory
        handle=wait(action.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('Execution rejected')
        execution_handle=handle
        future=handle.get_result_async();rclpy.spin_until_future_complete(n,future,timeout_sec=25)
        if not future.done():
            wait(handle.cancel_goal_async(),3);raise RuntimeError('Execution timeout; cancellation requested')
        code=future.result().result.error_code.val
        print('Execution result:',code,flush=True)
        if code!=1:raise RuntimeError(f'Execution failed: {code}; no further approach allowed')
    except KeyboardInterrupt:
        if execution_handle is not None and rclpy.ok():
            wait(execution_handle.cancel_goal_async(),3)
        raise
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
