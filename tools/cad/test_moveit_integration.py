"""Run only against the CAD mock demo in ROS_DOMAIN_ID=87."""
import os
if os.environ.get("ROS_DOMAIN_ID") != "87":
    raise RuntimeError("CAD integration tests require ROS_DOMAIN_ID=87")
import rclpy,json
from rclpy.action import ActionClient
from moveit_msgs.srv import GetStateValidity,GetMotionPlan
from moveit_msgs.msg import Constraints,JointConstraint
from moveit_msgs.action import ExecuteTrajectory
rclpy.init();n=rclpy.create_node('cad_integration_check')
names=['base_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_rotation_joint']
def call(typ,path,req):
 c=n.create_client(typ,path)
 if not c.wait_for_service(timeout_sec=20):raise RuntimeError(path+' unavailable')
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=30)
 if not f.done():raise RuntimeError(path+' timeout')
 return f.result()
r=GetStateValidity.Request();r.group_name='arm';r.robot_state.joint_state.name=names;r.robot_state.joint_state.position=[0.]*5
v=call(GetStateValidity,'/check_state_validity',r);print('zero valid',v.valid,'contacts',[(x.contact_body_1,x.contact_body_2) for x in v.contacts],flush=True)
if not v.valid:raise RuntimeError('CAD zero collision')
p=GetMotionPlan.Request();m=p.motion_plan_request;m.group_name='arm';m.pipeline_id='ompl';m.allowed_planning_time=10.;m.num_planning_attempts=3;m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1;m.start_state=r.robot_state
c=Constraints()
for name,value in zip(names,[.2,.15,-.3,.15,0.]):c.joint_constraints.append(JointConstraint(joint_name=name,position=value,tolerance_above=.001,tolerance_below=.001,weight=1.))
m.goal_constraints=[c]
res=call(GetMotionPlan,'/plan_kinematic_path',p).motion_plan_response
print('plan',res.error_code.val,'points',len(res.trajectory.joint_trajectory.points),flush=True)
assert res.error_code.val==1
ac=ActionClient(n,ExecuteTrajectory,'/execute_trajectory');assert ac.wait_for_server(timeout_sec=10)
f=ac.send_goal_async(ExecuteTrajectory.Goal(trajectory=res.trajectory));rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result();assert h.accepted
f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=90)
assert f.done();print('execute',f.result().result.error_code.val,flush=True);assert f.result().result.error_code.val==1
n.destroy_node();rclpy.shutdown()
