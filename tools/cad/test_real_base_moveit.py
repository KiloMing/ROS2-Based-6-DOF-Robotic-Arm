#!/usr/bin/env python3
"""One +2 degree MoveIt base test. Requires real.launch commissioning_base_test:=true."""
import argparse
import json
import math
import os
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from rcl_interfaces.srv import GetParameters

parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
if os.environ.get('ROS_DOMAIN_ID','0')!='0':raise RuntimeError('This test expects real domain 0')
rclpy.init();node=rclpy.create_node('cad_real_base_test');samples=[]
sub=node.create_subscription(JointState,'/joint_states',lambda msg:samples.append((time.monotonic(),msg)),10)
names=['base_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_rotation_joint']
report={'test':'MoveIt +2 degree base only','executed':False}
def wait(future,timeout=15):
 rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
 if not future.done():raise RuntimeError('ROS request timed out')
 return future.result()
def service(typ,path,req):
 client=node.create_client(typ,path)
 if not client.wait_for_service(timeout_sec=15):raise RuntimeError(path+' unavailable')
 return wait(client.call_async(req))
def feedback():
 samples.clear();deadline=time.monotonic()+5
 while not samples and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
 if not samples:raise RuntimeError('No real feedback')
 return samples[-1][1]
try:
 req=GetParameters.Request();req.names=['robot_description']
 xml=service(GetParameters,'/robot_state_publisher/get_parameters',req).values[0].string_value
 import xml.etree.ElementTree as ET
 control=ET.fromstring(xml).find('ros2_control')
 assert control.find('hardware/plugin').text=='arm_hardware/ArmSystem','Not serial hardware'
 assert control.find("hardware/param[@name='commissioning_base_test']").text=='true','Base commissioning mode required'
 msg=feedback();start=dict(zip(msg.name,msg.position));report['start_rad']=start
 req=GetStateValidity.Request();req.robot_state.joint_state=msg
 valid=service(GetStateValidity,'/check_state_validity',req)
 if not valid.valid:raise RuntimeError('Start in collision: '+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts]))
 plan=GetMotionPlan.Request();m=plan.motion_plan_request
 m.group_name='arm';m.pipeline_id='ompl';m.allowed_planning_time=10.;m.num_planning_attempts=3
 m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1;m.start_state.joint_state=msg
 goal=Constraints();locked=Constraints()
 for name in names:
  goal.joint_constraints.append(JointConstraint(joint_name=name,position=start[name]+(math.radians(2) if name=='base_joint' else 0.),tolerance_above=.0001,tolerance_below=.0001,weight=1.))
  if name!='base_joint':locked.joint_constraints.append(JointConstraint(joint_name=name,position=start[name],tolerance_above=.0005,tolerance_below=.0005,weight=1.))
 m.goal_constraints=[goal];m.path_constraints=locked
 result=service(GetMotionPlan,'/plan_kinematic_path',plan).motion_plan_response
 report['plan_code']=result.error_code.val
 if result.error_code.val!=1:raise RuntimeError(f'Planning failed {result.error_code.val}')
 trajectory=result.trajectory.joint_trajectory
 assert trajectory.points and set(trajectory.joint_names)==set(names)
 for point in trajectory.points:
  for name,q in zip(trajectory.joint_names,point.positions):
   if abs(q-start[name])>(math.radians(2.2) if name=='base_joint' else .001):raise RuntimeError('Unexpected trajectory excursion '+name)
 report['trajectory_points']=len(trajectory.points)
 print('PLAN PASSED',json.dumps(report),flush=True)
 if args.execute:
  fresh=feedback();now=dict(zip(fresh.name,fresh.position))
  if any(abs(now[name]-start[name])>.003 for name in names):raise RuntimeError('State changed after planning')
  client=ActionClient(node,ExecuteTrajectory,'/execute_trajectory');assert client.wait_for_server(timeout_sec=10)
  handle=wait(client.send_goal_async(ExecuteTrajectory.Goal(trajectory=result.trajectory)))
  assert handle.accepted,'Execution rejected'
  try:out=wait(handle.get_result_async(),30)
  except Exception:
   wait(handle.cancel_goal_async(),5);raise
  report['execute_code']=out.result.error_code.val;report['executed']=True
  if out.status!=4 or out.result.error_code.val!=1:raise RuntimeError(f'Execution failed {out.result.error_code.val}')
  deadline=time.monotonic()+1.5
  while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
  end=feedback();final=dict(zip(end.name,end.position));report['end_rad']=final
  report['delta_deg']={name:math.degrees(final[name]-start[name]) for name in names}
  if abs(report['delta_deg']['base_joint']-2)>.5:raise RuntimeError('Controller succeeded but measured base displacement differs from target')
  if any(abs(report['delta_deg'][name])>.3 for name in names[1:]):raise RuntimeError('Unexpected feedback movement on another joint')
  print('REAL FEEDBACK VERIFIED',json.dumps(report),flush=True)
except Exception as exc:
 report['error']=str(exc);raise
finally:
 Path('/tmp/cad_real_base_test_result.json').write_text(json.dumps(report,indent=2)+'\n')
 node.destroy_node();rclpy.shutdown()
