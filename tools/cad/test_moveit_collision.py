"""Run only against the CAD mock demo in ROS_DOMAIN_ID=87."""
import os
if os.environ.get("ROS_DOMAIN_ID") != "87":
    raise RuntimeError("CAD integration tests require ROS_DOMAIN_ID=87")
import rclpy
from moveit_msgs.srv import ApplyPlanningScene,GetStateValidity
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
rclpy.init();n=rclpy.create_node('collision_check')
def call(t,path,r):
 c=n.create_client(t,path);assert c.wait_for_service(timeout_sec=10)
 f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=15);assert f.done();return f.result()
obj=CollisionObject();obj.id='verification_obstacle';obj.header.frame_id='base_link';obj.operation=obj.ADD
box=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.12,.12,.12]);pose=Pose();pose.orientation.w=1.;pose.position.z=.06
obj.primitives=[box];obj.primitive_poses=[pose]
r=ApplyPlanningScene.Request();r.scene.is_diff=True;r.scene.world.collision_objects=[obj]
assert call(ApplyPlanningScene,'/apply_planning_scene',r).success
try:
 q=GetStateValidity.Request();q.group_name='arm';q.robot_state.joint_state.name=['base_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_rotation_joint'];q.robot_state.joint_state.position=[0.]*5
 v=call(GetStateValidity,'/check_state_validity',q);print('Obstacle validity:',v.valid,'contacts:',[(c.contact_body_1,c.contact_body_2) for c in v.contacts]);assert not v.valid
finally:
 obj.operation=obj.REMOVE;r.scene.world.collision_objects=[obj];assert call(ApplyPlanningScene,'/apply_planning_scene',r).success
n.destroy_node();rclpy.shutdown()
