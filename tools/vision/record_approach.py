"""Read-only manual approach recorder. No action clients or motion commands."""
import argparse
import json
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState, CompressedImage
from geometry_msgs.msg import PointStamped
from capture_helpers import marker_label

TOPICS=['/joint_states','/tf','/tf_static','/vision/ball_surface_point',
        '/vision/ball_surface_point_base','/vision/ball_status','/vision/preview/compressed',
        '/arm_trajectory_controller/controller_state',
        '/gripper_trajectory_controller/controller_state','/display_planned_path','/rosout']

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path)
    args=ap.parse_args()
    root=Path(__file__).resolve().parents[2]
    output=args.output or root/'tools/vision/approach_captures'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    config=output/'config'; config.mkdir()
    for name in ('camera_handeye.json','fingertip_approx.json','real_joint_limits.yaml'):
        shutil.copy2(root/'arm_ws/src/arm_moveit_config/config'/name,config/name)
    (output/'session.json').write_text(json.dumps({
        'topics':TOPICS,'created':datetime.now().astimezone().isoformat(),
        'camera_timestamp':'host receipt before inference, not hardware synchronization',
        'instructions':'Keep ball and base fixed. Move through MoveIt. Pause before markers.',
        'physical_endpoint_measurement':'Record separately: fingertip/surface or grasp-center/ball-center.'},indent=2))
    rclpy.init(args=[]); node=rclpy.create_node('manual_approach_recorder')
    latest={}; subscriptions=[]
    def receive(name,msg): latest[name]=(msg,node.get_clock().now().nanoseconds)
    for name,typ in (('/joint_states',JointState),('/vision/ball_surface_point',PointStamped),
                     ('/vision/ball_surface_point_base',PointStamped),('/vision/preview/compressed',CompressedImage)):
        subscriptions.append(node.create_subscription(typ,name,lambda msg,n=name:receive(n,msg),qos_profile_sensor_data))
    log=(output/'recorder.log').open('w')
    marks=(output/'markers.jsonl').open('w')
    bag=None
    try:
        bag=subprocess.Popen(['ros2','bag','record','-o',str(output/'bag'),*TOPICS],
                             stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print(f'Recording: {output}\nWait for data, then type key + ENTER:\n'
              's=start | m=waypoint | e=end | status=data ages | q=save and quit',flush=True)
        count=0
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if bag.poll() is not None:
                raise RuntimeError(f'Bag recorder exited {bag.returncode}; see {output}/recorder.log')
            if not select.select([sys.stdin],[],[],0)[0]: continue
            line=sys.stdin.readline()
            if not line:break
            command=line.strip().lower()
            if command=='q':break
            now=node.get_clock().now().nanoseconds
            ages={name:round((now-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9,3)
                  for name,(msg,received) in latest.items()}
            print('Message ages (seconds):',ages,flush=True)
            label=marker_label(command)
            if label is None:continue
            count+=1
            record={'index':count,'label':label,'ros_time_ns':now,'message_age_s':ages,
                    'missing_topics':[name for name in ('/joint_states','/vision/ball_surface_point',
                    '/vision/ball_surface_point_base','/vision/preview/compressed') if name not in latest],
                    'snapshot':{name:message_to_ordereddict(msg) for name,(msg,received) in latest.items()
                                if not isinstance(msg,CompressedImage)}}
            if '/vision/preview/compressed' in latest:
                filename=f'{count:03d}_{label}.jpg'
                (output/filename).write_bytes(bytes(latest['/vision/preview/compressed'][0].data))
                record['image']=filename
            marks.write(json.dumps(record)+'\n');marks.flush()
            print(f'Saved {count}: {label}',flush=True)
            if record['missing_topics'] or any(age<0 or age>1 for age in ages.values()):
                print('WARNING: missing/stale data; this marker is not a complete synchronized sample.',flush=True)
    except KeyboardInterrupt:pass
    finally:
        if bag is not None and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bag.terminate();bag.wait(timeout=5)
                print('WARNING: recorder shutdown timed out; inspect bag before using it.')
        marks.close();log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print(f'Saved session: {output}',flush=True)

if __name__=='__main__':main()
