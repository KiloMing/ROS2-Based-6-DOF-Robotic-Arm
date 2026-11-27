"""Check sampled full-robot collisions, MoveIt execution and coupled feedback."""
import os
import subprocess
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from gripper_linkage import NAMES
assert os.environ.get('ROS_DOMAIN_ID') == '87', 'CAD mock only'
root = Path(__file__).resolve().parents[2]
urdf = ET.parse(root/'arm_ws/src/arm_description_cad/urdf/arm_cad.urdf')
factors = [1.] + [float(urdf.find(f"joint[@name='{name}']/mimic").get('multiplier')) for name in NAMES[1:]]
rclpy.init()
node = rclpy.create_node('check_cad_gripper_coupling')
latest = []
sub = node.create_subscription(JointState, '/joint_states', lambda msg: latest.append(msg), 10)
def fresh_state():
    latest.clear()
    deadline = time.monotonic()+10
    while not latest and time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    assert latest, 'No feedback'
    return dict(zip(latest[-1].name, latest[-1].position))
try:
    state = fresh_state()
    client = node.create_client(GetStateValidity, '/check_state_validity')
    assert client.wait_for_service(timeout_sec=10)
    for q in np.linspace(-.153, .58, 101):
        sample = {**state, **dict(zip(NAMES, q*np.array(factors)))}
        req = GetStateValidity.Request()
        req.robot_state.joint_state.name = list(sample)
        req.robot_state.joint_state.position = list(sample.values())
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and future.result().valid, f'Collision at {q}: {future.result()}'
    print('101 full-robot collision samples passed', flush=True)
    for option, target in [('--close', .58), ('--open', -.14844025287)]:
        subprocess.run([sys.executable, str(root/'tools/cad/gripper_move.py'), option, '--execute'], check=True, timeout=75)
        state = fresh_state()
        assert abs(state[NAMES[0]]-target)<.001, state
        np.testing.assert_allclose([state[name] for name in NAMES], state[NAMES[0]]*np.array(factors), atol=1e-6)
        print(f'{option}: all six joint feedback values follow one driver', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
