"""Model contract tests for the new independent contact planning group."""
import importlib.util
import json
from pathlib import Path
import unittest
import sys
import xml.etree.ElementTree as ET
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools/vision'))
spec = importlib.util.spec_from_file_location('contact_moveit', ROOT / 'tools/vision/contact_moveit.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class ContactModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = xacro.process_file(str(ROOT / 'arm_ws/src/arm_description/urdf/arm.urdf.xacro')).toxml()
        cls.robot = ET.fromstring(cls.xml)

    def test_five_finite_joint_ranges(self):
        lo, hi = module.joint_bounds(self.xml)
        self.assertEqual(len(lo), 5)
        self.assertTrue(all(a < b for a, b in zip(lo, hi)))
        self.assertGreater(min(hi - lo), .9)  # not the old +/-0.45 local range

    def test_missing_or_invalid_joint_limits_rejected(self):
        with self.assertRaises(ValueError):
            module.joint_bounds('<robot/>')
        root = ET.fromstring(self.xml)
        root.find("joint[@name='base_joint']/limit").set('lower', 'nan')
        with self.assertRaises(ValueError):
            module.joint_bounds(ET.tostring(root, encoding='unicode'))

    def test_contact_reference_is_on_roll_axis_and_matches_fingertips(self):
        joint = self.robot.find("joint[@name='contact_tcp_approx_joint']")
        xyz = list(map(float, joint.find('origin').get('xyz').split()))
        self.assertEqual(joint.find('parent').get('link'), 'wrist_rotation_link')
        self.assertEqual(xyz[:2], [0., 0.])
        self.assertAlmostEqual(xyz[2], .110)
        # The reference must agree with the existing model, not an unrelated
        # camera-derived estimate. This does not validate physical dimensions.
        def z(name):
            return float(self.robot.find(f"joint[@name='{name}']/origin").get('xyz').split()[2])
        finger = self.robot.find("link[@name='left_finger_link']/visual/geometry/box")
        finger_length = float(finger.get('size').split()[2])
        self.assertAlmostEqual(xyz[2], z('gripper_mount_joint') + z('left_finger_joint') + finger_length)
        import numpy as np
        from scipy.spatial.transform import Rotation
        for angle in [-2., -.5, 0., .5, 2.]:
            np.testing.assert_allclose(Rotation.from_euler('z', angle).apply(xyz), xyz, atol=1e-12)

    def test_independent_position_only_group(self):
        folder = ROOT / 'arm_ws/src/arm_moveit_config/config'
        srdf = ET.parse(next(folder.glob('*.srdf'))).getroot()
        self.assertEqual(srdf.find("group[@name='contact_arm']/chain").get('tip_link'), module.TCP)
        self.assertEqual(srdf.find("group[@name='arm']/chain").get('tip_link'), 'tool0')
        config = yaml.safe_load((folder / 'kinematics.yaml').read_text())
        self.assertTrue(config['contact_arm']['position_only_ik'])
        self.assertNotIn('position_only_ik', config['arm'])

if __name__ == '__main__':
    unittest.main()
