"""Validate CAD hardware launch generation without connecting to serial."""
import copy
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'arm_ws/src/arm_cad_moveit_config'
spec=importlib.util.spec_from_file_location('cad_hw',BASE/'launch/cad_hardware_description.py')
hw=importlib.util.module_from_spec(spec);spec.loader.exec_module(hw)
URDF=ROOT/'arm_ws/src/arm_description_cad/urdf/arm_cad.urdf'
class HardwareDescriptionTest(unittest.TestCase):
 def setUp(self):
  self.profile=yaml.safe_load((BASE/'config/hardware_calibration.yaml').read_text())
  # Synthetic numbers for testing only; never saved as the real calibration.
  self.profile['joints']['gripper_drive_joint'].update(raw_zero=1200,radians_per_raw=.002)
 def test_missing_calibration(self):
  self.profile['joints']['gripper_drive_joint']['raw_zero']=None
  with self.assertRaisesRegex(ValueError,'gripper_drive_joint.raw_zero'):
   hw.build_description(URDF,self.profile,'/nonexistent')
 def test_unverified_motion(self):
  self.profile['calibration_verified']=False
  self.profile['mapping_accepted_for_control']=False
  with self.assertRaisesRegex(ValueError,'not verified'):
   hw.build_description(URDF,self.profile,'/nonexistent',True,True)
 def test_serial_interfaces(self):
  xml=hw.build_description(URDF,self.profile,'/nonexistent')
  root=ET.fromstring(xml);control=root.find('ros2_control')
  self.assertEqual(control.find('hardware/plugin').text,'arm_hardware/ArmSystem')
  joints=control.findall('joint');self.assertEqual([j.get('name') for j in joints],hw.MOTORS+hw.FOLLOWERS)
  self.assertEqual(len(control.findall('joint/command_interface')),6)
  self.assertTrue(all(j.find('command_interface') is None for j in joints[6:]))
  shoulder=root.find("joint[@name='shoulder_joint']/limit")
  self.assertAlmostEqual(float(shoulder.get('lower')),-800*.00235619449)
  self.assertEqual(control.find("hardware/param[@name='enable_motion']").text,'false')
 def test_nonfinite(self):
  self.profile['joints']['gripper_drive_joint']['radians_per_raw']=float('nan')
  with self.assertRaisesRegex(ValueError,'Nonfinite'):
   hw.build_description(URDF,self.profile,'/nonexistent')
 def test_reversed_arm_mapping_roundtrip_and_limits(self):
  root=ET.fromstring(hw.build_description(URDF,self.profile,'/nonexistent'))
  for name,sign in zip(hw.MOTORS[:5],[1,1,-1,1,1]):
   config=self.profile['joints'][name]
   zero=config['raw_zero'];scale=config['radians_per_raw']
   self.assertGreater(sign*scale,0)
   control=root.find(f"ros2_control/joint[@name='{name}']")
   self.assertEqual(float(control.find("param[@name='radians_per_raw']").text),scale)
   limit=root.find(f"joint[@name='{name}']/limit")
   lower=float(limit.get('lower'));upper=float(limit.get('upper'))
   for raw in [config['raw_min'],zero,zero+10,config['raw_max']]:
    q=(raw-zero)*scale
    self.assertEqual(round(zero+q/scale),raw)
    self.assertLessEqual(lower,q)
    self.assertGreaterEqual(upper,q)
   self.assertGreater((zero+.02/scale-zero)*sign,0)
if __name__=='__main__':unittest.main()
