import unittest
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as ET
from gripper_linkage import A,B,C,D,NAMES,solve

def rot(q):return np.array([[np.cos(q),np.sin(q)],[-np.sin(q),np.cos(q)]])
class LinkageTest(unittest.TestCase):
 def test_moveit_mimic_approximation(self):
  root=Path(__file__).resolve().parents[2]
  tree=ET.parse(root/'arm_ws/src/arm_description_cad/urdf/arm_cad.urdf')
  factors=[1.]
  for name in NAMES[1:]:
   mimic=tree.find(f"joint[@name='{name}']/mimic")
   self.assertEqual(mimic.get('joint'),NAMES[0])
   factors.append(float(mimic.get('multiplier')))
  for q in np.linspace(-.153,.58,1001):
   v=q*np.array(factors)
   np.testing.assert_allclose(v[3:],-v[:3],atol=1e-12)
   self.assertLess(np.max(np.abs(v-solve(q)[0])),np.deg2rad(.08))
   lower=A+rot(v[0])@(C-A)
   error=np.linalg.norm(lower+rot(v[0]+v[2])@(D-C)-(B+rot(v[1])@(D-B)))
   self.assertLess(error,.025) # mm; linear mimic approximation, not exact closure
 def test_reference(self):np.testing.assert_allclose(solve(0)[0],[0]*6,atol=1e-12)
 def test_closed_loop_entire_path(self):
  for q in np.linspace(-.153,.58,501):
   v,residual=solve(q)
   lower=A+rot(v[0])@(C-A)
   upper_on_finger=lower+rot(v[0]+v[2])@(D-C)
   upper_on_rocker=B+rot(v[1])@(D-B)
   np.testing.assert_allclose(upper_on_finger,upper_on_rocker,atol=1e-9)
   np.testing.assert_allclose(v[3:],-np.array(v[:3]),atol=1e-12)
 def test_interpolated_dense_trajectory(self):
  samples=[solve(-.153+.733*(3*u*u-2*u*u*u))[0] for u in np.linspace(0,1,161)]
  for a,b in zip(samples,samples[1:]):
   v=(np.array(a)+b)/2
   lower=A+rot(v[0])@(C-A)
   error=np.linalg.norm(lower+rot(v[0]+v[2])@(D-C)-(B+rot(v[1])@(D-B)))
   self.assertLess(error,.001) # mm, at segment midpoints
if __name__=='__main__':unittest.main()
