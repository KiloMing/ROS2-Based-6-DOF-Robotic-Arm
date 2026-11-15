"""Run with the same virtual environment and ROS setup as run.sh."""
import unittest
from types import SimpleNamespace
import numpy as np
from ball_point import project_surface, surface_depth

class GeometryTests(unittest.TestCase):
    def test_optical_axes_and_metres(self):
        k=SimpleNamespace(fx=500.,fy=500.,cx=320.,cy=240.)
        d=SimpleNamespace(**dict.fromkeys(('k1','k2','k3','k4','k5','k6','p1','p2'),0.))
        np.testing.assert_allclose(project_surface(370,190,2.,k,d),(.2,-.2,2.))
        np.testing.assert_allclose(project_surface(320,240,1.,k,d),(0,0,1.))

    def test_depth_rejects_missing_and_mixed_surfaces(self):
        box=(0,0,20,20)
        self.assertIsNone(surface_depth(np.zeros((20,20)),box))
        depth=np.full((20,20),1000.)
        self.assertEqual(surface_depth(depth,box),1.)
        depth[:,10:]=2000.
        self.assertIsNone(surface_depth(depth,box))

if __name__=='__main__': unittest.main()
