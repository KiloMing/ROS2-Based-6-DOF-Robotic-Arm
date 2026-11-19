import unittest
import numpy as np
from grasp_geometry import grasp_targets, finger_position, check_dimensions

class GraspGeometryTests(unittest.TestCase):
    def test_surface_center_and_tip_are_different(self):
        center, pre, final = grasp_targets([0,0,.3], [0,0,0], [0,0,1], .06, .05, .02)
        np.testing.assert_allclose(center, [0,0,.33])
        np.testing.assert_allclose(pre, [0,0,.25])
        np.testing.assert_allclose(final, [0,0,.35])
        np.testing.assert_allclose(final - np.array([0,0,1])*.02, center)
    def test_physical_opening_maps_to_controller_half_opening(self):
        self.assertAlmostEqual(finger_position(.07,.07,.03), .03)
        self.assertAlmostEqual(finger_position(.035,.07,.03), .015)
        with self.assertRaises(ValueError): finger_position(.071,.07,.03)
    def test_fit_and_invalid_dimensions(self):
        self.assertAlmostEqual(check_dimensions(.06,.06,.02,.002),0.)
        for values in [(.07,.06,.02,.002), (.06,.06,.08,.002), (.06,.06,.02,.07), (float('nan'),.06,.02,.002)]:
            with self.assertRaises(ValueError): check_dimensions(*values)
    def test_invalid_camera_ray_rejected(self):
        with self.assertRaises(ValueError): grasp_targets([0,0,0],[0,0,0],[0,0,1],.06,.05,.02)
