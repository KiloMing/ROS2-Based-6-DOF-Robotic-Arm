import copy
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contact_path import reverse_trajectory, join_trajectories, check_straight_pose, unit, angle_deg, set_seconds, seconds


def trajectory(positions):
    t = RobotTrajectory(); t.joint_trajectory.joint_names = ['joint']
    for i, value in enumerate(positions):
        p = JointTrajectoryPoint(); p.positions = [float(value)]
        p.velocities = [0. if i in (0, len(positions)-1) else .1]
        p.accelerations = [.02]; set_seconds(p.time_from_start, float(i))
        t.joint_trajectory.points.append(p)
    return t


class PathTests(unittest.TestCase):
    def test_reverse_time_velocity_and_acceleration(self):
        original = trajectory([0, 1, 2]); reversed_path = reverse_trajectory(original)
        points = reversed_path.joint_trajectory.points
        self.assertEqual([p.positions[0] for p in points], [2, 1, 0])
        self.assertEqual([seconds(p.time_from_start) for p in points], [0, 1, 2])
        self.assertEqual(list(points[1].velocities), [-.1])
        self.assertEqual(list(points[1].accelerations), [.02])
        self.assertEqual(list(original.joint_trajectory.points[0].positions), [0.])

    def test_reverse_normalizes_only_tiny_endpoint_velocity(self):
        original = trajectory([0, 1, 2])
        original.joint_trajectory.points[0].velocities = [1.0000000000000004e-5]
        result = reverse_trajectory(original)
        self.assertEqual(list(result.joint_trajectory.points[-1].velocities), [0.])
        self.assertEqual(list(result.joint_trajectory.points[1].velocities), [-.1])
        self.assertNotEqual(original.joint_trajectory.points[0].velocities[0], 0.)
        for value in (.01, float('nan'), float('inf')):
            original.joint_trajectory.points[0].velocities = [value]
            with self.assertRaises(ValueError):
                reverse_trajectory(original)

    def test_join_keeps_strict_time_and_no_duplicate_boundary(self):
        a = trajectory([0, 1, 2]); b = trajectory([2, 3, 4])
        result = join_trajectories(a, b).joint_trajectory.points
        self.assertEqual([p.positions[0] for p in result], [0, 1, 2, 3, 4])
        self.assertEqual([seconds(p.time_from_start) for p in result], [0, 1, 2, 3, 4])

    def test_join_rejects_discontinuous_position_and_velocity(self):
        a, b = trajectory([0, 1]), trajectory([2, 3])
        with self.assertRaises(ValueError): join_trajectories(a, b)
        b.joint_trajectory.points[0].positions = [1.]
        b.joint_trajectory.points[0].velocities = [.1]
        with self.assertRaises(ValueError): join_trajectories(a, b)
        b.joint_trajectory.joint_names = ['different']
        with self.assertRaises(ValueError): join_trajectories(a, b)

    def test_line_orientation_and_overshoot_checks(self):
        p = Pose(); p.orientation.w = 1.; p.position.x = .015
        start, end = np.array([0.,0.,0.]), np.array([.03,0.,0.])
        check_straight_pose(p, start, end, Rotation.identity())
        p.position.y = .003
        with self.assertRaises(ValueError): check_straight_pose(p, start, end, Rotation.identity())
        p.position.y = 0.; p.position.x = .04
        with self.assertRaises(ValueError): check_straight_pose(p, start, end, Rotation.identity())
        p.position.x = .015; q = Rotation.from_euler('z', 20, degrees=True).as_quat()
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = q
        with self.assertRaises(ValueError): check_straight_pose(p, start, end, Rotation.identity())

    def test_invalid_axes_and_opposite_direction(self):
        for axis in ([0,0,0], [float('nan'),0,1]):
            with self.assertRaises(ValueError): unit(axis)
        self.assertAlmostEqual(angle_deg([0,0,1], [0,0,-1]),180.)

if __name__ == '__main__': unittest.main()
