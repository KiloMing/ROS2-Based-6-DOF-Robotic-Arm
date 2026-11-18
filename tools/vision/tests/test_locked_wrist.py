import unittest
from contact_moveit import locked_wrist_constraints, validate_locked_wrist, NAMES
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class LockedWristTests(unittest.TestCase):
    def test_constraint_uses_current_angle(self):
        constraint = locked_wrist_constraints(.37).joint_constraints[0]
        self.assertEqual(constraint.joint_name, 'wrist_rotation_joint')
        self.assertEqual(constraint.position, .37)
        self.assertLessEqual(constraint.tolerance_above, .0001)

    def test_every_point_checked_by_name(self):
        t = RobotTrajectory(); t.joint_trajectory.joint_names = list(reversed(NAMES))
        for angle in [.37, .37, .37]:
            p = JointTrajectoryPoint(); p.positions = [angle, 0., 0., 0., 0.]
            t.joint_trajectory.points.append(p)
        validate_locked_wrist(t, .37)
        for angle in [.4, float('nan')]:
            t.joint_trajectory.points[1].positions[0] = angle
            with self.assertRaises(ValueError): validate_locked_wrist(t, .37)
