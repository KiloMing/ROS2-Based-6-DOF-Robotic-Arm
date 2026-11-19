import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from grasp_ball import GraspSession
from contact_moveit import NAMES
from contact_path import set_seconds

class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.s=GraspSession.__new__(GraspSession)
        self.s.node=object();self.s.handle=None
        self.s.wrist=0.;self.s.lower=np.full(5,-2.);self.s.upper=np.full(5,2.)
    def trajectory(self):
        tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(NAMES)
        for t in [0.,1.,2.]:
            p=JointTrajectoryPoint();p.positions=[0.]*5;p.velocities=[0.]*5
            set_seconds(p.time_from_start,t);tr.joint_trajectory.points.append(p)
        return tr
    def test_bad_internal_points_and_timestamps_rejected(self):
        tr=self.trajectory();self.s.validate(tr,np.zeros(5))
        tr.joint_trajectory.points[1].positions[2]=float('nan')
        with self.assertRaises(ValueError):self.s.validate(tr,np.zeros(5))
        tr=self.trajectory();tr.joint_trajectory.points[1].time_from_start.sec=0
        with self.assertRaises(ValueError):self.s.validate(tr,np.zeros(5))
        tr=self.trajectory();tr.joint_trajectory.points[1].positions[4]=.01
        with self.assertRaises(ValueError):self.s.validate(tr,np.zeros(5))
    def test_aborted_action_is_never_treated_as_success(self):
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:'result')
        client=SimpleNamespace(wait_for_server=lambda **kw:True,send_goal_async=lambda g:'handle')
        self.s.wait=lambda future,timeout=5:handle if future=='handle' else SimpleNamespace(status=6,result='hardware failure')
        with patch('grasp_ball.ActionClient',return_value=client):
            with self.assertRaises(RuntimeError):self.s.action(object,'/test',object(),5)
    def test_timeout_preserves_handle_for_cancellation(self):
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:'result')
        client=SimpleNamespace(wait_for_server=lambda **kw:True,send_goal_async=lambda g:'handle')
        def wait(future,timeout=5):
            if future=='handle':return handle
            raise RuntimeError('timeout')
        self.s.wait=wait
        with patch('grasp_ball.ActionClient',return_value=client):
            with self.assertRaises(RuntimeError):self.s.action(object,'/test',object(),5)
        self.assertIs(self.s.handle,handle)
