import unittest
from mirobot_sketch import start_pose as sp
from mirobot_sketch.virtual_robot import VirtualMirobotLink
from mirobot_sketch.draw_job import DrawJob
from tests.test_requested_start import cfg_for_test

class ReturnStandbyTest(unittest.TestCase):
    def test_job_returns_to_joint_standby_when_requested(self):
        cfg = cfg_for_test()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = 'Idle'
        robot.joints = [0, 2, -3, 0, -128, -3]
        job = DrawJob(None, cfg, lambda *a, **kw: None)
        job.link = robot
        self.assertEqual(job._return_to_origin(None), (True, ''))
        self.assertEqual(robot.joints, cfg['requested_start_pose']['joints_deg'])

    def test_virtual_robot_returns_to_requested_joints(self):
        cfg = cfg_for_test()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = 'Idle'
        robot.joints = [0, 2, -3, 0, -128, -3]
        result = sp.execute_requested_return(robot, cfg)
        self.assertEqual(result['joints_deg'], cfg['requested_start_pose']['joints_deg'])
        self.assertEqual(len(robot.sent), 1)

    def test_already_in_standby_does_not_move(self):
        cfg = cfg_for_test()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = 'Idle'
        robot.joints = list(cfg['requested_start_pose']['joints_deg'])
        status = sp.execute_requested_return(robot, cfg)
        self.assertEqual(status['joints_deg'], robot.joints)
        self.assertEqual(robot.sent, [])

    def test_cancel_prevents_return_command(self):
        cfg = cfg_for_test()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = 'Idle'
        robot.joints = [0, 1, -3, 0, -128, -3]
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_return(robot, cfg, should_stop=lambda: True)
        self.assertEqual(robot.sent, [])

    def test_unverified_physical_setup_does_not_move(self):
        cfg = cfg_for_test()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.virtual = False
        robot.state = 'Idle'
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_return(robot, cfg)
        self.assertEqual(robot.sent, [])
