import unittest
import numpy as np
from mirobot_sketch import start_pose as sp
from mirobot_sketch.virtual_robot import VirtualMirobotLink
from tests.test_requested_start import cfg_for_test

class ContactPreparationTest(unittest.TestCase):
    def cfg(self):
        cfg = cfg_for_test()
        cfg['requested_start_pose'].update(pen_length_mm=100, wall_distance_mm=311.632,
                                           contact_joints_deg=[0,10,-12,0,-128,-3])
        return cfg
    def test_prepares_virtual_contact_and_drawing_configuration(self):
        cfg = self.cfg()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = 'Idle'
        robot.joints = list(cfg['requested_start_pose']['joints_deg'])
        result = sp.prepare_requested_drawing(robot, cfg)
        self.assertEqual(robot.joints, cfg['requested_start_pose']['contact_joints_deg'])
        self.assertEqual(result['simulation_start_joints_deg'], robot.joints)
        self.assertEqual(result['pen']['up_clearance_mm'], 20.0)
        from mirobot_sketch import mirobot_sim as ms
        t, _ = ms.fk(np.radians(robot.joints))
        np.testing.assert_allclose(ms.controller_rotation(list(result['orientation_deg'].values())),
                                   t[:3, :3], atol=0.0001)
    def test_cancel_does_not_move(self):
        cfg = self.cfg()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state='Idle'
        robot.joints=list(cfg['requested_start_pose']['joints_deg'])
        with self.assertRaises(sp.StartPoseError):
            sp.prepare_requested_drawing(robot, cfg, should_stop=lambda: True)
        self.assertEqual(robot.sent, [])
