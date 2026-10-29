import unittest
from mirobot_sketch import start_pose as sp
from tests.test_requested_start import cfg_for_test

class ContactSampleConfigTest(unittest.TestCase):
    def test_sample_must_match_contact_pose_and_sets_pose_reference(self):
        cfg=cfg_for_test()
        r=cfg['requested_start_pose']
        r['contact_joints_deg']=[0,8,-8,0,-128,0]
        r['contact_status_sample']={'joints_deg':[0,8.002,-8,0,-127.992,0],
             'tcp_mm':[232.840,0,268.888], 'abc_deg':[0,-127.990,0]}
        out=sp.drawing_config_from_recorded_contact(cfg)
        self.assertEqual(out['paper_center_tcp_mm']['x'],232.840)
        self.assertEqual(out['orientation_deg']['b'],-127.990)
        self.assertEqual(out['simulation_start_joints_deg'],r['contact_status_sample']['joints_deg'])
    def test_missing_sample_rejected(self):
        with self.assertRaises(sp.StartPoseError):
            sp.drawing_config_from_recorded_contact(cfg_for_test())
