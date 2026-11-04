import unittest
from mirobot_sketch import draw_executor as de, start_pose as sp
from tests.test_requested_start import cfg_for_test

class ContactDrawingConfigTest(unittest.TestCase):
    def test_contact_pose_sets_drawing_reference_without_mutating_original(self):
        cfg = cfg_for_test()
        cfg['requested_start_pose']['contact_joints_deg'] = [0, 8, -8, 0, -128, 0]
        status = {'state':'Idle','joints_deg':[0,8,-8,0,-128,0],
                  'tcp':(230.5,1.0,271.2),'abc_deg':(0,-128,0)}
        new = sp.drawing_config_from_contact(cfg, status)
        self.assertEqual(new['paper_center_tcp_mm'], {'x':230.5,'y':1.0,'z':271.2})
        self.assertEqual(new['orientation_deg'], {'a':0,'b':-128,'c':0})
        self.assertEqual(new['simulation_start_joints_deg'], [0,8,-8,0,-128,0])
        self.assertNotEqual(cfg['paper_center_tcp_mm'], new['paper_center_tcp_mm'])
        self.assertEqual(new['pen']['up_clearance_mm'], 20.0)

    def test_wrong_contact_or_missing_status_is_rejected(self):
        cfg = cfg_for_test()
        cfg['requested_start_pose']['contact_joints_deg'] = [0,8,-8,0,-128,0]
        for status in ({'state':'Alarm','joints_deg':[0,8,-8,0,-128,0], 'tcp':(1,2,3), 'abc_deg':(0,-128,0)},
                       {'state':'Idle','joints_deg':[0,0,0,0,0,0], 'tcp':(1,2,3), 'abc_deg':(0,0,0)},
                       {'state':'Idle','joints_deg':[0,8,-8,0,-128,0], 'tcp':None, 'abc_deg':(0,-128,0)}):
            with self.assertRaises(sp.StartPoseError):
                sp.drawing_config_from_contact(cfg, status)
