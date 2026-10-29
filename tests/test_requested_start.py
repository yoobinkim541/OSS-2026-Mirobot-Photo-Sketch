import unittest

import numpy as np

from mirobot_sketch import draw_executor as de, start_pose as sp
from mirobot_sketch.virtual_robot import VirtualMirobotLink


def cfg_for_test():
    cfg = de.load_config("mirobot_sketch/data/drawing_config.json")
    cfg["requested_start_pose"] = {"joints_deg": [0, 0, -3, 0, -128, -3],
                                   "wall_distance_mm": 300, "pen_length_mm": 60}
    return cfg


class RequestedStartTest(unittest.TestCase):
    def test_plan_reaches_requested_angles_using_side_clearance(self):
        plan = sp.plan_requested_start(cfg_for_test())
        self.assertEqual(plan["joints_deg"][-1], [0.0, 0.0, -3.0, 0.0, -128.0, -3.0])
        self.assertEqual(plan["joints_deg"][0][0], -90.0)
        self.assertGreaterEqual(plan["min_tip_clearance_mm"], 15.0)

    def test_missing_geometry_does_not_create_a_motion_plan(self):
        cfg = cfg_for_test()
        del cfg["requested_start_pose"]["pen_length_mm"]
        with self.assertRaises(sp.StartPoseError):
            sp.plan_requested_start(cfg)

    def test_100mm_pen_at_300mm_wall_does_not_send_motion(self):
        cfg = cfg_for_test()
        cfg["requested_start_pose"]["pen_length_mm"] = 100
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = "Idle"
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_start(robot, cfg)
        self.assertEqual(robot.sent, [])

    def test_virtual_executes_plan_and_checks_reported_joints(self):
        cfg = cfg_for_test()
        robot = VirtualMirobotLink(cfg, speed=10000, homing_s=0)
        robot.wait_for_homing(2, progress=lambda *_: None)
        status = sp.execute_requested_start(robot, cfg, should_stop=lambda: False)
        np.testing.assert_allclose(status["joints_deg"], cfg["requested_start_pose"]["joints_deg"], atol=0.01)

    def test_non_homed_robot_receives_no_motion(self):
        cfg = cfg_for_test()
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = "Idle"
        robot.joints[0] = 3
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_start(robot, cfg, should_stop=lambda: False)
        self.assertEqual(robot.sent, [])

    def test_contact_plan_uses_second_joint_pose(self):
        cfg = cfg_for_test()
        cfg["requested_start_pose"].update(pen_length_mm=100, wall_distance_mm=311.632,
                                           contact_joints_deg=[0, 10, -12, 0, -128, -3])
        plan = sp.plan_requested_contact(cfg)
        self.assertEqual(plan["joints_deg"], [0.0, 10.0, -12.0, 0.0, -128.0, -3.0])

    def test_contact_move_requires_first_pose(self):
        cfg = cfg_for_test()
        cfg["requested_start_pose"].update(pen_length_mm=100, wall_distance_mm=311.632,
                                           contact_joints_deg=[0, 10, -12, 0, -128, -3])
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = "Idle"
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_contact(robot, cfg)
        self.assertEqual(robot.sent, [])

    def test_virtual_moves_from_first_to_second_pose(self):
        cfg = cfg_for_test()
        cfg["requested_start_pose"].update(pen_length_mm=100, wall_distance_mm=311.632,
                                           contact_joints_deg=[0, 10, -12, 0, -128, -3])
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = "Idle"
        robot.joints = list(cfg["requested_start_pose"]["joints_deg"])
        status = sp.execute_requested_contact(robot, cfg)
        np.testing.assert_allclose(status["joints_deg"], cfg["requested_start_pose"]["contact_joints_deg"])

    def test_tip_uses_calibrated_holder_offset_in_tool_frame(self):
        from mirobot_sketch import mirobot_sim as ms
        q = np.radians([0, 10, -12, 0, -128, -3])
        offset = np.array([-33.68, 1.76, 17.1])
        transform, _ = ms.fk(q)
        actual = sp._tip(q, 110, offset)
        np.testing.assert_allclose(actual, transform[:3, 3] + transform[:3, :3] @ offset)

    def test_unverified_offset_cannot_drive_a_real_move(self):
        cfg = cfg_for_test()
        cfg["requested_start_pose"].update(tool_offset_mm=[-33.68, 1.76, 17.1],
                                           tool_offset_verified=False)
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = "Idle"
        robot.virtual = False
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_start(robot, cfg)
        self.assertEqual(robot.sent, [])

    def test_refuses_plan_when_first_pose_gap_disagrees_with_observation(self):
        cfg = cfg_for_test()
        cfg["requested_start_pose"].update(wall_distance_mm=270, pen_length_mm=110,
                                           tool_offset_mm=[-33.68, 1.76, 17.1],
                                           first_pose_paper_gap_mm=15.0)
        with self.assertRaises(sp.StartPoseError):
            sp.plan_requested_start(cfg)
