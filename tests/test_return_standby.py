"""그림이 끝나면 1번 대기 자세를 거쳐 호밍 자세(관절 0)로 돌아와, 다음 그림을 같은 순서로 바로 시작할 수 있다."""

import unittest

from mirobot_sketch import start_pose as sp
from mirobot_sketch.draw_job import DrawJob
from mirobot_sketch.virtual_robot import VirtualMirobotLink
from tests.test_requested_start import cfg_for_test

HOME = [0.0] * 6


def idle_robot(cfg, joints):
    robot = VirtualMirobotLink(cfg, speed=10000)
    robot.state = "Idle"
    robot.joints = list(joints)
    return robot


class ReturnHomeTest(unittest.TestCase):
    def test_job_returns_through_standby_to_home(self):
        cfg = cfg_for_test()
        robot = idle_robot(cfg, [0, 2, -3, 0, -128, -3])
        job = DrawJob(None, cfg, lambda *a, **kw: None)
        job.link = robot
        self.assertEqual(job._return_to_origin(None), (True, ""))
        self.assertEqual(robot.joints, HOME)

    def test_return_reverses_the_checked_start_path(self):
        cfg = cfg_for_test()
        robot = idle_robot(cfg, [0, 2, -3, 0, -128, -3])
        sp.execute_requested_return(robot, cfg)
        start = sp.plan_requested_start(cfg)["commands"]
        self.assertEqual(robot.sent[1:], start[-2::-1] + [sp.joint_line(HOME, sp.settings(cfg)["joint_feed"])])
        self.assertIn(sp.joint_line(cfg["requested_start_pose"]["joints_deg"], sp.settings(cfg)["joint_feed"]),
                      robot.sent[0])                                          # 먼저 1번 대기 자세
        # 다음 그림의 시작 검사(호밍 자세에서만 출발)를 그대로 통과
        sp.execute_requested_start(robot, cfg)

    def test_already_in_standby_goes_straight_home(self):
        cfg = cfg_for_test()
        robot = idle_robot(cfg, cfg["requested_start_pose"]["joints_deg"])
        status = sp.execute_requested_return(robot, cfg)
        self.assertEqual(status["joints_deg"], HOME)
        self.assertEqual(len(robot.sent), 3)

    def test_cancel_prevents_return_command(self):
        cfg = cfg_for_test()
        robot = idle_robot(cfg, [0, 1, -3, 0, -128, -3])
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_return(robot, cfg, should_stop=lambda: True)
        self.assertEqual(robot.sent, [])

    def test_unverified_physical_setup_does_not_move(self):
        cfg = cfg_for_test()
        robot = idle_robot(cfg, [0, 1, -3, 0, -128, -3])
        robot.virtual = False
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_return(robot, cfg)
        self.assertEqual(robot.sent, [])


if __name__ == "__main__":
    unittest.main()
