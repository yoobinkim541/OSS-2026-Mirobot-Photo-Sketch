"""시작 자세 T2: 호밍 자세(펜 아래)에서 펜 끝을 위로 올린 대기 자세로 바꾸는 순서 테스트.

    python -m unittest tests.test_start_pose -v
"""

import copy
import re
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import mirobot_sim as ms  # noqa: E402
from mirobot_sketch import start_pose as sp  # noqa: E402
from mirobot_sketch.virtual_robot import VirtualMirobotLink  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()
JOINT_LINE = re.compile(r"^M21 G90 G01 X([-\d.]+) Y([-\d.]+) Z([-\d.]+) A([-\d.]+) B([-\d.]+) C([-\d.]+) F\d+$")


def joints_of(line):
    return np.array([float(v) for v in JOINT_LINE.match(line).groups()])


class StatusParseTest(unittest.TestCase):
    def test_home_status_gives_zero_joints_and_home_pose(self):
        raw = ("<Idle,Angle(ABCDXYZ):0,0,0,0,0,0,0,"
               "Cartesian coordinate(XYZ RxRyRz):198.668,0.000,230.477,0.000,0.000,0.000,"
               "Pump PWM:0,Valve PWM:0,Motion_MODE:0>")
        s = de.parse_status(raw)
        self.assertEqual(s["state"], "Idle")
        self.assertEqual(s["joints_deg"], [0.0] * 6)
        self.assertEqual(s["tcp"], (198.668, 0.0, 230.477))
        self.assertEqual(s["abc_deg"], (0.0, 0.0, 0.0))

    def test_angle_field_order_is_abcd_then_xyz(self):
        # 컨트롤러는 J4,J5,J6,D(레일),J1,J2,J3 순서로 보고함
        raw = ("<Idle,Angle(ABCDXYZ):4.0,5.0,6.0,0.0,1.0,2.0,3.0,"
               "Cartesian coordinate(XYZ RxRyRz):1.0,2.0,3.0,10.0,-20.0,30.0,Pump PWM:0>")
        s = de.parse_status(raw)
        self.assertEqual(s["joints_deg"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(s["abc_deg"], (10.0, -20.0, 30.0))


class RaisePlanTest(unittest.TestCase):
    def setUp(self):
        self.plan = sp.plan_raise(CFG)

    def test_commands_are_joint_moves(self):
        self.assertTrue(self.plan["commands"])
        for line, _ in self.plan["commands"]:
            self.assertRegex(line, JOINT_LINE)

    def test_first_move_turns_only_the_base_to_the_side(self):
        first = joints_of(self.plan["commands"][0][0])
        side = sp.settings(CFG)["side_j1_deg"]
        self.assertTrue(np.allclose(first, [side, 0, 0, 0, 0, 0]))

    def test_ends_facing_the_wall_with_the_pen_tip_raised(self):
        last = joints_of(self.plan["commands"][-1][0])
        self.assertAlmostEqual(last[0], 0.0, places=3)
        flange, _ = ms.fk(np.radians(last))
        up = np.radians(sp.settings(CFG)["pen_up_deg"])
        self.assertTrue(np.allclose(flange[:3, 2], [np.cos(up), 0, np.sin(up)], atol=1e-3))

    def test_pen_tip_keeps_the_clearance_from_the_paper(self):
        s = sp.settings(CFG)
        self.assertGreaterEqual(self.plan["min_tip_clearance_mm"], s["clearance_mm"] - sp.GAP_TOL_MM)
        self.assertGreaterEqual(self.plan["min_joint_margin_deg"], s["min_joint_margin_deg"])
        self.assertGreaterEqual(self.plan["standby_singularity_deg"], s["min_singularity_deg"])

    def test_refuses_when_the_paper_is_too_close_for_the_pen(self):
        cfg = copy.deepcopy(CFG)
        cfg["start_pose"] = {"paper_x_mm": 200.0, "pen_len_max_mm": 100.0}
        with self.assertRaises(sp.StartPoseError) as cm:
            sp.plan_raise(cfg)
        self.assertIn("mm", str(cm.exception))

    def test_refuses_a_side_angle_outside_the_base_range(self):
        cfg = copy.deepcopy(CFG)
        cfg["start_pose"] = {"side_j1_deg": -130.0}      # J1 범위 -110°~+160°
        with self.assertRaises(sp.StartPoseError):
            sp.plan_raise(cfg)


class VirtualJointMoveTest(unittest.TestCase):
    def test_virtual_robot_follows_joint_moves(self):
        link = VirtualMirobotLink(CFG, speed=1e6)
        plan = sp.plan_raise(CFG)
        for line, _ in plan["commands"]:
            link.send_and_ack(line, 5.0)
        expected = ms.fk(np.radians(joints_of(plan["commands"][-1][0])))[0][:3, 3]
        self.assertTrue(np.allclose(link.pos, expected, atol=1e-3))

    def test_virtual_status_reports_joints_like_the_controller(self):
        link = VirtualMirobotLink(CFG, speed=1e6)
        link.send_and_ack("M21 G90 G01 X-90.000 Y0.000 Z0.000 A0.000 B0.000 C0.000 F600", 5.0)
        s = de.parse_status(link.query_status()[2])
        self.assertEqual(s["joints_deg"], [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertTrue(np.allclose(s["tcp"], link.pos, atol=1e-3))

    def test_raise_runs_end_to_end_on_the_virtual_robot(self):
        link = VirtualMirobotLink(CFG, speed=1e6)
        link.joints = [0.0] * 6
        result = sp.run_raise(link, CFG, confirm=lambda plan: True, progress=lambda *_: None)
        self.assertEqual(result["result"], "completed")
        self.assertTrue(np.allclose(result["reported"]["tcp"], result["plan"]["standby_tcp_mm"], atol=1e-2))


class FakeLink:
    """관절 각도를 보고하는 가짜 연결: 보낸 M21 명령대로 관절 값이 바뀜."""

    def __init__(self, joints=(0, 0, 0, 0, 0, 0), fail_on=None):
        self.joints = list(joints)
        self.sent = []
        self.fail_on = fail_on

    def query_status(self, timeout=2.0):
        j = self.joints
        raw = (f"<Idle,Angle(ABCDXYZ):{j[3]},{j[4]},{j[5]},0,{j[0]},{j[1]},{j[2]},"
               "Cartesian coordinate(XYZ RxRyRz):1.0,2.0,3.0,4.0,5.0,6.0,Pump PWM:0>")
        return "Idle", (1.0, 2.0, 3.0), raw

    def send_and_ack(self, line, timeout, should_stop=None):
        self.sent.append(line)
        if self.fail_on and self.fail_on in line:
            raise de.ControllerError("컨트롤러 오류 응답: Soft limit:B")
        self.joints = list(joints_of(line))

    def wait_idle(self, timeout):
        return (1.0, 2.0, 3.0)


class RunRaiseTest(unittest.TestCase):
    def test_refuses_unless_the_robot_is_at_the_homing_pose(self):
        link = FakeLink(joints=(0, 0, 0, 0, -30, 0))
        with self.assertRaises(sp.StartPoseError):
            sp.run_raise(link, CFG, confirm=lambda plan: True, progress=lambda *_: None)
        self.assertEqual(link.sent, [])

    def test_sends_nothing_when_the_person_declines(self):
        link = FakeLink()
        result = sp.run_raise(link, CFG, confirm=lambda plan: False, progress=lambda *_: None)
        self.assertEqual(result["result"], "cancelled")
        self.assertEqual(link.sent, [])

    def test_sends_the_plan_and_reports_the_controller_pose(self):
        link = FakeLink()
        result = sp.run_raise(link, CFG, confirm=lambda plan: True, progress=lambda *_: None)
        self.assertEqual(result["result"], "completed")
        self.assertEqual(link.sent, [c for c, _ in sp.plan_raise(CFG)["commands"]])
        self.assertEqual(result["reported"]["abc_deg"], (4.0, 5.0, 6.0))

    def test_stops_on_a_controller_error(self):
        link = FakeLink(fail_on="Y0.000 Z0.000")      # 첫 명령(J1만 옆으로)에서 오류
        result = sp.run_raise(link, CFG, confirm=lambda plan: True, progress=lambda *_: None)
        self.assertEqual(result["result"], "stopped_on_error")
        self.assertEqual(len(link.sent), 1)


if __name__ == "__main__":
    unittest.main()
