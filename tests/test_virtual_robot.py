"""로봇 없이 같은 실행 흐름을 주는 가상 시뮬레이션."""

import re
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch.virtual_robot import VirtualMirobotLink  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()
SQUARE = [[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0), (-10.0, -10.0)]]


class VirtualLinkTest(unittest.TestCase):
    def test_homing_reports_idle_at_paper_center(self):
        v = VirtualMirobotLink(CFG, homing_s=0.05)
        state, tcp = v.wait_for_homing(5, progress=lambda *_: None)
        c = CFG["paper_center_tcp_mm"]
        self.assertEqual(state, "Idle")
        self.assertAlmostEqual(tcp[0], c["x"])
        v2 = VirtualMirobotLink(CFG, homing_s=0.05, start_offset_mm=7.0)
        self.assertGreater(abs(v2.wait_for_homing(5, progress=lambda *_: None)[1][2] - c["z"]), 6.9)

    def test_homing_can_be_cancelled_and_can_fail(self):
        v = VirtualMirobotLink(CFG, homing_s=10)
        t0 = time.monotonic()
        self.assertEqual(v.wait_for_homing(20, lambda *_: None, should_cancel=lambda: True)[0], "cancelled")
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertEqual(VirtualMirobotLink(CFG, homing_s=0.05, homing_fails=True)
                         .wait_for_homing(0.3, lambda *_: None)[0], "Alarm")

    def test_ack_takes_expected_time_scaled_by_speed(self):
        cmds = de.Planner(CFG).plan(SQUARE)
        v = VirtualMirobotLink(CFG, speed=50)
        v.wait_for_homing(1, lambda *_: None)
        t0 = time.monotonic()
        result = de.execute(v, cmds, CFG, progress=lambda *_: None)
        took = time.monotonic() - t0
        lat = CFG["timing"]["assumed_command_latency_s"]      # 기대값 = 명령마다 (거리/속도 + 지연) / 배속
        pos, expected = VirtualMirobotLink(CFG).pos, 0.0
        for line, _ in cmds:
            xyz = tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())
            feed = float(re.search(r"F([\d.]+)", line).group(1))
            expected += (sum((a - b) ** 2 for a, b in zip(xyz, pos)) ** 0.5 / (feed / 60) + lat) / 50
            pos = xyz
        self.assertEqual(result["result"], "completed")
        self.assertEqual(len(v.sent), len(cmds))
        self.assertLess(abs(took - expected), max(0.5, expected * 0.5))

    def test_fail_and_timeout(self):
        cmds = de.Planner(CFG).plan(SQUARE)
        r = de.execute(VirtualMirobotLink(CFG, speed=100, fail_at=3), cmds, CFG, progress=lambda *_: None)
        self.assertEqual((r["result"], r["commands_sent"]), ("stopped_on_error", 2))
        cfg = {**CFG, "ack_timeout_s": 0.2}
        r = de.execute(VirtualMirobotLink(cfg, speed=100, timeout_at=2), cmds, cfg, progress=lambda *_: None)
        self.assertEqual((r["result"], r["commands_sent"]), ("stopped_on_error", 1))
        self.assertIn("시간 초과", r["error"])

    def test_stop_interrupts_a_long_move_quickly(self):
        cmds = de.Planner(CFG).plan(SQUARE)
        v = VirtualMirobotLink(CFG, speed=1)                  # 1배속: 한 명령이 몇 초
        stop = threading.Event()
        threading.Timer(0.3, stop.set).start()
        t0 = time.monotonic()
        r = de.execute(v, cmds, CFG, progress=lambda *_: None, should_stop=stop.is_set)
        self.assertEqual(r["result"], "stopped_by_user")
        self.assertLess(time.monotonic() - t0, 0.9)
        self.assertEqual(len(v.sent), r["commands_sent"] + 1)   # 멈춘 명령 뒤로는 안 보냄

    def test_on_ack_called_for_every_command(self):
        cmds = de.Planner(CFG).plan(SQUARE)
        seen = []
        de.execute(VirtualMirobotLink(CFG, speed=500), cmds, CFG, progress=lambda *_: None,
                   on_ack=lambda a, n: seen.append((a, n)))
        self.assertEqual(seen, [(i + 1, len(cmds)) for i in range(len(cmds))])


if __name__ == "__main__":
    unittest.main()
