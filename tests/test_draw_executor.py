"""로봇 실행기 테스트 (실제 시리얼 없이 가짜 컨트롤러 사용).

    python -m unittest discover -s tests -v
"""

import copy
import json
import re
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mirobot_sketch import draw_executor as de  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()


class FakeSerial:
    """명령마다 정해진 응답을 돌려주는 가짜 Mirobot."""

    IDLE = ("<Idle,Angle(ABCDXYZ):0,0,0,0,0,0,0,"
            "Cartesian coordinate(XYZ RxRyRz):198.668,0.000,230.477,0.000,0.000,0.000,"
            "Pump PWM:0,Valve PWM:0,Motion_MODE:0>")

    def __init__(self, fail_at=None, fail_msg="Soft limit:B"):
        self.lines = []
        self.pending = []
        self.fail_at = fail_at
        self.fail_msg = fail_msg
        self.written = []

    def write(self, data):
        text = data.decode().strip()
        self.written.append(text)
        if text == "?":
            self.pending.append(self.IDLE)
            return
        n = sum(1 for w in self.written if w != "?")
        self.pending.append(self.fail_msg if n == self.fail_at else "ok")

    def readline(self):
        return (self.pending.pop(0) + "\r\n").encode() if self.pending else b""

    def reset_input_buffer(self):
        self.pending.clear()

    def close(self):
        pass


SQUARE = [[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0), (-10.0, -10.0)]]


class PlannerTest(unittest.TestCase):
    def test_pen_sequence_and_retract_direction(self):
        planner = de.Planner(CFG)
        cmds = planner.plan(SQUARE)
        labels = [label for _, label in cmds]
        self.assertEqual(labels[0], "center pen-up")
        self.assertEqual(labels[1:3], ["stroke 0 travel", "stroke 0 pen-down"])
        self.assertEqual(labels[-2:], ["stroke 0 pen-up", "return center pen-up"])
        up_x = planner.pose(0, 0, False)[0]
        down_x = planner.pose(0, 0, True)[0]
        self.assertAlmostEqual(down_x, CFG["paper_center_tcp_mm"]["x"])
        self.assertAlmostEqual(down_x - up_x, CFG["pen"]["up_clearance_mm"])  # 펜업은 벽에서 멀어짐

    def _high_pen_up_cfg(self, up=10.0):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["up_clearance_mm"] = up
        return cfg

    def test_high_pen_up_descends_fast_then_slow_only_near_paper(self):
        cfg = self._high_pen_up_cfg(10.0)
        f = cfg["feeds_mm_per_min"]
        planner = de.Planner(cfg)
        cmds = planner.plan(SQUARE)
        labels = [label for _, label in cmds]
        self.assertEqual(labels[1:4], ["stroke 0 travel", "stroke 0 descend", "stroke 0 pen-down"])
        x_of = lambda line: float(re.search(r"X([-\d.]+)", line).group(1))
        feed_of = lambda line: float(re.search(r"F([\d.]+)", line).group(1))
        travel, descend, down = cmds[1][0], cmds[2][0], cmds[3][0]
        sign = cfg["pen"]["retract_x_sign"]
        contact = planner.pose(SQUARE[0][0][0], SQUARE[0][0][1], True)[0]
        self.assertAlmostEqual(sign * (x_of(travel) - contact), 10.0, places=2)           # 펜업은 10 mm 위
        self.assertAlmostEqual(sign * (x_of(descend) - contact), cfg["pen"]["slow_zone_mm"], places=2)
        self.assertEqual([feed_of(travel), feed_of(descend), feed_of(down)], [f["travel"], f["travel"], f["approach"]])
        up = next(line for line, label in cmds if label == "stroke 0 pen-up")
        self.assertEqual(feed_of(up), f["travel"])                                         # 올리는 건 빠르게
        self.assertAlmostEqual(sign * (x_of(up) - planner.pose(*SQUARE[0][-1], True)[0]), 10.0, places=2)

    def test_pen_up_within_slow_zone_keeps_single_slow_approach(self):
        planner = de.Planner(CFG)                                   # 기본 설정: 펜업 2.5 mm = 느린 구간
        labels = [label for _, label in planner.plan(SQUARE)]
        self.assertNotIn("stroke 0 descend", labels)
        self.assertEqual(planner.plan(SQUARE)[-2][0].split(" F")[1], str(int(CFG["feeds_mm_per_min"]["approach"])))

    def test_air_mode_has_no_descent_waypoint_even_with_high_pen_up(self):
        planner = de.Planner(self._high_pen_up_cfg(10.0), air=True)
        self.assertNotIn("stroke 0 descend", [label for _, label in planner.plan(SQUARE)])

    def test_axis_signs(self):
        planner = de.Planner(CFG)
        c = CFG["paper_center_tcp_mm"]
        ry, rz = planner.robot_yz(10, 5)
        self.assertAlmostEqual(ry, c["y"] + CFG["paper_x_to_robot_y_sign"] * 10)
        self.assertAlmostEqual(rz, c["z"] + 5)

    def test_plane_compensation(self):
        cfg = copy.deepcopy(CFG)
        cfg["plane_compensation"]["a_per_mm_y"] = 0.01
        planner = de.Planner(cfg)
        x_left = planner.pose(-50, 0, True)[0]
        x_right = planner.pose(50, 0, True)[0]
        self.assertAlmostEqual(abs(x_left - x_right), 1.0)

    def test_out_of_limits_rejected(self):
        self.assertTrue(de.check_limits([[(0, 0), (80, 0)]], CFG))
        self.assertFalse(de.check_limits(SQUARE, CFG))

    def test_gcode_format(self):
        line = de.Planner(CFG).plan(SQUARE)[0][0]
        self.assertRegex(line, r"^M20 G90 G01 X[-\d.]+ Y[-\d.]+ Z[-\d.]+ A[-\d.]+ B[-\d.]+ C[-\d.]+ F\d+$")


class TimingTest(unittest.TestCase):
    def test_command_count_matches_plan(self):
        strokes = SQUARE + [[(0.0, 0.0), (5.0, 5.0)]]
        t = de.estimate_time(strokes, CFG)
        self.assertEqual(t["command_count"], len(de.Planner(CFG).plan(strokes)))

    def test_command_count_and_lift_time_match_plan_for_high_pen_up(self):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["up_clearance_mm"] = 10.0
        strokes = SQUARE + [[(0.0, 0.0), (5.0, 5.0)]]
        t = de.estimate_time(strokes, cfg)
        self.assertEqual(t["command_count"], len(de.Planner(cfg).plan(strokes)))
        f, n = cfg["feeds_mm_per_min"], len(strokes)
        expected = (10.0 / f["approach"] + n * (7.5 / f["travel"] + 2.5 / f["approach"] + 10.0 / f["travel"])) * 60
        self.assertAlmostEqual(t["pen_lift_s"], expected, places=1)
        slow_all = 10.0 * (2 * n + 1) / f["approach"] * 60                   # 전 구간 느린 속도였다면
        self.assertLess(t["pen_lift_s"], slow_all * 0.6)                    # 처음 중심 펜업은 느려서 획이 적으면 절반까지는 안 줄어듦
        air = de.estimate_time(strokes, cfg, air=True)
        self.assertEqual(air["command_count"], len(de.Planner(cfg, air=True).plan(strokes)))

    def test_pen_lift_grows_with_stroke_count(self):
        # 같은 길이를 1획으로 그릴 때와 10획으로 쪼갤 때: 펜 올림·내림 시간만 늘어남
        one = [[(float(x), 0.0) for x in range(-10, 11, 2)]]
        many = [[(float(x), 0.0), (float(x + 2), 0.0)] for x in range(-10, 10, 2)]
        t1, t10 = de.estimate_time(one, CFG), de.estimate_time(many, CFG)
        self.assertAlmostEqual(t1["draw_s"], t10["draw_s"], places=1)
        per_stroke = 2 * CFG["pen"]["up_clearance_mm"] / CFG["feeds_mm_per_min"]["approach"] * 60
        self.assertAlmostEqual(t10["pen_lift_s"] - t1["pen_lift_s"], 9 * per_stroke, places=0)
        self.assertGreater(t10["total_s"], t1["total_s"])


class SafetyOptionTest(unittest.TestCase):
    def test_air_mode_never_touches_paper(self):
        planner = de.Planner(CFG, air=True)
        for line, _ in planner.plan(SQUARE):
            parts = {part[0]: float(part[1:]) for part in line.split() if part[0] in "XYZ"}
            contact = planner.contact_x(parts["Y"], parts["Z"])
            self.assertGreaterEqual(abs(parts["X"] - contact),
                                    max(8.0, CFG["pen"]["up_clearance_mm"]) - 0.001)

    def test_pending_limits_only_with_flag(self):
        wide = [[(-120.0, -80.0), (120.0, -80.0), (120.0, 44.0), (0.0, 57.0), (-120.0, 44.0), (-120.0, -80.0)]]
        self.assertTrue(de.check_limits(wide, CFG))                    # 기본 ±50: 거부
        self.assertFalse(de.check_limits(wide, CFG, pending=True))     # 넓은 범위(지붕 모양): 허용
        self.assertTrue(de.check_limits([[(0.0, 0.0), (0.0, 58.0)]], CFG, pending=True))      # 지붕 위
        self.assertTrue(de.check_limits([[(0.0, 0.0), (124.0, 44.0)]], CFG, pending=True))    # 끝은 더 낮음

    def test_border_test_file_matches_pending_limits(self):
        from mirobot_sketch import limits
        path = Path(__file__).resolve().parent.parent / "trajectories" / "border-test-wide.json"
        _, strokes = de.load_strokes(path)
        self.assertTrue(de.check_limits(strokes, CFG))
        self.assertFalse(de.check_limits(strokes, CFG, pending=True))
        pts = np.array([p for s in strokes for p in s])
        r = limits.pending_region(CFG)
        self.assertAlmostEqual(pts[:, 0].max(), r.x_max)               # 테두리를 실제로 따라감
        self.assertAlmostEqual(pts[:, 1].min(), r.y_min)
        self.assertAlmostEqual(pts[:, 1].max(), r.top(0))


class GuidedCalibrationCLITest(unittest.TestCase):
    def test_physical_cli_run_calibrates_only_with_flag(self):
        from mirobot_sketch import calibration

        cfg = copy.deepcopy(CFG)
        center = dict(cfg["paper_center_tcp_mm"])
        ready = dict(center)
        ready["x"] += cfg["pen"]["retract_x_sign"] * cfg["pen"]["up_clearance_mm"]
        measured = {"result": "ready", "center_tcp_mm": center, "ready_tcp_mm": ready,
                    "selected_half_size_mm": 30.0, "max_residual_mm": 1.0,
                    "plane_fit": {"a_per_mm_y": 0.0, "b_per_mm_z": 0.0,
                                  "max_abs_residual_mm": 0.0}}
        with tempfile.TemporaryDirectory() as tmp:
            strokes_path = Path(tmp) / "line.json"
            strokes_path.write_text(json.dumps({"kind": "sketch_strokes", "units": "mm",
                                                "strokes": [{"points_xy_mm": [[-5, 0], [5, 0]]}]}),
                                    encoding="utf-8")
            link = mock.Mock()
            with mock.patch.object(sys, "argv", ["mirobot-draw", str(strokes_path), "--execute", "--calibrate"]), \
                 mock.patch.object(de, "load_config", return_value=cfg), \
                 mock.patch.object(de, "open_link", return_value=link), \
                 mock.patch.object(calibration, "run_calibration", return_value=measured) as run_cal, \
                 mock.patch.object(calibration, "save_report", return_value=Path(tmp) / "calibration.json"), \
                 mock.patch.object(calibration, "save_config") as save_cfg, \
                 mock.patch.object(de, "execute", return_value={"result": "completed"}) as execute, \
                 mock.patch.object(de, "write_run_record", return_value=Path(tmp) / "run.json"), \
                 mock.patch("builtins.input", return_value="yes"):
                self.assertEqual(de.main(), 0)
            run_cal.assert_called_once()
            save_cfg.assert_called_once()
            execute.assert_called_once()
            self.assertEqual(save_cfg.call_args.args[1]["plane_compensation"]["status"], "verified")

    def test_physical_cli_run_without_flag_skips_calibration_and_draws(self):
        from mirobot_sketch import calibration

        cfg = copy.deepcopy(CFG)
        c = cfg["paper_center_tcp_mm"]
        with tempfile.TemporaryDirectory() as tmp:
            strokes_path = Path(tmp) / "line.json"
            strokes_path.write_text(json.dumps({"kind": "sketch_strokes", "units": "mm",
                                                "strokes": [{"points_xy_mm": [[-5, 0], [5, 0]]}]}),
                                    encoding="utf-8")
            with mock.patch.object(sys, "argv", ["mirobot-draw", str(strokes_path), "--execute"]), \
                 mock.patch.object(de, "load_config", return_value=cfg), \
                 mock.patch.object(de, "open_link", return_value=mock.Mock()), \
                 mock.patch.object(de, "connect_and_home", return_value=(c["x"], c["y"], c["z"])), \
                 mock.patch.object(calibration, "run_calibration") as run_cal, \
                 mock.patch.object(calibration, "save_config") as save_cfg, \
                 mock.patch.object(de, "execute", return_value={"result": "completed"}) as execute, \
                 mock.patch.object(de, "write_run_record", return_value=Path(tmp) / "run.json"), \
                 mock.patch("builtins.input", return_value="yes"):
                self.assertEqual(de.main(), 0)
            run_cal.assert_not_called()
            save_cfg.assert_not_called()
            execute.assert_called_once()


class ExecuteTest(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(CFG)
        self.cfg["ack_timeout_s"] = 0.5
        self.cfg["idle_timeout_s"] = 1.0
        self.cmds = de.Planner(self.cfg).plan(SQUARE)

    def test_status_parsing(self):
        state, tcp, _ = de.MirobotLink(FakeSerial()).query_status()
        self.assertEqual(state, "Idle")
        self.assertEqual(tcp, (198.668, 0.0, 230.477))

    def test_waits_for_manual_homing(self):
        # 포트를 열면 리셋 -> Alarm -> (사용자 호밍) -> Home -> Idle
        fake = FakeSerial()
        states = iter(["Alarm", "Alarm", "Home", "Idle"])

        def write(data, _orig=fake.write):
            if data.decode().strip() == "?":
                st = next(states)
                fake.pending.append(FakeSerial.IDLE.replace("<Idle", f"<{st}"))
                return
            _orig(data)

        fake.write = write
        de.time.sleep, orig_sleep = (lambda *_: None), de.time.sleep
        try:
            state, tcp = de.MirobotLink(fake).wait_for_homing(5, progress=lambda *_: None)
        finally:
            de.time.sleep = orig_sleep
        self.assertEqual(state, "Idle")
        self.assertEqual(tcp, (198.668, 0.0, 230.477))

    def test_completes_with_acks(self):
        fake = FakeSerial()
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        self.assertEqual(result["result"], "completed")
        self.assertEqual(result["commands_sent"], len(self.cmds))

    def test_stops_immediately_on_soft_limit(self):
        fake = FakeSerial(fail_at=4)
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        self.assertEqual(result["result"], "stopped_on_error")
        self.assertIn("Soft limit", result["error"])
        sent = [w for w in fake.written if w != "?"]
        self.assertEqual(len(sent), 4)  # 오류 이후 명령을 더 보내지 않음

    def test_timeout_without_ack(self):
        fake = FakeSerial(fail_at=2, fail_msg="")
        fake.readline = lambda: b""
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        self.assertEqual(result["result"], "stopped_on_error")


class OrientationTestFileTest(unittest.TestCase):
    def test_orientation_file_is_valid_and_in_limits(self):
        path = Path(__file__).resolve().parent.parent / "trajectories" / "orientation-test-F.json"
        _, strokes = de.load_strokes(path)
        self.assertEqual(len(strokes), 2)
        self.assertFalse(de.check_limits(strokes, CFG))


class StepFunctionsTest(unittest.TestCase):
    def test_draw_steps_order(self):
        self.assertEqual([s[0] for s in de.DRAW_STEPS], ["preflight", "connect", "start", "confirm", "drawing", "done"])

    def test_preflight_rejects_out_of_range_with_hint(self):
        far = [[(0.0, 0.0), (58.0, 0.0)]]
        with self.assertRaises(de.DrawError) as cm:
            de.preflight(far, CFG)
        self.assertEqual(cm.exception.step, "preflight")
        self.assertIn("넓은 범위", cm.exception.hint)
        self.assertTrue(de.preflight(far, CFG, pending=True)["cmds"])

    def test_connect_and_check_start_with_virtual_link(self):
        from mirobot_sketch.virtual_robot import VirtualMirobotLink
        link = VirtualMirobotLink(CFG, homing_s=0.01)
        tcp = de.connect_and_home(link, CFG, progress=lambda *_: None)
        de.check_start(tcp, CFG)
        bad = VirtualMirobotLink(CFG, homing_s=0.01, start_offset_mm=9.0)
        with self.assertRaises(de.DrawError) as cm:
            de.check_start(de.connect_and_home(bad, CFG, progress=lambda *_: None), CFG)
        self.assertEqual(cm.exception.step, "start")
        with self.assertRaises(de.DrawError) as cm:
            de.connect_and_home(VirtualMirobotLink(CFG, homing_s=5), CFG, progress=lambda *_: None,
                                should_cancel=lambda: True)
        self.assertEqual(cm.exception.step, "connect")

    def test_check_start_after_calibration_accepts_pen_up_retract_only_away_from_wall(self):
        c = CFG["paper_center_tcp_mm"]
        sign = CFG["pen"]["retract_x_sign"]
        at = lambda back, dy=0.0, dz=0.0: (c["x"] + sign * back, c["y"] + dy, c["z"] + dz)
        self.assertGreater(8.0, CFG["max_start_offset_mm"])            # 보정 후 후퇴 자세는 일반 5 mm 기준을 넘는다
        with self.assertRaises(de.DrawError):
            de.check_start(at(8.0), CFG)                                # 일반 검사는 막지만
        self.assertAlmostEqual(de.check_start(at(8.0), CFG, retracted=True), 8.0)   # 보정 직후에는 통과
        de.check_start(at(0.0), CFG, retracted=True)
        for bad in (at(25.0), at(-5.0), at(5.0, dy=7.0), at(5.0, dz=-6.0)):        # 너무 멀리 물러남 / 벽 쪽 / 옆으로 어긋남
            with self.assertRaises(de.DrawError) as cm:
                de.check_start(bad, CFG, retracted=True)
            self.assertEqual(cm.exception.step, "start")

    def test_cli_virtual_run_writes_record(self):
        import tempfile
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            doc = Path(d) / "s.json"
            doc.write_text(json.dumps({"kind": "sketch_strokes", "units": "mm", "strokes": [
                {"points_xy_mm": [[-5, -5], [5, -5], [5, 5]]}]}), encoding="utf-8")
            with mock.patch.object(de.paths, "runs_dir", lambda: Path(d) / "runs"),                     mock.patch("builtins.input", return_value="yes"),                     mock.patch.object(sys, "argv", ["mirobot-draw", str(doc), "--execute", "--virtual",
                                                    "--virtual-speed", "200"]):
                self.assertEqual(de.main(), 0)
            rec = json.loads(next((Path(d) / "runs").glob("run-*.json")).read_text(encoding="utf-8"))
            self.assertTrue(rec["virtual"])
            self.assertEqual(rec["result"], "completed")


if __name__ == "__main__":
    unittest.main()
