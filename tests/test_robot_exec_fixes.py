"""로봇 실행 층 결함 회귀: 일시정지 선택, 컨트롤러 설정, 복귀 오류, 누름량, 취소, 상태 표시."""

import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import robot_status  # noqa: E402
from mirobot_sketch import start_pose as sp  # noqa: E402
from mirobot_sketch.draw_job import DrawJob  # noqa: E402
from mirobot_sketch.draw_window import DrawWindow  # noqa: E402
from mirobot_sketch.virtual_robot import VirtualMirobotLink  # noqa: E402

import golden  # noqa: E402
from test_requested_start import cfg_for_test  # noqa: E402


HOME = ("<Idle,Angle(ABCDXYZ):0,0,0,0,0,0,0,"
        "Cartesian coordinate(XYZ RxRyRz):1.000,2.000,3.000,0,0,0>")
RUN = ("<Run,Angle(ABCDXYZ):0,0,0,0,0,0,0,"
       "Cartesian coordinate(XYZ RxRyRz):1.000,2.000,3.000,0,0,0>")
SETTINGS_OK = (
    "$3=123\n"
    "$20=1 (soft limits, inches)\n"
    "$23=60\n"
    "ok\n"
)


class _Var:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


def _join(thread, seconds=1.0):
    thread.start()
    thread.join(seconds)
    return thread.is_alive()


class DecisionDrainTest(unittest.TestCase):
    def job(self):
        return DrawJob(SimpleNamespace(), golden.default_cfg(), lambda *a, **k: None)

    def _decision(self, job):
        got = []
        alive = _join(threading.Thread(target=lambda: got.append(job._wait_pause_decision(2, 8, True))))
        self.assertFalse(alive, "펜을 떼는 동안 넣은 선택이 버려지고 일시정지에서 기다림")
        return got[0]

    def test_finish_pressed_during_lift_wins_and_is_not_dropped(self):
        job = self.job()
        job._decision.put("resume")
        job._decision.put("end")
        job._decision.put("finish")
        self.assertEqual(self._decision(job), "finish")

    def test_end_pressed_during_lift_returns_without_waiting(self):
        job = self.job()
        job._decision.put("resume")
        job._decision.put("end")
        self.assertEqual(self._decision(job), "end")

    def test_latest_resume_pressed_during_lift_is_kept(self):
        job = self.job()
        job._decision.put("resume")
        self.assertEqual(self._decision(job), "resume")


class SettingsSerial:
    """$$ 응답만 돌려주는 가짜 시리얼. 포트를 열지 않는다."""

    def __init__(self, text):
        self.text = text.encode()
        self.buf = b""
        self.written = []

    @property
    def in_waiting(self):
        return len(self.buf)

    def read(self, n=1):
        n = max(int(n or 1), 1)
        chunk, self.buf = self.buf[:n], self.buf[n:]
        return chunk

    def reset_input_buffer(self):
        self.buf = b""

    def write(self, data):
        self.written.append(data)
        if data.startswith(b"$$"):
            self.buf = self.text


class ControllerSettingsTest(unittest.TestCase):
    def test_read_settings_ignores_soft_limit_comment(self):
        link = de.MirobotLink(SettingsSerial(SETTINGS_OK))
        got = link.read_settings(timeout=1.0)
        self.assertEqual(got["$3"], "123")
        self.assertEqual(got["$23"], "60")
        self.assertTrue(any(item.startswith(b"$$") for item in link.ser.written))

    def test_match_allows_motion_gate_to_pass(self):
        link = SimpleNamespace(sent=[], read_settings=lambda timeout=10.0: {"$3": "123", "$23": "60"})
        de.require_controller_settings(link, golden.default_cfg())
        self.assertEqual(link.sent, [])

    def test_mismatch_names_read_and_expected_and_sends_no_motion(self):
        link = SimpleNamespace(sent=[], read_settings=lambda timeout=10.0: {"$3": "127", "$23": "56"})
        with self.assertRaises(de.DrawError) as caught:
            de.require_controller_settings(link, golden.default_cfg())
        text = caught.exception.message
        self.assertIn("127", text)
        self.assertIn("123", text)
        self.assertIn("56", text)
        self.assertIn("60", text)
        self.assertEqual(link.sent, [])

    def test_missing_setting_line_refuses(self):
        link = SimpleNamespace(sent=[], read_settings=lambda timeout=10.0: {"$3": "123"})
        with self.assertRaises(de.DrawError) as caught:
            de.require_controller_settings(link, golden.default_cfg())
        self.assertIn("$23", caught.exception.message)
        self.assertIn("없음", caught.exception.message)
        self.assertEqual(link.sent, [])

    def test_explicit_null_skips_the_read_but_a_missing_key_still_checks(self):
        def boom(timeout=10.0):
            raise AssertionError("읽으면 안 됨")
        link = SimpleNamespace(sent=[], read_settings=boom)
        for off in (None, {}):
            self.assertIsNone(de.require_controller_settings(link, {**golden.default_cfg(), "controller_settings_expected": off}))
        cfg = {k: v for k, v in golden.default_cfg().items() if k != "controller_settings_expected"}
        bad = SimpleNamespace(sent=[], read_settings=lambda timeout=10.0: {"$3": "127", "$23": "56"})
        with self.assertRaises(de.DrawError):                       # 키가 없으면 기본값(123/60)으로 확인
            de.require_controller_settings(bad, cfg)


class RequestedReturnOsErrorTest(unittest.TestCase):
    def test_usb_unplug_during_requested_return_is_reported_not_raised(self):
        job = DrawJob(SimpleNamespace(), golden.default_cfg(), lambda *a, **k: None)
        job.cfg = {**job.cfg, "requested_start_pose": {"joints_deg": [0, 0, 0, 0, 0, 0]}}
        with mock.patch("mirobot_sketch.start_pose.execute_requested_return", side_effect=OSError(5, "USB")):
            try:
                ok, why = job._return_to_origin(None)
            except OSError as e:
                self.fail(f"USB 끊김이 완료된 그림의 복귀를 탈출함: {e}")
        self.assertFalse(ok)
        self.assertIn("USB", why)


class PenDownExtraTest(unittest.TestCase):
    def status(self):
        return {"state": "Idle", "joints_deg": [0, 8, -8, 0, -128, 0],
                "tcp": (230.5, 1.0, 271.2), "abc_deg": (0, -128, 0)}

    def cfg(self, extra):
        cfg = cfg_for_test()
        cfg["requested_start_pose"]["contact_joints_deg"] = [0, 8, -8, 0, -128, 0]
        cfg["requested_start_pose"]["requested_pen_down_extra_mm"] = extra
        return cfg

    def test_forty_mm_is_rejected_before_any_motion(self):
        cfg = self.cfg(40)
        with self.assertRaises(sp.StartPoseError):
            sp.drawing_config_from_contact(cfg, self.status())
        robot = VirtualMirobotLink(cfg, speed=10000)
        robot.state = "Idle"
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_start(robot, cfg)
        self.assertEqual(robot.sent, [])

    def test_current_and_boundary_values_are_copied(self):
        for extra in (-0.4, -10.0, 7.0, 8.0):
            new = sp.drawing_config_from_contact(self.cfg(extra), self.status())
            self.assertEqual(new["pen"]["down_extra_mm"], extra)

    def test_just_outside_the_typo_bound_is_rejected(self):
        for extra in (-10.1, 8.1):
            with self.assertRaises(sp.StartPoseError):
                sp.drawing_config_from_contact(self.cfg(extra), self.status())


class CancelDuringIdleWaitTest(unittest.TestCase):
    def test_wait_idle_honors_stop_when_status_never_becomes_idle(self):
        class NeverIdle:
            def __init__(self):
                self.pending = []
                self.written = []

            def write(self, data):
                self.written.append(data)
                self.pending.append(RUN)

            def readline(self):
                return (self.pending.pop(0) + "\r\n").encode() if self.pending else b""

            def reset_input_buffer(self):
                self.pending.clear()

        link = de.MirobotLink(NeverIdle())
        polls = {"n": 0}

        def should_stop():
            polls["n"] += 1
            return polls["n"] > 2

        started = time.monotonic()
        with self.assertRaises(de.StopRequested):
            link.wait_idle(180, should_stop=should_stop)
        self.assertLess(time.monotonic() - started, 2.0)

    def test_stop_during_joint_wait_sends_no_further_command(self):
        class Scripted:
            def __init__(self):
                self.pending = []
                self.motion = 0

            def write(self, data):
                text = data.decode().strip()
                if text == "?":
                    self.pending.append(RUN if self.motion else HOME)
                else:
                    self.motion += 1
                    self.pending.append("ok")

            def readline(self):
                return (self.pending.pop(0) + "\r\n").encode() if self.pending else b""

            def reset_input_buffer(self):
                self.pending.clear()

        ser = Scripted()
        link = de.MirobotLink(ser)
        link.virtual = True
        cfg = cfg_for_test()
        cfg["idle_timeout_s"] = 30
        cfg["ack_timeout_s"] = 2

        def should_stop():
            return ser.motion > 0

        started = time.monotonic()
        with self.assertRaises(sp.StartPoseError):
            sp.execute_requested_start(link, cfg, should_stop=should_stop)
        self.assertLess(time.monotonic() - started, 3.0)
        self.assertEqual(ser.motion, 1)


class RobotStatusAfterFailedReturnTest(unittest.TestCase):
    def test_completed_drawing_is_not_idle_when_return_failed_or_state_is_not_idle(self):
        events = []
        reporter = robot_status.ControllerStatusReporter(lambda **event: events.append(event))
        reporter.connection_started()
        reporter.progress("  컨트롤러 상태: Idle")
        reporter.drawing_started()
        reporter.finished("completed", link_open=True, returned_to_origin=False)
        self.assertNotEqual(events[-1]["state"], "Idle")
        self.assertIn(events[-1]["state"], ("unknown", "Idle")[:1] or ("unknown",))
        self.assertEqual(events[-1]["state"], "unknown")

        events.clear()
        running = robot_status.ControllerStatusReporter(lambda **event: events.append(event))
        running.connection_started()
        running.progress("  컨트롤러 상태: Run")
        running.finished("completed", link_open=True)
        self.assertEqual(events[-1]["state"], "Run")


class ConfirmEstimateTest(unittest.TestCase):
    def test_confirm_line_follows_the_air_checkbox(self):
        strokes = [[(0.0, float(i)), (40.0, float(i))] for i in range(12)]
        cfg = golden.default_cfg()
        air_s = de.estimate_time(strokes, cfg, air=True)["total_s"]
        pen_s = de.estimate_time(strokes, cfg, air=False)["total_s"]
        self.assertNotEqual(f"{air_s / 60:.1f}", f"{pen_s / 60:.1f}")
        shown = {}
        view = SimpleNamespace(
            job=SimpleNamespace(
                state="confirm",
                summary={"stroke_count": 12, "command_count": 99, "estimated_s": air_s,
                         "drawing_mm": [40.0, 11.0], "port": "가상", "virtual": True,
                         "plane_verified": True},
                timing_for=lambda air: de.estimate_time(strokes, cfg, air=air),
            ),
            air_var=_Var(True),
            cfg={},
            start_pose_var=_Var(False),
            detail=SimpleNamespace(configure=lambda **kw: shown.update(kw)),
        )
        view._show_confirm_detail = lambda: DrawWindow._show_confirm_detail(view)
        DrawWindow._show_confirm_detail(view)
        self.assertIn(f"예상 {air_s / 60:.1f}분", shown["text"])
        view.air_var = _Var(False)
        DrawWindow._on_air_changed(view)
        self.assertIn(f"예상 {pen_s / 60:.1f}분", shown["text"])
        self.assertNotIn(f"예상 {air_s / 60:.1f}분", shown["text"])


if __name__ == "__main__":
    unittest.main()
