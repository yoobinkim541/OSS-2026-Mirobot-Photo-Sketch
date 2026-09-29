"""로봇 USB 감지와 컨트롤러 상태 문구 변환."""

import importlib
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


class RobotStatusTest(unittest.TestCase):
    def api(self):
        try:
            return importlib.import_module("mirobot_sketch.robot_status")
        except ImportError:
            self.fail("GUI의 로봇 연결 상태 확인 기능이 아직 없습니다")

    def reporter(self):
        status = self.api()
        try:
            return status.ControllerStatusReporter
        except AttributeError:
            self.fail("실제 컨트롤러 상태 전달 흐름이 아직 없습니다")

    def test_detects_configured_port_from_pyserial_port_info(self):
        status = self.api()
        ports = [SimpleNamespace(device="COM5"), SimpleNamespace(device="com9")]
        self.assertTrue(status.port_is_present("COM9", ports))

    def test_does_not_match_a_different_port_with_same_prefix(self):
        status = self.api()
        self.assertFalse(status.port_is_present("COM9", ["COM90"]))

    def test_missing_port_configuration_is_not_detected(self):
        status = self.api()
        self.assertFalse(status.port_is_present("", ["COM9"]))

    def test_extracts_controller_state_only_from_status_progress(self):
        status = self.api()
        self.assertEqual(status.controller_state_from_progress("  컨트롤러 상태: Alarm"), "Alarm")
        self.assertEqual(status.controller_state_from_progress("  컨트롤러 상태: Idle"), "Idle")
        self.assertIsNone(status.controller_state_from_progress("  -> 중앙 버튼을 눌러 호밍하세요"))

    def test_labels_unknown_and_recent_controller_states_distinctly(self):
        status = self.api()
        self.assertEqual(status.controller_status_text(), "제어기 상태: 미확인")
        self.assertEqual(status.controller_status_text("Idle"), "제어기 상태: Idle")
        self.assertEqual(status.controller_status_text("Idle", recent=True), "최근 제어기 상태: Idle")

    def test_reports_live_controller_states_and_final_idle(self):
        events = []
        reporter = self.reporter()(lambda **event: events.append(event))
        reporter.connection_started()
        reporter.progress("  컨트롤러 상태: Alarm")
        reporter.progress("  컨트롤러 상태: Idle")
        reporter.drawing_started()
        reporter.finished("completed", link_open=True)

        self.assertEqual(events, [
            {"state": "연결 중"},
            {"state": "Alarm"},
            {"state": "Idle"},
            {"state": "그리는 중"},
            {"state": "Idle", "recent": True},
        ])

    def test_reports_connection_failure_and_clears_stale_state_after_interrupted_draw(self):
        events = []
        reporter_type = self.reporter()
        failed = reporter_type(lambda **event: events.append(event))
        failed.connection_started()
        failed.finished("error", link_open=False)
        self.assertEqual(events[-1], {"state": "연결 실패", "recent": True})

        interrupted = reporter_type(lambda **event: events.append(event))
        interrupted.connection_started()
        interrupted.progress("  컨트롤러 상태: Idle")
        interrupted.drawing_started()
        interrupted.finished("stopped_by_user", link_open=True)
        self.assertEqual(events[-1], {"state": None, "recent": True})


if __name__ == "__main__":
    unittest.main()
