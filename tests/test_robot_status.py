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


def usb(device, vid=0x1A86, description="USB-SERIAL CH340"):
    return SimpleNamespace(device=device, vid=vid, description=description, hwid="")


BLUETOOTH = SimpleNamespace(device="COM5", vid=None, description="표준 Bluetooth에서 직렬 링크(COM5)",
                            hwid="BTHENUM\\{00001101}")


class RobotPortResolutionTest(unittest.TestCase):
    def resolve(self, configured, ports):
        from mirobot_sketch import robot_status
        return robot_status.resolve_robot_port(configured, ports)

    def test_moved_robot_is_found_when_configured_port_is_gone(self):
        choice = self.resolve("COM9", [BLUETOOTH, usb("COM7")])
        self.assertEqual((choice.port, choice.source), ("COM7", "auto"))

    def test_auto_setting_ignores_bluetooth_and_other_usb_serial(self):
        other = usb("COM3", vid=0x0403, description="USB Serial Port (FTDI)")
        self.assertEqual(self.resolve("auto", [BLUETOOTH, other, usb("COM8")]).port, "COM8")
        self.assertEqual(self.resolve("", [BLUETOOTH, other]).source, "missing")

    def test_configured_robot_port_wins_when_several_robot_cables_exist(self):
        choice = self.resolve("com8", [usb("COM7"), usb("COM8")])
        self.assertEqual((choice.port, choice.source), ("COM8", "configured"))

    def test_several_robot_cables_without_matching_setting_are_not_guessed(self):
        choice = self.resolve("COM9", [usb("COM8"), usb("COM7")])
        self.assertIsNone(choice.port)
        self.assertEqual((choice.source, choice.candidates), ("ambiguous", ("COM7", "COM8")))

    def test_configured_non_wch_adapter_is_used_only_when_no_robot_cable_exists(self):
        other = usb("COM3", vid=0x0403, description="USB Serial Port")
        self.assertEqual(self.resolve("COM3", [other]).port, "COM3")
        self.assertEqual(self.resolve("COM3", [other, usb("COM7")]).port, "COM7")

    def test_recognises_wch_chip_from_description_when_vid_is_missing(self):
        port = SimpleNamespace(device="COM4", vid=None, description="USB-SERIAL CH340 (COM4)", hwid="")
        self.assertEqual(self.resolve("auto", [port]).port, "COM4")


class OpenLinkPortTest(unittest.TestCase):
    def test_opens_detected_port_and_reports_missing_robot(self):
        from unittest import mock

        from mirobot_sketch import draw_executor as de
        cfg = {"port": "COM9", "baud": 115200}
        with mock.patch("serial.tools.list_ports.comports", return_value=[BLUETOOTH, usb("COM7")]), \
                mock.patch.object(de.MirobotLink, "open", return_value=SimpleNamespace()) as open_:
            link = de.open_link(cfg)
        open_.assert_called_once_with("COM7", 115200, False)
        self.assertEqual(link.port, "COM7")
        with mock.patch("serial.tools.list_ports.comports", return_value=[BLUETOOTH]), \
                mock.patch.object(de.MirobotLink, "open") as open_:
            with self.assertRaises(de.DrawError) as err:
                de.open_link(cfg)
        open_.assert_not_called()
        self.assertEqual(err.exception.step, "connect")
        self.assertIn("찾지 못했습니다", err.exception.message)


if __name__ == "__main__":
    unittest.main()
