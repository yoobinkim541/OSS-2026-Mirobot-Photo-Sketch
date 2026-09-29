"""평면 캘리브레이션 계산과 안전한 사각형 탐색 계획 테스트."""

import copy
import contextlib
import io
import json
import re
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mirobot_sketch import calibration, draw_executor  # noqa: E402


CFG = draw_executor.load_config(Path(__file__).resolve().parent.parent /
                                "mirobot_sketch" / "data" / "drawing_config.json")


class CalibrationPlanTest(unittest.TestCase):
    def test_command_defaults_to_plan_without_opening_serial_port(self):
        import sys
        from unittest import mock

        with mock.patch.object(sys, "argv", ["mirobot-calibrate"]), \
             mock.patch.object(draw_executor, "open_link") as open_link, \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(calibration.main(), 0)
        open_link.assert_not_called()
        self.assertIn("60", output.getvalue())

    def test_staged_square_sizes_start_conservatively_and_stop_at_configured_limit(self):
        self.assertEqual(calibration.staged_half_sizes(CFG), [30, 35, 40, 45, 50])

    def test_staged_square_sizes_can_be_bounded_by_user(self):
        self.assertEqual(calibration.staged_half_sizes(CFG, start_mm=25, step_mm=10, max_mm=45),
                         [25, 35, 45])

    def test_staged_square_sizes_reject_invalid_or_out_of_bounds_values(self):
        with self.assertRaises(ValueError):
            calibration.staged_half_sizes(CFG, start_mm=0)
        with self.assertRaises(ValueError):
            calibration.staged_half_sizes(CFG, max_mm=51)

    def test_square_outline_is_closed_and_segmented(self):
        points = calibration.square_outline(30, segment_mm=10)
        self.assertEqual(points[0], points[-1])
        self.assertGreater(len(points), 5)
        self.assertTrue(all(np.linalg.norm(np.subtract(b, a)) <= 10.000001
                            for a, b in zip(points, points[1:])))


class PlaneFitTest(unittest.TestCase):
    def setUp(self):
        self.center = {"x": 200.0, "y": 3.0, "z": 220.0}
        self.a, self.b = 0.012, -0.018
        h = 30.0
        # 종이 좌표의 모서리를 로봇 Y/Z 좌표로 변환한 실측 샘플을 흉내 냅니다.
        sy, sz = CFG["paper_x_to_robot_y_sign"], CFG["paper_y_to_robot_z_sign"]
        self.samples = []
        for name, px, py in calibration.contact_points(h)[1:]:
            ry = self.center["y"] + sy * px
            rz = self.center["z"] + sz * py
            self.samples.append({"name": name, "paper_xy_mm": [px, py],
                                 "tcp_mm": {"x": self.center["x"] + self.a * (ry - self.center["y"])
                                                   + self.b * (rz - self.center["z"]),
                                            "y": ry, "z": rz}})

    def test_fit_recovers_axis_slopes_and_near_zero_residual(self):
        fit = calibration.fit_contact_plane(self.center, self.samples)
        self.assertAlmostEqual(fit["a_per_mm_y"], self.a, places=9)
        self.assertAlmostEqual(fit["b_per_mm_z"], self.b, places=9)
        self.assertLess(fit["max_abs_residual_mm"], 1e-9)

    def test_fit_reports_nonplanar_samples(self):
        samples = copy.deepcopy(self.samples)
        samples[-1]["tcp_mm"]["x"] += 3.0
        fit = calibration.fit_contact_plane(self.center, samples)
        self.assertGreater(fit["max_abs_residual_mm"], 0.5)

    def test_fit_rejects_collinear_or_missing_corner_samples(self):
        with self.assertRaises(ValueError):
            calibration.fit_contact_plane(self.center, self.samples[:1])

    def test_apply_updates_center_plane_and_confirmed_square_limits(self):
        cfg = copy.deepcopy(CFG)
        fit = calibration.fit_contact_plane(self.center, self.samples)
        updated = calibration.apply_calibration(cfg, self.center, fit, 30)
        self.assertEqual(updated["paper_center_tcp_mm"], self.center)
        self.assertEqual(updated["plane_compensation"]["status"], "verified")
        self.assertAlmostEqual(updated["plane_compensation"]["a_per_mm_y"], self.a)
        self.assertEqual(updated["limits"], {"max_abs_paper_x_mm": 30.0,
                                              "max_abs_paper_y_mm": 30.0})
        self.assertEqual(cfg["limits"]["max_abs_paper_x_mm"], 50.0)

    def test_apply_refuses_nonplanar_calibration(self):
        samples = copy.deepcopy(self.samples)
        samples[-1]["tcp_mm"]["x"] += 3.0
        fit = calibration.fit_contact_plane(self.center, samples)
        with self.assertRaises(ValueError):
            calibration.apply_calibration(CFG, self.center, fit, 30, max_residual_mm=1.0)

    def test_contact_points_rect_lists_center_and_four_paper_corners(self):
        pts = calibration.contact_points_rect(-60, -65, 60, 55)
        self.assertEqual([p[0] for p in pts], ["center", "bottom_left", "bottom_right", "top_right", "top_left"])
        self.assertEqual([(p[1], p[2]) for p in pts[1:]], [(-60, -65), (60, -65), (60, 55), (-60, 55)])
        with self.assertRaises(ValueError):
            calibration.contact_points_rect(60, -65, -60, 55)

    def test_apply_accepts_off_center_120mm_square_and_stores_asymmetric_limits(self):
        fit = calibration.fit_contact_plane(self.center, self.samples)
        updated = calibration.apply_calibration(CFG, self.center, fit, [-60, -65, 60, 55])
        self.assertEqual(updated["limits"], {"x_max_mm": 60.0, "y_min_mm": -65.0,
                                              "roof_mm": [[0.0, 55.0], [60.0, 55.0]]})
        self.assertEqual(updated["calibration"]["rect_mm"], [-60.0, -65.0, 60.0, 55.0])
        from mirobot_sketch import limits
        region = limits.executor_region(updated)
        for x, y in ((-60, -65), (60, -65), (60, 55), (-60, 55), (0, 0)):
            self.assertTrue(region.contains(x, y), (x, y))
        self.assertFalse(region.contains(0, 56))                       # 위쪽 한계를 넘으면 밖
        self.assertFalse(region.contains(61, 0))
        with self.assertRaises(ValueError):
            calibration.apply_calibration(CFG, self.center, fit, [-60, -65, 60, 60])   # 도달 영역(위쪽 55~57.5) 밖

    def test_apply_accepts_measured_rectangular_region(self):
        fit = calibration.fit_contact_plane(self.center, self.samples)
        updated = calibration.apply_calibration(CFG, self.center, fit, [60, 37])
        self.assertEqual(updated["limits"], {"max_abs_paper_x_mm": 60.0,
                                              "max_abs_paper_y_mm": 37.0})
        self.assertEqual(updated["calibration"]["half_extents_mm"], [60.0, 37.0])
        with self.assertRaises(ValueError):
            calibration.apply_calibration(CFG, self.center, fit, [60, 60])


class FakeCalibrationLink:
    """Cartesian fake serial endpoint; it never opens the physical robot port."""

    def __init__(self, tcp):
        self.tcp = tuple(tcp)
        self.commands = []

    def wait_for_homing(self, timeout, progress=print):
        return "Idle", self.tcp

    def send_and_ack(self, line, timeout):
        self.commands.append(line)
        values = re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line)
        self.tcp = tuple(float(v) for v in values.groups())

    def wait_idle(self, timeout):
        return self.tcp

    def query_status(self):
        return "Idle", self.tcp, "<Idle,Cartesian coordinate(XYZ RxRyRz):...>"


class CalibrationWorkflowTest(unittest.TestCase):
    def test_target_rectangle_checks_air_path_and_four_contacts(self):
        link = FakeCalibrationLink((198.668, 0.0, 230.477))
        answers = ["yes", "", "", "", "", ""]
        for _ in range(4):
            answers.extend(["+" * 32, ""])
        result = calibration.run_calibration(
            link, copy.deepcopy(CFG), target_half_extents_mm=(60, 37),
            input_fn=lambda _: answers.pop(0), output_fn=lambda _: None)
        self.assertEqual(result["result"], "ready")
        self.assertEqual(result["selected_half_size_mm"], [60.0, 37.0])
        self.assertEqual(len(result["samples"]), 4)
        self.assertEqual(result["tested_half_sizes_mm"], [[60.0, 37.0]])

    def test_off_center_rect_checks_air_path_and_four_contacts(self):
        link = FakeCalibrationLink((198.668, 0.0, 230.477))
        answers = ["yes", "", "", "", "", ""]
        for _ in range(4):
            answers.extend(["+" * 32, ""])
        result = calibration.run_calibration(
            link, copy.deepcopy(CFG), target_rect_mm=(-60, -65, 60, 55),
            input_fn=lambda _: answers.pop(0), output_fn=lambda _: None)
        self.assertEqual(result["result"], "ready")
        self.assertEqual(result["selected_half_size_mm"], [-60.0, -65.0, 60.0, 55.0])
        self.assertEqual(result["selected_rect_mm"], [-60.0, -65.0, 60.0, 55.0])
        self.assertEqual([s["paper_xy_mm"] for s in result["samples"]],
                         [[-60.0, -65.0], [60.0, -65.0], [60.0, 55.0], [-60.0, 55.0]])
        # 펜업 탐색이 실제로 y=-65 ~ 55 (로봇 Z = 중심 + y) 전체를 돌았는지
        zs = [float(re.search(r"Z([-\d.]+)", c).group(1)) for c in link.commands]
        self.assertAlmostEqual(min(zs), 230.477 - 65.0, places=2)
        self.assertAlmostEqual(max(zs), 230.477 + 55.0, places=2)
        with self.assertRaises(ValueError):
            calibration.run_calibration(link, copy.deepcopy(CFG), target_rect_mm=(-60, -65, 60, 70),
                                        input_fn=lambda _: "q", output_fn=lambda _: None)

    def test_contact_travel_stays_retracted_after_early_contact(self):
        link = FakeCalibrationLink((198.668, 0.0, 230.477))
        answers = ["yes", "", "", "", "", "", "", "q"]
        result = calibration.run_calibration(link, copy.deepcopy(CFG), start_mm=30,
                                             max_mm=30, input_fn=lambda _: answers.pop(0),
                                             output_fn=lambda _: None)
        self.assertEqual(result["result"], "aborted")
        xs = [float(re.search(r"X([-\d.]+)", cmd).group(1)) for cmd in link.commands]
        center_x = 198.668
        self.assertLessEqual(xs[7], center_x - 5.0 + 0.001)
        self.assertLessEqual(xs[-1], center_x - 7.5 + 0.001)

    def test_cancel_at_paper_alignment_sends_no_motion(self):
        link = FakeCalibrationLink((198.668, 0.0, 230.477))
        result = calibration.run_calibration(link, copy.deepcopy(CFG), input_fn=lambda _: "q",
                                             output_fn=lambda _: None)
        self.assertEqual(result["result"], "cancelled")
        self.assertEqual(link.commands, [])

    def test_complete_five_point_measurement_returns_plane_without_writing_config(self):
        cfg = copy.deepcopy(CFG)
        link = FakeCalibrationLink((198.668, 0.0, 230.477))
        answers = ["yes", "", "", "", "", ""]
        for _ in range(4):
            answers.extend(["++++++++++++++++++++", ""])
        result = calibration.run_calibration(link, cfg, start_mm=30, step_mm=5, max_mm=30,
                                             input_fn=lambda _: answers.pop(0),
                                             output_fn=lambda _: None)
        self.assertEqual(result["result"], "ready")
        self.assertEqual(result["selected_half_size_mm"], 30.0)
        self.assertEqual(len(result["samples"]), 4)
        self.assertAlmostEqual(result["ready_tcp_mm"]["x"], 198.668 - 5.0)
        self.assertAlmostEqual(result["plane_fit"]["a_per_mm_y"], 0.0, places=8)
        self.assertAlmostEqual(result["plane_fit"]["b_per_mm_z"], 0.0, places=8)
        self.assertEqual(cfg["plane_compensation"]["status"], "pending_measurement")
        self.assertEqual(len(link.commands), 1 + 5 + 1 + 4 * (1 + 20 + 1) + 1)

    def test_can_choose_last_safe_size_while_expanding(self):
        link = FakeCalibrationLink((198.668, 0.0, 230.477))
        answers = ["yes", "", "", "", "", "", "", "c"]
        for _ in range(4):
            answers.extend(["++++++++++++++++++++", ""])
        result = calibration.run_calibration(link, copy.deepcopy(CFG), start_mm=30, step_mm=5,
                                             max_mm=35, input_fn=lambda _: answers.pop(0),
                                             output_fn=lambda _: None)
        self.assertEqual(result["result"], "ready")
        self.assertEqual(result["selected_half_size_mm"], 30.0)
        self.assertEqual(result["tested_half_sizes_mm"], [30.0])

    def test_config_write_is_atomic_and_valid_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "drawing_config.json"
            calibration.save_config(path, {"calibration": {"status": "verified"}})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")),
                             {"calibration": {"status": "verified"}})
            self.assertFalse(path.with_name(path.name + ".tmp").exists())


if __name__ == "__main__":
    unittest.main()
