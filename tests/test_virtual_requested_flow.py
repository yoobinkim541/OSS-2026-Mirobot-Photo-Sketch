"""저장된 대기(1번)·접촉(2번) 자세로 그리기: 가상 실행 순서와, 실물 연결에서 움직이기 전에 멈추는 조건."""

import re
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import mirobot_sim as ms  # noqa: E402
from mirobot_sketch.draw_job import DrawJob  # noqa: E402
from mirobot_sketch.session import SketchSession  # noqa: E402
from mirobot_sketch.virtual_robot import VirtualMirobotLink  # noqa: E402

import golden  # noqa: E402

HOME = [0.0] * 6
STANDBY = [0.0, 0.0, 0.0, 0.0, -128.0, 0.0]
PRESS_MM = 7.0                          # 사용자 요청: 접촉점보다 1 mm → 최소 6 mm 더 앞으로 (총 7 mm)
CONTACT = [0.0, 8.0, -8.0, 0.0, -128.0, 0.0]
CONTACT_TCP = [232.84, 0.0, 268.888]     # 사용자가 보낸 컨트롤러 상태 응답
CONTACT_LINE = "X0.000 Y8.000 Z-8.000 A0.000 B-128.000 C0.000"
M20_X = re.compile(r"^M20 G90 G0[01] X(-?[\d.]+)")


def requested_cfg(verified=False, first_gap_mm=18.0):
    """사용자가 2026-10-29에 측정한 대기·접촉 자세 (컨트롤러 상태 응답 포함). 1번 자세 펜 간격은 펜 방향으로 잰 값."""
    cfg = golden.default_cfg()
    cfg["plane_compensation"].update(a_per_mm_y=0.0, b_per_mm_z=0.0)
    cfg["requested_start_pose"] = {
        "joints_deg": STANDBY, "contact_joints_deg": CONTACT,
        "wall_distance_mm": 270.0, "pen_length_mm": 110.0, "tool_offset_mm": [-33.68, 1.76, 17.1],
        "first_pose_paper_gap_mm": first_gap_mm,
        "tool_offset_verified": verified, "requested_pen_down_extra_mm": PRESS_MM,
        "contact_status_sample": {"joints_deg": [0, 8.002, -8, 0, -127.992, 0],
                                  "tcp_mm": CONTACT_TCP, "abc_deg": [0, -127.99, 0]}}
    return cfg


class RequestedFlowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        cls.pic = cls.root / "line.png"
        cv2.imwrite(str(cls.pic), golden.synthetic_images()["line"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_job(self, cfg, virtual=True, air=False, recalibrate=False, outside=False):
        session = SketchSession(cfg)
        session.set_image(self.pic)
        session.update_params({"box_mm": 10, "epsilon_px": 4.0})
        session.run_current()
        ready, finished, events, holder = threading.Event(), threading.Event(), [], {}

        def callback(kind, **data):
            events.append((kind, data))
            if kind == "step" and data["id"] == "confirm" and data["status"] == "active":
                ready.set()
            if kind == "finished":
                finished.set()

        def open_link(config, virtual, speed):
            holder["link"] = VirtualMirobotLink(config, speed=5000, homing_s=0)
            holder["link"].port = "COM9"
            return holder["link"]
        job = DrawJob(session, cfg, callback, progress_path=self.root / "progress.json")
        if outside:         # 사전 검사(넓은 범위)는 통과하지만 펜 접촉 범위(limits)는 벗어나는 그림
            cfg["limits"] = {"x_max_mm": 1.0, "y_min_mm": -1.0, "roof_mm": [[0, 1.0], [1.0, 1.0]]}
        with mock.patch.object(de, "open_link", side_effect=open_link) as opened, \
                mock.patch.object(de.paths, "runs_dir", return_value=self.root / "runs"), \
                mock.patch.object(de.paths, "output_dir", return_value=self.root), \
                mock.patch.object(job, "_open_rviz"):
            job.start(virtual=virtual, virtual_speed=5000, start_pose=True, recalibrate=recalibrate, pending=outside)
            if ready.wait(30):
                job.confirm(air=air, checked=True)
            self.assertTrue(finished.wait(60), events[-5:])
        result = next(d["result"] for k, d in events if k == "finished")
        failed = [d["message"] for k, d in events if k == "step" and d["status"] == "failed"]
        return result, holder.get("link"), opened, failed

    def assert_standby_contact_draw_return(self, link):
        joint_moves = [line for line in link.sent if line.startswith("M21")]
        self.assertIn("B-128", joint_moves[0] + joint_moves[1])                     # 1번 대기 자세로
        contact = next(i for i, line in enumerate(link.sent) if CONTACT_LINE in line)
        drawing = [i for i, line in enumerate(link.sent) if line.startswith("M20")]
        self.assertLess(contact, drawing[0])                                         # 2번 접촉 자세에서 그리기 시작
        standby = next(i for i, line in enumerate(link.sent) if i > drawing[-1] and "X0.000 Y0.000 Z0.000 A0.000 B-128" in line)
        self.assertLess(drawing[-1], standby)                                        # 끝나면 1번 자세를 거쳐
        self.assertEqual(link.joints, HOME)                                          # 호밍 자세(관절 0)로
        contact_x = float(ms.fk(np.radians(CONTACT))[0][0, 3])                    # 접촉 자세에서 보고된 TCP
        xs = [float(M20_X.match(line).group(1)) for line in link.sent if M20_X.match(line)]
        self.assertAlmostEqual(max(xs), contact_x + PRESS_MM, places=2)             # 접촉점보다 더 눌러 그림
        self.assertAlmostEqual(min(xs), contact_x - 20.0, places=2)                 # 획 사이에는 종이 면에서 20 mm 뗌
        descend = [float(M20_X.match(line).group(1)) for line in link.sent if M20_X.match(line) and "F600" in line]
        self.assertLess(max(descend), contact_x)                                     # 종이 면 앞에서부터는 느리게 접근
        lifts = [line for line in link.sent if line.startswith("M20") and "F200" in line]
        self.assertTrue(lifts)                                                       # 뗄 때는 접근 속도로 천천히
        self.assertLess(float(M20_X.match(lifts[0]).group(1)), contact_x)            # 종이 면을 지난 뒤에야 빠르게 이동

    def test_virtual_moves_standby_then_contact_draws_and_returns(self):
        result, link, _, _ = self.run_job(requested_cfg())
        self.assertEqual(result["result"], "completed")
        self.assertTrue(result["returned_to_origin"])
        self.assert_standby_contact_draw_return(link)

    def test_air_mode_never_goes_to_the_contact_pose(self):
        result, link, _, _ = self.run_job(requested_cfg(), air=True)
        self.assertEqual(result["result"], "completed")
        self.assertFalse(any(CONTACT_LINE in line for line in link.sent))
        self.assertEqual(link.joints, HOME)

    def test_unverified_real_robot_stops_before_opening_the_port(self):
        result, _, opened, failed = self.run_job(requested_cfg(verified=False), virtual=False)
        self.assertEqual(result["result"], "not_started")
        opened.assert_not_called()
        self.assertIn("움직이지 않았습니다", failed[0])

    def test_first_pose_gap_mismatch_stops_real_robot_even_when_marked_verified(self):
        result, _, opened, failed = self.run_job(requested_cfg(verified=True, first_gap_mm=30.0), virtual=False)
        self.assertEqual(result["result"], "not_started")
        opened.assert_not_called()
        self.assertIn("첫 자세 펜 간격", failed[0])

    def test_recalibration_is_refused_before_connecting(self):
        result, _, opened, _ = self.run_job(requested_cfg(verified=True), virtual=False,
                                            recalibrate=True)
        self.assertEqual(result["result"], "not_started")
        opened.assert_not_called()

    def test_verified_real_path_runs_the_same_sequence(self):
        result, link, opened, _ = self.run_job(requested_cfg(verified=True), virtual=False)
        opened.assert_called_once()
        self.assertEqual(result["result"], "completed")
        self.assertTrue(result["returned_to_origin"])
        self.assert_standby_contact_draw_return(link)

    def test_out_of_range_drawing_stops_before_touching_the_paper(self):
        result, link, _, _ = self.run_job(requested_cfg(verified=True), virtual=False,
                                          outside=True)
        self.assertEqual(result["result"], "not_started")
        self.assertFalse(any(CONTACT_LINE in line for line in link.sent))


if __name__ == "__main__":
    unittest.main()
