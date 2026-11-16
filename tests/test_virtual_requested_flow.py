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

    def run_job(self, cfg, virtual=True, air=False, recalibrate=False, outside=False, on_send=None, on_pause=None):
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
            if kind == "paused" and on_pause:   # 일시정지에서 사람이 고르는 것 (이어서·끝내기)
                on_pause(job)

        def open_link(config, virtual, speed):
            link = VirtualMirobotLink(config, speed=5000, homing_s=0)
            link.port = "COM9"
            if on_send:     # 보낸 명령마다 시험 쪽에서 [멈춤]을 누를 수 있게
                original = link.send_and_ack

                def send_and_ack(line, timeout, should_stop=None):
                    original(line, timeout, should_stop)
                    on_send(job, link, line)
                link.send_and_ack = send_and_ack
            holder["link"] = link
            return link
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

    def _stop_while_pressed(self, press_again_on_return=False, presses=1, on_pause=lambda job: job.finish()):
        """종이를 누른 채 그리는 명령에서 [멈춤]을 presses번, 일시정지가 되면 on_pause(job).
        반환: (결과, 연결, 멈춘 명령 번호, 접촉 X)."""
        contact_x = float(ms.fk(np.radians(CONTACT))[0][0, 3])
        marks = {}

        def on_send(job, link, line):
            m = M20_X.match(line)
            if "stop" not in marks and m and abs(float(m.group(1)) - (contact_x + PRESS_MM)) < 0.01:
                marks["stop"] = len(link.sent) - 1
                for _ in range(presses):
                    job.stop()
            elif press_again_on_return and "stop" in marks and "again" not in marks and line.startswith("M21"):
                marks["again"] = len(link.sent) - 1
                job.stop()

        result, link, _, _ = self.run_job(requested_cfg(), on_send=on_send, on_pause=on_pause)
        return result, link, marks["stop"], contact_x

    def test_stop_lifts_the_pen_straight_off_and_finish_returns_to_home(self):
        result, link, stop_at, contact_x = self._stop_while_pressed()
        self.assertEqual((result["result"], result["returned_to_origin"]), ("stopped_by_user", True))
        pressed, lift = link.sent[stop_at], link.sent[stop_at + 1]
        self.assertAlmostEqual(float(M20_X.match(lift).group(1)), contact_x - 20.0, places=2)   # 펜업 높이까지 뗌
        self.assertEqual(lift.split(" Y")[1].split(" A")[0], pressed.split(" Y")[1].split(" A")[0])  # 옆으로 가지 않음
        self.assertIn("F200", lift)                                                         # 종이에서 천천히
        after = link.sent[stop_at + 2:]
        self.assertTrue(after and all(line.startswith("M21") for line in after))            # 끝내기 뒤로는 관절 이동만
        self.assertIn("B-128", after[0])                                                    # 1번 대기 자세를 거쳐
        self.assertEqual(link.joints, HOME)                                                 # 호밍 자세(관절 0)

    def test_pressing_stop_twice_before_it_takes_effect_still_lifts_the_pen_and_returns(self):
        # 멈춤은 보내던 명령의 ok 뒤에야 걸려, 반응이 없어 보여 한 번 더 누르는 경우: 펜을 누른 채 남기지 않는다
        result, link, stop_at, contact_x = self._stop_while_pressed(presses=2)
        self.assertEqual((result["result"], result["returned_to_origin"]), ("stopped_by_user", True))
        self.assertAlmostEqual(float(M20_X.match(link.sent[stop_at + 1]).group(1)), contact_x - 20.0, places=2)
        self.assertEqual(link.joints, HOME)

    def test_stop_while_returning_after_finish_leaves_the_robot_where_it_is(self):
        result, link, stop_at, contact_x = self._stop_while_pressed(press_again_on_return=True)
        self.assertEqual((result["result"], result["returned_to_origin"]), ("stopped_by_user", False))
        self.assertIn("그 자리에", result["return_error"])
        self.assertEqual(len(link.sent), stop_at + 3)                # 펜 떼기, 첫 관절 이동 뒤로는 보내지 않음
        self.assertNotEqual(link.joints, HOME)

    def test_resume_puts_the_pen_back_where_it_stopped_and_completes(self):
        result, link, stop_at, contact_x = self._stop_while_pressed(on_pause=lambda job: job.resume())
        self.assertEqual((result["result"], result["returned_to_origin"], result["pauses"]), ("completed", True, 1))
        pressed, lift, slow, touch = link.sent[stop_at:stop_at + 4]
        yz = lambda line: line.split(" Y")[1].split(" A")[0]
        self.assertAlmostEqual(float(M20_X.match(lift).group(1)), contact_x - 20.0, places=2)
        self.assertEqual({yz(lift), yz(slow), yz(touch)}, {yz(pressed)})         # 옆으로 가지 않고 같은 점에서
        self.assertLess(float(M20_X.match(slow).group(1)), contact_x)               # 종이 앞까지 빠르게
        self.assertEqual(touch.split(" F")[0], pressed.split(" F")[0])            # 멈춘 접촉점에 다시 댐
        self.assertIn("F200", touch)                                                # 닿는 구간은 천천히
        self.assert_standby_contact_draw_return(link)

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
