"""카메라 화면 ↔ 종이 좌표 맞추기 (손으로 찍은 귀퉁이 4개)."""

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import limits, vision  # noqa: E402

import golden  # noqa: E402

# 왼쪽에서 비스듬히 본 A4: 가까운(왼쪽) 변이 더 김
CLICKS = [(120, 90), (520, 140), (500, 380), (100, 420)]
SIZE = (640, 480)


class RegistrationTest(unittest.TestCase):
    def reg(self, paper=(297.0, 210.0)):
        return vision.Registration.from_pixels("Logitech BRIO", CLICKS, SIZE, paper)

    def test_paper_corners_land_on_the_clicked_points(self):
        reg = self.reg()
        got = vision.project(reg.matrix(SIZE), vision.paper_corners_mm(reg.paper_mm))
        np.testing.assert_allclose(got, CLICKS, atol=1e-3)

    def test_paper_center_is_where_the_diagonals_cross(self):
        # 원근에서는 종이 중심이 네 점 평균이 아니라 대각선 교점에 찍혀야 함
        p = np.array(CLICKS, float)
        d1, d2 = p[2] - p[0], p[3] - p[1]
        cross = lambda a, b: a[0] * b[1] - a[1] * b[0]  # noqa: E731
        t = cross(p[1] - p[0], d2) / cross(d1, d2)
        crossing = p[0] + t * d1
        center = vision.project(self.reg().matrix(SIZE), [(0.0, 0.0)])[0]
        np.testing.assert_allclose(center, crossing, atol=1e-3)
        self.assertGreater(np.linalg.norm(center - p.mean(axis=0)), 3)

    def test_corrected_registration_puts_plan_points_where_the_robot_actually_drew(self):
        reg = self.reg()
        th = np.radians(-2.0)                                            # 그린 선이 등록보다 오른쪽 7·아래 6 mm, −2°, 1.05배
        a = np.array([[1.05 * np.cos(th), -1.05 * np.sin(th), 7.0], [1.05 * np.sin(th), 1.05 * np.cos(th), -6.0]])
        new = vision.corrected_registration(reg, SIZE, a)
        pts = np.array([(0.0, 0.0), (50.0, -30.0), (-80.0, 40.0)])
        np.testing.assert_allclose(vision.project(new.matrix(SIZE), pts),
                                   vision.project(reg.matrix(SIZE), pts @ a[:, :2].T + a[:, 2]), atol=0.05)
        self.assertEqual((new.camera, new.paper_mm, new.frame_size), (reg.camera, reg.paper_mm, SIZE))

    def test_same_registration_works_at_another_resolution(self):
        reg = self.reg()
        small = vision.project(reg.matrix(SIZE), [(50.0, -30.0)])[0]
        big = vision.project(reg.matrix((1280, 960)), [(50.0, -30.0)])[0]
        np.testing.assert_allclose(big, small * 2, atol=1e-3)

    def test_paper_size_changes_where_a_mm_point_lands(self):
        corner_a4 = vision.project(self.reg().matrix(SIZE), [(148.5, 105.0)])[0]
        inner = vision.project(self.reg((400.0, 300.0)).matrix(SIZE), [(148.5, 105.0)])[0]
        np.testing.assert_allclose(corner_a4, CLICKS[1], atol=1e-3)
        self.assertGreater(np.linalg.norm(inner - np.array(CLICKS[1])), 20)

    def test_registration_from_another_aspect_ratio_is_not_used(self):
        # 640×480(4:3)에서 찍은 등록을 4K(16:9)에 쓰면 보이는 범위가 달라 계획이 엉뚱한 곳에 겹친다
        reg = self.reg()
        self.assertEqual(reg.frame_size, SIZE)
        self.assertTrue(reg.fits((1280, 960)))
        self.assertFalse(reg.fits((3840, 2160)))

    def test_old_saved_registration_without_frame_size_still_loads(self):
        old = {"camera": "Logitech BRIO", "corners": [[0.1, 0.2], [0.9, 0.2], [0.9, 0.8], [0.1, 0.8]],
               "paper_mm": [297.0, 210.0], "saved_at": "2026-11-10T18:01:51+09:00"}
        reg = vision.Registration.from_dict(old)
        self.assertEqual(reg.frame_size, ())
        self.assertTrue(reg.fits((3840, 2160)))
        r = self.reg()
        self.assertEqual(vision.Registration.from_dict(r.to_dict()), r)     # 새 등록은 크기까지 저장했다 읽음


class CornerCheckTest(unittest.TestCase):
    def test_good_quad_passes(self):
        self.assertIsNone(vision.check_corners(CLICKS, SIZE))

    def test_crossed_order_is_rejected(self):
        crossed = [CLICKS[0], CLICKS[2], CLICKS[1], CLICKS[3]]
        self.assertIn("꼬였습니다", vision.check_corners(crossed, SIZE))

    def test_counter_clockwise_order_is_rejected(self):
        self.assertIn("거꾸로", vision.check_corners(CLICKS[::-1], SIZE))

    def test_tiny_quad_is_rejected(self):
        tiny = [(10, 10), (30, 10), (30, 30), (10, 30)]
        self.assertIn("작습니다", vision.check_corners(tiny, SIZE))

    def test_points_outside_the_frame_are_rejected(self):
        outside = [(-40, -30), (500, 20), (480, 400), (10, 380)]
        self.assertIn("밖", vision.check_corners(outside, SIZE))

    def test_first_point_must_be_the_top_left(self):
        rotated = [CLICKS[3], CLICKS[0], CLICKS[1], CLICKS[2]]
        self.assertIn("왼쪽 위", vision.check_corners(rotated, SIZE))

    def test_short_side_is_rejected(self):
        thin = [(100, 400), (1800, 420), (1780, 480), (120, 470)]
        self.assertIn("짧", vision.check_corners(thin, (1920, 1080)))

    def test_aspect_far_from_the_paper_is_rejected(self):
        wide = [(40, 160), (600, 150), (590, 250), (50, 260)]
        self.assertIn("비율", vision.check_corners(wide, SIZE))

    def test_sharp_corner_is_rejected(self):
        sharp = [(90, 50), (520, 70), (500, 400), (200, 180)]
        self.assertIn("각", vision.check_corners(sharp, SIZE))


class StoreTest(unittest.TestCase):
    def test_each_camera_keeps_its_own_registration(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.object(vision.paths, "user_dir", lambda: Path(d)):
            brio = vision.Registration.from_pixels("Logitech BRIO", CLICKS, SIZE)
            webcam = vision.Registration.from_pixels("USB2.0 HD UVC WebCam", [(0, 0), (600, 0), (600, 400), (0, 400)],
                                                     SIZE)
            vision.save(brio)
            vision.save(webcam)
            self.assertEqual(vision.load("Logitech BRIO"), brio)
            self.assertEqual(vision.load("USB2.0 HD UVC WebCam"), webcam)
            self.assertIsNone(vision.load("OBS Virtual Camera"))
            vision.forget("Logitech BRIO")
            self.assertIsNone(vision.load("Logitech BRIO"))
            self.assertEqual(vision.load("USB2.0 HD UVC WebCam"), webcam)

    def test_broken_file_means_not_registered(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.object(vision.paths, "user_dir", lambda: Path(d)):
            (Path(d) / vision.SETTINGS_FILE).write_text("{not json", encoding="utf-8")
            self.assertIsNone(vision.load("Logitech BRIO"))


class OverlayTest(unittest.TestCase):
    def test_strokes_are_drawn_where_they_project_and_input_is_untouched(self):
        frame = np.full((SIZE[1], SIZE[0], 3), 255, np.uint8)
        reg = vision.Registration.from_pixels("cam", CLICKS, SIZE)
        stroke = [np.array([(-20.0, 0.0), (20.0, 0.0)])]
        regions = ((limits.executor_region(golden.default_cfg()), vision.PEN_RANGE_COLOR),)
        out = vision.Overlay().draw(frame, reg, stroke, regions)
        self.assertTrue((frame == 255).all())
        mid = np.round(vision.project(reg.matrix(SIZE), [(0.0, 5.0)])[0]).astype(int)   # 획에서 5 mm 떨어진 곳
        on = np.round(vision.project(reg.matrix(SIZE), [(10.0, 0.0)])[0]).astype(int)
        np.testing.assert_allclose(out[on[1], on[0]], vision.STROKE_COLOR, atol=40)   # 선 위 (가장자리 부드럽게 칠함)
        self.assertEqual(tuple(out[mid[1], mid[0]]), (255, 255, 255))

    def test_snapshot_puts_photo_and_overlay_side_by_side(self):
        import cv2
        frame = np.full((SIZE[1], SIZE[0], 3), 200, np.uint8)
        reg = vision.Registration.from_pixels("cam", CLICKS, SIZE)
        with tempfile.TemporaryDirectory() as d:
            path = vision.save_snapshot(frame, reg, [np.array([(-20.0, 0.0), (20.0, 0.0)])], (),
                                        Path(d) / "한글" / "vision.png")
            saved = cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(saved.shape, (SIZE[1], SIZE[0] * 2, 3))
        self.assertTrue((saved[:, :SIZE[0]] == 200).all())


class LineCheckTest(unittest.TestCase):
    """연한 펜 선이 있는 획은 보이고, 안 그린 획은 안 보이고, 팔·홀더(색 있는 큰 물체)에 가린 획은 판정하지 않음."""

    BIG = (2560, 1920)                       # A4가 약 5 px/mm로 찍힘 (4K 카메라와 비슷)

    def setUp(self):
        import cv2
        self.reg = vision.Registration.from_pixels("cam", [(x * 4, y * 4) for x, y in CLICKS], self.BIG)
        self.strokes = [np.array([(-40.0, 30.0), (0.0, 35.0), (40.0, 30.0)]),     # 그림
                        np.array([(-40.0, 0.0), (40.0, 0.0)]),                     # 안 그림
                        np.array([(-40.0, -35.0), (40.0, -35.0)])]                 # 그렸지만 홀더에 가림
        m = self.reg.matrix(self.BIG)
        frame = np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8)
        for i in (0, 2):
            pts = np.round(vision.project(m, self.strokes[i])).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [pts], False, (196, 196, 196), 2, cv2.LINE_AA)     # 종이보다 4%만 어두운 선
        holder = np.round(vision.project(m, [(-55, -25), (55, -25), (55, -48), (-55, -48)])).astype(np.int32)
        cv2.fillPoly(frame, [holder], (40, 60, 200))
        self.frames = [np.clip(frame + np.random.default_rng(k).normal(0, 2.0, frame.shape), 0, 255).astype(np.uint8)
                       for k in range(3)]

    def test_drawn_missing_and_hidden_strokes(self):
        check = vision.LineCheck(self.reg, self.strokes, self.BIG)
        for f in self.frames:
            check.add(f)
        got = {i: check.measure(i) for i in range(3)}
        self.assertGreaterEqual(got[0], vision.LINE_SEEN)
        self.assertLess(got[1], vision.LINE_SEEN)
        self.assertIsNone(got[2])
        self.assertEqual(vision.line_summary(got), (1, 1, 1))

    def test_frame_of_another_size_is_refused(self):
        check = vision.LineCheck(self.reg, self.strokes, self.BIG)
        with self.assertRaises(ValueError):
            check.add(self.frames[0][::2, ::2])

    def test_dark_pen_line_is_measured_visible(self):
        import cv2
        stroke = np.array([(-40.0, 20.0), (40.0, 20.0)])
        frame = np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8)
        pts = np.round(vision.project(self.reg.matrix(self.BIG), stroke)).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(frame, [pts], False, (155, 155, 155), 2, cv2.LINE_AA)
        check = vision.LineCheck(self.reg, [stroke], self.BIG)
        check.add(frame)
        got = check.measure(0)
        self.assertIsNotNone(got)
        self.assertGreaterEqual(got, vision.LINE_SEEN)

    def test_warm_paper_is_not_occluded(self):
        stroke = np.array([(-40.0, 20.0), (40.0, 20.0)])
        frame = np.full((self.BIG[1], self.BIG[0], 3), (180, 200, 230), np.uint8)
        check = vision.LineCheck(self.reg, [stroke], self.BIG)
        check.add(frame)
        self.assertIsNotNone(check.measure(0))

    def test_thick_arm_blob_is_still_occluded(self):
        import cv2
        strokes = [np.array([(-40.0, 20.0), (40.0, 20.0)]),
                   np.array([(-40.0, -40.0), (40.0, -40.0)])]
        frame = np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8)
        blob = np.round(vision.project(self.reg.matrix(self.BIG),
                                       [(-55, 10), (55, 10), (55, 30), (-55, 30)])).astype(np.int32)
        cv2.fillPoly(frame, [blob], (30, 30, 30))
        check = vision.LineCheck(self.reg, strokes, self.BIG)
        check.add(frame)
        self.assertIsNone(check.measure(0))


class EndTickTest(unittest.TestCase):
    """획 시작·끝점에서 세로로 뻗은 줄(펜을 세게 대거나 뗄 때 미끄러진 자국)을 재고, 획 자체·옆 획은 세지 않음."""

    BIG = LineCheckTest.BIG

    def setUp(self):
        import cv2
        self.reg = vision.Registration.from_pixels("cam", [(x * 4, y * 4) for x, y in CLICKS], self.BIG)
        self.strokes = [np.array([(-40.0, 30.0), (40.0, 30.0)]),        # 시작점 아래로 4 mm 세로 줄
                        np.array([(-40.0, 0.0), (40.0, 0.0)]),          # 깨끗한 획
                        np.array([(-30.0, -40.0), (-30.0, -20.0)])]     # 세로 획: 획 자체는 세로 줄이 아님
        m = self.reg.matrix(self.BIG)
        frame = np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8)
        lines = [*self.strokes, np.array([(-40.0, 30.0), (-40.0, 26.0)])]     # 첫 획 시작점의 세로 줄
        for s in lines:
            pts = np.round(vision.project(m, s)).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [pts], False, (196, 196, 196), 2, cv2.LINE_AA)     # 실제처럼 연한 펜 선
        self.frame = np.clip(frame + np.random.default_rng(0).normal(0, 2.0, frame.shape), 0, 255).astype(np.uint8)

    def test_tick_found_only_at_the_marked_end(self):
        check = vision.LineCheck(self.reg, self.strokes, self.BIG)
        check.add(self.frame)
        start, end = check.end_ticks(0)
        self.assertGreaterEqual(start, vision.TICK_MIN_MM)
        self.assertAlmostEqual(start, 4.0, delta=1.0)
        self.assertLess(end, vision.TICK_MIN_MM)
        for i in (1, 2):
            self.assertTrue(all(t < vision.TICK_MIN_MM for t in check.end_ticks(i)), i)
        ticks = {i: check.end_ticks(i) for i in range(3)}
        cov = {i: check.measure(i) for i in range(3)}
        self.assertEqual(vision.tick_summary(ticks, cov), {"start": (1, 3), "end": (0, 3)})

    def test_tick_direction_is_split_into_up_and_down_on_the_paper(self):
        check = vision.LineCheck(self.reg, self.strokes, self.BIG)
        check.add(self.frame)
        (up, down), end = check.end_tick_dirs(0)              # 첫 획 시작점: 종이 아래로 4mm
        self.assertAlmostEqual(down, 4.0, delta=1.0)
        self.assertLess(up, vision.TICK_MIN_MM)
        self.assertLess(max(end), vision.TICK_MIN_MM)
        self.assertEqual(check.end_tick_dirs(2), (None, None))  # 세로 획 끝은 위·아래를 가를 수 없음

    def _shifted_frame(self, dx, dy):
        import cv2
        m = self.reg.matrix(self.BIG)
        frame = np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8)
        for s in [*self.strokes, np.array([(-40.0, 30.0), (-40.0, 26.0)])]:
            pts = np.round(vision.project(m, s + (dx, dy))).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [pts], False, (196, 196, 196), 2, cv2.LINE_AA)
        return np.clip(frame + np.random.default_rng(1).normal(0, 2.0, frame.shape), 0, 255).astype(np.uint8)

    def test_align_finds_a_paper_offset_before_measuring(self):
        check = vision.LineCheck(self.reg, self.strokes, self.BIG)
        check.add(self._shifted_frame(5.0, -4.0))                 # 종이를 다시 붙여 그림이 5mm 오른쪽·4mm 아래
        self.assertLess(check.measure(1), vision.LINE_SEEN)        # 맞추기 전: 그린 획도 안 보임으로 나옴
        score = check.align()
        self.assertGreater(score, 0.8)
        self.assertAlmostEqual(check.shift_mm[0], 5.0, delta=0.4)
        self.assertAlmostEqual(check.shift_mm[1], -4.0, delta=0.4)
        self.assertTrue(all(check.measure(i) >= vision.LINE_SEEN for i in range(3)))
        (up, down), _ = check.end_tick_dirs(0)
        self.assertAlmostEqual(down, 4.0, delta=1.0)               # 맞춘 뒤에도 세로 줄은 그대로 잡힘

    def test_align_leaves_a_blank_paper_alone(self):
        check = vision.LineCheck(self.reg, self.strokes, self.BIG)
        check.add(np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8))
        self.assertLess(check.align(), vision.ALIGN_MIN_SCORE)
        self.assertEqual(check.shift_mm, (0.0, 0.0))

    def test_align_finds_a_rotated_and_scaled_camera(self):
        # 11-18: 카메라가 틀어져 그린 선이 등록보다 +16, −12 mm, −2°, 1.06배. 이동만 찾으면 겹침이 낮아 확인·보정이 멈췄다
        import cv2
        rng = np.random.default_rng(3)
        strokes = [np.array([(x, y), (x + rng.uniform(8, 25), y + rng.uniform(-3, 3))])
                   for x, y in rng.uniform(-45, 30, (17, 2))]
        strokes += [np.array([(x, y), (x + rng.uniform(-3, 3), y + rng.uniform(8, 25))])
                    for x, y in rng.uniform(-45, 30, (17, 2))]
        th = np.radians(-2.0)
        a = np.array([[1.06 * np.cos(th), -1.06 * np.sin(th), 16.0], [1.06 * np.sin(th), 1.06 * np.cos(th), -12.0]])
        m = self.reg.matrix(self.BIG)
        frame = np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8)
        for s in strokes:
            pts = np.round(vision.project(m, s @ a[:, :2].T + a[:, 2])).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [pts], False, (196, 196, 196), 2, cv2.LINE_AA)
        frame = np.clip(frame + rng.normal(0, 2.0, frame.shape), 0, 255).astype(np.uint8)
        check = vision.LineCheck(self.reg, strokes, self.BIG)
        check.add(frame)
        self.assertGreater(check.align(), 0.8)
        self.assertAlmostEqual(check.rotation_deg, -2.0, delta=0.6)
        self.assertAlmostEqual(check.scale, 1.06, delta=0.015)
        plan = np.vstack(strokes)
        err = np.linalg.norm(plan @ check.affine_mm[:, :2].T + check.affine_mm[:, 2] - (plan @ a[:, :2].T + a[:, 2]), axis=1)
        self.assertLess(err.max(), 1.0)                              # 그린 자리를 1 mm 안으로 맞춤
        self.assertGreater(np.mean([check.measure(i) for i in range(len(strokes))]), 0.8)
        early = vision.LineCheck(self.reg, strokes, self.BIG)       # 그리는 중 앞부분(획 10개): 이동만 맞춤
        early.add(frame)
        early.align(upto=10)
        self.assertEqual((early.rotation_deg, early.scale), (0.0, 1.0))

    def _diagonal(self, extra):
        import cv2
        direction = np.array([np.cos(np.radians(60)), np.sin(np.radians(60))])
        start = np.array([-12.0, -8.0])
        end = start + direction * 36.0
        stroke = np.vstack([start, end])
        frame = np.full((self.BIG[1], self.BIG[0], 3), 205, np.uint8)
        m = self.reg.matrix(self.BIG)
        for s in (stroke, extra(end, direction)):
            pts = np.round(vision.project(m, s)).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [pts], False, (196, 196, 196), 2, cv2.LINE_AA)
        check = vision.LineCheck(self.reg, [stroke], self.BIG)
        check.add(frame)
        return check.end_ticks(0)

    def test_extension_along_a_diagonal_stroke_is_not_a_tick(self):
        _start, end = self._diagonal(lambda end, direction: np.vstack([end, end + direction * 3.0]))
        self.assertLess(end, vision.TICK_MIN_MM)

    def test_perpendicular_tick_on_a_diagonal_stroke_is_detected(self):
        _start, end = self._diagonal(
            lambda end, direction: np.vstack([end, end + np.array([-direction[1], direction[0]]) * 4.0]))
        self.assertGreaterEqual(end, vision.TICK_MIN_MM)


class PenCorrectionTest(unittest.TestCase):
    from mirobot_sketch import pen_correction as pc

    def test_frequent_start_ticks_back_off_only_the_start(self):
        ticks = {i: (3.0 if i % 2 else 0.0, 0.0) for i in range(10)}
        cov = {i: 0.9 for i in range(10)}
        new, why = self.pc.next_backoff({"start": 0.0, "end": 0.2}, ticks, cov)
        self.assertEqual(new, {"start": 0.2, "end": 0.2})
        self.assertIn("시작점", why)
        # 내릴 때 값이 시작점 줄을 맡으면 시작점 물림은 그 줄로 올리지 않음 (같은 줄로 두 값이 움직여 엇갈림 멈춤이 걸리지 않게)
        self.assertEqual(self.pc.next_backoff({"start": 0.0, "end": 0.2}, ticks, cov, kinds=("end",)),
                         ({"start": 0.0, "end": 0.2}, ""))

    def test_too_few_endpoints_or_rare_ticks_change_nothing(self):
        cur = {"start": 0.4, "end": 0.4}
        few = {i: (3.0, 3.0) for i in range(self.pc.MIN_ENDPOINTS - 1)}
        self.assertEqual(self.pc.next_backoff(cur, few, {i: 0.9 for i in few}), (cur, ""))
        rare = {i: (3.0 if i == 0 else 0.0, 0.0) for i in range(20)}
        self.assertEqual(self.pc.next_backoff(cur, rare, {i: 0.9 for i in rare}), (cur, ""))

    def test_reduce_waits_until_min_endpoints(self):
        cur = {"start": 0.4, "end": 0.4}
        few = {i: (0.0, 0.0) for i in range(self.pc.MIN_ENDPOINTS // 2)}
        self.assertEqual(self.pc.next_backoff(cur, few, {i: 0.0 for i in few}), (cur, ""))
        enough = {i: (0.0, 0.0) for i in range(self.pc.MIN_ENDPOINTS)}
        new, why = self.pc.next_backoff(cur, enough, {i: 0.0 for i in enough})
        self.assertEqual(new, {"start": 0.2, "end": 0.2})
        self.assertIn("덜 물림", why)

    def test_missing_lines_undo_the_backoff_only_when_worse_than_before(self):
        ticks = {i: (3.0, 3.0) for i in range(20)}
        cov = {i: (0.1 if i < 8 else 0.9) for i in range(20)}             # 이번 구간 40% 안 보임
        new, why = self.pc.next_backoff({"start": 0.4, "end": 0.1}, ticks, cov, baseline_missing=0.1)
        self.assertEqual(new, {"start": 0.2, "end": 0.0})
        self.assertIn("덜 물림", why)
        # 앞 구간도 비슷하게 안 보였으면(종이 위치 오차·연한 선) 물림 탓이 아님: 세로 줄 규칙대로 더 물림
        new, _ = self.pc.next_backoff({"start": 0.4, "end": 0.1}, ticks, cov, baseline_missing=0.35)
        self.assertEqual(new, {"start": 0.6, "end": 0.3})

    def test_backoff_never_exceeds_the_planner_limit(self):
        from mirobot_sketch.draw_executor import END_BACKOFF_MAX_MM
        ticks = {i: (3.0, 3.0) for i in range(10)}
        new, _ = self.pc.next_backoff({"start": END_BACKOFF_MAX_MM, "end": END_BACKOFF_MAX_MM - 0.1}, ticks,
                                      {i: 0.9 for i in range(10)})
        self.assertEqual(new, {"start": END_BACKOFF_MAX_MM, "end": END_BACKOFF_MAX_MM})

    def test_downward_ticks_raise_the_slant_and_upward_ticks_lower_it(self):
        cov = {i: 0.9 for i in range(40)}
        down = {i: (None, (0.0, 3.0) if i < 12 else (0.0, 0.0)) for i in range(40)}     # 40끝점 중 아래 12
        new, why = self.pc.next_edge_rise(2.0, down, cov)
        self.assertEqual(new, 2.5)
        self.assertIn("아래로", why)
        up = {i: (None, (3.0, 0.0) if i < 12 else (0.0, 0.0)) for i in range(40)}
        new, why = self.pc.next_edge_rise(2.0, up, cov)
        self.assertEqual(new, 1.5)
        self.assertIn("위로", why)
        self.assertEqual(self.pc.next_edge_rise(self.pc.RISE_STEP_MM, up, cov)[0], self.pc.RISE_STEP_MM)  # 꺼지지 않음
        start_only = {i: ((0.0, 3.0), (0.0, 0.0)) for i in range(40)}     # 시작점 아래 줄은 뗄 때 값과 무관
        self.assertEqual(self.pc.next_edge_rise(2.0, start_only, cov), (2.0, ""))

    def test_slant_left_alone_when_off_balanced_or_unseen(self):
        cov = {i: 0.9 for i in range(40)}
        both = {i: (None, (3.0, 3.0) if i < 16 else (0.0, 0.0)) for i in range(40)}
        down = {i: (None, (0.0, 3.0) if i < 16 else (0.0, 0.0)) for i in range(40)}
        self.assertEqual(self.pc.next_edge_rise(2.0, both, cov), (2.0, ""))          # 양쪽 다: 방향 문제 아님
        self.assertEqual(self.pc.next_edge_rise(0.0, down, cov), (0.0, ""))          # 꺼 둠: 명령 수를 안 바꿈
        self.assertEqual(self.pc.next_edge_rise(2.0, down, {i: 0.1 for i in range(40)}), (2.0, ""))  # 선이 안 보인 획

    def test_start_ticks_move_the_touch_down_slant(self):
        # 시작점 아래 줄이 많으면 더 위에서 내려오며, 위 줄이 많으면 덜. 끝점 줄은 이 값과 상관없음
        cov = {i: 0.9 for i in range(40)}
        down = {i: ((0.0, 3.0) if i < 12 else (0.0, 0.0), (0.0, 3.0)) for i in range(40)}
        new, why = self.pc.next_edge_land(-2.0, 1.5, down, cov)
        self.assertEqual(new, -2.5)
        self.assertIn("시작점", why)
        up = {i: ((3.0, 0.0) if i < 12 else (0.0, 0.0), None) for i in range(40)}
        self.assertEqual(self.pc.next_edge_land(-2.0, 1.5, up, cov)[0], -1.5)
        self.assertEqual(self.pc.next_edge_land(0.0, 1.5, up, cov), (0.0, ""))        # 0 위로는 안 감
        self.assertEqual(self.pc.next_edge_land(-3.0, 1.5, down, cov), (-3.0, ""))    # 끝(−3)에서 멈춤
        self.assertEqual(self.pc.next_edge_land(-2.0, 0.0, down, cov), (-2.0, ""))    # 뗄 때 꺼짐: 명령 수를 안 바꿈
        end_only = {i: ((0.0, 0.0), (0.0, 3.0)) for i in range(40)}
        self.assertEqual(self.pc.next_edge_land(-2.0, 1.5, end_only, cov), (-2.0, ""))

    def test_slant_waits_for_enough_endpoints_and_a_clear_majority(self):
        # 11-18 그림들: 구간마다 시작점 10~24개(15:53 앱 표시 "아래 5/24, 위 2")로 판단해 같은 −1 mm를 그림마다 반대로 봤다
        def starts(n, up, down):
            return {i: ((3.0, 0.0) if i < up else (0.0, 3.0) if i < up + down else (0.0, 0.0), None) for i in range(n)}
        cov = {i: 0.9 for i in range(120)}
        self.assertEqual(self.pc.next_edge_land(-1.0, 1.75, starts(24, 2, 5), cov), (-1.0, ""))   # 끝점이 모자람
        self.assertEqual(self.pc.next_edge_land(-1.0, 1.75, starts(40, 2, 6), cov), (-1.0, ""))   # 6 대 2는 우연일 수 있음
        self.assertEqual(self.pc.next_edge_land(-1.0, 1.75, starts(40, 6, 2), cov), (-1.0, ""))
        # 20:44 그림을 끝난 뒤 다시 잰 −0.75 구간: 101개 중 위 10·아래 21 → 더 위에서 내려오며 대기로
        self.assertEqual(self.pc.next_edge_land(-0.75, 1.75, starts(101, 10, 21), cov)[0], -1.25)

    def test_lower_paper_slant_follows_the_lower_endpoints_only(self):
        # 획마다 시작·끝이 같은 높이: 0~39번은 위쪽(y 10), 40~79번은 아래쪽(y −60)
        strokes = [[(0.0, 10.0), (5.0, 10.0)]] * 40 + [[(0.0, -60.0), (5.0, -60.0)]] * 40
        cov = {i: 0.9 for i in range(80)}
        dirs = {i: ((0.0, 0.0), (0.0, 3.0 if (i >= 40 and i % 4 == 0) else 0.0)) for i in range(80)}   # 아래쪽만 아래 줄
        new, why = self.pc.next_edge_rise_low(0.0, 2.0, dirs, cov, strokes)
        self.assertEqual(new, 0.5)
        self.assertIn("아래쪽", why)
        self.assertEqual(self.pc.next_edge_rise(2.0, dirs, cov, strokes), (2.0, ""))   # 위쪽 값은 그대로
        self.assertEqual(self.pc.next_edge_rise_low(1.0, 2.0, dirs, cov, strokes)[0], 1.0)  # 합 3 mm에서 멈춤
        self.assertEqual(self.pc.next_edge_rise_low(0.5, 0.0, dirs, cov, strokes), (0.5, ""))  # 꺼 둠
        up = {i: ((0.0, 0.0), (3.0 if (i >= 40 and i % 4 == 0) else 0.0, 0.0)) for i in range(80)}
        self.assertEqual(self.pc.next_edge_rise_low(1.0, 2.0, up, cov, strokes)[0], 0.5)

    def test_back_and_forth_settles_in_the_middle_and_locks_that_value(self):
        # 15:53 그림: 댈 때 −2 → −1.5 → −1(위 줄이 많다고 봄) 다음 아래 줄로 −1.5를 원함 → 그 사이(−1.25)로 고정
        old = {"start": 0.6, "end": 0.2, "rise": 1.5, "rise_low": 0.0, "land": -2.0}
        vals, locked, dirs, flipped = self.pc.settle_changes(set(), {}, old, {**old, "land": -1.5})
        self.assertEqual((vals["land"], flipped), (-1.5, []))
        vals, locked, dirs, flipped = self.pc.settle_changes(locked, dirs, vals, {**vals, "land": -1.0})
        self.assertEqual((vals["land"], flipped), (-1.0, []))                 # 같은 방향은 그대로
        vals, locked, dirs, flipped = self.pc.settle_changes(locked, dirs, vals, {**vals, "land": -1.5, "rise": 2.0})
        self.assertEqual((vals["land"], vals["rise"], flipped), (-1.25, 2.0, ["land"]))   # 다른 값은 계속 고침
        vals2, *_ = self.pc.settle_changes(locked, dirs, vals, {**vals, "land": -2.0})
        self.assertEqual(vals2["land"], -1.25)                                # 고정한 값은 더 안 바뀜

    def test_save_changes_only_the_pen_correction_keys(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "drawing_config.json"
            path.write_text(json.dumps({"pen": {"down_extra_mm": 0.3}, "port": "COM7"}), encoding="utf-8")
            self.pc.save_pen({"start": 0.2, "end": 0.4, "rise": 2.5}, path)
            data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(data, {"pen": {"down_extra_mm": 0.3, "start_backoff_mm": 0.2, "end_backoff_mm": 0.4,
                                        "edge_rise_mm": 2.5}, "port": "COM7"})


if __name__ == "__main__":
    unittest.main()
