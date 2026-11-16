"""종이 X·펜 끝 관찰. 접촉을 확인했다고 하거나 중심을 옮기지 않는지."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import camera_marks, robot_status  # noqa: E402


def paper(crosses, pen=None, level=235, ink=15):
    img = np.full((480, 640, 3), level, np.uint8)
    for cx, cy in crosses:
        cv2.line(img, (cx - 18, cy - 18), (cx + 18, cy + 18), (ink, ink, ink), 3)
        cv2.line(img, (cx - 18, cy + 18), (cx + 18, cy - 18), (ink, ink, ink), 3)
    if pen is not None:
        cv2.circle(img, pen, 5, (10, 10, 10), -1)
    return img


class MarkObservationTest(unittest.TestCase):
    def test_reports_perspective_correct_center_and_pen_location(self):
        obs = camera_marks.observe_setup(paper([(80, 80), (520, 90), (90, 380), (500, 370)], pen=(300, 230)))
        self.assertAlmostEqual(obs["paper_center_px"][0], 304.74, delta=0.1)
        self.assertAlmostEqual(obs["paper_center_px"][1], 235.18, delta=0.1)
        self.assertAlmostEqual(obs["pen_tip_px"][0], 300, delta=1)
        self.assertAlmostEqual(obs["pen_tip_px"][1], 230, delta=1)

    def test_missing_marks_never_reports_a_center(self):
        obs = camera_marks.observe_setup(paper([(80, 80), (520, 90), (90, 380)], pen=(300, 230)))
        self.assertIsNone(obs["paper_center_px"])

    def test_four_crosses_and_pen_tip_are_seen_without_claiming_contact(self):
        obs = camera_marks.observe_setup(paper([(80, 80), (520, 90), (90, 380), (500, 370)], pen=(300, 230)))
        self.assertTrue(obs["crosses_stable"])
        self.assertTrue(obs["pen_tip_seen"])
        self.assertFalse(obs["contact_checked"])
        self.assertFalse(obs["center_moved"])
        self.assertNotIn("mm", obs)

    def test_three_crosses_do_not_count_as_stable(self):
        obs = camera_marks.observe_setup(paper([(80, 80), (520, 90), (90, 380)]))
        self.assertFalse(obs["crosses_stable"])
        self.assertEqual(obs["crosses"], 3)

    def test_blank_frame_moves_nothing(self):
        obs = camera_marks.observe_setup(np.full((200, 200, 3), 240, np.uint8))
        self.assertEqual(obs["crosses"], 0)
        self.assertFalse(obs["center_moved"])


class ReadyAttachmentTest(unittest.TestCase):
    def test_keeps_robot_guide_ready_and_adds_usb_and_camera(self):
        guide = {"ready": True, "issues": [], "notes": ["기존 안내"], "plane_status": "needs_recalibration"}
        choice = robot_status.PortChoice("COM7", "auto", ("COM7",))
        obs = camera_marks.observe_setup(paper([(80, 80), (520, 90), (90, 380), (500, 370)], pen=(300, 230)))
        report = camera_marks.attach_camera_check(guide, choice, obs)
        self.assertTrue(report["ready"])
        self.assertEqual(report["plane_status"], "needs_recalibration")
        self.assertEqual(report["usb"]["port"], "COM7")
        self.assertIn("기존 안내", report["notes"])
        self.assertFalse(report["contact_checked"])
        self.assertFalse(report["center_moved"])
        self.assertIn("접촉", report["notes"][-1])

    def test_unstable_crosses_do_not_become_a_move(self):
        guide = {"ready": False, "issues": ["시뮬레이션이 아직 없다"], "notes": []}
        choice = robot_status.PortChoice(None, "missing", ())
        obs = camera_marks.observe_setup(paper([(80, 80), (520, 90), (90, 380)]))
        report = camera_marks.attach_camera_check(guide, choice, obs)
        self.assertFalse(report["ready"])
        self.assertFalse(report["center_moved"])
        self.assertTrue(any("옮기지 않습니다" in n for n in report["notes"]))

    def test_button_appears_only_after_a_camera_frame(self):
        self.assertFalse(camera_marks.ready_button_visible(None, None))
        self.assertFalse(camera_marks.ready_button_visible(0, None))
        self.assertTrue(camera_marks.ready_button_visible(0, np.zeros((4, 4, 3), np.uint8)))


PAPER_MM = (297.0, 210.0)
QUAD = np.array([[500, 300], [1450, 210], [1425, 850], [550, 940]], np.float32)     # 원근으로 찍힌 종이 귀퉁이 (1920×1080)


def board_photo(kinds=("x", "+", "x", "+"), seed=0, marks=True, jitter_px=2.0):
    """원근 사진: 연한 그림 선, 꽉 찬 펜 몸통, 종이 귀퉁이에서 안쪽에 손으로 그린 표시. 반환: (사진, 실제 교점 4개)."""
    from mirobot_sketch import vision
    rng = np.random.default_rng(seed)
    img = np.clip(np.full((1080, 1920, 3), 200, np.int16) + rng.normal(0, 2, (1080, 1920, 3)), 0, 255).astype(np.uint8)
    h = cv2.getPerspectiveTransform(vision.paper_corners_mm(PAPER_MM).astype(np.float32), QUAD)
    for _ in range(30):                                   # 로봇 펜 선: 종이보다 8단계쯤만 어두움
        a = rng.uniform(-60, 60, 2)
        p = vision.project(h, [a, a + rng.uniform(-25, 25, 2)]).astype(np.int32)
        cv2.line(img, tuple(p[0]), tuple(p[1]), (192, 192, 192), 2)
    cv2.rectangle(img, (1150, 580), (1300, 650), (40, 70, 40), -1)
    truth = []
    for kind, (cx, cy) in zip(kinds, vision.project(h, camera_marks.mark_positions_mm(PAPER_MM))):
        s = 30
        arms = [((-s, -s), (s, s)), ((-s, s), (s, -s))] if kind == "x" else [((-s, 0), (s, 0)), ((0, -s), (0, s))]
        drawn = []
        for a, b in arms:
            p1 = np.round(np.array(a) + [cx, cy] + rng.normal(0, jitter_px, 2))
            p2 = np.round(np.array(b) + [cx, cy] + rng.normal(0, jitter_px, 2))
            if marks:
                cv2.line(img, tuple(p1.astype(int)), tuple(p2.astype(int)), (60, 60, 60), 4, cv2.LINE_AA)
            drawn.append((p1, p2))
        (a1, a2), (b1, b2) = drawn
        t = np.linalg.solve(np.array([a2 - a1, -(b2 - b1)]).T, b1 - a1)
        truth.append(a1 + t[0] * (a2 - a1))
    return img, np.array(truth)


class CornerMarkRegistrationTest(unittest.TestCase):
    def test_hand_drawn_x_and_plus_marks_are_found_at_their_crossing(self):
        for seed in range(3):
            img, truth = board_photo(seed=seed)
            marks, _n = camera_marks.find_corner_marks(img)
            self.assertIsNotNone(marks)
            np.testing.assert_allclose(marks, truth, atol=1.0)        # 왼쪽 위부터 시계 방향, 1px 안

    def test_paper_corners_follow_from_the_marks(self):
        img, _truth = board_photo(jitter_px=0.0)       # 표시를 정확한 자리에 그렸을 때 (손 떨림은 귀퉁이에서 커짐)
        marks, _n = camera_marks.find_corner_marks(img)
        reg = camera_marks.registration_from_marks("cam", marks, (1920, 1080), PAPER_MM)
        np.testing.assert_allclose(reg.corners_px((1920, 1080)), QUAD, atol=3.0)
        self.assertEqual(reg.frame_size, (1920, 1080))

    def test_no_marks_means_no_change(self):
        # 표시가 없으면 None: 앱은 저장된 종이 위치로 그대로 진행
        img, _truth = board_photo(marks=False)
        self.assertIsNone(camera_marks.find_corner_marks(img)[0])

    def test_t_and_l_shapes_are_not_marks(self):
        img = np.full((1080, 1920, 3), 200, np.uint8)
        for cx, cy in ((400, 300), (1500, 300), (1500, 800), (400, 800)):
            cv2.line(img, (cx - 30, cy - 30), (cx + 30, cy - 30), (60, 60, 60), 4)     # T
            cv2.line(img, (cx, cy - 30), (cx, cy + 30), (60, 60, 60), 4)
        self.assertIsNone(camera_marks.find_corner_marks(img)[0])

    def test_saved_position_limits_where_marks_may_be(self):
        img, truth = board_photo()
        near, _ = camera_marks.find_corner_marks(img, expected_px=truth + 15, radius_px=40)
        np.testing.assert_allclose(near, truth, atol=1.0)
        far, _ = camera_marks.find_corner_marks(img, expected_px=truth + [[200, 0]] * 4, radius_px=40)
        self.assertIsNone(far)

    def test_paper_frame_is_measured_against_the_robot_registration(self):
        # 저장된 등록(로봇이 그리는 자리)은 그대로 두고, 표시로 종이의 자리·회전을 잰다
        from mirobot_sketch import vision
        robot = vision.Registration.from_pixels("cam", QUAD, (1920, 1080), PAPER_MM)
        th = np.radians(1.5)
        rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        marks_mm = camera_marks.mark_positions_mm(PAPER_MM) @ rot.T + [5.0, -3.0]
        f = camera_marks.paper_frame_mm(robot, vision.project(robot.matrix((1920, 1080)), marks_mm), (1920, 1080),
                                        PAPER_MM)
        self.assertAlmostEqual(f["cx"], 5.0, places=3)
        self.assertAlmostEqual(f["cy"], -3.0, places=3)
        self.assertAlmostEqual(f["deg"], 1.5, places=3)
        self.assertAlmostEqual(f["width_mm"], 297.0, places=2)
        self.assertAlmostEqual(f["scale"], 1.0, places=3)

    def test_smaller_paper_gives_its_size_and_a_smaller_drawing_scale(self):
        # A5 종이에 같은 규칙(표시 중심 = 귀퉁이에서 MARK_INSET_MM 안쪽)으로 표시하면 종이 크기와 그림 배율(가로·세로 중 작은 쪽)이 나온다
        from mirobot_sketch import vision
        robot = vision.Registration.from_pixels("cam", QUAD, (1920, 1080), PAPER_MM)
        a5 = (210.0, 148.0)
        marks_px = vision.project(robot.matrix((1920, 1080)), camera_marks.mark_positions_mm(a5) + [-20.0, 10.0])
        f = camera_marks.paper_frame_mm(robot, marks_px, (1920, 1080), PAPER_MM)
        self.assertAlmostEqual(f["width_mm"], 210.0, places=2)
        self.assertAlmostEqual(f["height_mm"], 148.0, places=2)
        self.assertAlmostEqual(f["scale"], min(210 / 297, 148 / 210), places=3)
        self.assertAlmostEqual(f["cx"], -20.0, places=3)
        self.assertEqual(f["paper"][0], "A5")

    def test_unknown_paper_size_keeps_the_drawing_size(self):
        # 판 귀퉁이를 A4로 찍은 등록처럼 배율이 틀리면 A4 종이가 약 218×154 mm로 잰다: 아는 크기가 아니므로 배율은 1
        from mirobot_sketch import vision
        robot = vision.Registration.from_pixels("cam", QUAD, (1920, 1080), PAPER_MM)
        marks_px = vision.project(robot.matrix((1920, 1080)), camera_marks.mark_positions_mm(PAPER_MM) / 1.36)
        f = camera_marks.paper_frame_mm(robot, marks_px, (1920, 1080), PAPER_MM)
        self.assertIsNone(f["paper"])
        self.assertEqual(f["scale"], 1.0)
        inset = camera_marks.MARK_INSET_MM
        self.assertAlmostEqual(f["width_mm"], (297 - 2 * inset) / 1.36 + 2 * inset, places=2)


def _rotated_corners(deg, hw=400.0, hh=280.0, center=(960.0, 540.0)):
    th = np.radians(deg)
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    mm = np.array([[-hw, hh], [hw, hh], [hw, -hh], [-hw, -hh]], np.float64)
    xy = mm @ rot.T
    return np.column_stack([center[0] + xy[:, 0], center[1] - xy[:, 1]])


class CornerOrderTest(unittest.TestCase):
    def test_rotated_paper_starts_at_the_top_left(self):
        for deg in (40, 45, 50):
            img = _rotated_corners(deg)
            ordered = camera_marks._order_corners(img)
            self.assertLess(float(np.linalg.norm(ordered[0] - img[0])), 1.0, deg)
            np.testing.assert_allclose(ordered, img, atol=1.0)


class OutlierQuadTest(unittest.TestCase):
    def test_outlier_mark_does_not_replace_a_true_corner(self):
        img = np.full((1080, 1920, 3), 210, np.uint8)
        true = [(400, 250), (1500, 230), (1480, 820), (420, 840)]
        outlier = (200, 80)

        def draw_x(cx, cy):
            cv2.line(img, (cx - 28, cy - 28), (cx + 28, cy + 28), (40, 40, 40), 4, cv2.LINE_AA)
            cv2.line(img, (cx - 28, cy + 28), (cx + 28, cy - 28), (40, 40, 40), 4, cv2.LINE_AA)

        for p in true + [outlier]:
            draw_x(*p)
        marks, n = camera_marks.find_corner_marks(img)
        self.assertEqual(n, 5)
        self.assertIsNotNone(marks)
        chosen = np.asarray(marks)
        self.assertFalse(any(np.linalg.norm(c - outlier) < 15 for c in chosen))
        self.assertLess(float(np.min(np.linalg.norm(chosen - true[0], axis=1))), 15)


class RelativeContrastTest(unittest.TestCase):
    def test_marks_on_paper_gray_130_are_found(self):
        img = np.full((1080, 1920, 3), 130, np.uint8)
        for cx, cy in ((400, 250), (1500, 230), (1480, 820), (420, 840)):
            cv2.line(img, (cx - 28, cy - 28), (cx + 28, cy + 28), (40, 40, 40), 4, cv2.LINE_AA)
            cv2.line(img, (cx - 28, cy + 28), (cx + 28, cy - 28), (40, 40, 40), 4, cv2.LINE_AA)
        marks, n = camera_marks.find_corner_marks(img)
        self.assertEqual(n, 4)
        self.assertIsNotNone(marks)

    def test_pencil_gray_100_is_seen_as_a_cross(self):
        obs = camera_marks.observe_setup(paper([(80, 80), (520, 90), (90, 380), (500, 370)], ink=100))
        self.assertTrue(obs["crosses_stable"])
        self.assertEqual(obs["crosses"], 4)
