"""종이 X·펜 끝 관찰. 접촉을 확인했다고 하거나 중심을 옮기지 않는지."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import camera_marks, robot_status  # noqa: E402


def paper(crosses, pen=None):
    img = np.full((480, 640, 3), 235, np.uint8)
    for cx, cy in crosses:
        cv2.line(img, (cx - 18, cy - 18), (cx + 18, cy + 18), (15, 15, 15), 3)
        cv2.line(img, (cx - 18, cy + 18), (cx + 18, cy - 18), (15, 15, 15), 3)
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
