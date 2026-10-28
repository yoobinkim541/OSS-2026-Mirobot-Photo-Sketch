"""실사 선화 경로: 선화로 선을 따고, 머리카락은 흐름선으로, 피부 조명 경계는 지운다."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import hair, stages  # noqa: E402


class LineartEdgeTest(unittest.TestCase):
    def test_photo_lineart_uses_the_drawing_not_canny(self):
        color = np.full((40, 60, 3), 180, np.uint8)
        drawn = np.full_like(color, 255)
        mask = np.zeros((40, 60), np.uint8)
        mask[10:30, 20] = 255
        params = stages.default_params()
        params["lineart"] = True
        prev = {"color": color, "gray": color[:, :, 0], "prep_color": color, "prep_gray": color[:, :, 0]}
        with mock.patch("mirobot_sketch.lineart.ink_mask", return_value=(drawn, mask)) as ink:
            out = stages._run_edges(prev, params)
        ink.assert_called_once()
        self.assertEqual(int(out["edges"][15, 20]), 255)
        self.assertEqual(int(out["edges"][0, 0]), 0)
        self.assertTrue(out["lineart_used"])


def face_at(x, y, w, h):
    lm = np.array([[x + 0.3 * w, y + 0.4 * h], [x + 0.5 * w, y + 0.4 * h], [x + 0.15 * w, y + 0.6 * h],
                   [x + 0.25 * w, y + 0.8 * h], [x + 0.45 * w, y + 0.8 * h]])   # 왼쪽을 보는 옆얼굴
    return SimpleNamespace(box=np.array([x, y, w, h], float), landmarks=lm)


class HairFlowTest(unittest.TestCase):
    def test_strands_follow_the_hair_direction_inside_the_dark_mass(self):
        img = np.full((300, 300, 3), (200, 210, 230), np.uint8)
        hair_area = np.zeros((300, 300), bool)
        hair_area[20:290, 170:290] = True
        stripes = ((np.arange(300) // 3) % 2 == 0)[None, :].repeat(300, 0)   # 세로 결
        img[hair_area & stripes] = (20, 20, 20)
        img[hair_area & ~stripes] = (45, 45, 45)
        face = face_at(60, 120, 100, 120)        # 코가 왼쪽 -> 뒤통수(머리카락)는 오른쪽
        mask = hair.hair_mask(img, [face])
        self.assertGreater(mask[150, 230], 0)
        strands = hair.hair_strokes(img, mask)
        self.assertGreater(len(strands), 3)
        vertical = [abs(d[1]) > 2 * abs(d[0]) for d in (s[-1] - s[0] for s in strands)]
        self.assertGreaterEqual(sum(vertical) / len(vertical), 0.8)     # 대부분 세로 결을 따라감


class SkinShadingTest(unittest.TestCase):
    def colors(self, img):
        b = cv2.GaussianBlur(img, (0, 0), 1.2)
        return (cv2.cvtColor(b, cv2.COLOR_BGR2LAB).astype(np.float32),
                cv2.cvtColor(b, cv2.COLOR_BGR2YCrCb).astype(np.float32))

    def test_brightness_step_on_skin_is_shading_but_lip_edge_is_not(self):
        line = np.array([[50.0, y] for y in range(10, 90)])
        skin = np.zeros((100, 100, 3), np.uint8)
        skin[:, :50] = (150, 170, 230)
        skin[:, 50:] = (110, 125, 175)          # 같은 피부색, 그늘
        self.assertTrue(stages._is_skin_shading(line, *self.colors(skin)))
        lip = skin.copy()
        lip[:, 50:] = (90, 60, 200)             # 붉은 입술
        self.assertFalse(stages._is_skin_shading(line, *self.colors(lip)))
