"""창 크기에 맞춘 위젯 배율 (화면 없이 계산만)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import ui_scale  # noqa: E402


class FitZoomTest(unittest.TestCase):
    def test_base_window_is_full_size_and_bigger_windows_grow_until_the_cap(self):
        self.assertEqual(ui_scale.fit_zoom(ui_scale.BASE_W, ui_scale.BASE_H), 1.0)
        self.assertEqual(ui_scale.fit_zoom(1600, 1000), 1.25)
        self.assertEqual(ui_scale.fit_zoom(10000, 10000), ui_scale.MAX_ZOOM)

    def test_the_tighter_side_decides_so_the_layout_always_fits(self):
        self.assertEqual(ui_scale.fit_zoom(2560, 800), 1.0)          # 가로로 긴 창: 높이 기준
        self.assertEqual(ui_scale.fit_zoom(1280, 1600), 1.0)         # 세로로 긴 창: 너비 기준

    def test_small_windows_shrink_only_to_the_floor(self):
        self.assertEqual(ui_scale.fit_zoom(1100, 700), 0.85)
        self.assertEqual(ui_scale.fit_zoom(400, 300), ui_scale.MIN_ZOOM)

    def test_user_zoom_multiplies_and_rounds_to_steps(self):
        self.assertEqual(ui_scale.fit_zoom(ui_scale.BASE_W, ui_scale.BASE_H, user=1.2), 1.2)
        self.assertEqual(ui_scale.fit_zoom(1400, 860, user=1.1), 1.2)   # 1.075 × 1.1 = 1.18 -> 1.2
        self.assertEqual(ui_scale.clamp_user(9), ui_scale.USER_MAX)
        self.assertEqual(ui_scale.clamp_user(0.1), ui_scale.USER_MIN)


if __name__ == "__main__":
    unittest.main()
