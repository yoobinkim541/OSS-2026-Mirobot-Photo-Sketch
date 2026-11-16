"""종이 위치 창의 클릭이 사진 안으로 잘리는지. 창은 열지 않습니다."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch.paper_register_window import PaperRegisterWindow  # noqa: E402


class ClickClampTest(unittest.TestCase):
    def test_new_click_is_clamped_inside_the_frame(self):
        win = SimpleNamespace(frame=np.zeros((100, 200, 3), np.uint8), points=[], _scale=1.0, _drag=None)
        win._nearest = lambda _x, _y: None
        win._redraw = lambda: None
        win._to_image = lambda x, y: PaperRegisterWindow._to_image(win, x, y)
        PaperRegisterWindow._press(win, SimpleNamespace(x=-30, y=800))
        self.assertEqual(win.points, [(0.0, 99.0)])
