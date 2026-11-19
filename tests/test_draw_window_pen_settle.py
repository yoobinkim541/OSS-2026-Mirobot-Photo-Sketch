"""그림이 끝난 뒤 자동 보정으로 바뀐 펜 값을 정리하는 규칙. 창은 열지 않습니다."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import pen_correction  # noqa: E402
from mirobot_sketch.draw_window import DrawWindow  # noqa: E402

START = {"start_backoff_mm": 0.6, "end_backoff_mm": 0.2, "edge_rise_mm": 1.5, "edge_rise_low_mm": 0.0, "edge_land_mm": -2.0}


def view(now):
    return SimpleNamespace(cfg={"pen": dict(now)}, _pen_start=dict(START), _pen_note="",
                           _pen_text=lambda: "", pen_status=SimpleNamespace(configure=lambda **kw: None))


class PenSettleTest(unittest.TestCase):
    def test_changed_values_are_saved_once_at_the_end(self):
        v = view({**START, "start_backoff_mm": 0.4, "edge_land_mm": -2.5})
        with mock.patch.object(pen_correction, "save_pen") as save:
            DrawWindow._settle_pen(v)
            DrawWindow._settle_pen(v)                      # 마지막 확인이 두 번 와도 한 번만
        save.assert_called_once_with({"start": 0.4, "end": 0.2, "rise": 1.5, "rise_low": 0.0, "land": -2.5})

    def test_nothing_saved_when_values_did_not_change(self):
        v = view(START)
        with mock.patch.object(pen_correction, "save_pen") as save:
            DrawWindow._settle_pen(v)
        save.assert_not_called()


class AutoRegisterTest(unittest.TestCase):
    """끝난 뒤 선 맞춤에서 찾은 이동·회전·크기로 종이 위치(등록)를 고쳐 저장하는지."""

    def setUp(self):
        from mirobot_sketch import vision
        self.vision = vision
        self.size = (640, 480)
        self.reg = vision.Registration.from_pixels("cam", [(120, 90), (520, 140), (500, 380), (100, 420)], self.size)
        strokes = [[(x, y), (x + 10.0, y)] for x in range(-40, 40, 8) for y in (-30.0, 0.0, 30.0)]   # 획 30개
        self.shown = {}
        self.view = SimpleNamespace(_registration=self.reg, job=SimpleNamespace(summary={}), _run_strokes=strokes,
                                    vision_status=SimpleNamespace(configure=lambda **kw: self.shown.update(kw)))

    def test_rotation_and_scale_are_saved(self):
        th = np.radians(-2.0)
        a = np.array([[1.06 * np.cos(th), -1.06 * np.sin(th), 16.0], [1.06 * np.sin(th), 1.06 * np.cos(th), -12.0]])
        with mock.patch.object(self.vision, "save") as save, \
                mock.patch.object(self.vision, "backup", return_value=None) as backup:
            DrawWindow._auto_register(self.view, 30, (0.9, a, (16.0, -12.0), -2.0, 1.06), self.reg, self.size)
        backup.assert_called_once()                                  # 덮어쓰기 전에 예전 위치를 남김
        save.assert_called_once()
        new = save.call_args[0][0]
        pts = np.array([(0.0, 0.0), (40.0, 30.0)])
        np.testing.assert_allclose(self.vision.project(new.matrix(self.size), pts),
                                   self.vision.project(self.reg.matrix(self.size), pts @ a[:, :2].T + a[:, 2]), atol=0.05)
        self.assertIs(self.view._registration, new)
        self.assertIn("회전 -2.0°", self.shown["text"])

    def test_tiny_or_weak_fits_are_not_saved(self):
        tiny = np.array([[1.0, 0.0, 0.2], [0.0, 1.0, 0.0]])
        with mock.patch.object(self.vision, "save") as save, mock.patch.object(self.vision, "backup") as backup:
            DrawWindow._auto_register(self.view, 30, (0.9, tiny, (0.2, 0.0), 0.0, 1.0), self.reg, self.size)
            DrawWindow._auto_register(self.view, 30, (0.5, tiny * 2, (0.4, 0.0), 0.0, 1.0), self.reg, self.size)
        save.assert_not_called()
        backup.assert_not_called()


if __name__ == "__main__":
    unittest.main()
