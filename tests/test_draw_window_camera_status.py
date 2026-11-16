"""카메라 상태 줄이 공유 실패를 보여 주는지. 창은 열지 않습니다."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch.draw_window import DrawWindow  # noqa: E402


class CameraStatusShareErrorTest(unittest.TestCase):
    def test_share_error_is_shown_on_the_camera_status_line(self):
        shown = {}
        view = SimpleNamespace(
            winfo_exists=lambda: True,
            camera=SimpleNamespace(latest=lambda: np.zeros((8, 8, 3), np.uint8), index=0, error=""),
            _registration=None,
            _sync_ready_button=lambda _frame: None,
            _sync_registration=lambda: None,
            _show_camera_frame=lambda _frame: None,
            _update_zoom=lambda: None,
            _share_frame=lambda: None,
            _camera_name=lambda _index: "BRIO",
            camera_status=SimpleNamespace(configure=lambda **kw: shown.update(kw)),
            _share=SimpleNamespace(error="OSError: 사진을 만들지 못했습니다"),
            _maybe_check_lines=lambda: None,
            _maybe_check_marks=lambda: None,
            after=lambda *_a, **_k: None,
        )
        view._tick_camera = lambda: None
        DrawWindow._tick_camera(view)
        self.assertIn("BRIO", shown["text"])
        self.assertIn("OSError: 사진을 만들지 못했습니다", shown["text"])
