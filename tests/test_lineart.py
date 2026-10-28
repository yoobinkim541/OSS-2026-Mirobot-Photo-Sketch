"""실사 선화는 기존 Canny 대신 선화 이미지를 어두운 선으로 추적한다."""

import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import stages  # noqa: E402


class LineartEdgeTest(unittest.TestCase):
    def test_photo_lineart_uses_the_drawing_not_canny(self):
        color = np.full((40, 60, 3), 180, np.uint8)
        drawn = np.full_like(color, 255)
        drawn[10:30, 20, :] = 0
        params = stages.default_params()
        params["lineart"] = True
        prev = {"color": color, "gray": color[:, :, 0], "prep_color": color, "prep_gray": color[:, :, 0]}
        with mock.patch("mirobot_sketch.lineart.line_drawing", return_value=drawn) as draw:
            out = stages._run_edges(prev, params)
        draw.assert_called_once()
        self.assertGreater(int(out["edges"][15, 20]), 0)
        self.assertEqual(int(out["edges"][0, 0]), 0)
        self.assertFalse(np.array_equal(out["prep_color"], color))
