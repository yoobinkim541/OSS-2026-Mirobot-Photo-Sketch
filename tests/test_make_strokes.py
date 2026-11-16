"""명령줄 make_strokes가 GUI와 같이 종류 프리셋의 선화·빗금을 쓰는지."""

import json
import sys
import tempfile
import unittest
from io import StringIO
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import lineart, make_strokes  # noqa: E402


def write_png(path, img):
    cv2.imencode(".png", img)[1].tofile(str(path))


def photo():
    img = np.full((240, 320), 255, np.uint8)
    cv2.circle(img, (160, 120), 55, 30, -1)
    return img


class PhotoPresetTest(unittest.TestCase):
    def invoke(self, img, extra, ink):
        with tempfile.TemporaryDirectory() as d:
            src = Path(d) / "photo.png"
            out = Path(d) / "strokes"
            write_png(src, img)
            argv = ["mirobot-strokes", str(src), *extra, "--out", str(out), "--max-side", "320", "--no-rembg"]
            buf = StringIO()
            with mock.patch("mirobot_sketch.lineart.ink_mask", ink), \
                    mock.patch("sys.argv", argv), mock.patch("sys.stdout", buf):
                code = make_strokes.main()
            doc = json.loads(out.with_suffix(".json").read_text(encoding="utf-8")) if code == 0 else None
            preview = out.with_name(out.name + "_preview.png").exists()
            return code, buf.getvalue(), doc, preview

    def test_photo_uses_lineart_and_leaves_the_dark_disc_unhatched(self):
        # 실사 프리셋은 채색 전 스케치라 어두운 면을 빗금으로 메우지 않음 (머리카락은 얼굴이 있을 때 결 선으로)
        line = np.zeros((240, 320), np.uint8)
        line[20, 30:290] = 255
        art = cv2.cvtColor(line, cv2.COLOR_GRAY2BGR)
        ink = mock.Mock(return_value=(art, line))
        code, text, doc, preview = self.invoke(photo(), ["--type", "photo"], ink)
        self.assertEqual(code, 0, text)
        ink.assert_called_once()
        self.assertTrue(preview)
        self.assertEqual(doc["source"]["params"]["tone_levels"], 0)
        self.assertTrue(doc["source"]["params"]["hair_flow"])
        tf = doc["placement"]["transform"]
        inside = False
        for stroke in doc["strokes"]:
            pts = [((x - tf["ox"]) / tf["scale"] + tf["cx"], tf["cy"] - (y - tf["oy"]) / tf["scale"])
                   for x, y in stroke["points_xy_mm"]]
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                for t in (0.25, 0.5, 0.75):
                    x = x0 + (x1 - x0) * t
                    y = y0 + (y1 - y0) * t
                    if (x - 160) ** 2 + (y - 120) ** 2 < 30 ** 2:
                        inside = True
        self.assertFalse(inside, "어두운 원 안에 빗금이 그어짐")

    def test_missing_lineart_model_falls_back_and_says_so(self):
        ink = mock.Mock(side_effect=lineart.LineartUnavailable("인터넷 없음."))
        code, text, doc, _preview = self.invoke(photo(), ["--type", "photo"], ink)
        self.assertEqual(code, 0, text)
        self.assertIn("선화 대신", text)
        self.assertTrue(doc["strokes"])

    def test_illustration_does_not_call_lineart(self):
        img = np.full((80, 200), 255, np.uint8)
        cv2.line(img, (10, 40), (190, 40), 0, 2)
        ink = mock.Mock(side_effect=lineart.LineartUnavailable("인터넷 없음."))
        code, text, doc, _preview = self.invoke(img, ["--type", "illustration"], ink)
        self.assertEqual(code, 0, text)
        ink.assert_not_called()
        self.assertNotIn("선화 대신", text)
        self.assertTrue(doc["strokes"])


if __name__ == "__main__":
    unittest.main()
