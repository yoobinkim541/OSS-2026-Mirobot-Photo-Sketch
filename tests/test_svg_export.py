"""획 JSON -> SVG 내보내기 테스트.

    python -m unittest discover -s tests -v
"""

import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mirobot_sketch import paper_mapping as pm  # noqa: E402
from mirobot_sketch import svg_export  # noqa: E402

NS = {"svg": "http://www.w3.org/2000/svg"}


def parse(svg_text):
    root = ET.fromstring(svg_text.encode("utf-8"))
    paths = []
    for p in root.iterfind(".//svg:path", NS):
        nums = [float(t) for t in p.get("d").replace("M", " ").replace("L", " ").split()]
        paths.append(list(zip(nums[0::2], nums[1::2])))
    return root, paths


class StrokesToSvgTest(unittest.TestCase):
    def test_document_is_paper_size_in_mm(self):
        root, _ = parse(svg_export.strokes_to_svg([np.array([[0, 0], [1, 1]])], paper_mm=(210, 297)))
        self.assertEqual(root.get("width"), "210mm")
        self.assertEqual(root.get("height"), "297mm")
        self.assertEqual(root.get("viewBox"), "0 0 210 297")

    def test_paper_center_origin_and_y_up_become_svg_top_left_and_y_down(self):
        # 종이 중심 (0,0) -> (148.5, 105), 오른쪽 위 (+x, +y) -> SVG 오른쪽 위 (큰 x, 작은 y)
        stroke = np.array([[0.0, 0.0], [10.0, 20.0], [-30.0, -40.0]])
        _, paths = parse(svg_export.strokes_to_svg([stroke]))
        self.assertEqual(paths, [[(148.5, 105.0), (158.5, 85.0), (118.5, 145.0)]])

    def test_one_path_per_stroke_in_drawing_order(self):
        strokes = [np.array([[i, 0.0], [i, 5.0]]) for i in (3.0, -7.0, 12.0)]
        _, paths = parse(svg_export.strokes_to_svg(strokes))
        self.assertEqual([p[0][0] for p in paths], [151.5, 141.5, 160.5])

    def test_single_point_strokes_are_skipped(self):
        _, paths = parse(svg_export.strokes_to_svg([np.array([[1.0, 1.0]]), np.array([[0, 0], [1, 0]])]))
        self.assertEqual(len(paths), 1)

    def test_json_converts_with_paper_size_from_placement(self):
        strokes_mm, placement = pm.pixels_to_paper([np.array([[0, 0], [100, 50]], float)], box_mm=(80, 80))
        doc = pm.build_strokes_document(strokes_mm, placement)
        with tempfile.TemporaryDirectory() as d:
            src = Path(d) / "s.json"
            pm.save_strokes_json(src, doc)
            loaded, strokes = pm.load_strokes_json(src)
            out = Path(d) / "s.svg"
            svg_export.save_svg(out, strokes, svg_export.paper_size_mm(loaded["placement"]))
            root, paths = parse(out.read_text(encoding="utf-8"))
        self.assertEqual(root.get("viewBox"), "0 0 297 210")
        # 80 x 40 mm 선이 종이 중심에 놓임: (148.5±40, 105±20), 이미지 아래로 가던 선은 SVG에서도 아래로
        self.assertEqual(paths, [[(108.5, 85.0), (188.5, 125.0)]])


if __name__ == "__main__":
    unittest.main()
