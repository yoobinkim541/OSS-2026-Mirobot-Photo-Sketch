"""명암 빗금(tone 단계)과 사진-그림 비교: 빗금 생성, 단계·세션 연결, 에이전트 도구."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import stages, tone  # noqa: E402
from mirobot_sketch.agent.tools import AgentToolbox, READ_ONLY_TOOLS, SYSTEM_PROMPT, TOOL_NAMES  # noqa: E402
from mirobot_sketch.session import SketchSession  # noqa: E402

import golden  # noqa: E402


def disc_and_gradient():
    """흰 바탕에 어두운 원(왼쪽)과 오른쪽으로 갈수록 어두워지는 띠."""
    img = np.full((400, 600), 255, np.uint8)
    cv2.circle(img, (150, 200), 90, 30, -1)
    for x in range(320, 580):
        img[100:300, x] = int(230 - (x - 320) / 260 * 200)
    return img


def total_length(strokes):
    return sum(float(np.linalg.norm(np.diff(s, axis=0), axis=1).sum()) for s in strokes)


class DarknessMapTest(unittest.TestCase):
    def test_white_background_is_zero_and_dark_is_high(self):
        d = tone.darkness_map(disc_and_gradient())
        self.assertEqual(float(d[10, 10]), 0.0)                    # 흰 바탕
        self.assertGreater(float(d[200, 150]), 0.9)                # 원 안
        self.assertGreater(float(d[200, 570]), float(d[200, 330]))  # 띠는 오른쪽이 더 어두움

    def test_blank_image_has_no_darkness(self):
        self.assertEqual(float(tone.darkness_map(np.full((50, 50), 255, np.uint8)).max()), 0.0)


class HatchTest(unittest.TestCase):
    def hatch(self, levels=2, **kw):
        args = dict(spacing_px=8, levels=levels, angle_deg=45, min_len_px=12)
        args.update(kw)
        return tone.hatch_strokes(disc_and_gradient(), **args)

    def test_off_makes_nothing(self):
        self.assertEqual(self.hatch(levels=0), ([], ""))

    def test_strokes_stay_on_the_dark_areas(self):
        strokes, note = self.hatch()
        self.assertEqual(note, "")
        pts = np.vstack(strokes)
        in_disc = np.hypot(pts[:, 0] - 150, pts[:, 1] - 200) <= 95
        in_bar = (pts[:, 0] >= 315) & (pts[:, 1] >= 95) & (pts[:, 1] <= 305)
        self.assertTrue(bool((in_disc | in_bar).all()), "빗금이 어두운 면 밖에 생김")
        self.assertTrue(bool(in_disc.any()) and bool(in_bar.any()))

    def test_more_levels_draw_more_and_darker_areas_get_crossed(self):
        lengths = [total_length(self.hatch(levels=n)[0]) for n in (1, 2, 3)]
        self.assertLess(lengths[0], lengths[1])
        self.assertLess(lengths[1], lengths[2])

    def test_spacing_and_min_length_and_bias_change_the_amount(self):
        base = total_length(self.hatch(levels=1)[0])
        self.assertGreater(total_length(self.hatch(levels=1, spacing_px=5)[0]), base)          # 촘촘하면 더 많이
        self.assertLess(total_length(self.hatch(levels=1, spacing_px=14)[0]), base)
        self.assertLess(total_length(self.hatch(levels=1, min_len_px=400)[0]), base)           # 짧은 것만 남으면 거의 없음
        self.assertLess(total_length(self.hatch(levels=1, bias=0.2)[0]), base)                 # +면 줄어듦
        self.assertGreater(total_length(self.hatch(levels=1, bias=-0.2)[0]), base)

    def test_neighbouring_lines_are_chained_into_zigzag_strokes(self):
        img = disc_and_gradient()
        mask = tone._clean_mask((tone.darkness_map(img, 8) > 0.45).astype(np.uint8), 8)
        runs = tone._runs_on_lines(mask, 45, 8, 12)
        chains = tone._chain_runs(runs, mask, 20)
        self.assertGreater(len(runs), 20)
        self.assertLess(len(chains), len(runs) / 3)                # 펜 올림이 크게 줄어듦
        for c in chains:
            self.assertGreaterEqual(len(c), 2)
            self.assertEqual(c.shape[1], 2)

    def test_dark_background_is_refused_with_a_hint(self):
        dark = np.full((300, 300), 20, np.uint8)
        cv2.circle(dark, (150, 150), 60, 200, -1)
        strokes, note = tone.hatch_strokes(dark, 8, 2)
        self.assertEqual(strokes, [])
        self.assertIn("배경 제거", note)

    def test_result_is_deterministic(self):
        a, b = self.hatch()[0], self.hatch()[0]
        self.assertEqual(len(a), len(b))
        self.assertTrue(all(np.array_equal(x, y) for x, y in zip(a, b)))


class CompareReportTest(unittest.TestCase):
    def report(self, levels):
        img = disc_and_gradient()
        color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        outline = [np.array([[60.0, 200.0], [150.0, 110.0], [240.0, 200.0], [150.0, 290.0], [60.0, 200.0]])]
        hatch = tone.hatch_strokes(img, 8, levels, 45, 12)[0] if levels else []
        edges = cv2.Canny(img, 50, 150)
        return tone.compare_report(img, color, outline + hatch, 1.0, edges, outline=outline)

    def test_hatching_improves_the_tone_match(self):
        none, one, two = (self.report(n)[0]["tone_match"] for n in (0, 1, 2))
        self.assertLess(none, one)
        self.assertLessEqual(one, two + 1e-9)
        self.assertGreater(two, 0.85)

    def test_metrics_image_and_hints(self):
        metrics, image = self.report(0)
        for key in ("tone_match", "under_shaded_pct", "over_shaded_pct", "ink_coverage_pct",
                    "edge_recall", "edge_precision", "hints"):
            self.assertIn(key, metrics)
        self.assertEqual(image.shape[0], 360)
        self.assertEqual(image.shape[2], 3)
        self.assertTrue(any("tone_levels" in h for h in metrics["hints"]), metrics["hints"])   # 명암이 비었다고 알려 줌
        self.assertGreater(metrics["under_shaded_pct"], 15)

    def test_edge_precision_ignores_hatching(self):
        without = self.report(0)[0]["edge_precision"]
        with_hatch = self.report(2)[0]["edge_precision"]
        self.assertAlmostEqual(without, with_hatch, places=3)      # 빗금 때문에 잡음으로 잘못 판정하지 않음


class ToneStageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        img = np.full((400, 500), 255, np.uint8)
        cv2.circle(img, (250, 200), 120, 0, 6)                  # 윤곽
        cv2.circle(img, (250, 200), 70, 40, -1)                 # 어두운 면
        cls.path = Path(cls.tmp.name) / "disc.png"
        cv2.imwrite(str(cls.path), img)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def session(self, **params):
        s = SketchSession(golden.default_cfg())
        s.set_image(self.path)
        s.update_params({"box_mm": 80, "epsilon_px": 3.0, **params})
        s.run_current()
        return s

    def test_stage_is_listed_before_edit_and_numbered(self):
        ids = [s.id for s in stages.ALL_STAGES]
        self.assertEqual(ids[ids.index("simplify") + 1:ids.index("simplify") + 3], ["tone", "edit"])
        self.assertIn("tone", stages.PIPELINE_IDS)
        self.assertTrue(stages.stage_title("paper").startswith("⑪"))
        for key in ("tone_levels", "tone_spacing_mm", "tone_angle", "tone_min_mm", "tone_bias"):
            self.assertIn(key, stages.PARAM_SPECS)

    def test_default_is_off_and_leaves_the_outline_strokes_unchanged(self):
        s = self.session()
        out = s.result["stages"]["tone"]
        self.assertEqual(out["hatch_strokes"], [])
        self.assertEqual(len(out["strokes"]), len(s.result["stages"]["simplify"]["strokes"]))
        self.assertEqual(s.state()["params"]["tone_levels"], 0)

    def test_hatching_adds_strokes_without_changing_the_edit_numbering(self):
        off = self.session()
        on = self.session(tone_levels=2)
        table_strokes = lambda s: sorted(i for i, e in s.table.items() if e["kind"] == "stroke")
        self.assertEqual(table_strokes(on), table_strokes(off))                  # 번호표에 빗금은 없음
        n_off, n_on = len(off.result["strokes_mm"]), len(on.result["strokes_mm"])
        self.assertGreater(n_on, n_off)                                          # 그리는 획에는 들어감
        self.assertEqual(n_on - n_off, len(on.result["stages"]["tone"]["hatch_strokes"]))
        self.assertGreater(on.result["timing"]["total_s"], off.result["timing"]["total_s"])   # 시간 추정에 반영
        self.assertEqual(on.result["out_of_limits"], 0)

    def test_hatching_stays_inside_the_paper_and_survives_edits_and_undo(self):
        s = self.session(tone_levels=2)
        hatch_n = len(s.result["stages"]["tone"]["hatch_strokes"])
        pts = np.vstack([np.asarray(p) for p in s.result["strokes_mm"]])
        self.assertLessEqual(float(np.abs(pts[:, 0]).max()), 40.0 + 1e-6)         # 80 mm 그림
        stroke_id = next(i for i, e in s.table.items() if e["kind"] == "stroke")
        s.propose_edits([{"op": "delete", "ids": [stroke_id]}], apply_now=True)
        self.assertEqual(len(s.result["stages"]["tone"]["hatch_strokes"]), hatch_n)
        self.assertGreaterEqual(len(s.result["strokes_mm"]), hatch_n)             # 빗금은 그대로 그려짐
        s.undo()
        self.assertEqual(s.table[stroke_id]["kind"], "stroke")

    def test_changing_paper_size_does_not_rerun_tone_when_it_is_off(self):
        s = self.session()
        before = s.pipeline.run_counts["tone"]
        s.update_params({"box_mm": 60})
        s.run_current()
        self.assertEqual(s.pipeline.run_counts["tone"], before)                   # uses_deps=False
        s.update_params({"tone_levels": 1})
        s.run_current()
        self.assertEqual(s.pipeline.run_counts["tone"], before + 1)

    def test_stage_summary_and_preview(self):
        s = self.session(tone_levels=1)
        self.assertRegex(s.stage_summaries()["tone"], r"빗금 \d+획")
        img = s.render("tone")
        self.assertEqual(img.shape[:2], s.result["base"].shape)
        self.assertEqual(self.session().stage_summaries()["tone"], "꺼짐")


class CompareToolTest(unittest.TestCase):
    def test_tool_returns_metrics_and_side_by_side_image_and_is_read_only(self):
        tmp = tempfile.TemporaryDirectory()
        try:
            img = np.full((300, 400), 255, np.uint8)
            cv2.circle(img, (200, 150), 90, 30, -1)
            cv2.circle(img, (200, 150), 110, 0, 4)
            p = Path(tmp.name) / "d.png"
            cv2.imwrite(str(p), img)
            s = SketchSession(golden.default_cfg())
            s.set_image(p)
            s.run_current()
            tb = AgentToolbox(s)
            self.assertIn("compare", TOOL_NAMES)
            self.assertIn("compare", READ_ONLY_TOOLS)
            parts, err = tb.call("compare", {})
            self.assertFalse(err, parts)
            metrics = json.loads(parts[0]["text"])
            self.assertIn("tone_match", metrics)
            self.assertEqual(parts[1]["type"], "image")
            s.drawing_lock = True                                                  # 그리는 중에도 읽기는 됨
            self.assertFalse(tb.call("compare", {})[1])
            s.drawing_lock = False
            parts, err = tb.call("set_params", {"tone_levels": 2})
            self.assertFalse(err, parts)
            self.assertGreater(json.loads(tb.call("compare", {})[0][0]["text"])["tone_match"], metrics["tone_match"])
        finally:
            tmp.cleanup()

    def test_compare_without_result_is_a_readable_error(self):
        parts, err = AgentToolbox(SketchSession(golden.default_cfg())).call("compare", {})
        self.assertTrue(err)
        self.assertTrue(parts[0]["text"].startswith("오류"))

    def test_prompt_teaches_the_look_alike_loop(self):
        for word in ("compare", "tone_levels", "rembg", "tone_match", "차이"):
            self.assertIn(word, SYSTEM_PROMPT)


if __name__ == "__main__":
    unittest.main()
