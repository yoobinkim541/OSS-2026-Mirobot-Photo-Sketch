"""단계 정의·캐시 파이프라인."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import stages  # noqa: E402

import golden  # noqa: E402


def inputs():
    gray = golden.synthetic_images()["line"]
    return {"gray": gray, "color": cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)}


class ParamSpecTest(unittest.TestCase):
    def test_every_default_is_valid_and_keys_unique(self):
        keys = [p.key for st in stages.ALL_STAGES for p in st.params]
        self.assertEqual(len(keys), len(set(keys)))
        for spec in stages.PARAM_SPECS.values():
            self.assertEqual(spec.clamp(spec.default), spec.default, spec.key)

    def test_clamp(self):
        s = stages.PARAM_SPECS
        self.assertEqual(s["blur_ksize"].clamp(4), 5)        # 홀수로
        self.assertEqual(s["blur_ksize"].clamp(99), 15)
        self.assertEqual(s["epsilon_px"].clamp(1.2345), 1.2)
        self.assertEqual(s["canny_low"].clamp(-5), 0)
        self.assertTrue(s["rembg"].clamp(1))
        with self.assertRaises(ValueError):
            s["edge_mode"].clamp("rainbow")


class StageTextTest(unittest.TestCase):
    def test_every_stage_has_number_and_description(self):
        for k, st in enumerate(stages.ALL_STAGES):
            self.assertTrue(st.desc, st.id)
            self.assertEqual(stages.stage_title(st.id), f"{'①②③④⑤⑥⑦⑧⑨⑩⑪'[k]} {st.label}")


class PipelineTest(unittest.TestCase):
    def test_late_change_only_recomputes_late_stages(self):
        pl, p = stages.Pipeline(), stages.default_params()
        pl.run(inputs(), "img", p)
        first = dict(pl.run_counts)
        pl.run(inputs(), "img", p)
        self.assertEqual(pl.run_counts, first)                         # 전부 캐시
        pl.run(inputs(), "img", {**p, "epsilon_px": 2.0})
        changed = {k for k in first if pl.run_counts[k] != first[k]}
        self.assertEqual(changed, {"simplify", "tone"})              # 윤곽이 바뀌면 (꺼진) 명암 단계 입력도 바뀜
        pl.run(inputs(), "img", {**p, "epsilon_px": 2.0, "canny_low": 60})
        self.assertEqual(pl.run_counts["prep"], first["prep"])
        self.assertEqual(pl.run_counts["edges"], first["edges"] + 1)
        self.assertEqual(pl.dirty_from("img", {**p, "dedupe_px": 2}), "edges")
        self.assertIsNone(pl.dirty_from("img", {**p, "epsilon_px": 2.0, "canny_low": 60}))

    def test_dirty_from_points_at_first_changed_stage(self):
        pl, p = stages.Pipeline(), stages.default_params()
        pl.run(inputs(), "img", p)
        self.assertEqual(pl.dirty_from("img", {**p, "dedupe_px": 2}), "dedupe")
        self.assertEqual(pl.dirty_from("other", p), "source")

    def test_stale_run_stops_before_computing(self):
        pl = stages.Pipeline()
        with self.assertRaises(stages.StaleRun):
            pl.run(inputs(), "img", stages.default_params(), is_current=lambda: False)

    def test_previews_are_same_size_bgr(self):
        inp = inputs()
        outs = stages.Pipeline().run(inp, "img", stages.default_params())
        for sid in stages.PIPELINE_IDS:
            img = stages.STAGE_BY_ID[sid].preview(outs[sid])
            self.assertEqual(img.shape, (*inp["gray"].shape, 3), sid)
            self.assertEqual(img.dtype, np.uint8)

    def test_candidates_collected(self):
        outs = stages.Pipeline().run(inputs(), "img", stages.default_params())
        reasons = {r for _, r in stages.candidates_of(outs)}
        self.assertTrue(reasons <= {"small", "spur", "overlap"})

    def test_lab_mode_runs(self):
        p = {**stages.default_params(), "edge_mode": "lab"}
        outs = stages.Pipeline().run(inputs(), "img", p)
        self.assertTrue(outs["simplify"]["strokes"])


if __name__ == "__main__":
    unittest.main()
