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

    def test_uniform_dark_block_is_hatched_but_a_light_block_is_not(self):
        dark = np.full((200, 300), 255, np.uint8)
        dark[40:160, 40:160] = 20
        strokes, note = tone.hatch_strokes(dark, 8, 1, min_len_px=8)
        self.assertEqual(note, "")
        self.assertTrue(strokes)
        pts = np.vstack(strokes)
        on_block = (pts[:, 0] >= 35) & (pts[:, 0] <= 165) & (pts[:, 1] >= 35) & (pts[:, 1] <= 165)
        self.assertTrue(bool(on_block.all()))
        light = np.full((200, 300), 255, np.uint8)
        light[40:160, 40:160] = 230
        self.assertEqual(tone.hatch_strokes(light, 8, 1, min_len_px=8)[0], [])

    def test_darker_levels_add_lines_in_the_same_direction_not_a_grid(self):
        dark = np.full((200, 300), 255, np.uint8)
        dark[40:160, 40:160] = 10                                   # 가장 어두운 단계까지 들어가는 면
        one = tone.hatch_strokes(dark, 8, 1, angle_deg=45, min_len_px=8)[0]
        two = tone.hatch_strokes(dark, 8, 2, angle_deg=45, min_len_px=8)[0]
        long_dirs = []
        for s in two:
            d = np.diff(np.asarray(s), axis=0)
            for dx, dy in d[np.hypot(d[:, 0], d[:, 1]) > 3 * 8]:  # 이음(짧은 지그재그)이 아닌 빗금 몸통
                long_dirs.append(np.degrees(np.arctan2(dy, dx)) % 180)
        self.assertTrue(long_dirs)
        self.assertTrue(all(abs(a - 45) < 3 for a in long_dirs))   # 교차(135°) 빗금 없음
        ink = lambda ss: (tone.ink_image(ss, dark.shape, 2) < 128).sum()
        self.assertGreater(ink(two), 1.5 * ink(one))               # 그래도 더 진해짐

    def test_keep_out_mask_gets_no_hatching(self):
        dark = np.full((200, 300), 255, np.uint8)
        dark[40:160, 40:160] = 20
        keep_out = np.zeros(dark.shape, bool)
        keep_out[:, :100] = True                                   # 블록 왼쪽 절반 (얼굴 피부 자리)
        strokes, _ = tone.hatch_strokes(dark, 8, 2, min_len_px=8, keep_out=keep_out)
        self.assertTrue(strokes)                                   # 오른쪽 절반은 그대로 빗금
        self.assertGreaterEqual(float(np.vstack(strokes)[:, 0].min()), 99.0)

    def test_hair_flow_follows_the_strand_direction_and_keeps_its_spacing(self):
        yy, xx = np.mgrid[0:200, 0:200].astype(np.float32)
        th = np.radians(30)                                         # 결(밝고 어두운 줄무늬)이 30° 방향
        gray = (60 + 40 * np.sin((-xx * np.sin(th) + yy * np.cos(th)) / 3.0)).astype(np.uint8)
        mask = np.zeros((200, 200), bool)
        mask[30:170, 30:170] = True
        lines = tone.hair_flow_strokes(gray, mask, 10.0, 15.0, 200.0)
        self.assertGreater(len(lines), 5)
        for q in lines:
            d = q[-1] - q[0]
            ang = np.degrees(np.arctan2(d[1], d[0])) % 180
            self.assertLess(min(abs(ang - 30), 180 - abs(ang - 30)), 8)       # 결 방향
            self.assertTrue(mask[np.round(q[:, 1]).astype(int), np.round(q[:, 0]).astype(int)].all())
        mids = [q[len(q) // 2] for q in lines]
        gaps = [min(np.linalg.norm(a - b) for j, b in enumerate(mids) if j != i) for i, a in enumerate(mids)]
        self.assertGreater(min(gaps), 4.0)                          # 간격의 절반보다 가까이 붙지 않음

    def test_hair_flow_fills_only_the_gaps_between_existing_strands(self):
        yy, xx = np.mgrid[0:200, 0:200].astype(np.float32)
        th = np.radians(30)
        gray = (60 + 40 * np.sin((-xx * np.sin(th) + yy * np.cos(th)) / 3.0)).astype(np.uint8)
        mask = np.zeros((200, 200), bool)
        mask[30:170, 30:170] = True
        t = np.linspace(-60, 60, 121)
        strand = np.column_stack([100 + t * np.cos(th), 100 + t * np.sin(th)])   # 선화가 이미 그린 결 한 줄
        lines = tone.hair_flow_strokes(gray, mask, 10.0, 15.0, 200.0, obstacles=[strand])
        self.assertGreater(len(lines), 3)
        for q in lines:
            pts = tone._dense(q)
            gap = np.min(np.linalg.norm(pts[:, None, :] - strand[None, :, :], axis=2))
            self.assertGreater(gap, 4.0)                            # 기존 결과 간격의 절반 넘게 떨어짐

    def test_hair_flow_direction_follows_a_new_photo_of_the_same_size(self):
        yy, xx = np.mgrid[0:200, 0:200].astype(np.float32)
        mask = np.zeros((200, 200), bool)
        mask[30:170, 30:170] = True
        for deg in (30, 120):                                       # 결 방향을 기억해 두므로 다른 사진에서 옛 방향을 쓰면 안 됨
            th = np.radians(deg)
            gray = (60 + 40 * np.sin((-xx * np.sin(th) + yy * np.cos(th)) / 3.0)).astype(np.uint8)
            lines = tone.hair_flow_strokes(gray, mask, 10.0, 15.0, 200.0)
            self.assertGreater(len(lines), 5)
            for q in lines:
                d = q[-1] - q[0]
                ang = np.degrees(np.arctan2(d[1], d[0])) % 180
                self.assertLess(min(abs(ang - deg), 180 - abs(ang - deg)), 8)

    def test_hair_mask_takes_the_hair_but_cuts_off_dark_clothes_joined_to_it(self):
        from mirobot_sketch import faces as fc
        from mirobot_sketch import stages
        img = np.full((400, 300, 3), 255, np.uint8)
        img[40:200, 60:240] = (30, 30, 30)                          # 머리 (얼굴 뒤까지)
        cv2.ellipse(img, (150, 160), (55, 70), 0, 0, 360, (150, 170, 220), -1)   # 얼굴
        img[200:400, 40:260] = (40, 30, 30)                         # 머리와 이어진 어두운 옷
        lm = np.array([[128, 145], [172, 145], [150, 170], [132, 195], [168, 195]], np.float64)
        face = fc.Face(np.array([100.0, 100.0, 100.0, 120.0]), lm, 0.9)
        prev = {"faces": [face], "prep_color": img, "prep_gray": cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)}
        mask = stages._hair_mask(prev, stages._face_skin_mask(prev))
        self.assertTrue(mask[60, 150] and mask[150, 75])            # 정수리·옆머리
        self.assertFalse(mask[320, 150])                            # 옷은 결을 그리지 않음
        self.assertFalse(mask[170, 150])                            # 얼굴 피부도 아님

    def test_face_keep_out_spares_hair_above_the_brows(self):
        from mirobot_sketch import faces as fc
        from mirobot_sketch import stages
        img = np.full((300, 300, 3), 255, np.uint8)
        cv2.ellipse(img, (150, 160), (60, 80), 0, 0, 360, (150, 170, 220), -1)   # 피부색 얼굴
        img[40:105, 80:220] = (30, 30, 30)                                       # 이마 위 머리카락
        lm = np.array([[125, 140], [175, 140], [150, 170], [130, 195], [170, 195]], np.float64)
        face = fc.Face(np.array([95.0, 90.0, 110.0, 140.0]), lm, 0.9)
        mask = stages._face_skin_mask({"faces": [face], "prep_color": img})
        self.assertTrue(mask[170, 120] and mask[200, 150])          # 볼·입 주변은 빗금 금지
        self.assertFalse(mask[70, 150])                            # 앞머리는 빗금 그대로

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

    def test_photo_edges_find_the_subject_outline_but_not_flat_areas_or_the_white_background(self):
        img = np.full((200, 200), 255, np.uint8)
        cv2.rectangle(img, (50, 50), (150, 150), 90, -1)          # 배경을 지운 사진의 인물처럼
        e = tone.photo_edges(img) > 0
        self.assertTrue(e[48:53, 60:140].any())                     # 윤곽
        self.assertFalse(e[70:130, 70:130].any())                   # 고른 면
        self.assertFalse(e[:30, :].any())                           # 흰 배경

    def test_lineart_hints_point_at_the_lineart_controls_not_canny(self):
        img = disc_and_gradient()
        color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        edges = cv2.Canny(img, 50, 150)
        tiny = [np.array([[5.0, 5.0], [8.0, 5.0]])]                       # 윤곽을 거의 안 그림 → 재현 안내가 나옴
        canny_hints = " ".join(tone.compare_report(img, color, tiny, 1.0, edges, outline=tiny)[0]["hints"])
        art_hints = " ".join(tone.compare_report(img, color, tiny, 1.0, edges, outline=tiny, lineart=True)[0]["hints"])
        self.assertIn("canny_low", canny_hints)
        self.assertNotIn("canny_low", art_hints)
        self.assertIn("declutter", art_hints)


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
        self.assertTrue(stages.stage_title("paper").startswith("⑫"))
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

    def test_turning_tone_on_reruns_it_even_after_a_paper_size_change(self):
        s = self.session()
        s.update_params({"box_mm": 60})
        s.run_current()
        # 겹침 제거가 종이 크기를 쓰므로 그 뒤 단계도 다시 돈다. 빗금이 꺼져 있으면 빗금 획은 없다.
        self.assertEqual(s.result["stages"]["tone"]["hatch_strokes"], [])
        self.assertIs(s.result["stages"]["tone"].get("uses_deps"), False)
        after = s.pipeline.run_counts["tone"]
        s.update_params({"tone_levels": 1})
        s.run_current()
        self.assertEqual(s.pipeline.run_counts["tone"], after + 1)
        self.assertTrue(s.result["stages"]["tone"]["hatch_strokes"])

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
