"""자동 세팅 탐색: 목표 시간 안에서 가장 닮은 후보를 고르고, 강도만으로 안 되면 선 개수 상한으로 맞추는지."""

import unittest
from unittest import mock

from mirobot_sketch import autotune


class FakeSession:
    """잔선 강도·상한·빗금에 따라 획 수와 시간이 바뀌는 가짜 세션 (파이프라인 없이 탐색 논리만 검사)."""

    def __init__(self, base_strokes, per_stroke_s=6.0):
        self.base = base_strokes
        self.per_stroke_s = per_stroke_s
        self.params = {"clutter_mm": 3.0, "clutter_pct": 60, "clutter_max": 0, "tone_levels": 1,
                       "tone_spacing_mm": 1.5, "declutter": True}
        self.result = None

    def update_params(self, changes):
        self.params.update(changes)

    def kept(self):
        p = self.params
        n = self.base - int(10 * (p["clutter_mm"] - 3.0)) - (p["clutter_pct"] - 60)
        return min(n, p["clutter_max"]) if p["clutter_max"] else n

    def hatch(self):
        p = self.params
        return 0 if not p["tone_levels"] else int(60 * p["tone_levels"] / p["tone_spacing_mm"])

    def run_current(self):
        strokes = [[(0, 0), (1, 1)]] * self.kept()
        hatch = [[(0, 0), (1, 1)]] * self.hatch()
        total_s = (len(strokes) + len(hatch)) * self.per_stroke_s
        self.result = {"timing": {"total_s": total_s},
                       "stages": {"simplify": {"strokes": strokes}, "clutter": {"strokes": strokes},
                                  "tone": {"hatch_strokes": hatch}}}
        return self.result


def fake_likeness(session):
    # 선이 많을수록, 빗금이 있을수록 닮음 (빗금 효과는 선 100개 몫)
    return {"score": session.kept() / 1000 + (0.1 if session.params["tone_levels"] else 0.0)}


class TuneTest(unittest.TestCase):
    def tune(self, session, target):
        with mock.patch.object(autotune, "likeness", fake_likeness):
            return autotune.tune(session, target_min=target)

    def test_picks_the_most_alike_candidate_within_the_target(self):
        s = FakeSession(base_strokes=400)
        rep = self.tune(s, target=45)
        self.assertTrue(rep["within_target"])
        self.assertLessEqual(rep["chosen"]["minutes"], 45)
        scored = [e for e in rep["tried"] if "likeness" in e and e["minutes"] <= 45]
        best = max(e["likeness"]["score"] for e in scored)
        self.assertEqual(rep["chosen"]["likeness"]["score"], best)
        for k, v in rep["chosen"]["params"].items():         # 고른 설정이 세션에 적용되어 있음
            self.assertEqual(s.params[k], v)
        self.assertLessEqual(s.run_current()["timing"]["total_s"] / 60, 45)

    def test_line_cap_is_used_when_the_strictest_ladder_step_is_too_slow(self):
        s = FakeSession(base_strokes=900)                    # 가장 엄격해도 800획 = 80분 이상
        rep = self.tune(s, target=45)
        self.assertTrue(rep["within_target"])
        cap = rep["chosen"]["params"]["clutter_max"]
        self.assertGreater(cap, 0)
        self.assertLessEqual(rep["chosen"]["minutes"], 45)
        # 상한은 목표 안에서 가장 많이 남기는 값: 하나 더 남기면 넘음
        s.update_params({"clutter_max": cap + 1})
        self.assertGreater(s.run_current()["timing"]["total_s"] / 60, 45)

    def test_reports_the_fastest_candidate_when_nothing_fits(self):
        s = FakeSession(base_strokes=900, per_stroke_s=600.0)  # 20획만 남겨도 200분
        rep = self.tune(s, target=45)
        self.assertFalse(rep["within_target"])
        self.assertEqual(rep["chosen"]["minutes"], min(e["minutes"] for e in rep["tried"]))

    def test_hair_strands_are_spaced_out_then_dropped_only_when_nothing_else_fits(self):
        class LongHair(FakeSession):
            """머리 결을 켜면 상한으로도 못 빼는 획이 floor개 + 결 선 80/간격(mm)개, 끄면 지켜지는 획이 200개뿐."""
            floor = 430

            def kept(self):
                return max(self.floor if self.params["hair_flow"] else 200, super().kept())

            def hatch(self):
                return int(80 / self.params["hair_spacing_mm"]) if self.params["hair_flow"] else 0

        def tuned(floor):
            s = LongHair(base_strokes=900)
            s.floor = floor
            s.params.update({"tone_levels": 0, "hair_flow": True, "hair_spacing_mm": 2.0})
            return self.tune(s, target=45)

        kept = tuned(300)                                     # 결 2mm로도 맞음: 그대로
        self.assertIs(kept["chosen"]["params"]["hair_flow"], True)
        self.assertEqual(kept["chosen"]["params"]["hair_spacing_mm"], 2.0)
        self.assertFalse(kept["hair_flow_dropped"])
        self.assertIsNone(kept["hair_gap_widened"])
        wide = tuned(430)                                     # 2mm 470획·3mm 456획은 넘고 4mm 450획(45분)은 맞음
        self.assertTrue(wide["within_target"])
        self.assertIs(wide["chosen"]["params"]["hair_flow"], True)
        self.assertEqual(wide["hair_gap_widened"], 4.0)
        self.assertIn("결 간격 4mm", autotune.summary(wide))
        off = tuned(440)                                      # 4mm로도 460획: 결 선을 끔
        self.assertTrue(off["within_target"])
        self.assertIs(off["chosen"]["params"]["hair_flow"], False)
        self.assertTrue(off["hair_flow_dropped"])
        self.assertIn("머리 결 선 끔", autotune.summary(off))


class OutNameTest(unittest.TestCase):
    def test_a_different_photo_with_the_same_name_does_not_overwrite_earlier_results(self):
        import json
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as d:
            a, b = str(Path(d) / "photo.webp"), str(Path(d) / "photo.jfif")
            self.assertEqual(autotune._out_name(d, a, set()), "photo")
            (Path(d) / "photo_strokes.json").write_text(json.dumps({"source": {"image": a}}), encoding="utf-8")
            self.assertEqual(autotune._out_name(d, a, set()), "photo")            # 같은 사진: 덮어씀
            self.assertEqual(autotune._out_name(d, b, set()), "photo_jfif")       # 다른 사진: 따로 (따로 돌린 실행)
            self.assertEqual(autotune._out_name(d, a, {"photo"}), "photo_webp")   # 한 실행 안에서 겹침


if __name__ == "__main__":
    unittest.main()
