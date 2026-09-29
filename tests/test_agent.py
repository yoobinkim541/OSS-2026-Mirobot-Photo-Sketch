"""에이전트 기능 테스트: 세션 편집, 도구, 로컬 브리지+MCP, OpenRouter 도구 루프, CLI 이벤트 해석.

LLM이나 네트워크 없이 돌아갑니다 (OpenRouter는 로컬 가짜 서버, CLI는 미리 준비한 이벤트).

    python -m unittest discover -s tests -v
"""

import json
import os
import subprocess
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mirobot_sketch import sketch_pipeline as sp  # noqa: E402
from mirobot_sketch import edits  # noqa: E402
from mirobot_sketch import stages  # noqa: E402
from mirobot_sketch.agent import backends as bk  # noqa: E402
from mirobot_sketch.agent.bridge import BridgeClient, BridgeServer  # noqa: E402
from mirobot_sketch.agent.tools import TOOLS, AgentToolbox  # noqa: E402
from mirobot_sketch.session import SessionError, SketchSession  # noqa: E402

import golden  # noqa: E402


def make_image(path):
    """테스트용 선화: 큰 사각형 + 원 + 왼쪽 위 작은 잡음 점들."""
    img = np.full((400, 400), 255, np.uint8)
    cv2.rectangle(img, (80, 80), (320, 320), 0, 3)
    cv2.circle(img, (200, 200), 60, 0, 3)
    for x in range(20, 60, 12):
        cv2.line(img, (x, 20), (x + 6, 30), 0, 2)
    cv2.imwrite(str(path), img)


class SessionTestBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import tempfile

        cls.tmp = tempfile.TemporaryDirectory()
        cls.img = Path(cls.tmp.name) / "t.png"
        make_image(cls.img)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def new_session(self):
        s = SketchSession(golden.default_cfg())
        s.set_image(self.img)
        s.apply_preset("illustration", "medium")
        s.run()
        return s


class SessionTest(SessionTestBase):
    def test_run_and_state(self):
        s = self.new_session()
        st = s.state()["result"]
        self.assertGreater(st["strokes"], 1)
        self.assertEqual(st["edits"], [])

    def test_original_is_color_and_aligned_with_gray(self):
        import tempfile
        s = self.new_session()
        orig = s.render("original")
        self.assertEqual(orig.ndim, 3)
        self.assertEqual(orig.shape[:2], s.result["base"].shape)   # 획 좌표를 겹쳐 그릴 수 있게 같은 크기
        # 컬러가 남아 있고(흑백 변환 안 함), 투명 PNG는 흰 배경으로 합성
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "rgba.png"
            img = np.zeros((40, 60, 4), np.uint8)
            img[:, :30] = (0, 0, 255, 255)          # 왼쪽: 불투명 빨강
            cv2.imwrite(str(p), img)                # 오른쪽: 완전 투명
            c = sp.load_color(p, max_side=60)
        self.assertEqual(c.shape, (40, 60, 3))
        self.assertEqual(tuple(int(v) for v in c[20, 5]), (0, 0, 255))
        self.assertEqual(tuple(int(v) for v in c[20, 55]), (255, 255, 255))

    def test_invalid_inputs_raise_session_error(self):
        s = self.new_session()
        with self.assertRaises(SessionError):
            s.update_params({"nope": 1})
        with self.assertRaises(SessionError):
            s.apply_preset("watercolor")

    def test_only_changed_stages_recompute(self):
        s = self.new_session()
        before = dict(s.pipeline.run_counts)
        s.update_params({"epsilon_px": 2.5})
        s.run()
        changed = {k for k in before if s.pipeline.run_counts[k] != before[k]}
        self.assertEqual(changed, {"simplify"})

    def test_line_source_is_mapped_to_edge_mode(self):
        s = self.new_session()
        self.assertEqual(s.update_params({"line_source": "dark"}), {"edge_mode": "dark"})
        self.assertEqual(s.update_params({"line_source": "canny"}), {"edge_mode": "luma"})
        with self.assertRaises(SessionError):
            s.update_params({"edge_mode": "rainbow"})

    def test_stale_run_returns_none(self):
        s = self.new_session()
        s.update_params({"canny_low": 70})
        real = s.pipeline.run

        def bump_then_run(*a, **k):
            s.generation += 1        # 계산 시작 직후 사용자가 값을 또 바꾼 상황
            return real(*a, **k)

        s.pipeline.run = bump_then_run
        self.assertIsNone(s.run())
        s.pipeline.run = real
        self.assertIsNotNone(s.run_current())

    def test_every_stage_renders_same_size_even_for_tiny_transparent_png(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "tiny.png"
            img = np.zeros((60, 90, 4), np.uint8)
            cv2.circle(img, (45, 30), 20, (0, 0, 0, 255), 2)   # 투명 바탕 위 검은 원
            cv2.imwrite(str(p), img)
            s = SketchSession(golden.default_cfg())
            s.set_image(p)
            s.run_current()
            h, w = s.result["base"].shape
            for kind in ("original", *stages.PIPELINE_IDS, "lines"):
                self.assertEqual(s.render(kind).shape[:2], (h, w), kind)
            eh, ew = s.render("edit").shape[:2]      # 편집 그림은 에이전트용으로 확대(최대 1000px), 비율은 같음
            self.assertAlmostEqual(eh / ew, h / w, delta=0.01)

    def test_default_box_fits_executor_limits_exactly(self):
        s = self.new_session()                 # 기본 그림 크기 100mm = 실행기 허용 범위(±50mm)와 같음
        lim = s.cfg["limits"]["max_abs_paper_x_mm"]
        worst = max(float(np.abs(np.asarray(st)).max()) for st in s.result["strokes_mm"])
        self.assertLessEqual(worst, lim + 1e-6)        # 반올림된 배율·중심 때문에 가장자리가 넘치면 안 됨
        self.assertEqual(s.result["out_of_limits"], 0)

    def test_stage_summaries_show_what_each_stage_produced(self):
        s = self.new_session()
        sm = s.stage_summaries()
        self.assertEqual(list(sm), [st.id for st in stages.ALL_STAGES])
        o = s.result["stages"]
        self.assertIn(f"획 {len(o['trace']['strokes'])}", sm["trace"])
        self.assertIn(f"버림 {len(o['trace']['discarded_trace'])}", sm["trace"])
        self.assertIn(f"경계 {int((o['edges']['edges'] > 0).sum()):,}px", sm["edges"])
        self.assertIn("분", sm["paper"])
        # 이전 단계 대비 변화: "이전 → 지금"
        ch = s.stage_change("dedupe")
        self.assertIn("→", ch)
        self.assertIn(sm["trace"].split(" · ")[0], ch)
        self.assertEqual(s.stage_change("source"), "")

    def test_state_lists_stages_with_values(self):
        st = self.new_session().state()
        ids = [x["id"] for x in st["stages"]]
        self.assertEqual(ids, [x.id for x in stages.ALL_STAGES])
        self.assertIn("edge_mode", st["stages"][2]["params"])

    def test_update_params_clamps_to_safe_range(self):
        s = self.new_session()
        applied = s.update_params({"box_mm": 500, "median_ksize": -3})
        self.assertEqual(applied["box_mm"], 250)       # 넓은 범위(실물 미확인) 폭 ±125mm가 상한
        self.assertEqual(applied["median_ksize"], 0)

class EditSessionTest(SessionTestBase):
    def strokes(self, s):
        return sorted(i for i, e in s.table.items() if e["kind"] == "stroke")

    def test_delete_is_proposed_then_applied_and_undone(self):
        s = self.new_session()
        ids = self.strokes(s)
        r = s.propose_edits([{"op": "delete", "ids": [ids[0]]}])
        self.assertEqual(r["proposed"], [ids[0]])
        self.assertEqual(self.strokes(s), ids)                    # 제안만으로는 안 바뀜
        s.apply_proposals()
        self.assertEqual(self.strokes(s), ids[1:])
        self.assertEqual(s.table[ids[0]]["reason"], "deleted")    # 같은 번호의 후보가 됨
        self.assertIsNotNone(s.undo())
        self.assertEqual(self.strokes(s), ids)

    def test_restore_candidate_and_exclude(self):
        s = self.new_session()
        cands = sorted(i for i, e in s.table.items() if e["kind"] == "candidate")
        self.assertTrue(cands, "합성 이미지의 잡음 점이 후보로 남아야 함")
        first = self.strokes(s)[0]
        s.propose_edits([{"op": "restore", "ids": [cands[0]]}, {"op": "delete", "ids": [first]}])
        out = s.apply_proposals(exclude=[first])
        self.assertEqual(out["applied"], [cands[0]])
        self.assertEqual(s.table[cands[0]]["kind"], "stroke")
        self.assertEqual(s.table[first]["kind"], "stroke")
        self.assertEqual(set(s.proposals), {first})                # 뺀 제안은 확인 전 제안으로 남음

    def test_point_edit_in_mm_and_bounds(self):
        s = self.new_session()
        sid = self.strokes(s)[0]
        pts = s.get_stroke(sid)["points_mm"]
        target = [pts[0][1] * 0.9, pts[0][2] * 0.9]
        expected_px = s.mm_to_px(target)          # 적용 후엔 테두리 상자가 바뀌어 mm 배치가 달라질 수 있어 px로 비교
        s.propose_edits([{"op": "move_point", "id": sid, "index": 0, "to_mm": target}], apply_now=True)
        self.assertTrue(np.allclose(s.table[sid]["poly"][0], expected_px))
        with self.assertRaises(SessionError):
            s.propose_edits([{"op": "move_point", "id": sid, "index": 0, "to_mm": [0, 70]}])   # 넓은 범위 지붕(+57.5) 위

    def test_invalid_batch_changes_nothing(self):
        s = self.new_session()
        before = {i: e["kind"] for i, e in s.table.items()}
        sid = self.strokes(s)[0]
        cand = next(i for i, e in s.table.items() if e["kind"] == "candidate")
        for bad in ([{"op": "delete", "ids": [sid]}, {"op": "delete", "ids": [10 ** 6]}],
                    [{"op": "delete", "ids": [cand]}],
                    [{"op": "delete_points", "id": sid, "indices": [999]}],
                    [{"op": "teleport"}],
                    [{"op": "delete", "ids": [sid]}] * (edits.MAX_OPS + 1)):
            with self.assertRaises(SessionError):
                s.propose_edits(bad)
        self.assertEqual({i: e["kind"] for i, e in s.table.items()}, before)
        self.assertEqual(s.proposals, {})

    def test_cannot_delete_every_stroke(self):
        s = self.new_session()
        s.propose_edits([{"op": "delete", "ids": self.strokes(s)}])
        with self.assertRaises(SessionError):
            s.apply_proposals()

    def test_edits_survive_recompute_and_pending_proposals_are_cancelled(self):
        s = self.new_session()
        ids = self.strokes(s)
        victim = s.table[ids[0]]["poly"].copy()
        s.propose_edits([{"op": "delete", "ids": [ids[0]]}], apply_now=True)
        s.propose_edits([{"op": "delete", "ids": [ids[1]]}])           # 적용 안 한 제안
        s.update_params({"canny_low": s.params["canny_low"] + 5})
        s.run_current()
        self.assertEqual(s.proposals, {})
        self.assertIn("취소", s.state()["edit"]["notice"])
        polys = [e["poly"] for e in s.table.values() if e["kind"] == "stroke"]
        close = [p for p in polys if edits.remove_matching([p], [victim], s.result["base"].shape)[0] == []]
        self.assertEqual(close, [])                                      # 지운 선은 다시 지워짐

    def test_box_change_keeps_numbers_and_proposals(self):
        s = self.new_session()
        sid = self.strokes(s)[0]
        s.propose_edits([{"op": "delete", "ids": [sid]}])
        s.update_params({"box_mm": 80})
        s.run_current()
        self.assertIn(sid, s.proposals)

    def test_split_join_group_all_or_nothing(self):
        s = self.new_session()
        a, b = self.strokes(s)[:2]
        s.propose_edits([{"op": "join", "a": a, "b": b}])
        with self.assertRaises(SessionError) as cm:           # 묶음의 일부만 고르면 적용할 것이 없음
            s.apply_proposals(exclude=[b])
        self.assertIn("묶", str(cm.exception))
        self.assertEqual(set(s.proposals), {a, b})

    def test_render_edit_region_and_overlay(self):
        s = self.new_session()
        img = s.render("edit", region_mm=[-10, -10, 10, 10], numbered=True, show_candidates=True, overlay=0.5)
        self.assertEqual(img.ndim, 3)
        self.assertLessEqual(max(img.shape[:2]), 1000)

    def test_list_strokes_is_truncated(self):
        s = self.new_session()
        r = s.list_strokes(include_candidates=True, limit=2)
        self.assertEqual(len(r["rows"]), 2)
        self.assertTrue(r["truncated"])
        self.assertEqual(set(r["rows"][0]), {"id", "kind", "reason", "length_mm", "bbox_mm", "points"})


class ReviewFixesTest(SessionTestBase):
    """최종 리뷰에서 나온 문제 재현 (원자성, 입력 검사, 좌표 고정, 캐시, 제안 보존)."""

    def strokes(self, s):
        return sorted(i for i, e in s.table.items() if e["kind"] == "stroke")

    def test_failed_recompute_keeps_previous_result(self):
        from mirobot_sketch import session as session_mod
        s = self.new_session()
        old_result, old_table = s.result, dict(s.table)
        s.update_params({"canny_low": 70})
        with mock.patch.object(session_mod.pm, "pixels_to_paper", side_effect=ValueError("변환 실패")):
            with self.assertRaises(ValueError):
                s.run_current()
        self.assertIs(s.result, old_result)
        self.assertEqual(s.table.keys(), old_table.keys())
        self.assertIn("estimated_minutes", s.state()["result"])      # 예전 결과로 상태 조회가 됨
        s.render("paper")

    def test_failed_apply_changes_nothing(self):
        from mirobot_sketch import session as session_mod
        s = self.new_session()
        sid = self.strokes(s)[0]
        s.propose_edits([{"op": "delete", "ids": [sid]}])
        with mock.patch.object(session_mod.sp, "order_strokes", side_effect=RuntimeError("순서 실패")):
            with self.assertRaises(RuntimeError):
                s.apply_proposals()
        self.assertEqual(s.table[sid]["kind"], "stroke")
        self.assertEqual(s.history, [])
        self.assertIn(sid, s.proposals)                                # 제안도 그대로

    def test_non_finite_and_wrong_type_inputs_are_rejected(self):
        s = self.new_session()
        sid = self.strokes(s)[0]
        for bad in ({"op": "add_stroke", "points_mm": [[float("nan"), 0], [1, 1]]},
                    {"op": "delete", "ids": "12"},
                    {"op": "delete", "ids": [3.7]},
                    {"op": "split", "id": sid, "index": True},
                    {"op": "move_point", "id": sid, "index": 0, "to_mm": [1]}):
            with self.assertRaises(SessionError, msg=str(bad)):
                s.propose_edits([bad], apply_now=True)
        self.assertEqual(s.history, [])

    def test_mm_coordinates_of_untouched_strokes_do_not_move(self):
        s = self.new_session()
        keep = self.strokes(s)[0]
        before = s.get_stroke(keep)["points_mm"]
        s.propose_edits([{"op": "add_stroke", "points_mm": [[0, 0], [100, -80]]}], apply_now=True)   # 넓은 범위 먼 곳
        self.assertEqual(s.get_stroke(keep)["points_mm"], before)
        added = max(i for i, e in s.table.items() if e["reason"] == "added")
        self.assertAlmostEqual(s.get_stroke(added)["points_mm"][1][1], 100, delta=0.2)   # [번호, x, y]
        self.assertAlmostEqual(s.get_stroke(added)["points_mm"][1][2], -80, delta=0.2)

    def test_rembg_result_of_previous_image_is_not_cached_for_new_image(self):
        from mirobot_sketch import session as session_mod
        import tempfile
        s = self.new_session()
        with tempfile.TemporaryDirectory() as d:
            other = Path(d) / "b.png"
            make_image(other)

            def slow_rembg(path, **kw):
                s.set_image(other)                   # 배경 제거 도중 사용자가 다른 이미지를 엶
                return np.zeros((10, 10, 3), np.uint8)

            with mock.patch.object(session_mod.sp, "remove_background_bgr", side_effect=slow_rembg):
                s._inputs(True)
            self.assertEqual(s.image_path, str(other))
            self.assertFalse(any(k[1] for k in s._inputs_cache))   # 새 이미지 캐시엔 rembg 결과 없음

    def test_empty_selection_is_refused_and_unselected_proposals_are_kept(self):
        s = self.new_session()
        a, b = self.strokes(s)[:2]
        s.propose_edits([{"op": "delete", "ids": [a, b]}])
        with self.assertRaises(SessionError):
            s.apply_proposals(exclude=[a, b])
        self.assertEqual(set(s.proposals), {a, b})
        self.assertEqual(s.history, [])
        s.apply_proposals(only=[a])
        self.assertEqual(set(s.proposals), {b})                         # 고르지 않은 제안은 남음
        self.assertEqual(s.table[a]["kind"], "candidate")
        self.assertEqual(s.table[b]["kind"], "stroke")
        s.apply_proposals()
        self.assertEqual(s.table[b]["kind"], "candidate")

    def test_apply_now_refuses_ids_with_unconfirmed_proposals(self):
        s = self.new_session()
        a, b = self.strokes(s)[:2]
        s.propose_edits([{"op": "smooth", "id": a, "strength": 2}])
        with self.assertRaises(SessionError):
            s.propose_edits([{"op": "delete", "ids": [a]}], apply_now=True)
        s.propose_edits([{"op": "delete", "ids": [b]}], apply_now=True)   # 다른 번호는 바로 적용되고
        self.assertIn(a, s.proposals)                                     # 기존 제안은 남음


class MinorFixesTest(SessionTestBase):
    """최종 리뷰 Minor 항목 재현."""

    def strokes(self, s):
        return sorted(i for i, e in s.table.items() if e["kind"] == "stroke")

    def bump(self, s):
        s.update_params({"canny_low": s.params["canny_low"] + 3})
        s.run_current()

    def test_proposal_views_wait_for_the_lock(self):
        s = self.new_session()
        s.propose_edits([{"op": "delete", "ids": [self.strokes(s)[0]]}])
        done = threading.Event()
        with s.lock:                       # 다른 스레드가 제안을 바꾸는 중인 상황
            t = threading.Thread(target=lambda: (s.proposal_views(), done.set()))
            t.start()
            self.assertFalse(done.wait(0.3))   # 잠금이 풀릴 때까지 기다려야 함
        t.join(5)
        self.assertTrue(done.is_set())

    def test_proposal_epoch_changes_only_on_renumber(self):
        s = self.new_session()
        e0 = s.proposal_epoch
        a, b = self.strokes(s)[:2]
        s.propose_edits([{"op": "delete", "ids": [a, b]}])
        s.apply_proposals(exclude=[b])
        self.assertEqual(s.proposal_epoch, e0)          # 적용 뒤 남은 제안은 같은 번호 체계
        self.bump(s)
        self.assertNotEqual(s.proposal_epoch, e0)       # 번호를 새로 매기면 바뀜

    def test_undo_after_recompute_restores_the_edit_record(self):
        s = self.new_session()
        n = len(self.strokes(s))
        s.propose_edits([{"op": "delete", "ids": [self.strokes(s)[0]]}], apply_now=True)
        self.bump(s)
        self.assertEqual(len(self.strokes(s)), n - 1)
        self.assertIsNotNone(s.undo())                   # 다시 계산한 뒤에도 되돌릴 수 있음
        self.assertEqual(len(self.strokes(s)), n)

    def test_deleted_edited_stroke_can_be_restored_after_recompute(self):
        s = self.new_session()
        sid = self.strokes(s)[0]
        s.propose_edits([{"op": "smooth", "id": sid, "strength": 3}], apply_now=True)
        edited = s.table[sid]["poly"]
        s.propose_edits([{"op": "delete", "ids": [sid]}], apply_now=True)
        self.bump(s)
        cand = [i for i, e in s.table.items() if e["kind"] == "candidate" and e["poly"] is edited]
        self.assertEqual(len(cand), 1)
        s.propose_edits([{"op": "restore", "ids": cand}], apply_now=True)
        self.bump(s)
        self.assertTrue(any(e["kind"] == "stroke" and e["poly"] is edited for e in s.table.values()))

    def test_restored_candidate_is_not_listed_twice_after_recompute(self):
        s = self.new_session()
        cid = min(i for i, e in s.table.items() if e["kind"] == "candidate")
        poly = s.table[cid]["poly"]
        s.propose_edits([{"op": "restore", "ids": [cid]}], apply_now=True)
        self.bump(s)
        close = [e["kind"] for e in s.table.values()
                 if e["poly"].shape == poly.shape and np.allclose(e["poly"], poly)]
        self.assertEqual(close, ["stroke"])

    def test_refused_apply_now_adds_no_proposals(self):
        s = self.new_session()
        with self.assertRaises(SessionError) as cm:
            s.propose_edits([{"op": "delete", "ids": self.strokes(s)}], apply_now=True)
        self.assertIn("추가하지 않았습니다", str(cm.exception))
        self.assertEqual(s.proposals, {})

    def test_delete_region_crossing_mode(self):
        s = self.new_session()
        region = [-5, -5, 5, 5]
        s.propose_edits([{"op": "delete_region", "region_mm": region, "mode": "crossing"}])
        crossing = set(s.proposals)
        s.discard_proposals()
        s.propose_edits([{"op": "delete_region", "region_mm": region, "mode": "inside"}])
        self.assertTrue(set(s.proposals) <= crossing)
        self.assertTrue(crossing)

    def test_error_wording(self):
        s = self.new_session()
        cand = next(i for i, e in s.table.items() if e["kind"] == "candidate")
        with self.assertRaises(SessionError) as cm:
            s.propose_edits([{"op": "delete", "ids": [cand]}])
        self.assertIn("획이 아닙니다", str(cm.exception))

    def test_concurrent_recompute_and_edits_stay_consistent(self):
        s = self.new_session()
        errors = []

        def recompute():
            for k in range(6):
                try:
                    s.update_params({"epsilon_px": 1.0 + 0.2 * k})
                    s.run()
                except Exception as e:   # 계산 스레드에서 난 예외는 모두 실패
                    errors.append(e)

        def edit():
            for _ in range(30):
                try:
                    ids = [i for i, e in s.table.items() if e["kind"] == "stroke"][:1]
                    s.propose_edits([{"op": "smooth", "id": ids[0]}])
                    s.proposal_views()
                    s.apply_proposals()
                except SessionError:
                    pass                 # 번호가 바뀌는 등 정상적인 거부
                except Exception as e:
                    errors.append(e)

        ts = [threading.Thread(target=recompute), threading.Thread(target=edit)]
        for t in ts:
            t.start()
        for t in ts:
            t.join(120)
        self.assertEqual(errors, [])
        s.run_current()
        self.assertIn("timing", s.result)
        s.render("edit")


class ToolboxTest(SessionTestBase):
    def test_tools_do_not_include_robot_execution(self):
        names = {t["name"] for t in TOOLS}
        self.assertFalse(names & {"execute", "draw", "run_robot", "send_gcode"})

    def test_view_returns_png_image(self):
        tb = AgentToolbox(self.new_session())
        parts, err = tb.call("view", {"kind": "strokes", "numbered": True})
        self.assertFalse(err)
        img = parts[1]
        self.assertEqual(img["mime"], "image/png")
        import base64
        self.assertTrue(base64.b64decode(img["data"]).startswith(b"\x89PNG"))

    def test_errors_are_returned_not_raised(self):
        tb = AgentToolbox(SketchSession())
        for name, args in (("view", {"kind": "paper"}), ("unknown", {}), ("propose_edits", {"wrong": 1})):
            parts, err = tb.call(name, args)
            self.assertTrue(err)
            self.assertTrue(parts[0]["text"].startswith("오류"))

    def test_set_params_schema_comes_from_stage_specs(self):
        props = next(t for t in TOOLS if t["name"] == "set_params")["parameters"]["properties"]
        self.assertEqual(props["edge_mode"]["enum"], ["luma", "lab", "dark"])
        self.assertEqual(props["blur_ksize"]["maximum"], 15)
        for key in stages.PARAM_SPECS:
            self.assertIn(key, props)

    def test_set_params_lab_and_view_each_stage(self):
        tb = AgentToolbox(self.new_session())
        parts, err = tb.call("set_params", {"edge_mode": "lab"})
        self.assertFalse(err, parts)
        self.assertEqual(json.loads(parts[0]["text"])["applied"], {"edge_mode": "lab"})
        for kind in stages.PIPELINE_IDS:
            parts, err = tb.call("view", {"kind": kind})
            self.assertFalse(err, kind)
            self.assertEqual(parts[1]["type"], "image")

    def test_propose_view_apply_flow(self):
        changes = []
        s = self.new_session()
        tb = AgentToolbox(s, on_change=changes.append)
        rows = json.loads(tb.call("list_strokes", {"include_candidates": True})[0][0]["text"])["rows"]
        stroke = next(r["id"] for r in rows if r["kind"] == "stroke")
        parts, err = tb.call("propose_edits", {"ops": [{"op": "delete", "ids": [stroke]}]})
        self.assertFalse(err, parts)
        self.assertEqual([p["type"] for p in parts], ["text", "image"])       # 바뀌는 부위 미리보기
        self.assertIn("proposals", changes)
        parts, err = tb.call("apply_proposals", {})
        self.assertFalse(err, parts)
        self.assertEqual(s.table[stroke]["kind"], "candidate")
        parts, err = tb.call("view", {"kind": "edit", "show_candidates": True, "overlay_original": 0.4})
        self.assertFalse(err)
        pts = json.loads(tb.call("get_stroke", {"id": stroke})[0][0]["text"])["points_mm"]
        self.assertEqual(pts[0][0], 0)

    def test_old_delete_tools_are_gone(self):
        names = {t["name"] for t in TOOLS}
        self.assertFalse(names & {"delete_strokes", "delete_region"})
        self.assertTrue({"list_strokes", "get_stroke", "propose_edits", "apply_proposals",
                         "discard_proposals"} <= names)

    def test_set_params_reports_recomputed_stages(self):
        tb = AgentToolbox(self.new_session())
        out = json.loads(tb.call("set_params", {"epsilon_px": 2.2})[0][0]["text"])
        self.assertEqual(out["recomputed"], ["simplify", "edit", "paper"])

    def test_tools_are_read_only_while_robot_draws(self):
        s = self.new_session()
        tb = AgentToolbox(s)
        s.drawing_lock = True
        parts, err = tb.call("set_params", {"epsilon_px": 2.0})
        self.assertTrue(err)
        self.assertIn("그리는 중", parts[0]["text"])
        for name, args in (("get_state", {}), ("view", {"kind": "paper"}), ("list_strokes", {})):
            self.assertFalse(tb.call(name, args)[1], name)
        s.drawing_lock = False
        self.assertFalse(tb.call("set_params", {"epsilon_px": 2.0})[1])

    def test_set_params_notifies_screen(self):
        changes = []
        tb = AgentToolbox(self.new_session(), on_change=changes.append)
        tb.call("set_params", {"detail": "low"})
        self.assertEqual(changes, ["result"])

    def guide(self, tb):
        parts, err = tb.call("robot_guide", {})
        self.assertFalse(err, parts)
        return json.loads(parts[0]["text"])

    def test_robot_guide_without_image_says_to_open_one(self):
        g = self.guide(AgentToolbox(SketchSession(golden.default_cfg())))
        self.assertFalse(g["ready"])
        self.assertIn("이미지", g["next"])
        self.assertIn("로봇으로 그리기", g["agent_cannot"])
        self.assertEqual(len(g["steps_for_the_person"]), 4)

    def test_robot_guide_asks_for_simulation_then_reports_pen_down_readiness(self):
        s = self.new_session()                         # 기본 100 mm = 펜으로 그릴 수 있는 범위(±50 mm)
        tb = AgentToolbox(s)
        g = self.guide(tb)
        self.assertFalse(g["ready"])
        self.assertTrue(any("simulate" in i for i in g["issues"]))
        self.assertFalse(g["pen_down_possible"])
        tb.call("simulate", {})
        g = self.guide(tb)
        self.assertTrue(g["ready"], g)
        self.assertTrue(g["pen_down_possible"])
        self.assertEqual(g["issues"], [])
        self.assertFalse(g["paper_calibrated"])        # 기본 설정은 종이 보정 전
        self.assertTrue(any("보정" in n for n in g["notes"]))
        self.assertEqual(g["pen_down_area_mm"]["x"], [-50.0, 50.0])
        self.assertGreater(g["drawing"]["strokes"], 0)

    def test_robot_guide_says_air_mode_when_drawing_is_outside_pen_down_range(self):
        s = self.new_session()
        tb = AgentToolbox(s)
        tb.call("set_params", {"box_mm": 110})         # ±55 mm: 펜 범위(±50) 밖, 넓은 범위 안
        tb.call("simulate", {})
        g = self.guide(tb)
        self.assertFalse(g["pen_down_possible"])
        self.assertTrue(g["ready"], g)                 # 공중 확인은 가능
        self.assertTrue(any("공중 모드" in n for n in g["notes"]))

    def test_robot_guide_and_state_are_readable_while_robot_draws(self):
        s = self.new_session()
        tb = AgentToolbox(s)
        s.drawing_lock = True
        self.assertTrue(self.guide(tb)["drawing_now"])
        st = json.loads(tb.call("get_state", {})[0][0]["text"])
        self.assertTrue(st["drawing"])
        self.assertIn("plane_status", st)
        self.assertEqual(st["executor_limit_outline_mm"][0], [-50.0, -50.0])
        s.drawing_lock = False

    def test_system_prompt_describes_the_real_robot_flow(self):
        from mirobot_sketch.agent.tools import SYSTEM_PROMPT, TOOL_NAMES, READ_ONLY_TOOLS
        self.assertIn("'로봇으로 그리기' 창", SYSTEM_PROMPT)
        self.assertNotIn("내보내기 후 실행기에서 직접 시작합니다", SYSTEM_PROMPT)
        for name in ("get_state", "view", "simulate", "robot_guide"):      # 프롬프트가 이름으로 부르는 도구는 실제로 있어야 함
            self.assertIn(name, SYSTEM_PROMPT)
            self.assertIn(name, TOOL_NAMES)
        self.assertIn("robot_guide", READ_ONLY_TOOLS)


class BridgeAndMcpTest(SessionTestBase):
    def test_bridge_requires_token(self):
        br = BridgeServer(AgentToolbox(self.new_session())).start()
        try:
            self.assertEqual(len(BridgeClient(br.url, br.token).tools()), len(TOOLS))
            with self.assertRaises(Exception):
                BridgeClient(br.url, "wrong-token").tools()
        finally:
            br.stop()

    def test_mcp_stdio_server_end_to_end(self):
        import importlib.util
        if importlib.util.find_spec("mcp") is None:
            self.skipTest("mcp 패키지 없음 (pip install -e \".[agent]\")")
        br = BridgeServer(AgentToolbox(self.new_session())).start()
        env = dict(os.environ, PYTHONPATH=str(ROOT), PYTHONIOENCODING="utf-8", **br.env())
        p = subprocess.Popen([sys.executable, "-m", "mirobot_sketch.agent.mcp_server"], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env, text=True, encoding="utf-8")

        def rpc(i, method, params):
            p.stdin.write(json.dumps({"jsonrpc": "2.0", "id": i, "method": method, "params": params}) + "\n")
            p.stdin.flush()
            return json.loads(p.stdout.readline())

        try:
            r = rpc(1, "initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                      "clientInfo": {"name": "t", "version": "0"}})
            self.assertEqual(r["result"]["serverInfo"]["name"], "mirobot")
            p.stdin.write(json.dumps({"jsonrpc": "2.0", "method": "notifications/initialized"}) + "\n")
            p.stdin.flush()
            r = rpc(2, "tools/list", {})
            self.assertEqual({t["name"] for t in r["result"]["tools"]}, {t["name"] for t in TOOLS})
            r = rpc(3, "tools/call", {"name": "view", "arguments": {"kind": "paper"}})
            self.assertEqual([c["type"] for c in r["result"]["content"]], ["text", "image"])
            r = rpc(4, "tools/call", {"name": "propose_edits",
                                      "arguments": {"ops": [{"op": "delete", "ids": [10 ** 6]}]}})
            self.assertTrue(r["result"]["isError"])
        finally:
            p.stdin.close()
            p.wait(timeout=10)
            p.stdout.close()
            p.stderr.close()
            br.stop()


class FakeOpenRouter:
    """chat/completions 흉내: 첫 요청엔 도구 호출(view), 다음 요청엔 최종 답변."""

    def __init__(self):
        self.requests = []
        fake = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                fake.requests.append({"auth": self.headers.get("Authorization"), "body": body})
                if len(fake.requests) == 1:
                    msg = {"role": "assistant", "content": "그림을 볼게요.", "tool_calls": [
                        {"id": "c1", "type": "function", "function": {"name": "view", "arguments": '{"kind":"paper"}'}}]}
                else:
                    msg = {"role": "assistant", "content": "확인했습니다."}
                data = json.dumps({"choices": [{"message": msg}], "usage": {"cost": 0.001}}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()


class OpenRouterBackendTest(SessionTestBase):
    def test_tool_loop_with_image_injection(self):
        fake = FakeOpenRouter()
        try:
            with mock.patch.object(bk, "OPENROUTER_URL", fake.url):
                b = bk.OpenRouterBackend(AgentToolbox(self.new_session()), lambda: "sk-test", "test/model")
                events = []
                b.send("결과를 봐줘", events.append)
        finally:
            fake.httpd.shutdown()
            fake.httpd.server_close()
        types_ = [e["type"] for e in events]
        self.assertEqual(types_, ["text", "tool", "tool_done", "text", "done"])
        self.assertEqual(fake.requests[0]["auth"], "Bearer sk-test")
        second = fake.requests[1]["body"]["messages"]
        self.assertEqual(second[-2]["role"], "tool")                    # 도구 결과(글자)
        self.assertEqual(second[-1]["content"][1]["type"], "image_url")  # 그림은 사용자 메시지로
        self.assertIn("$0.0020", events[-1]["info"])

    def test_missing_key_reports_error(self):
        b = bk.OpenRouterBackend(AgentToolbox(SketchSession()), lambda: None)
        events = []
        b.send("안녕", events.append)
        self.assertEqual(events[0]["type"], "error")
        self.assertIn("API 키", events[0]["text"])


class CliEventParsingTest(unittest.TestCase):
    def run_backend(self, backend_cls, lines, exit_code=0):
        br = mock.Mock()
        br.env.return_value = {"MIROBOT_BRIDGE_URL": "http://x", "MIROBOT_BRIDGE_TOKEN": "t"}
        b = backend_cls(br)

        def fake_run(cmd, stdin_text, on_line):
            fake_run.cmd, fake_run.stdin = cmd, stdin_text
            for ev in lines:
                on_line(ev)
            return exit_code, ""

        events = []
        with mock.patch.object(backend_cls, "available", staticmethod(lambda: "cli.exe")), \
                mock.patch.object(bk.ClaudeCodeBackend, "logged_in", staticmethod(lambda exe: True)), \
                mock.patch.object(b, "_run", fake_run):
            b.send("획 줄여줘", events.append)
        return b, events, fake_run

    def test_claude_code_stream_json(self):
        lines = [
            {"type": "system", "subtype": "init", "session_id": "s-1",
             "mcp_servers": [{"name": "mirobot", "status": "connected"}]},
            {"type": "assistant", "message": {"content": [
                {"type": "text", "text": "설정을 바꿀게요."},
                {"type": "tool_use", "name": "mcp__mirobot__set_params", "input": {"detail": "low"}}]}},
            {"type": "user", "message": {"content": [{"type": "tool_result", "is_error": False}]}},
            {"type": "result", "subtype": "success", "is_error": False, "session_id": "s-1", "total_cost_usd": 0.01},
        ]
        b, events, run = self.run_backend(bk.ClaudeCodeBackend, lines)
        self.assertEqual([e["type"] for e in events], ["text", "tool", "tool_done", "done"])
        self.assertEqual(events[1]["name"], "set_params")
        self.assertEqual(b.session_id, "s-1")
        self.assertIn("--strict-mcp-config", run.cmd)
        self.assertEqual(run.cmd[run.cmd.index("--tools") + 1], "")      # 기본 도구 모두 끔
        self.assertEqual(run.stdin, "획 줄여줘")                          # 요청은 stdin으로

    def test_claude_code_auth_error_is_explained_once(self):
        msg = "Failed to authenticate: OAuth session expired"
        lines = [{"type": "assistant", "message": {"content": [{"type": "text", "text": msg}]}},
                 {"type": "result", "is_error": True, "result": msg}]
        _, events, _ = self.run_backend(bk.ClaudeCodeBackend, lines, exit_code=1)
        errors = [e["text"] for e in events if e["type"] == "error"]
        self.assertEqual(len(errors), 1)
        self.assertIn("claude auth login", errors[0])
        self.assertNotIn(msg, errors[0])   # 같은 문구를 두 번 보여주지 않음

    def test_not_logged_in_is_reported_before_running(self):
        b = bk.ClaudeCodeBackend(mock.Mock())
        events = []
        with mock.patch.object(bk.ClaudeCodeBackend, "available", staticmethod(lambda: "claude.exe")), \
                mock.patch.object(bk.ClaudeCodeBackend, "logged_in", staticmethod(lambda exe: False)), \
                mock.patch.object(b, "_run") as run:
            b.send("안녕", events.append)
        run.assert_not_called()   # 모델 호출(사용량) 없이 바로 안내
        self.assertEqual(events[0]["action"], "login")   # 패널이 [로그인 창 열기] 버튼을 붙임

    def test_login_console_runs_login_command_in_new_window(self):
        with mock.patch.object(bk.ClaudeCodeBackend, "available", staticmethod(lambda: "C:/x/claude.exe")):
            argv = bk.ClaudeCodeBackend(mock.Mock()).login_argv()
        self.assertEqual(argv, ["C:/x/claude.exe", "auth", "login"])
        with mock.patch.object(bk.subprocess, "Popen") as popen:
            self.assertTrue(bk.open_login_console(argv))
        cmd = popen.call_args.args[0]
        self.assertIn("auth login", " ".join(cmd))
        self.assertNotIn("CLAUDECODE", popen.call_args.kwargs["env"])
        with mock.patch.object(bk.subprocess, "Popen", side_effect=OSError):
            self.assertFalse(bk.open_login_console(argv))

    def test_host_session_env_is_removed_for_child_cli(self):
        host = {"PATH": "x", "CLAUDECODE": "1", "CLAUDE_CODE_ENTRYPOINT": "desktop",
                "CLAUDE_CODE_SDK_HAS_HOST_AUTH_REFRESH": "1", "ANTHROPIC_BASE_URL": "http://127.0.0.1:1",
                "ANTHROPIC_API_KEY": "user-key", "OPENROUTER_API_KEY": "k"}
        self.assertEqual(set(bk.clean_cli_env(host)), {"PATH", "ANTHROPIC_API_KEY", "OPENROUTER_API_KEY"})
        # 다른 Claude Code 세션 안이 아니면 사용자가 직접 정한 ANTHROPIC_BASE_URL은 유지
        self.assertIn("ANTHROPIC_BASE_URL", bk.clean_cli_env({"ANTHROPIC_BASE_URL": "https://proxy"}))

    def test_codex_json_events_and_resume(self):
        lines = [
            {"type": "thread.started", "thread_id": "th-9"},
            {"type": "item.started", "item": {"type": "mcp_tool_call", "tool": "view", "arguments": {"kind": "paper"}}},
            {"type": "item.completed", "item": {"type": "mcp_tool_call", "tool": "view", "status": "completed"}},
            {"type": "item.completed", "item": {"type": "agent_message", "text": "확인했어요."}},
            {"type": "turn.completed", "usage": {"input_tokens": 10, "output_tokens": 5}},
        ]
        b, events, run = self.run_backend(bk.CodexBackend, lines)
        self.assertEqual([e["type"] for e in events], ["tool", "tool_done", "text", "done"])
        self.assertEqual(b.session_id, "th-9")
        self.assertIn("mcp_servers.mirobot.command=", " ".join(run.cmd))
        self.assertTrue(run.stdin.startswith("당신은"))   # 첫 요청에 시스템 지시 포함
        # 같은 백엔드로 다음 요청을 보내면 저장한 세션으로 이어감
        seen = {}
        with mock.patch.object(bk.CodexBackend, "available", staticmethod(lambda: "cli.exe")), \
                mock.patch.object(b, "_run", lambda cmd, s, f: (seen.update(cmd=cmd, stdin=s), (0, ""))[1]):
            b.send("다음", lambda e: None)
        self.assertEqual(seen["cmd"][1:4], ["exec", "resume", "th-9"])
        self.assertEqual(seen["stdin"], "다음")   # 이어갈 때는 시스템 지시를 다시 붙이지 않음


if __name__ == "__main__":
    unittest.main()
