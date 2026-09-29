"""3D 시뮬레이터(기구학·관절 한계) 테스트.

    python -m unittest discover -s tests -v
"""

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mirobot_sketch import mirobot_sim as ms  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()


class KinematicsTest(unittest.TestCase):
    def test_home_matches_controller_tcp(self):
        t, _ = ms.fk(np.zeros(6))
        self.assertTrue(np.allclose(t[:3, 3], ms.HOME_TCP_MM, atol=1e-3))

    def test_tool_offset_is_flange_distance(self):
        # URDF 손목 중심 -> 플랜지 약 24.5mm (링크 길이 반올림 차이 0.23mm)
        self.assertAlmostEqual(ms.TOOL_OFFSET_MM[2], 24.5, delta=0.1)
        self.assertLess(abs(ms.TOOL_OFFSET_MM[0]), 0.3)

    def test_ik_roundtrip(self):
        q_true = np.radians([10, 5, -15, 0, 10, -10])
        t, _ = ms.fk(q_true)
        q, err, ok = ms.ik(t[:3, 3], np.zeros(6), r_goal=t[:3, :3])
        self.assertTrue(ok)
        self.assertLess(err, 0.05)


class PathTest(unittest.TestCase):
    def test_orientation_f_passes(self):
        _, strokes = ms.de.load_strokes(ROOT / "trajectories" / "orientation-test-F.json")
        result = ms.simulate(ms.plan_targets(strokes, CFG))
        self.assertEqual(ms.verdict(result), "PASS")

    def test_high_on_paper_hits_b_axis_limit(self):
        # 종이 중심에서 80mm 위: B(J5)가 30도 한계를 넘음 (실물 하트 시도의 Soft limit:B와 같은 축)
        strokes = [[(0.0, 60.0), (0.0, 80.0)]]
        result = ms.simulate(ms.plan_targets(strokes, CFG))
        self.assertTrue(ms.verdict(result).startswith("FAIL"))
        self.assertEqual(result["violations"][0]["axis"], "B(J5)")

    def test_executor_box_is_safe(self):
        corners = [[(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0), (-50.0, -50.0)]]
        result = ms.simulate(ms.plan_targets(corners, CFG), step_mm=5.0)
        self.assertFalse(ms.verdict(result).startswith("FAIL"))


class TrajectoryCmdTest(unittest.TestCase):
    def test_points_carry_command_index(self):
        de = ms.de
        strokes = [[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0)]]
        res = ms.simulate(ms.plan_targets(strokes, CFG))
        doc = ms.trajectory_doc(res, CFG, "t")
        cmds = [p["cmd"] for p in doc["points"]]
        n = len(de.Planner(CFG).plan(strokes))
        self.assertEqual(cmds[0], -1)
        self.assertEqual(sorted(set(cmds)), list(range(-1, n)))        # 명령마다 점이 1개 이상
        self.assertEqual(cmds, sorted(cmds))                           # 순서대로 늘어남
        self.assertEqual(doc["command_count"], n)
        self.assertEqual(len(doc["cmd_feed_mm_min"]), n)
        self.assertEqual(doc["cmd_feed_mm_min"][0], CFG["feeds_mm_per_min"]["approach"])


if __name__ == "__main__":
    unittest.main()
