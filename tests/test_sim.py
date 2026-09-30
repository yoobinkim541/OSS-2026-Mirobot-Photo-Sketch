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


class FastSolverTest(unittest.TestCase):
    """속도를 위해 바꾼 FK·IK가 예전(스칼라) 구현과 같은 결과를 내는지."""

    @staticmethod
    def reference_ik(target_mm, q0, r_goal=ms.HOME_ROT, iters=100, tol_mm=1e-3):
        # 배치 이전의 구현 그대로 (기준값)
        q = np.array(q0, float)
        w_rot = 100.0
        for _ in range(iters):
            t, _ = ms.fk(q)
            e = np.concatenate([target_mm - t[:3, 3], w_rot * ms._rot_error(t[:3, :3], r_goal)])
            if np.linalg.norm(e[:3]) < tol_mm and np.linalg.norm(e[3:]) < w_rot * 1e-5:
                break
            jac = np.zeros((6, 6))
            h = 1e-6
            for i in range(6):
                dq = q.copy()
                dq[i] += h
                td, _ = ms.fk(dq)
                ed = np.concatenate([target_mm - td[:3, 3], w_rot * ms._rot_error(td[:3, :3], r_goal)])
                jac[:, i] = (e - ed) / h
            q = q + jac.T @ np.linalg.solve(jac @ jac.T + 0.5 ** 2 * np.eye(6), e)
        t, _ = ms.fk(q)
        return q, float(np.linalg.norm(target_mm - t[:3, 3]))

    def test_batched_fk_matches_the_chain_fk(self):
        rng = np.random.default_rng(3)
        qs = rng.uniform(ms.JOINT_LIMITS_RAD[:, 0], ms.JOINT_LIMITS_RAD[:, 1], size=(25, 6))
        batch = ms.fk_batch(qs)
        for q, t in zip(qs, batch):
            self.assertLess(np.abs(t - ms.fk(q)[0]).max(), 1e-9)

    def test_batched_ik_matches_the_reference_solver_from_the_same_start(self):
        q0 = np.zeros(6)
        for target in (np.array([198.668, 0.0, 230.477]), np.array([190.0, -25.0, 240.0]),
                       np.array([185.0, 30.0, 215.0])):
            q, err, ok = ms.ik(target, q0)
            ref_q, ref_err = self.reference_ik(target, q0)
            self.assertLess(np.abs(q - ref_q).max(), 1e-8)
            self.assertAlmostEqual(err, ref_err, places=8)
            self.assertTrue(ok)

    def test_simulate_with_extrapolated_start_gives_the_same_joint_path(self):
        targets = [(np.array([198.668, 0.0, 230.477]), "start", False, 0.0)]
        for i, (y, z) in enumerate([(0, 0), (25, 15), (-20, 30), (10, -25), (0, 0)], start=1):
            targets.append((np.array([198.668, float(y), 230.477 + z]), f"move {i}", True, 300.0))
        res = ms.simulate(targets)
        q = np.zeros(6)
        ref = []
        prev = targets[0][0]
        for xyz, *_ in targets:
            n = max(1, int(np.ceil(np.linalg.norm(xyz - prev) / 1.0)))
            for k in range(1, n + 1):
                q, _ = self.reference_ik(prev + (xyz - prev) * (k / n), q)
                ref.append(q)
            prev = xyz
        got = np.array([s[0] for s in res["samples"]])
        self.assertEqual(len(got), len(ref))
        self.assertLess(np.abs(got - np.array(ref)).max(), 1e-4)     # 해는 같고 시작 추정만 다름 (rad)
        self.assertEqual(ms.verdict(res).split(":")[0], "PASS")

    def test_progress_callback_can_cancel_the_simulation(self):
        targets = [(np.array([198.668, 0.0, 230.477]), "start", False, 0.0)]
        targets += [(np.array([198.668, float(i), 230.477]), f"m{i}", True, 300.0) for i in range(1, 40)]
        calls = []

        def cancel_at_five(done, total):
            calls.append(done)
            if done >= 5:
                raise ms.SimulationCancelled()

        with self.assertRaises(ms.SimulationCancelled):
            ms.simulate(targets, progress=cancel_at_five)
        self.assertEqual(calls, [0, 1, 2, 3, 4, 5])                   # 남은 34개 명령은 계산하지 않음

    def test_progress_callback_reports_every_command(self):
        targets = [(np.array([198.668, 0.0, 230.477]), "start", False, 0.0),
                   (np.array([198.668, 10.0, 230.477]), "a", True, 300.0),
                   (np.array([198.668, 10.0, 240.477]), "b", True, 300.0)]
        seen = []
        ms.simulate(targets, progress=lambda done, total: seen.append((done, total)))
        self.assertEqual(seen, [(0, 3), (1, 3), (2, 3)])


if __name__ == "__main__":
    unittest.main()
