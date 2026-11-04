"""펜을 벽 쪽으로 눕힌 자세(시작 자세 설계 T1)의 기구학 검사 테스트.

    python -m unittest tests.test_pen_pose -v
"""

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mirobot_sketch import mirobot_sim as ms  # noqa: E402
from mirobot_sketch import pen_pose as pp  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()


class PenRotationTest(unittest.TestCase):
    def test_level_pen_points_at_the_wall(self):
        self.assertTrue(np.allclose(pp.pen_rotation(0.0)[:, 2], [1, 0, 0], atol=1e-4))

    def test_tilt_lowers_the_pen_tip(self):
        t = np.radians(15.0)
        self.assertTrue(np.allclose(pp.pen_rotation(15.0)[:, 2], [np.cos(t), 0, -np.sin(t)], atol=1e-4))


class SingularityTest(unittest.TestCase):
    def test_home_pose_is_far_from_wrist_singularity(self):
        # 홈: J4 축(팔뚝, +X)과 J6 축(플랜지, 아래)이 직각
        self.assertAlmostEqual(pp.singularity_deg(np.zeros(6)), 90.0, delta=0.5)

    def test_flange_along_forearm_is_singular(self):
        # J2·J3가 0이면 팔뚝이 수평이라 J5=-90°에서 플랜지 축이 J4 축과 겹침
        self.assertLess(pp.singularity_deg(np.radians([0, 0, 0, 0, -90, 0])), 0.5)


class WrapTest(unittest.TestCase):
    def test_j5_beyond_minus_180_stays_inside_its_range(self):
        # J5 범위는 -200°~+30°: -190°를 ±180°로 접어 +170°(범위 밖)로 바꾸면 안 됨
        q = pp._wrap(np.radians([0, 0, 0, 0, -190, 0]))
        self.assertAlmostEqual(np.degrees(q[4]), -190.0, places=6)

    def test_turns_beyond_a_full_circle_come_back_into_range(self):
        q = pp._wrap(np.radians([370, 0, 0, 0, 0, 0]))
        self.assertAlmostEqual(np.degrees(q[0]), 10.0, places=6)


class SimulateOrientationTest(unittest.TestCase):
    def test_simulate_keeps_the_requested_pen_orientation(self):
        rot = pp.pen_rotation(15.0)
        targets = [(np.array([180.0, 0.0, 230.0]), "start", False, 0.0),
                   (np.array([180.0, 20.0, 230.0]), "line", True, 300.0)]
        result = ms.simulate(targets, step_mm=5.0, r_goal=rot, q0=pp.LEVEL_SEED)
        self.assertEqual(result["failures"], [])
        flange, _ = ms.fk(result["samples"][-1][0])
        self.assertTrue(np.allclose(flange[:3, 2], rot[:, 2], atol=1e-3))


class StudyTest(unittest.TestCase):
    def test_level_pen_at_paper_center_is_near_singular(self):
        r = pp.study(CFG, center_x=180.0, center_z=230.0, tilt_down_deg=0.0)
        self.assertEqual(r["unreachable"], 0)
        self.assertLess(r["min_singularity_deg"], 8.0)

    def test_small_tilt_only_moves_the_singularity_inside_the_area(self):
        # 그리기 범위 안에서 팔뚝 기울기가 위아래로 약 ±20° 바뀌어, 펜을 15° 기울여도 어딘가에서 팔뚝과 겹침
        r = pp.study(CFG, center_x=180.0, center_z=230.0, tilt_down_deg=15.0)
        self.assertLess(r["min_singularity_deg"], 5.0)

    def test_large_tilt_keeps_the_whole_area_away_from_the_singularity(self):
        r = pp.study(CFG, center_x=180.0, center_z=230.0, tilt_down_deg=40.0)
        self.assertGreater(r["min_singularity_deg"], 15.0)
        self.assertGreater(r["min_margin_deg"], 10.0)

    def test_flange_too_close_to_the_base_breaks_the_j2_limit(self):
        r = pp.study(CFG, center_x=140.0, center_z=230.0, tilt_down_deg=40.0)
        self.assertLess(r["min_margin_deg"], 0.0)
        self.assertEqual(r["worst_axis"], "Y(J2)")

    def test_wall_distance_adds_the_pen_reach(self):
        # 펜 끝이 종이에 닿는 X = 플랜지 X + 펜 길이 * cos(기울기)
        self.assertAlmostEqual(pp.wall_x(center_x=180.0, pen_len_mm=60.0, tilt_down_deg=0.0), 240.0)
        self.assertAlmostEqual(pp.wall_x(center_x=180.0, pen_len_mm=60.0, tilt_down_deg=60.0), 210.0)


if __name__ == "__main__":
    unittest.main()
