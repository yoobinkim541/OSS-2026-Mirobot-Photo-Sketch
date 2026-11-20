"""로봇 실행기 테스트 (실제 시리얼 없이 가짜 컨트롤러 사용).

    python -m unittest discover -s tests -v
"""

import copy
import json
import re
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mirobot_sketch import draw_executor as de  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()


class FakeSerial:
    """명령마다 정해진 응답을 돌려주는 가짜 Mirobot."""

    IDLE = ("<Idle,Angle(ABCDXYZ):0,0,0,0,0,0,0,"
            "Cartesian coordinate(XYZ RxRyRz):198.668,0.000,230.477,0.000,0.000,0.000,"
            "Pump PWM:0,Valve PWM:0,Motion_MODE:0>")

    def __init__(self, fail_at=None, fail_msg="Soft limit:B"):
        self.lines = []
        self.pending = []
        self.fail_at = fail_at
        self.fail_msg = fail_msg
        self.written = []
        self.x, self.y, self.z = "198.668", "0.000", "230.477"
        self.held_x = None

    def _idle(self):
        x = self.held_x if self.held_x is not None else self.x
        self.held_x = None
        return ("<Idle,Angle(ABCDXYZ):0,0,0,0,0,0,0,"
                f"Cartesian coordinate(XYZ RxRyRz):{x},{self.y},{self.z},0.000,0.000,0.000,"
                "Pump PWM:0,Valve PWM:0,Motion_MODE:0>")

    def write(self, data):
        text = data.decode().strip()
        self.written.append(text)
        m = re.search(r"^M20 .*X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", text)
        if m:
            if m.group(1) != self.x:
                self.held_x = self.x
            self.x, self.y, self.z = m.groups()
        if text == "?":
            self.pending.append(self._idle())
            return
        n = sum(1 for w in self.written if w != "?")
        self.pending.append(self.fail_msg if n == self.fail_at else "ok")

    def readline(self):
        return (self.pending.pop(0) + "\r\n").encode() if self.pending else b""

    def reset_input_buffer(self):
        self.pending.clear()

    def close(self):
        pass


SQUARE = [[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0), (-10.0, -10.0)]]


class PlannerTest(unittest.TestCase):
    def test_radial_retraction_uses_distance_and_caps_at_three_mm(self):
        cfg = copy.deepcopy(CFG)
        cfg['plane_compensation'].update(a_per_mm_y=0.0, b_per_mm_z=0.0,
                                         radial_retract_max_mm=3.0, radial_retract_radius_mm=60.0)
        p = de.Planner(cfg)
        center = p.pose(0, 0, True)[0]
        for x, y, amount in [(0, 0, 0), (30, 0, 1.5), (0, -30, 1.5),
                             (36, 48, 3), (90, 0, 3)]:
            self.assertAlmostEqual(p.pose(x, y, True)[0], center - amount)
        self.assertAlmostEqual(p.pose(30, 0, False)[0] - p.pose(30, 0, True)[0],
                               -cfg['pen']['up_clearance_mm'])

    def test_last_10mm_approach_and_first_10mm_retract_are_normal_and_slow(self):
        cfg = self._high_pen_up_cfg(20.0)
        cfg['pen']['slow_zone_mm'] = 10.0
        cfg['pen']['down_extra_mm'] = 0.0
        cfg['plane_compensation']['a_per_mm_y'] = 0.013
        cfg['plane_compensation']['b_per_mm_z'] = -0.02
        planner = de.Planner(cfg)
        cmds = planner.plan([[(3.0, 4.0), (5.0, 7.0)]])
        by_label = {label: line for line, label in cmds}
        def pose(label):
            return tuple(float(v) for v in re.search(r'X([-\d.]+) Y([-\d.]+) Z([-\d.]+).* F([\d.]+)', by_label[label]).groups())
        descent, down, end, lift = (pose('stroke 0 descend'), pose('stroke 0 pen-down'),
                                    pose('stroke 0 draw'), pose('stroke 0 lift'))
        self.assertEqual(descent[1:3], down[1:3])
        self.assertAlmostEqual(abs(descent[0] - down[0]), 10.0, places=3)
        self.assertEqual(end[1:3], lift[1:3])
        self.assertAlmostEqual(abs(end[0] - lift[0]), 10.0, places=3)
        self.assertEqual(down[3], cfg['feeds_mm_per_min']['approach'])
        self.assertEqual(lift[3], cfg['feeds_mm_per_min']['approach'])

    def test_pen_sequence_and_retract_direction(self):
        planner = de.Planner(CFG)
        cmds = planner.plan(SQUARE)
        labels = [label for _, label in cmds]
        self.assertEqual(labels[0], "center pen-up")
        self.assertEqual(labels[1:3], ["stroke 0 travel", "stroke 0 pen-down"])
        self.assertEqual(labels[-2:], ["stroke 0 pen-up", "return center pen-up"])
        up_x = planner.pose(0, 0, False)[0]
        down_x = planner.pose(0, 0, True)[0]
        self.assertAlmostEqual(down_x, CFG["paper_center_tcp_mm"]["x"])
        self.assertAlmostEqual(down_x - up_x, CFG["pen"]["up_clearance_mm"])  # 펜업은 벽에서 멀어짐

    def test_end_backoff_moves_only_first_and_last_contact_points_away_from_paper(self):
        cfg = copy.deepcopy(CFG)
        cfg["pen"].update(start_backoff_mm=0.4, end_backoff_mm=0.6)
        base = de.Planner(CFG).plan(SQUARE)
        backed = de.Planner(cfg).plan(SQUARE)
        self.assertEqual([l for _, l in base], [l for _, l in backed])     # 명령 수·순서 그대로
        xyz = lambda line: tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())
        sign = CFG["pen"]["retract_x_sign"]
        draws = [i for i, (_, l) in enumerate(base) if l == "stroke 0 draw"]
        down = next(i for i, (_, l) in enumerate(base) if l == "stroke 0 pen-down")
        expected = {down: 0.4, draws[-1]: 0.6}
        for i, ((a, _), (b, _)) in enumerate(zip(base, backed)):
            pa, pb = xyz(a), xyz(b)
            self.assertEqual(pa[1:], pb[1:])
            self.assertAlmostEqual((pb[0] - pa[0]) * sign, expected.get(i, 0.0), places=3)

    def test_end_backoff_outside_zero_to_one_mm_is_refused(self):
        for bad in (-0.1, 1.5):
            cfg = copy.deepcopy(CFG)
            cfg["pen"]["end_backoff_mm"] = bad
            with self.assertRaises(ValueError):
                de.Planner(cfg)

    def test_pen_lifts_in_place_before_the_next_move(self):
        cfg = self._high_pen_up_cfg(20.0)
        cfg["pen"]["down_extra_mm"] = 8.0
        cmds = de.Planner(cfg).plan(SQUARE)
        yz = lambda line: re.search(r"Y([-\d.]+) Z([-\d.]+)", line).groups()
        draw = next(line for line, label in reversed(cmds) if label == "stroke 0 draw")
        lift = next(line for line, label in cmds if label == "stroke 0 lift")
        up = next(line for line, label in cmds if label == "stroke 0 pen-up")
        self.assertEqual(yz(lift), yz(draw))          # 뗄 때 Y·Z는 그대로, X만 뒤로
        self.assertEqual(yz(up), yz(draw))
        labels = [label for _, label in cmds]
        self.assertLess(labels.index("stroke 0 pen-up"), labels.index("return center pen-up"))

    def test_travel_between_strokes_stays_off_the_paper(self):
        cfg = self._high_pen_up_cfg(20.0)
        cfg["pen"]["down_extra_mm"] = 8.0
        cfg["plane_compensation"]["b_per_mm_z"] = 0.05
        strokes = [[(0.0, 0.0), (10.0, 0.0)], [(0.0, 30.0), (10.0, 30.0)]]
        planner = de.Planner(cfg)
        cmds = planner.plan(strokes)
        labels = [label for _, label in cmds]
        up_at = labels.index("stroke 0 pen-up")
        self.assertEqual(labels[up_at + 1], "stroke 1 travel")
        self.assertLess(labels.index("stroke 1 travel"), labels.index("stroke 1 pen-down"))
        x_of = lambda line: float(re.search(r"X([-\d.]+)", line).group(1))
        yz = lambda line: re.search(r"Y([-\d.]+) Z([-\d.]+)", line).groups()
        sign = cfg["pen"]["retract_x_sign"]
        up = next(line for line, label in cmds if label == "stroke 0 pen-up")
        travel = next(line for line, label in cmds if label == "stroke 1 travel")
        arrive = planner.pose(0.0, 30.0, False)
        self.assertNotAlmostEqual(x_of(up), arrive[0], places=2)
        self.assertEqual(yz(travel), (f"{arrive[1]:.3f}", f"{arrive[2]:.3f}"))
        self.assertAlmostEqual(x_of(travel), planner._farther_x(x_of(up), arrive[0]), places=3)
        self.assertGreaterEqual(sign * (x_of(travel) - x_of(up)), -1e-6)
        last = next(line for line, label in cmds if label == "stroke 1 pen-up")
        back = next(line for line, label in cmds if label == "return center pen-up")
        home = planner.pose(0.0, 0.0, False)
        self.assertEqual(yz(back), (f"{home[1]:.3f}", f"{home[2]:.3f}"))
        self.assertAlmostEqual(x_of(back), planner._farther_x(x_of(last), home[0]), places=3)
        self.assertGreaterEqual(sign * (x_of(back) - x_of(last)), -1e-6)
    def _high_pen_up_cfg(self, up=10.0):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["up_clearance_mm"] = up
        return cfg

    def test_high_pen_up_descends_fast_then_slow_only_near_paper(self):
        cfg = self._high_pen_up_cfg(10.0)
        f = cfg["feeds_mm_per_min"]
        planner = de.Planner(cfg)
        cmds = planner.plan(SQUARE)
        labels = [label for _, label in cmds]
        self.assertEqual(labels[1:4], ["stroke 0 travel", "stroke 0 descend", "stroke 0 pen-down"])
        x_of = lambda line: float(re.search(r"X([-\d.]+)", line).group(1))
        feed_of = lambda line: float(re.search(r"F([\d.]+)", line).group(1))
        travel, descend, down = cmds[1][0], cmds[2][0], cmds[3][0]
        sign = cfg["pen"]["retract_x_sign"]
        contact = planner.pose(SQUARE[0][0][0], SQUARE[0][0][1], True)[0]
        self.assertAlmostEqual(sign * (x_of(travel) - contact), 10.0, places=2)           # 펜업은 10 mm 위
        self.assertAlmostEqual(sign * (x_of(descend) - contact), cfg["pen"]["slow_zone_mm"], places=2)
        self.assertEqual([feed_of(travel), feed_of(descend), feed_of(down)], [f["travel"], f["travel"], f["approach"]])
        up = next(line for line, label in cmds if label == "stroke 0 pen-up")
        self.assertEqual(feed_of(up), f["travel"])                                         # 올리는 건 빠르게
        self.assertAlmostEqual(sign * (x_of(up) - planner.pose(*SQUARE[0][-1], True)[0]), 10.0, places=2)

    def test_pen_up_within_slow_zone_keeps_single_slow_approach(self):
        planner = de.Planner(CFG)                                   # 기본 설정: 펜업 2.5 mm = 느린 구간
        labels = [label for _, label in planner.plan(SQUARE)]
        self.assertNotIn("stroke 0 descend", labels)
        self.assertEqual(planner.plan(SQUARE)[-2][0].split(" F")[1], str(int(CFG["feeds_mm_per_min"]["approach"])))

    def test_air_mode_has_no_descent_waypoint_even_with_high_pen_up(self):
        planner = de.Planner(self._high_pen_up_cfg(10.0), air=True)
        self.assertNotIn("stroke 0 descend", [label for _, label in planner.plan(SQUARE)])

    def test_return_to_origin_puts_the_pen_back_on_the_paper_center(self):
        cfg = self._high_pen_up_cfg(10.0)
        planner = de.Planner(cfg)
        cmds = planner.return_to_origin()
        self.assertEqual([label for _, label in cmds], ["return origin descend", "return origin"])
        x_of = lambda line: float(re.search(r"X([-\d.]+)", line).group(1))
        feed_of = lambda line: float(re.search(r"F([\d.]+)", line).group(1))
        c = cfg["paper_center_tcp_mm"]
        sign = cfg["pen"]["retract_x_sign"]
        self.assertAlmostEqual(x_of(cmds[-1][0]), c["x"], places=3)                      # 처음 자세: 종이 중심, 펜 끝이 닿음
        self.assertAlmostEqual(sign * (x_of(cmds[0][0]) - c["x"]), cfg["pen"]["slow_zone_mm"], places=3)
        self.assertEqual(feed_of(cmds[-1][0]), cfg["feeds_mm_per_min"]["approach"])       # 닿기 직전은 천천히
        for line, _ in cmds:
            self.assertIn(f"Y{c['y']:.3f}", line)
            self.assertIn(f"Z{c['z']:.3f}", line)

    def test_return_to_origin_is_one_slow_move_when_pen_up_is_within_the_slow_zone(self):
        cmds = de.Planner(CFG).return_to_origin()                                        # 기본 설정: 펜업 2.5 = 느린 구간
        self.assertEqual([label for _, label in cmds], ["return origin"])

    def test_return_to_origin_is_empty_in_air_mode(self):
        self.assertEqual(de.Planner(self._high_pen_up_cfg(10.0), air=True).return_to_origin(), [])

    def test_axis_signs(self):
        planner = de.Planner(CFG)
        c = CFG["paper_center_tcp_mm"]
        ry, rz = planner.robot_yz(10, 5)
        self.assertAlmostEqual(ry, c["y"] + CFG["paper_x_to_robot_y_sign"] * 10)
        self.assertAlmostEqual(rz, c["z"] + 5)

    def test_drawing_warp_moves_sent_points_and_limit_check_uses_them(self):
        cfg = copy.deepcopy(CFG)
        from mirobot_sketch.limits import Region
        x_max = Region.from_cfg(cfg["limits"]).x_max
        outside = [[(x_max + 2.0, 0.0), (x_max + 2.0, 1.0)]]
        self.assertTrue(de.check_limits(outside, cfg))
        cfg["drawing_warp"] = {"x": [0.9, 0.0, 0.0, 0.002, 0.0], "y": [0.0, 1.1, 0.0, 0.0, 0.0]}
        c = cfg["paper_center_tcp_mm"]
        ry, rz = de.Planner(cfg).robot_yz(10.0, 5.0)
        self.assertAlmostEqual(ry, c["y"] + cfg["paper_x_to_robot_y_sign"] * (0.9 * 10.0 + 0.002 * 50.0))
        self.assertAlmostEqual(rz, c["z"] + cfg["paper_y_to_robot_z_sign"] * 5.5)
        self.assertEqual(de.check_limits(outside, cfg), [])      # 실제로 보낼 좌표(0.9배)는 범위 안
        line = [[(0.0, 0.0), (20.0, 0.0)]]
        self.assertAlmostEqual(de.estimate_time(line, cfg)["pen_down_mm"], 18.0, places=1)   # 시간도 보낼 길이로
        cfg["drawing_warp"]["x"][0] = 2.0
        with self.assertRaises(ValueError):
            de.Planner(cfg)

    def test_plane_compensation(self):
        cfg = copy.deepcopy(CFG)
        cfg["plane_compensation"]["a_per_mm_y"] = 0.01
        planner = de.Planner(cfg)
        x_left = planner.pose(-50, 0, True)[0]
        x_right = planner.pose(50, 0, True)[0]
        self.assertAlmostEqual(abs(x_left - x_right), 1.0)

    def test_out_of_limits_rejected(self):
        self.assertTrue(de.check_limits([[(0, 0), (80, 0)]], CFG))
        self.assertFalse(de.check_limits(SQUARE, CFG))

    def test_gcode_format(self):
        line = de.Planner(CFG).plan(SQUARE)[0][0]
        self.assertRegex(line, r"^M20 G90 G01 X[-\d.]+ Y[-\d.]+ Z[-\d.]+ A[-\d.]+ B[-\d.]+ C[-\d.]+ F\d+$")

    def test_j1_offset_turns_targets_against_arrival_and_takes_up_where_j1_reverses(self):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["j1_direction_offset_deg"] = 0.6
        stroke = [[(0.0, 0.0), (10.0, 0.0), (0.0, 0.0)]]        # 가로로 갔다가 같은 줄로 되돌아옴
        planner = de.Planner(cfg)
        cmds = planner.plan(stroke)
        base = de.Planner(CFG).plan(stroke)
        labels = [label for _, label in cmds]
        self.assertEqual(labels.count("stroke 0 j1-takeup draw"), 1)              # 종이 위에서는 되돌아오는 점에서만
        self.assertIn("stroke 0 j1-approach travel", labels)                       # 첫 방향은 공중에서 미리 맞춤
        self.assertEqual([l for l in labels if "j1-" not in l], [l for _, l in base])
        self.assertEqual(planner.plan(stroke), cmds)                               # 다시 계획해도 같음
        xyz = lambda line: tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())
        j1 = lambda p: np.degrees(np.arctan2(p[1], p[0]))
        sent = {label: [] for label in labels}
        for line, label in cmds:
            sent[label].append(xyz(line))
        wanted = [xyz(line) for line, label in base if label == "stroke 0 draw"]
        out_and_back = sent["stroke 0 draw"]
        going = 1 if j1(wanted[0]) > j1(xyz(base[2][0])) else -1                  # 첫 그리기 이동의 J1 방향
        self.assertAlmostEqual(j1(out_and_back[0]) - j1(wanted[0]), -0.3 * going, places=3)
        self.assertAlmostEqual(j1(out_and_back[1]) - j1(wanted[1]), 0.3 * going, places=3)
        takeup = sent["stroke 0 j1-takeup draw"][0]                                # 되돌아오는 점에 새 보정값으로 먼저 감
        self.assertAlmostEqual(j1(takeup) - j1(wanted[0]), 0.3 * going, places=3)
        self.assertAlmostEqual(np.hypot(*takeup[:2]), np.hypot(*wanted[0][:2]), places=2)

    def test_j1_offset_keeps_j1_still_while_the_pen_goes_down_touches_and_lifts(self):
        cfg = self._high_pen_up_cfg(10.0)
        cfg["pen"].update(j1_direction_offset_deg=0.6, start_backoff_mm=0.4, end_backoff_mm=0.4)
        cmds = de.Planner(cfg).plan([[(20.0, 10.0), (30.0, 10.0), (30.0, 16.0)]])   # 종이 중심에서 떨어진 획
        j1 = {}
        for line, label in cmds:
            x, y = (float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+)", line).groups())
            j1.setdefault(label, []).append(np.degrees(np.arctan2(y, x)))
        touching = (" pen-down", " draw", " lift")
        self.assertFalse([l for l in j1 if "j1-takeup" in l and l.endswith(touching)])   # 닿아 있을 때는 메우지 않음
        down = [j1["stroke 0 travel"][0], j1["stroke 0 descend"][0], j1["stroke 0 pen-down"][0]]
        self.assertLess(np.ptp(down), 2e-3)                                        # 내려가는 동안 J1 그대로
        up = [j1["stroke 0 draw"][-1], j1["stroke 0 lift"][0], j1["stroke 0 pen-up"][0]]
        self.assertLess(np.ptp(up), 2e-3)                                          # 들어올리는 동안 J1 그대로

    def test_j1_offset_in_air_mode_follows_the_drawing_shape(self):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["j1_direction_offset_deg"] = 0.6
        stroke = [[(20.0, 10.0), (30.0, 10.0), (30.0, 16.0), (20.0, 16.0)]]
        pick = lambda cmds: [tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+)", line).groups())
                             for line, label in cmds if label == "stroke 0 draw"]
        on, off = pick(de.Planner(cfg, air=True).plan(stroke)), pick(de.Planner(CFG, air=True).plan(stroke))
        for a, b in zip(on, off):                     # 같은 반지름에서 J1만 보정의 절반만큼 돌아감 (모양이 줄지 않음)
            self.assertAlmostEqual(np.hypot(*a), np.hypot(*b), places=2)
            self.assertAlmostEqual(abs(np.degrees(np.arctan2(a[1], a[0]) - np.arctan2(b[1], b[0]))), 0.3, places=2)

    def test_j1_offset_plan_keeps_its_commands_when_end_backoff_changes(self):
        # 그리는 중 끝점 물림을 바꿔 다시 계획할 때(draw_job.set_pen_correction) 명령 수·순서가 같아야 바꿔 끼울 수 있다
        cfg = self._high_pen_up_cfg(10.0)
        cfg["pen"]["j1_direction_offset_deg"] = 0.6
        strokes = [[(20.0, 10.0), (30.0, 10.0), (20.0, 12.0)], [(-30.0, -20.0), (-40.0, -20.0)]]
        backed = copy.deepcopy(cfg)
        backed["pen"].update(start_backoff_mm=1.0, end_backoff_mm=1.0)
        self.assertEqual([l for _, l in de.Planner(cfg).plan(strokes)], [l for _, l in de.Planner(backed).plan(strokes)])

    def test_j1_offset_command_count_estimate_matches_plan(self):
        cfg = self._high_pen_up_cfg(10.0)
        cfg["pen"]["j1_direction_offset_deg"] = 0.6
        strokes = SQUARE + [[(20.0, 10.0), (30.0, 10.0), (20.0, 12.0)]]
        self.assertEqual(de.estimate_time(strokes, cfg)["command_count"], len(de.Planner(cfg).plan(strokes)))

    def test_j1_offset_beyond_three_degrees_is_refused(self):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["j1_direction_offset_deg"] = 3.5
        with self.assertRaises(ValueError):
            de.Planner(cfg)



class ResumeTest(unittest.TestCase):
    STROKES = [[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0)], [(20.0, 0.0), (30.0, 0.0)]]

    def setUp(self):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["up_clearance_mm"] = 10.0                 # 느린 구간이 따로 있는 2단 내림
        self.planner = de.Planner(cfg)
        self.cmds = self.planner.plan(self.STROKES)
        self.labels = [label for _, label in self.cmds]

    @staticmethod
    def xyz(line):
        return tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())

    def test_stop_with_the_pen_on_paper_puts_it_back_on_the_same_point_then_continues(self):
        k = self.labels.index("stroke 0 draw") + 1             # 첫 그리기 이동까지 보내고 멈춤 (다음도 그리기)
        prefix, start = de.resume_commands(self.planner, self.cmds, k)
        self.assertEqual(start, k)                              # 남은 그리기부터, 그은 부분은 다시 긋지 않음
        (slow, _), (touch, _) = prefix
        contact = self.xyz(self.cmds[k - 1][0])
        self.assertEqual(self.xyz(touch), contact)              # 멈춘 접촉점에 다시 댐
        self.assertEqual(self.xyz(slow)[1:], contact[1:])       # 바로 위에서 종이 쪽으로만 내려옴
        self.assertAlmostEqual(self.xyz(slow)[0] - contact[0], self.planner.slow_dx, places=3)
        self.assertIn(f"F{self.planner.feeds['approach']}", touch)   # 닿는 구간은 느리게

    def test_stop_after_the_last_draw_of_a_stroke_skips_its_lift_and_goes_on_to_the_next(self):
        k = len(self.labels) - 1 - self.labels[::-1].index("stroke 0 draw") + 1
        prefix, start = de.resume_commands(self.planner, self.cmds, k)
        self.assertEqual(prefix, [])                            # 이미 뗐으니 다시 대지도, 다시 떼지도 않음
        self.assertEqual(self.labels[start], "stroke 1 travel")

    def test_stop_before_the_pen_goes_down_comes_fast_to_the_slow_zone_first(self):
        k = self.labels.index("stroke 1 pen-down")
        prefix, start = de.resume_commands(self.planner, self.cmds, k)
        self.assertEqual(start, k)
        (slow, _), = prefix
        target = self.xyz(self.cmds[k][0])
        self.assertEqual(self.xyz(slow)[1:], target[1:])
        self.assertIn(f"F{self.planner.feeds['travel']}", slow)

    def test_stop_in_the_air_needs_nothing_before_continuing(self):
        k = self.labels.index("stroke 1 travel")
        self.assertEqual(de.resume_commands(self.planner, self.cmds, k), ([], k))
        air = de.Planner(self.planner.cfg, air=True)
        cmds = air.plan(self.STROKES)
        self.assertEqual(de.resume_commands(air, cmds, 5), ([], 5))



class TimingTest(unittest.TestCase):
    def test_command_count_matches_plan(self):
        strokes = SQUARE + [[(0.0, 0.0), (5.0, 5.0)]]
        t = de.estimate_time(strokes, CFG)
        self.assertEqual(t["command_count"], len(de.Planner(CFG).plan(strokes)))

    def test_command_count_and_lift_time_match_plan_for_high_pen_up(self):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["up_clearance_mm"] = 10.0
        strokes = SQUARE + [[(0.0, 0.0), (5.0, 5.0)]]
        t = de.estimate_time(strokes, cfg)
        self.assertEqual(t["command_count"], len(de.Planner(cfg).plan(strokes)))
        f, n = cfg["feeds_mm_per_min"], len(strokes)
        expected = (10.0 / f["approach"] + n * (2 * 7.5 / f["travel"] + 2 * 2.5 / f["approach"])) * 60
        self.assertAlmostEqual(t["pen_lift_s"], expected, places=1)
        slow_all = 10.0 * (2 * n + 1) / f["approach"] * 60                   # 전 구간 느린 속도였다면
        self.assertLessEqual(t["pen_lift_s"], slow_all * 0.6)
        air = de.estimate_time(strokes, cfg, air=True)
        self.assertEqual(air["command_count"], len(de.Planner(cfg, air=True).plan(strokes)))

    def test_pen_lift_grows_with_stroke_count(self):
        # 같은 길이를 1획으로 그릴 때와 10획으로 쪼갤 때: 펜 올림·내림 시간만 늘어남
        one = [[(float(x), 0.0) for x in range(-10, 11, 2)]]
        many = [[(float(x), 0.0), (float(x + 2), 0.0)] for x in range(-10, 10, 2)]
        t1, t10 = de.estimate_time(one, CFG), de.estimate_time(many, CFG)
        self.assertAlmostEqual(t1["draw_s"], t10["draw_s"], places=1)
        per_stroke = 2 * CFG["pen"]["up_clearance_mm"] / CFG["feeds_mm_per_min"]["approach"] * 60
        self.assertAlmostEqual(t10["pen_lift_s"] - t1["pen_lift_s"], 9 * per_stroke, places=0)
        self.assertGreater(t10["total_s"], t1["total_s"])


class EdgeRiseTest(unittest.TestCase):
    """비스듬히 대고 떼기: 끝점마다 아래로 끌린 세로 줄을 위쪽으로 상쇄."""
    LINE = [[(0.0, 0.0), (10.0, 0.0)]]

    @staticmethod
    def cfg(two_stage):
        cfg = copy.deepcopy(CFG)
        cfg["pen"]["edge_rise_mm"], cfg["pen"]["edge_rise_depth_mm"] = 2.0, 1.5
        if two_stage:
            cfg["pen"]["up_clearance_mm"] = 10.0
        return cfg

    @staticmethod
    def xyz(line):
        return tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())

    def test_lands_from_below_and_lifts_upward_near_the_paper(self):
        cfg = self.cfg(True)
        cfg["pen"]["edge_land_mm"] = 1.0
        planner = de.Planner(cfg)
        cmds = planner.plan(self.LINE)
        labels = [label for _, label in cmds]
        self.assertEqual(labels[1:9], ["stroke 0 travel", "stroke 0 descend", "stroke 0 land", "stroke 0 pen-down",
                                       "stroke 0 draw", "stroke 0 lift", "stroke 0 lift-out", "stroke 0 pen-up"])
        land, lift = self.xyz(cmds[3][0]), self.xyz(cmds[6][0])
        for (x, y, z), (px, py) in ((land, (0.0, -1.0)), (lift, (10.0, 2.0))):   # 내릴 때 1 mm 아래, 뗄 때 2 mm 위
            ry, rz = planner.robot_yz(px, py)
            self.assertAlmostEqual(y, ry, places=2)
            self.assertAlmostEqual(z, rz, places=2)
            off = planner.retract_sign * (x - planner.contact_x(ry, rz))
            self.assertAlmostEqual(off, 1.5, places=2)                          # 종이에서 1.5mm 떨어진 점
        self.assertEqual(self.xyz(cmds[4][0])[1:], tuple(round(v, 3) for v in planner.robot_yz(0.0, 0.0)))
        straight = de.Planner(self.cfg(True))                                  # edge_land_mm 기본 0: 바로 앞에서 곧게 댐
        land0 = self.xyz(straight.plan(self.LINE)[3][0])
        self.assertEqual(land0[1:], tuple(round(v, 3) for v in straight.robot_yz(0.0, 0.0)))
        above = self.cfg(True)
        above["pen"]["edge_land_mm"] = -2.0                                    # −: 위에서 내려오며 댐
        p = de.Planner(above)
        land_above = self.xyz(p.plan(self.LINE)[3][0])
        self.assertAlmostEqual(land_above[2], p.robot_yz(0.0, 2.0)[1], places=2)
        only = self.cfg(True)
        only["pen"]["edge_rise_mm"], only["pen"]["edge_land_mm"] = 0.0, -2.0  # 뗄 때 끄고 댈 때만 위에서
        q = de.Planner(only)
        cmds = q.plan(self.LINE)
        land_only = self.xyz(next(g for g, l in cmds if l.endswith(" land")))
        self.assertAlmostEqual(land_only[2], q.robot_yz(0.0, 2.0)[1], places=2)
        self.assertEqual(de.estimate_time(self.LINE, only)["command_count"], len(cmds))

    def test_virtual_run_completes_and_estimate_counts_every_command(self):
        from mirobot_sketch.virtual_robot import VirtualMirobotLink
        strokes = SQUARE + self.LINE
        for two_stage in (False, True):
            cfg = self.cfg(two_stage)
            cmds = de.Planner(cfg).plan(strokes)
            self.assertEqual(de.estimate_time(strokes, cfg)["command_count"], len(cmds))
            link = VirtualMirobotLink(cfg, speed=1000, homing_s=0.0)
            link.wait_for_homing(1, lambda *_: None)
            r = de.execute(link, cmds, cfg, progress=lambda *_: None)
            self.assertEqual(r["result"], "completed", (two_stage, r.get("error"), r.get("failed_command")))

    def test_resume_after_the_last_draw_skips_the_lift_steps(self):
        planner = de.Planner(self.cfg(True))
        cmds = planner.plan(self.LINE)
        last_draw = max(i for i, (_, label) in enumerate(cmds) if label.endswith(" draw"))
        self.assertEqual(de.resume_commands(planner, cmds, last_draw + 1), ([], last_draw + 4))

    def test_depth_must_clear_the_end_backoff(self):
        cfg = self.cfg(True)
        cfg["pen"]["start_backoff_mm"], cfg["pen"]["edge_rise_depth_mm"] = 0.6, 0.5
        with self.assertRaises(ValueError):
            de.Planner(cfg)

    def test_changing_the_slant_keeps_the_commands_so_it_can_be_swapped_mid_run(self):
        cfg, more = self.cfg(True), self.cfg(True)
        more["pen"]["edge_rise_mm"] = 2.5
        strokes = SQUARE + self.LINE
        a, b = de.Planner(cfg).plan(strokes), de.Planner(more).plan(strokes)
        self.assertEqual([l for _, l in a], [l for _, l in b])
        self.assertNotEqual([g for g, _ in a], [g for g, _ in b])

    def test_range_check_includes_the_shifted_points(self):
        from mirobot_sketch.limits import Region
        cfg = self.cfg(False)
        top = Region.from_cfg(de.active_limits(cfg)).top(0.0)
        near_top = [[(0.0, top - 3.0), (0.0, top - 0.5)]]               # 끝점 위 2mm는 범위 밖
        self.assertTrue(de.check_limits(near_top, cfg))
        cfg["pen"]["edge_rise_mm"] = 0.0
        self.assertFalse(de.check_limits(near_top, cfg))

    def test_lower_endpoints_get_the_extra_slant(self):
        pen = {"edge_rise_mm": 2.0, "edge_rise_low_mm": 1.0}
        self.assertEqual([de.edge_rise_at(pen, y) for y in (10.0, -20.0, -40.0, -60.0, -90.0)], [2.0, 2.0, 2.5, 3.0, 3.0])
        self.assertEqual(de.edge_rise_at({"edge_rise_mm": 0.0, "edge_rise_low_mm": 1.0}, -80.0), 0.0)   # 꺼 두면 아래쪽도 0
        cfg = self.cfg(True)
        cfg["pen"]["edge_rise_low_mm"] = 1.0
        planner = de.Planner(cfg)
        low = [[(0.0, -60.0), (10.0, -60.0)]]
        cmds = planner.plan(low)
        lift = self.xyz(next(g for g, l in cmds if l.endswith(" lift")))
        self.assertAlmostEqual(lift[2], planner.robot_yz(10.0, -57.0)[1], places=2)       # 2 + 1 mm 위로 빼고
        land = self.xyz(next(g for g, l in cmds if l.endswith(" land")))
        self.assertAlmostEqual(land[2], planner.robot_yz(0.0, -60.0)[1], places=2)        # 댈 때는 아래쪽 추가 없음
        same = self.cfg(True)
        self.assertEqual([l for _, l in cmds], [l for _, l in de.Planner(same).plan(low)])  # 명령 수는 같음


class SafetyOptionTest(unittest.TestCase):
    def test_air_mode_never_touches_paper(self):
        planner = de.Planner(CFG, air=True)
        for line, _ in planner.plan(SQUARE):
            parts = {part[0]: float(part[1:]) for part in line.split() if part[0] in "XYZ"}
            contact = planner.contact_x(parts["Y"], parts["Z"])
            self.assertGreaterEqual(abs(parts["X"] - contact),
                                    max(8.0, CFG["pen"]["up_clearance_mm"]) - 0.001)

    def test_pending_limits_only_with_flag(self):
        wide = [[(-120.0, -80.0), (120.0, -80.0), (120.0, 44.0), (0.0, 57.0), (-120.0, 44.0), (-120.0, -80.0)]]
        self.assertTrue(de.check_limits(wide, CFG))                    # 기본 ±50: 거부
        self.assertFalse(de.check_limits(wide, CFG, pending=True))     # 넓은 범위(지붕 모양): 허용
        self.assertTrue(de.check_limits([[(0.0, 0.0), (0.0, 58.0)]], CFG, pending=True))      # 지붕 위
        self.assertTrue(de.check_limits([[(0.0, 0.0), (124.0, 44.0)]], CFG, pending=True))    # 끝은 더 낮음

    def test_border_test_file_matches_pending_limits(self):
        from mirobot_sketch import limits
        path = Path(__file__).resolve().parent.parent / "trajectories" / "border-test-wide.json"
        _, strokes = de.load_strokes(path)
        self.assertTrue(de.check_limits(strokes, CFG))
        self.assertFalse(de.check_limits(strokes, CFG, pending=True))
        pts = np.array([p for s in strokes for p in s])
        r = limits.pending_region(CFG)
        self.assertAlmostEqual(pts[:, 0].max(), r.x_max)               # 테두리를 실제로 따라감
        self.assertAlmostEqual(pts[:, 1].min(), r.y_min)
        self.assertAlmostEqual(pts[:, 1].max(), r.top(0))


class GuidedCalibrationCLITest(unittest.TestCase):
    def test_physical_cli_run_calibrates_only_with_flag(self):
        from mirobot_sketch import calibration

        cfg = copy.deepcopy(CFG)
        center = dict(cfg["paper_center_tcp_mm"])
        ready = dict(center)
        ready["x"] += cfg["pen"]["retract_x_sign"] * cfg["pen"]["up_clearance_mm"]
        measured = {"result": "ready", "center_tcp_mm": center, "ready_tcp_mm": ready,
                    "selected_half_size_mm": 30.0, "max_residual_mm": 1.0,
                    "plane_fit": {"a_per_mm_y": 0.0, "b_per_mm_z": 0.0,
                                  "max_abs_residual_mm": 0.0}}
        with tempfile.TemporaryDirectory() as tmp:
            strokes_path = Path(tmp) / "line.json"
            strokes_path.write_text(json.dumps({"kind": "sketch_strokes", "units": "mm",
                                                "strokes": [{"points_xy_mm": [[-5, 0], [5, 0]]}]}),
                                    encoding="utf-8")
            link = mock.Mock()
            with mock.patch.object(sys, "argv", ["mirobot-draw", str(strokes_path), "--execute", "--calibrate"]), \
                 mock.patch.object(de, "load_config", return_value=cfg), \
                 mock.patch.object(de, "open_link", return_value=link), \
                 mock.patch.object(calibration, "run_calibration", return_value=measured) as run_cal, \
                 mock.patch.object(calibration, "save_report", return_value=Path(tmp) / "calibration.json"), \
                 mock.patch.object(calibration, "save_config") as save_cfg, \
                 mock.patch.object(de, "execute", return_value={"result": "completed"}) as execute, \
                 mock.patch.object(de, "write_run_record", return_value=Path(tmp) / "run.json"), \
                 mock.patch("builtins.input", return_value="yes"):
                self.assertEqual(de.main(), 0)
            run_cal.assert_called_once()
            save_cfg.assert_called_once()
            execute.assert_called_once()
            self.assertEqual(save_cfg.call_args.args[1]["plane_compensation"]["status"], "verified")

    def test_physical_cli_run_without_flag_skips_calibration_and_draws(self):
        from mirobot_sketch import calibration

        cfg = copy.deepcopy(CFG)
        c = cfg["paper_center_tcp_mm"]
        with tempfile.TemporaryDirectory() as tmp:
            strokes_path = Path(tmp) / "line.json"
            strokes_path.write_text(json.dumps({"kind": "sketch_strokes", "units": "mm",
                                                "strokes": [{"points_xy_mm": [[-5, 0], [5, 0]]}]}),
                                    encoding="utf-8")
            with mock.patch.object(sys, "argv", ["mirobot-draw", str(strokes_path), "--execute"]), \
                 mock.patch.object(de, "load_config", return_value=cfg), \
                 mock.patch.object(de, "open_link", return_value=mock.Mock()), \
                 mock.patch.object(de, "connect_and_home", return_value=(c["x"], c["y"], c["z"])), \
                 mock.patch.object(calibration, "run_calibration") as run_cal, \
                 mock.patch.object(calibration, "save_config") as save_cfg, \
                 mock.patch.object(de, "execute", return_value={"result": "completed"}) as execute, \
                 mock.patch.object(de, "write_run_record", return_value=Path(tmp) / "run.json"), \
                 mock.patch("builtins.input", return_value="yes"):
                self.assertEqual(de.main(), 0)
            run_cal.assert_not_called()
            save_cfg.assert_not_called()
            execute.assert_called_once()


class ExecuteTest(unittest.TestCase):
    def test_waits_for_retract_travel_and_descent_before_following_move(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['pen']['up_clearance_mm'] = 20.0
        cfg['pen']['slow_zone_mm'] = 10.0
        commands = de.Planner(cfg).plan([[(0.0, 0.0), (4.0, 3.0)]])
        serial = FakeSerial()
        result = de.execute(de.MirobotLink(serial), commands, cfg, progress=lambda *_: None)
        self.assertEqual(result['result'], 'completed')
        by_label = {label: line for line, label in commands}
        for first, second in [('center pen-up', 'stroke 0 travel'),
                              ('stroke 0 travel', 'stroke 0 descend'),
                              ('stroke 0 descend', 'stroke 0 pen-down')]:
            start = serial.written.index(by_label[first])
            end = serial.written.index(by_label[second], start + 1)
            self.assertIn('?', serial.written[start + 1:end], first)
            self.assertTrue(any(s['command'] == first for s in result['controller_trace']))

    def test_contact_and_stroke_end_are_reached_before_direction_changes(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['pen']['up_clearance_mm'] = 20.0
        cfg['pen']['slow_zone_mm'] = 10.0
        commands = de.Planner(cfg).plan([[(0.0, 0.0), (4.0, 3.0)]])
        serial = FakeSerial()
        result = de.execute(de.MirobotLink(serial), commands, cfg, progress=lambda *_: None)
        self.assertEqual(result['result'], 'completed')
        by_label = {label: line for line, label in commands}
        down = serial.written.index(by_label['stroke 0 pen-down'])
        draw = serial.written.index(by_label['stroke 0 draw'])
        lift = serial.written.index(by_label['stroke 0 lift'])
        self.assertIn('?', serial.written[down + 1:draw])
        self.assertIn('?', serial.written[draw + 1:lift])

    def setUp(self):
        self.cfg = copy.deepcopy(CFG)
        self.cfg["ack_timeout_s"] = 0.5
        self.cfg["idle_timeout_s"] = 1.0
        self.cmds = de.Planner(self.cfg).plan(SQUARE)

    def test_status_parsing(self):
        state, tcp, _ = de.MirobotLink(FakeSerial()).query_status()
        self.assertEqual(state, "Idle")
        self.assertEqual(tcp, (198.668, 0.0, 230.477))

    def test_waits_for_manual_homing(self):
        # 포트를 열면 리셋 -> Alarm -> (사용자 호밍) -> Home -> Idle
        fake = FakeSerial()
        states = iter(["Alarm", "Alarm", "Home", "Idle"])

        def write(data, _orig=fake.write):
            if data.decode().strip() == "?":
                st = next(states)
                fake.pending.append(FakeSerial.IDLE.replace("<Idle", f"<{st}"))
                return
            _orig(data)

        fake.write = write
        de.time.sleep, orig_sleep = (lambda *_: None), de.time.sleep
        try:
            state, tcp = de.MirobotLink(fake).wait_for_homing(5, progress=lambda *_: None)
        finally:
            de.time.sleep = orig_sleep
        self.assertEqual(state, "Idle")
        self.assertEqual(tcp, (198.668, 0.0, 230.477))

    def test_completes_with_acks(self):
        fake = FakeSerial()
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        self.assertEqual(result["result"], "completed")
        self.assertEqual(result["commands_sent"], len(self.cmds))

    def test_pen_up_waits_until_idle_before_the_next_move(self):
        fake = FakeSerial()
        de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        pen_up = next(line for line, label in self.cmds if label == "stroke 0 pen-up")
        i = fake.written.index(pen_up)
        self.assertEqual(fake.written[i + 1], "?")          # 펜업 ok 다음, 옆 이동 전에 Idle 확인

    def test_execution_records_reported_retract_coordinates(self):
        fake = FakeSerial()
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        samples = result['controller_trace']
        self.assertTrue(any(s['command'].endswith('pen-up') and s['tcp_mm'] is not None for s in samples))
        self.assertTrue(all('state' in s and 'time_s' in s for s in samples))

    def test_stops_immediately_on_soft_limit(self):
        fake = FakeSerial(fail_at=4)
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        self.assertEqual(result["result"], "stopped_on_error")
        self.assertIn("Soft limit", result["error"])
        sent = [w for w in fake.written if w != "?"]
        self.assertEqual(len(sent), 4)  # 오류 이후 명령을 더 보내지 않음

    def test_timeout_without_ack(self):
        fake = FakeSerial(fail_at=2, fail_msg="")
        fake.readline = lambda: b""
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        self.assertEqual(result["result"], "stopped_on_error")

    def test_usb_read_failure_stops_with_a_result_instead_of_raising(self):
        fake = FakeSerial()
        real_readline, reads = fake.readline, []

        def readline():
            reads.append(1)
            if len(reads) > 6:
                raise OSError(22, "ReadFile failed")      # serial.SerialException도 OSError
            return real_readline()

        fake.readline = readline
        result = de.execute(de.MirobotLink(fake), self.cmds, self.cfg, progress=lambda *_: None)
        self.assertEqual(result["result"], "stopped_on_error")
        self.assertIn("USB", result["error"])
        self.assertLess(result["commands_sent"], len(self.cmds))


def _status(state, x=None, y=0.0, z=230.477):
    if x is None:
        return f"<{state},Angle(ABCDXYZ):0,0,0,0,0,0,0>"
    return (f"<{state},Angle(ABCDXYZ):0,0,0,0,0,0,0,"
            f"Cartesian coordinate(XYZ RxRyRz):{x:.3f},{y:.3f},{z:.3f},0.000,0.000,0.000>")


class ScriptedStatus:
    """? 마다 정해 둔 상태 줄을 돌려준다."""

    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.last = self.statuses[-1] if self.statuses else "<Idle>"
        self.pending = []
        self.written = []

    def write(self, data):
        text = data.decode().strip()
        self.written.append(text)
        if text == "?":
            self.pending.append(self.statuses.pop(0) if self.statuses else self.last)
            if self.statuses:
                self.last = self.statuses[0]
        else:
            self.pending.append("ok")

    def readline(self):
        return (self.pending.pop(0) + "\r\n").encode() if self.pending else b""

    def reset_input_buffer(self):
        self.pending.clear()


class RetractConfirmTest(unittest.TestCase):
    def test_wait_for_xyz_rejects_moving_and_wrong_yz(self):
        fake = ScriptedStatus([_status('Idle', 230, y=5), _status('Run', 230, y=0), _status('Idle', 230, y=0)])
        result = de.MirobotLink(fake).wait_until_pose((230, 0, 230.477), 2.0)
        self.assertEqual(result, (230, 0, 230.477))
        self.assertEqual(fake.written.count('?'), 3)

    def test_wait_for_xyz_honors_stop_without_querying(self):
        fake = ScriptedStatus([_status('Run', 230.0)])
        with self.assertRaises(de.StopRequested):
            de.MirobotLink(fake).wait_until_pose((230, 0, 230.477), 2.0, should_stop=lambda: True)
        self.assertEqual(fake.written, [])

    def test_observer_receives_each_actual_status_sample(self):
        fake = ScriptedStatus([_status("Idle", 240.0), _status("Run", 230.0), _status("Idle", 230.0)])
        link = de.MirobotLink(fake)
        seen = []
        link.status_observer = lambda state, tcp, raw: seen.append((state, tcp, raw))
        link.wait_until_left_contact(240.0, 2.0, -1.0, 10.0)
        self.assertEqual([sample[0] for sample in seen], ["Idle", "Run", "Idle"])
        self.assertEqual(seen[-1][1], (230.0, 0.0, 230.477))

    def test_retract_does_not_complete_while_controller_is_still_moving(self):
        fake = ScriptedStatus([_status("Idle", 240.0), _status("Run", 230.0), _status("Idle", 230.0)])
        link = de.MirobotLink(fake)
        tcp = link.wait_until_left_contact(240.0, 2.0, -1.0, 10.0)
        self.assertEqual(tcp[0], 230.0)
        self.assertEqual(fake.written.count("?"), 3)

    def _run(self, statuses, timeout=1.0):
        clock = {"t": 0.0}

        def monotonic():
            return clock["t"]

        def sleep(dt):
            clock["t"] += dt

        link = de.MirobotLink(ScriptedStatus(statuses))
        with mock.patch.object(de.time, "monotonic", monotonic), mock.patch.object(de.time, "sleep", sleep):
            return link.wait_until_x(190.0, timeout)

    def test_first_sample_already_at_target_passes_without_run(self):
        tcp = self._run([_status("Idle", 190.0)])
        self.assertEqual(tcp[0], 190.0)

    def test_idle_without_coordinates_does_not_pass(self):
        with self.assertRaises(de.ControllerError):
            self._run([_status("Idle")] * 30)

    def test_x_that_left_the_target_then_returns_passes_without_run(self):
        tcp = self._run([_status("Idle", 218.0), _status("Idle", 190.0)])
        self.assertEqual(tcp[0], 190.0)

    def test_non_idle_state_counts_as_motion(self):
        tcp = self._run([_status("Idle"), _status("Jog", 218.0), _status("Idle", 190.0)])
        self.assertEqual(tcp[0], 190.0)




    def test_target_x_alone_does_not_mean_this_retract_started(self):
        with self.assertRaises(de.ControllerError):
            self._left([_status("Idle", 212.0)] * 30, 240.0)

    def test_leaves_after_this_contact_is_seen(self):
        tcp = self._left([_status("Idle", 240.0), _status("Idle", 220.0)], 240.0, min_mm=5.0)
        self.assertEqual(tcp[0], 220.0)

    def _left(self, statuses, contact_x, timeout=1.0, min_mm=5.0):
        clock = {"t": 0.0}

        def monotonic():
            return clock["t"]

        def sleep(dt):
            clock["t"] += dt

        link = de.MirobotLink(ScriptedStatus(statuses))
        with mock.patch.object(de.time, "monotonic", monotonic), mock.patch.object(de.time, "sleep", sleep):
            return link.wait_until_left_contact(contact_x, timeout, -1.0, min_mm)



class OrientationTestFileTest(unittest.TestCase):
    def test_orientation_file_is_valid_and_in_limits(self):
        path = Path(__file__).resolve().parent.parent / "trajectories" / "orientation-test-F.json"
        _, strokes = de.load_strokes(path)
        self.assertEqual(len(strokes), 2)
        self.assertFalse(de.check_limits(strokes, CFG))


class StepFunctionsTest(unittest.TestCase):
    def test_draw_steps_order(self):
        self.assertEqual([s[0] for s in de.DRAW_STEPS], ["preflight", "connect", "start", "confirm", "drawing", "done"])

    def test_preflight_rejects_out_of_range_with_hint(self):
        far = [[(0.0, 0.0), (58.0, 0.0)]]
        with self.assertRaises(de.DrawError) as cm:
            de.preflight(far, CFG)
        self.assertEqual(cm.exception.step, "preflight")
        self.assertIn("넓은 범위", cm.exception.hint)
        self.assertTrue(de.preflight(far, CFG, pending=True)["cmds"])

    def test_connect_and_check_start_with_virtual_link(self):
        from mirobot_sketch.virtual_robot import VirtualMirobotLink
        link = VirtualMirobotLink(CFG, homing_s=0.01)
        tcp = de.connect_and_home(link, CFG, progress=lambda *_: None)
        de.check_start(tcp, CFG)
        bad = VirtualMirobotLink(CFG, homing_s=0.01, start_offset_mm=9.0)
        with self.assertRaises(de.DrawError) as cm:
            de.check_start(de.connect_and_home(bad, CFG, progress=lambda *_: None), CFG)
        self.assertEqual(cm.exception.step, "start")
        with self.assertRaises(de.DrawError) as cm:
            de.connect_and_home(VirtualMirobotLink(CFG, homing_s=5), CFG, progress=lambda *_: None,
                                should_cancel=lambda: True)
        self.assertEqual(cm.exception.step, "connect")

    def test_check_start_after_calibration_accepts_pen_up_retract_only_away_from_wall(self):
        c = CFG["paper_center_tcp_mm"]
        sign = CFG["pen"]["retract_x_sign"]
        at = lambda back, dy=0.0, dz=0.0: (c["x"] + sign * back, c["y"] + dy, c["z"] + dz)
        self.assertGreater(8.0, CFG["max_start_offset_mm"])            # 보정 후 후퇴 자세는 일반 5 mm 기준을 넘는다
        with self.assertRaises(de.DrawError):
            de.check_start(at(8.0), CFG)                                # 일반 검사는 막지만
        self.assertAlmostEqual(de.check_start(at(8.0), CFG, retracted=True), 8.0)   # 보정 직후에는 통과
        de.check_start(at(0.0), CFG, retracted=True)
        for bad in (at(25.0), at(-5.0), at(5.0, dy=7.0), at(5.0, dz=-6.0)):        # 너무 멀리 물러남 / 벽 쪽 / 옆으로 어긋남
            with self.assertRaises(de.DrawError) as cm:
                de.check_start(bad, CFG, retracted=True)
            self.assertEqual(cm.exception.step, "start")

    def test_cli_virtual_run_writes_record(self):
        import tempfile
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            doc = Path(d) / "s.json"
            doc.write_text(json.dumps({"kind": "sketch_strokes", "units": "mm", "strokes": [
                {"points_xy_mm": [[-5, -5], [5, -5], [5, 5]]}]}), encoding="utf-8")
            with mock.patch.object(de.paths, "runs_dir", lambda: Path(d) / "runs"),                     mock.patch("builtins.input", return_value="yes"),                     mock.patch.object(sys, "argv", ["mirobot-draw", str(doc), "--execute", "--virtual",
                                                    "--virtual-speed", "200"]):
                self.assertEqual(de.main(), 0)
            rec = json.loads(next((Path(d) / "runs").glob("run-*.json")).read_text(encoding="utf-8"))
            self.assertTrue(rec["virtual"])
            self.assertEqual(rec["result"], "completed")


if __name__ == "__main__":
    unittest.main()
