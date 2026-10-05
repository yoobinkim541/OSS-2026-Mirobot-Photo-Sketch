"""시작 자세 T4: 펜 끝을 종이 가운데에 맞춘 자세(ready_pose)의 검사·저장·불러오기 테스트.

    python -m unittest tests.test_ready_pose -v
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import start_pose as sp  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()


def status(x, y=0.0, z=130.0, abc=(0.0, -90.0, 0.0), joints=(0.0, -1.4, 46.4, 0.0, -135.0, 0.0)):
    return {"state": "Idle", "tcp": (x, y, z), "abc_deg": abc, "joints_deg": list(joints)}


REFERENCE = status(185.0)        # 펜을 수평으로 올린 직후(대기 자세)에 컨트롤러가 보고한 값이라고 가정


class CheckReadyTest(unittest.TestCase):
    def test_jog_along_x_and_y_only_is_accepted(self):
        r = sp.check_ready(CFG, status(200.0, y=3.0), REFERENCE)
        self.assertTrue(r["ok"], r["problems"])
        self.assertGreater(r["min_margin_deg"], 5.0)

    def test_height_change_is_rejected(self):
        r = sp.check_ready(CFG, status(200.0, z=133.0), REFERENCE)
        self.assertFalse(r["ok"])
        self.assertTrue(any("Z" in p for p in r["problems"]))

    def test_orientation_change_is_rejected(self):
        r = sp.check_ready(CFG, status(200.0, abc=(0.0, -85.0, 0.0)), REFERENCE)
        self.assertFalse(r["ok"])
        self.assertTrue(any("자세" in p for p in r["problems"]))

    def test_paper_too_close_to_the_base_asks_to_move_the_robot_back(self):
        # 플랜지 X 160 mm: 펜이 수평인 자세에서 그리기 범위 끝이 관절 한계에 걸림
        r = sp.check_ready(CFG, status(160.0), REFERENCE)
        self.assertFalse(r["ok"])
        self.assertGreater(r["move_back_mm"], 0.0)
        self.assertTrue(any("뒤로" in p for p in r["problems"]))


class SaveLoadTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "drawing_config.json"
        raw = json.loads((ROOT / "mirobot_sketch" / "data" / "drawing_config.json").read_text(encoding="utf-8"))
        raw["_user_note"] = "사용자가 손으로 적은 값"
        self.path.write_text(json.dumps(raw, ensure_ascii=False, indent=1), encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def test_saved_pose_comes_back_after_reloading_the_config(self):
        saved = sp.save_ready_pose(self.path, status(182.0, y=3.0))
        loaded = sp.settings(de.load_config(self.path))["ready_pose"]
        self.assertEqual(loaded, saved)
        self.assertEqual(loaded["tcp_mm"], [182.0, 3.0, 130.0])
        self.assertEqual(loaded["abc_deg"], [0.0, -90.0, 0.0])

    def test_saving_keeps_the_other_settings(self):
        sp.save_ready_pose(self.path, status(182.0))
        raw = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertEqual(raw["_user_note"], "사용자가 손으로 적은 값")
        self.assertIn("limits", raw)

    def test_no_saved_pose_by_default(self):
        self.assertIsNone(sp.settings(de.load_config(self.path))["ready_pose"])


class ScriptedLink:
    """query_status마다 정해 둔 자세를 차례로 보고 (사람이 화살표로 움직이는 것을 흉내)."""

    def __init__(self, poses):
        self.poses = list(poses)
        self.sent = []

    def send_and_ack(self, line, timeout, should_stop=None):
        self.sent.append(line)

    def wait_idle(self, timeout):
        return self.poses[0]

    def query_status(self, timeout=2.0):
        x, y, z = self.poses.pop(0) if len(self.poses) > 1 else self.poses[0]
        raw = ("<Idle,Angle(ABCDXYZ):0.0,-144.4,0.0,0.0,0.0,-27.3,46.7,"
               f"Cartesian coordinate(XYZ RxRyRz):{x},{y},{z},0.0,-90.0,0.0,Pump PWM:0>")
        return "Idle", (x, y, z), raw


class JogSessionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "drawing_config.json"
        self.path.write_text((ROOT / "mirobot_sketch" / "data" / "drawing_config.json").read_text(encoding="utf-8"),
                             encoding="utf-8")
        self.out = []

    def tearDown(self):
        self.tmp.cleanup()

    def run_session(self, poses, keys):
        keys = iter(keys)
        self.link = ScriptedLink(poses)
        return sp.jog_session(self.link, CFG, REFERENCE, self.path,
                              input_fn=lambda _: next(keys), output_fn=self.out.append)

    def test_enter_shows_the_current_position_and_s_saves(self):
        result = self.run_session([(190.0, 0.0, 130.0), (200.0, 2.0, 130.0)], ["", "s"])
        self.assertEqual(result["result"], "saved")
        self.assertTrue(any("190.0" in line for line in self.out))
        saved = sp.settings(de.load_config(self.path))["ready_pose"]
        self.assertEqual(saved["tcp_mm"], [200.0, 2.0, 130.0])

    def test_a_bad_pose_is_not_saved(self):
        result = self.run_session([(200.0, 0.0, 135.0)], ["s", "q"])
        self.assertEqual(result["result"], "quit")
        self.assertIsNone(sp.settings(de.load_config(self.path))["ready_pose"])
        self.assertTrue(any("Z" in line for line in self.out))


    def test_w_moves_toward_the_wall_keeping_height_and_orientation(self):
        self.run_session([(185.0, 0.0, 130.0)], ["w", "q"])
        step = sp.settings(CFG)["jog_steps_mm"][sp.settings(CFG)["jog_default_step"]]
        x, y, z = REFERENCE["tcp"]
        a, b, c = REFERENCE["abc_deg"]
        feed = CFG["feeds_mm_per_min"]["approach"]
        self.assertEqual(self.link.sent, [f"M20 G90 G01 X{x + step:.3f} Y{y:.3f} Z{z:.3f} "
                                          f"A{a:.3f} B{b:.3f} C{c:.3f} F{feed:.0f}"])

    def test_step_keys_change_the_step_and_moves_add_up(self):
        self.run_session([(185.0, 0.0, 130.0)], ["3", "aa", "1", "d", "q"])
        steps = sp.settings(CFG)["jog_steps_mm"]
        ys = [float(line.split(" Y")[1].split(" ")[0]) for line in self.link.sent]
        self.assertEqual(ys, [steps[2], 2 * steps[2], 2 * steps[2] - steps[0]])

    def test_moves_outside_the_jog_box_are_refused(self):
        limit = sp.settings(CFG)["jog_max_y_mm"]
        keys = ["3"] + ["a"] * int(limit / sp.settings(CFG)["jog_steps_mm"][2] + 2) + ["q"]
        self.run_session([(185.0, 0.0, 130.0)], keys)
        ys = [float(line.split(" Y")[1].split(" ")[0]) for line in self.link.sent]
        self.assertLessEqual(max(ys), limit + 1e-9)
        self.assertTrue(any("범위" in line for line in self.out))


if __name__ == "__main__":
    unittest.main()
