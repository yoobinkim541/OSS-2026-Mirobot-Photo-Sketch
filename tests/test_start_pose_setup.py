"""시작 자세 T5: 연결 뒤 저장된 시작 자세로 자동 이동 + 앱에서 X·Y·Z 조그 + 기억하고 계속.

    python -m unittest tests.test_start_pose_setup -v
"""

import json
import re
import sys
import tempfile
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import start_pose as sp  # noqa: E402

import golden  # noqa: E402

CFG = golden.default_cfg()
HOME = (198.67, 0.0, 230.5)
MOVE = re.compile(r"^M20 G90 G01 X([-\d.]+) Y([-\d.]+) Z([-\d.]+) A([-\d.]+) B([-\d.]+) C([-\d.]+) F(\d+)$")


class FakeRobot:
    """직각 홀더 로봇: 관절 0 = 호밍 자세. M20 직교 이동을 받은 대로 TCP를 바꾸고 상태로 보고."""

    def __init__(self, tcp=HOME, homed=True):
        self.tcp = tuple(tcp)
        self.joints = [0.0] * 6 if homed else [5.0, 0, 0, 0, 0, 0]
        self.sent = []
        self.lock = threading.Lock()

    def query_status(self, timeout=2.0):
        with self.lock:
            (x, y, z), j = self.tcp, self.joints
        raw = (f"<Idle,Angle(ABCDXYZ):{j[3]},{j[4]},{j[5]},0,{j[0]},{j[1]},{j[2]},"
               f"Cartesian coordinate(XYZ RxRyRz):{x},{y},{z},0.0,0.0,0.0,Pump PWM:0>")
        return "Idle", (x, y, z), raw

    def send_and_ack(self, line, timeout, should_stop=None):
        m = MOVE.match(line)
        assert m, line
        with self.lock:
            self.sent.append(line)
            self.tcp = tuple(float(v) for v in m.groups()[:3])
            if self.tcp != HOME:
                self.joints = [1.0] * 6      # 호밍 자세를 벗어나면 관절 값도 바뀜

    def wait_idle(self, timeout):
        return self.tcp


class Events:
    def __init__(self):
        self.items, self.cond = [], threading.Condition()

    def __call__(self, kind, **data):
        with self.cond:
            self.items.append((kind, data))
            self.cond.notify_all()

    def wait_for(self, pred, timeout=10.0):
        with self.cond:
            ok = self.cond.wait_for(lambda: any(pred(k, d) for k, d in self.items), timeout)
        if not ok:
            raise AssertionError(f"event not seen: {self.items[-5:]}")


class SetupTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "drawing_config.json"
        self.path.write_text((ROOT / "mirobot_sketch" / "data" / "drawing_config.json").read_text(encoding="utf-8"),
                             encoding="utf-8")
        self.events = Events()

    def tearDown(self):
        self.tmp.cleanup()

    def start(self, robot, cfg=CFG):
        self.setup = sp.StartPoseSetup(robot, cfg, self.path, self.events)
        self.result, self.error = None, None

        def work():
            try:
                self.result = self.setup.run(should_stop=lambda: False)
            except Exception as e:      # 테스트에서 확인
                self.error = e

        self.thread = threading.Thread(target=work, daemon=True)
        self.thread.start()
        self.events.wait_for(lambda k, d: k == "start_pose" and d["state"] == "jog")

    def finish(self):
        self.thread.join(10)
        self.assertFalse(self.thread.is_alive())

    def saved(self):
        return sp.settings(de.load_config(self.path))["ready_pose"]

    def test_without_saved_pose_jog_xyz_then_remember(self):
        robot = FakeRobot()
        self.start(robot)
        self.setup.jog(2.0, 0.0, 0.0)
        self.events.wait_for(lambda k, d: k == "start_pose" and abs(d.get("tcp", (0,))[0] - (HOME[0] + 2.0)) < 1e-6)
        self.setup.jog(0.0, -1.0, 0.5)          # 오른쪽 + 위
        self.events.wait_for(lambda k, d: k == "start_pose" and abs(d.get("tcp", (0, 0, 0))[2] - (HOME[2] + 0.5)) < 1e-6)
        self.setup.accept()
        self.finish()
        self.assertIsNone(self.error)
        self.assertEqual(self.result, (HOME[0] + 2.0, -1.0, HOME[2] + 0.5))
        self.assertEqual(self.saved()["tcp_mm"], [HOME[0] + 2.0, -1.0, HOME[2] + 0.5])
        for line in robot.sent:                  # 자세는 호밍 값(A·B·C = 0) 고정
            self.assertTrue(line.split(" A")[1].startswith("0.000 B0.000 C0.000"), line)

    def test_saved_pose_is_reached_automatically_from_the_homing_pose(self):
        sp.save_ready_pose(self.path, {"tcp": (205.0, 3.0, 225.0), "abc_deg": (0.0, 0.0, 0.0), "joints_deg": None})
        robot = FakeRobot()
        self.start(robot, de.load_config(self.path))
        targets = [tuple(float(v) for v in MOVE.match(line).groups()[:3]) for line in robot.sent]
        self.assertEqual(targets[-1], (205.0, 3.0, 225.0))
        self.assertTrue(all(x <= 205.0 for x, _, _ in targets))           # 종이 쪽으로 넘어가지 않음
        self.assertLess(targets[-2][0], 205.0)                            # 마지막은 뒤에서 앞으로 다가감
        self.setup.accept()
        self.finish()
        self.assertEqual(self.result, (205.0, 3.0, 225.0))

    def test_already_at_the_saved_pose_does_not_move(self):
        sp.save_ready_pose(self.path, {"tcp": (205.0, 3.0, 225.0), "abc_deg": (0.0, 0.0, 0.0), "joints_deg": None})
        robot = FakeRobot(tcp=(205.0, 3.0, 225.0), homed=False)
        self.start(robot, de.load_config(self.path))
        self.assertEqual(robot.sent, [])
        self.setup.accept()
        self.finish()
        self.assertEqual(self.result, (205.0, 3.0, 225.0))

    def test_unknown_pose_is_refused_without_moving(self):
        robot = FakeRobot(tcp=(150.0, 40.0, 200.0), homed=False)
        setup = sp.StartPoseSetup(robot, CFG, self.path, self.events)
        with self.assertRaises(sp.StartPoseError):
            setup.run(should_stop=lambda: False)
        self.assertEqual(robot.sent, [])

    def test_jog_outside_the_box_is_refused(self):
        robot = FakeRobot()
        self.start(robot)
        limit = sp.settings(CFG)["jog_max_z_mm"]
        self.setup.jog(0.0, 0.0, limit + 1.0)
        self.events.wait_for(lambda k, d: k == "start_pose" and "범위" in d.get("message", ""))
        self.assertEqual(robot.sent, [])
        self.setup.cancel()
        self.finish()
        self.assertIsInstance(self.error, sp.StartPoseError)

    def test_a_pose_that_fails_the_check_is_not_saved(self):
        robot = FakeRobot()
        self.start(robot)
        self.setup.jog(-30.0, 0.0, 0.0)          # 플랜지 X 168.7: 그리기 범위 끝 여유는 있지만 확인용으로 더 뒤로
        self.events.wait_for(lambda k, d: k == "start_pose" and abs(d.get("tcp", (0,))[0] - (HOME[0] - 30.0)) < 1e-6)
        cfg_far = json.loads(self.path.read_text(encoding="utf-8"))
        cfg_far.setdefault("start_pose", {})["min_joint_margin_deg"] = 60.0     # 일부러 못 맞추는 기준
        self.path.write_text(json.dumps(cfg_far, ensure_ascii=False), encoding="utf-8")
        self.setup.cfg = de.load_config(self.path)
        self.setup.accept()
        self.events.wait_for(lambda k, d: k == "start_pose" and d.get("problems"), timeout=60.0)
        self.assertIsNone(self.saved())
        self.setup.cancel()
        self.finish()


class CheckReadyZTest(unittest.TestCase):
    def test_moving_up_or_down_is_allowed_now(self):
        home = {"tcp": HOME, "abc_deg": (0.0, 0.0, 0.0)}
        r = sp.check_ready(CFG, {"tcp": (205.0, 0.0, 220.0), "abc_deg": (0.0, 0.0, 0.0)}, home)
        self.assertTrue(r["ok"], r["problems"])


if __name__ == "__main__":
    unittest.main()
