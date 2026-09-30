# GUI 로봇 드로잉 + RViz 실시간 따라가기 — 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** GUI에서 안전 절차(①~⑥)를 거쳐 로봇(또는 가상 시뮬레이션)으로 그리게 하고, 그리는 동안 RViz의 로봇팔·펜 자국이 명령 응답 기준으로 같은 위치를 따라가게 한다.

**Architecture:**
- **실행기:** `draw_executor.py`를 단계 함수(`preflight`, `connect_and_home`, `check_start`, `execute`, `write_run_record`)로 나눈다. 명령줄과 새 `DrawJob`(화면과 무관한 실행 작업)이 같은 함수를 쓴다.
- **진행 전달:** `ok` 응답마다 진행 파일(`live_progress.json`)을 원자적으로 쓴다. WSL의 `rviz_playback.py --follow`가 그 파일을 0.05초마다 읽고, `live_progress.follow_index()`로 궤적 위치를 정한다.
- **가상 시뮬레이션:** `VirtualMirobotLink`가 로봇 없이 같은 흐름을 제공한다.

**Tech Stack:** Python 3.11+, pyserial, NumPy, CustomTkinter, ROS 2 Humble(rclpy, WSL), unittest

**Spec:** `docs/superpowers/specs/2026-09-26-live-draw-rviz-follow-design.md`

## Global Constraints

- **테스트:** `python -m unittest discover -s tests`가 전부 통과하고, `python -m pyflakes mirobot_sketch tests`가 깨끗해야 한다.
- **코드 스타일:** 주석과 docstring은 한국어, 한 줄 120자 이하.
- **명령줄 호환:** 기존 `tests/test_draw_executor.py`의 테스트를 고치지 않고 통과해야 한다. 예외는 새 인자를 추가하는 경우뿐이다. `execute(link, cmds, cfg, progress=...)`의 반환 형식(`result`, `commands_sent`, `elapsed_s`, …)을 유지한다.
- **자동 복구 금지:** 멈춤이나 오류 뒤에 로봇을 움직이는 명령을 보내지 않는다.
- **에이전트:** 에이전트 도구에 로봇 실행을 넣지 않는다(`TOOLS`에 변화 없음).
- **진행 파일:** 같은 폴더의 임시 파일에 쓴 뒤 `os.replace`로 바꾼다. 읽는 쪽은 JSON 오류가 나면 직전 값을 쓴다.
- **궤적 명령 번호:** 궤적 점의 `cmd`는 G-code 명령 번호(0부터)이고, 시작 자세 점은 −1이다. `acked` = `ok`를 받은 명령 수다.
- **가상 시뮬레이션 기록:** 실행 기록에 `"virtual": true`와 `"virtual_speed"`를 남긴다.
- **화면 캡처:** 사용자가 PC를 쓰는 중이면 전체 화면을 캡처하지 않는다.
- **커밋:** 메시지 끝에 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`을 넣는다. 브랜치는 `feature/live-draw`다.

## Review Focus

1. **[멈춤]을 명령 응답 대기 중에 누름:** 가상 시뮬레이션 1배속의 긴 이동 중이라도, 다음 명령은 보내지 않고 0.3초 안에 ⑥(사용자 멈춤)으로 가야 한다. → Task 3·5 테스트
2. **RViz가 명령 응답보다 앞서 가는 보간:** 다음 명령의 끝 점을 절대 넘지 않아야 한다. 응답이 오래 안 오면 그 점에서 멈춰 기다린다. → Task 2 테스트
3. **실행 중 앱 종료:** 창을 닫으면 멈추고 포트를 닫은 뒤 종료해야 한다. 스레드가 남아 포트를 붙잡으면 안 된다. → Task 7 테스트
4. **WSL/ROS가 없는 PC:** 드로잉은 정상으로 되고, RViz 버튼만 흐리게 이유와 함께 보여야 한다. → Task 5·7 테스트
5. **진행 파일을 읽는 도중 교체:** Windows에서 `os.replace`가 잠깐 실패하면 다시 시도하고, 읽는 쪽은 절대 반쪽 JSON을 쓰지 않아야 한다. → Task 2 테스트

---

### Task 1: 궤적 점에 명령 번호 붙이기

**Files:**
- Modify: `mirobot_sketch/mirobot_sim.py` (`simulate`, `trajectory_doc`)
- Test: `tests/test_sim.py`

**Interfaces:**
- Produces:
  - `simulate()`의 `samples` 원소가 `(q, tcp, label, pen_down, cmd)`로 5개가 된다. `cmd`는 int이고 시작 자세는 −1이다.
  - `trajectory_doc(...)["points"][i]["cmd"]`를 추가한다.
  - `trajectory_doc(...)["cmd_feed_mm_min"]`: 명령별 F 값 목록(길이 = 명령 수)
  - `trajectory_doc(...)["command_count"]`

- [ ] **Step 1: 실패하는 테스트**

```python
class TrajectoryCmdTest(unittest.TestCase):
    def test_points_carry_command_index(self):
        cfg = de.load_config()
        strokes = [[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0)]]
        res = ms.simulate(ms.plan_targets(strokes, cfg))
        doc = ms.trajectory_doc(res, cfg, "t")
        cmds = [p["cmd"] for p in doc["points"]]
        n = len(de.Planner(cfg).plan(strokes))
        self.assertEqual(cmds[0], -1)
        self.assertEqual(sorted(set(cmds)), list(range(-1, n)))        # 명령마다 점이 1개 이상
        self.assertEqual(cmds, sorted(cmds))                           # 순서대로 늘어남
        self.assertEqual(doc["command_count"], n)
        self.assertEqual(len(doc["cmd_feed_mm_min"]), n)
        self.assertEqual(doc["cmd_feed_mm_min"][0], cfg["feeds_mm_per_min"]["approach"])
```

(`tests/test_sim.py`에 `from mirobot_sketch import draw_executor as de`와 `mirobot_sim as ms` import가 없으면 추가한다.)

- [ ] **Step 2: 실패 확인** — `cd tests && python -m unittest test_sim.TrajectoryCmdTest -v` → `KeyError: 'cmd'`

- [ ] **Step 3: 구현**

명령별 속도(F)는 `simulate` 결과만으로는 알 수 없으므로, `simulate`의 입력인 `plan_targets`가 명령별 F를 함께 돌려주게 바꾼다.
- `plan_targets`는 `(xyz, label, pen_down)` 대신 `(xyz, label, pen_down, feed)`를 돌려준다. 시작 자세의 feed는 0이다.
- `simulate`는 feed를 `result["cmd_feed"]` 목록에 모은다(시작 자세 제외).
- `trajectory_doc`은 다음처럼 만든다.

```python
def trajectory_doc(result, cfg, source):
    """... (위 docstring)"""
    tip = pen_tip_offset(cfg)
    return {
        "joint_names": JOINT_NAMES, "units": "rad", "step_mm": 1.0, "source": str(source),
        "pen_tip_offset_mm": tip.tolist(),
        "command_count": len(result["cmd_feed"]),
        "cmd_feed_mm_min": [float(f) for f in result["cmd_feed"]],
        "points": [{"q": [round(float(v), 5) for v in s[0]], "tcp_mm": [round(float(v), 3) for v in s[1]],
                    "pen_tip_mm": [round(float(v), 3) for v in s[1] + tip],
                    "pen_down": bool(s[3]), "command": s[2], "cmd": int(s[4])} for s in result["samples"]],
    }
```

`plan_targets`:

```python
_GCODE_F = __import__("re").compile(r"F([\d.]+)")


def plan_targets(strokes, cfg):
    """실행기와 같은 계획 -> [(xyz_mm, 설명, pen_down, feed)]. 시작점은 종이 중심 펜다운(=사용자가 둔 위치)."""
    planner = de.Planner(cfg)
    out = [(np.array(planner.pose(0, 0, True)), "start (pen at paper center)", False, 0.0)]
    for line, label in planner.plan(strokes):
        xyz = np.array([float(v) for v in _GCODE_XYZ.search(line).groups()])
        out.append((xyz, label, label.endswith(" draw"), float(_GCODE_F.search(line).group(1))))
    return out
```

`simulate`의 반복문:

```python
    cmd_feed = []
    for t, (xyz, label, pen_down, feed) in enumerate(targets):
        if t > 0:
            cmd_feed.append(feed)
        ...
            samples.append((q.copy(), p.copy(), label, pen_down, t - 1))
```

`return` 사전에 `"cmd_feed": cmd_feed`를 추가한다. `plan_targets`를 부르는 다른 곳(`strokes_from_shape`, `session.simulate`)은 결과를 그대로 `simulate`에 넘기므로 영향이 없다. `mirobot_sim`에서 `targets`를 푸는 다른 곳이 있는지 `grep -n "for .* in targets\|targets\[" mirobot_sketch/*.py`로 확인하고, 있으면 4개로 풀게 고친다.

- [ ] **Step 4: 통과 확인** — `python -m unittest discover -s tests` → OK (`test_rviz_launch`의 궤적 형식 테스트는 `"cmd"`를 쓰지 않으므로 그대로 통과. 만약 `samples` 4튜플을 직접 만드는 테스트가 있으면 5번째 값 0을 더한다)

- [ ] **Step 5: 커밋**

```bash
git add mirobot_sketch/mirobot_sim.py tests/test_sim.py tests/test_rviz_launch.py
git commit -m "Trajectory points carry G-code command index and per-command feed

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: 진행 파일과 따라가기 위치 계산 (`live_progress.py`)

**Files:**
- Create: `mirobot_sketch/live_progress.py`
- Test: `tests/test_live_progress.py`

**Interfaces:**
- Produces:
  - `ProgressWriter(path)`: `.write(**fields) -> dict`. `run_id`, `total` 등은 이전 값을 이어받고, 매번 `t=time.time()`을 넣는다.
  - `read_progress(path) -> dict | None`: JSON 오류이거나 파일이 없으면 None
  - `class FollowTrack(points, cmd_feed_mm_min, step_mm=1.0)`: `.index(progress, now) -> int`, `.cmd_end: list[int]`
  - `to_local_path(p) -> str`: Windows 경로를 Linux(WSL)에서 `/mnt/c/...`로 바꾼다. Windows에서는 그대로 둔다.
  - **의존성:** 표준 라이브러리만 쓴다. WSL의 ROS 파이썬에서도 불러야 하기 때문이다.

- [ ] **Step 1: 실패하는 테스트**

```python
# tests/test_live_progress.py
"""진행 파일(원자적 쓰기)과 RViz 따라가기 위치 계산."""

import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import live_progress as lp  # noqa: E402


def points(cmds):
    """cmd 번호 목록 -> 궤적 점 (1mm 간격 가정)."""
    return [{"cmd": c} for c in cmds]


# 시작 자세 1점, 명령0: 3점, 명령1: 10점, 명령2: 2점
PTS = points([-1] + [0] * 3 + [1] * 10 + [2] * 2)
FEED = [600.0, 60.0, 600.0]          # mm/min -> 명령1은 1 mm/s


class FollowTrackTest(unittest.TestCase):
    def setUp(self):
        self.tr = lp.FollowTrack(PTS, FEED)

    def test_command_end_indices(self):
        self.assertEqual(self.tr.cmd_end, [3, 13, 15])

    def test_acked_commands_are_final(self):
        self.assertEqual(self.tr.index({"state": "running", "acked": 0, "t": 100.0}, 100.0), 0)
        self.assertEqual(self.tr.index({"state": "running", "acked": 1, "t": 100.0}, 100.0), 3)

    def test_interpolates_but_never_passes_next_command_end(self):
        p = {"state": "running", "acked": 1, "t": 100.0, "speed": 1.0}
        self.assertEqual(self.tr.index(p, 103.0), 6)              # 1 mm/s * 3 s = 3점 앞으로
        self.assertEqual(self.tr.index(p, 1000.0), 13)            # 응답이 늦어도 명령1의 끝에서 기다림
        self.assertEqual(self.tr.index({**p, "speed": 2.0}, 103.0), 9)   # 배속 반영

    def test_stopped_error_done_hold_position(self):
        for st in ("stopped", "error", "done"):
            self.assertEqual(self.tr.index({"state": st, "acked": 1, "t": 100.0}, 200.0), 3, st)
        self.assertEqual(self.tr.index({"state": "done", "acked": 3, "t": 1.0}, 2.0), 15)

    def test_missing_progress_stays_at_start(self):
        self.assertEqual(self.tr.index(None, 5.0), 0)


class ProgressFileTest(unittest.TestCase):
    def test_write_and_read_roundtrip_keeps_previous_fields(self):
        with tempfile.TemporaryDirectory() as d:
            w = lp.ProgressWriter(Path(d) / "p.json")
            w.write(run_id="r1", total=10, state="running", acked=0)
            w.write(acked=4)
            got = lp.read_progress(Path(d) / "p.json")
            self.assertEqual((got["run_id"], got["total"], got["acked"]), ("r1", 10, 4))
            self.assertIn("t", got)

    def test_reader_never_sees_half_written_json(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "p.json"
            w = lp.ProgressWriter(path)
            w.write(run_id="r", total=5000, state="running", acked=0, note="x" * 5000)
            stop, bad = threading.Event(), []

            def reader():
                while not stop.is_set():
                    try:
                        json.loads(path.read_text(encoding="utf-8"))
                    except (json.JSONDecodeError, PermissionError, FileNotFoundError) as e:
                        if isinstance(e, json.JSONDecodeError):
                            bad.append(e)

            t = threading.Thread(target=reader)
            t.start()
            for i in range(300):
                w.write(acked=i)
            stop.set()
            t.join()
            self.assertEqual(bad, [])

    def test_replace_is_retried_when_file_is_briefly_locked(self):
        with tempfile.TemporaryDirectory() as d:
            w = lp.ProgressWriter(Path(d) / "p.json")
            real = lp.os.replace
            calls = []

            def flaky(a, b):
                calls.append(1)
                if len(calls) < 3:
                    raise PermissionError("locked")
                return real(a, b)

            with mock.patch.object(lp.os, "replace", side_effect=flaky):
                w.write(state="running")
            self.assertEqual(len(calls), 3)
            self.assertEqual(lp.read_progress(Path(d) / "p.json")["state"], "running")

    def test_read_missing_or_broken(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(lp.read_progress(Path(d) / "none.json"))
            (Path(d) / "b.json").write_text("{", encoding="utf-8")
            self.assertIsNone(lp.read_progress(Path(d) / "b.json"))

    def test_to_local_path(self):
        with mock.patch.object(lp.sys, "platform", "linux"):
            self.assertEqual(lp.to_local_path("C:\\Users\\a b\\x.json"), "/mnt/c/Users/a b/x.json")
            self.assertEqual(lp.to_local_path("/home/u/x.json"), "/home/u/x.json")
        with mock.patch.object(lp.sys, "platform", "win32"):
            self.assertEqual(lp.to_local_path("C:\\x.json"), "C:\\x.json")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 실패 확인** — `cd tests && python -m unittest test_live_progress -v` → `ImportError`

- [ ] **Step 3: 구현**

```python
# mirobot_sketch/live_progress.py
"""
실시간 드로잉 진행 파일과 RViz 따라가기 위치 계산
=================================================
GUI(Windows)의 실행 스레드가 로봇의 ok 응답마다 진행 파일을 쓰고, WSL의 sim/rviz_playback.py
--follow가 그 파일을 읽어 로봇팔·펜 자국을 같은 명령 위치로 옮깁니다.

표준 라이브러리만 씁니다 (WSL의 ROS 파이썬에서도 불러 쓰기 때문).
진행 파일 형식: {"run_id", "state": homing|running|stopped|done|error, "acked", "total", "t", "speed",
                "tcp_mm": null(실제 좌표 읽기 자리), "trajectory", "message"}
"""

import bisect
import json
import os
import sys
import time

FINAL_STATES = ("stopped", "error", "done")


class ProgressWriter:
    """임시 파일에 쓴 뒤 os.replace로 바꿔 넣음 (읽는 쪽이 반쪽 JSON을 보지 않게)."""

    def __init__(self, path):
        self.path = str(path)
        self.tmp = self.path + ".tmp"
        self.fields = {"tcp_mm": None}

    def write(self, **fields):
        self.fields.update(fields)
        self.fields["t"] = time.time()
        data = json.dumps(self.fields, ensure_ascii=False)
        with open(self.tmp, "w", encoding="utf-8") as f:
            f.write(data)
        for attempt in range(20):   # Windows: 읽는 쪽이 잠깐 열고 있으면 교체가 실패할 수 있음
            try:
                os.replace(self.tmp, self.path)
                break
            except PermissionError:
                time.sleep(0.005 * (attempt + 1))
        return dict(self.fields)


def read_progress(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def to_local_path(p):
    """진행 파일에 적힌 Windows 경로를 이 컴퓨터에서 열 수 있는 경로로 (WSL이면 /mnt/c/...)."""
    s = str(p)
    if sys.platform != "win32" and len(s) > 2 and s[1] == ":":
        return "/mnt/" + s[0].lower() + s[2:].replace("\\", "/")
    return s


class FollowTrack:
    """궤적 점(cmd 번호)과 명령별 속도로 '지금 보여 줄 점 번호'를 계산."""

    def __init__(self, points, cmd_feed_mm_min, step_mm=1.0):
        cmds = [int(p["cmd"]) for p in points]
        self.n = len(cmds)
        self.cmd_feed = list(cmd_feed_mm_min)
        self.step = float(step_mm)
        count = max(cmds) + 1 if cmds else 0
        self.cmd_end = [bisect.bisect_right(cmds, k) - 1 for k in range(count)]   # 명령 k의 마지막 점

    def _done_index(self, acked):
        if acked <= 0 or not self.cmd_end:
            return 0
        return self.cmd_end[min(acked, len(self.cmd_end)) - 1]

    def index(self, progress, now):
        if not progress:
            return 0
        acked = int(progress.get("acked", 0))
        base = self._done_index(acked)
        if progress.get("state") != "running" or acked >= len(self.cmd_end):
            return base
        limit = self.cmd_end[acked]                           # 지금 실행 중인 명령의 끝
        feed = self.cmd_feed[acked] if acked < len(self.cmd_feed) else 0.0
        elapsed = max(0.0, now - float(progress.get("t", now)))
        ahead = int(elapsed * feed / 60.0 * float(progress.get("speed", 1.0)) / self.step)
        return min(base + ahead, limit)
```

`test_interpolates…`: `acked=1`이면 기준점은 명령0의 끝(3)이고, 명령1(1mm/s)을 3초 진행하면 3+3=6이다. 한도는 `cmd_end[1] = 13`이다.

- [ ] **Step 4: 통과 확인** — `cd tests && python -m unittest test_live_progress -v` → PASS

- [ ] **Step 5: 커밋**

```bash
git add mirobot_sketch/live_progress.py tests/test_live_progress.py
git commit -m "Live progress file (atomic write, retry) and RViz follow index calculation

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: 가상 시뮬레이션 (`virtual_robot.py`)

**Files:**
- Create: `mirobot_sketch/virtual_robot.py`
- Modify: `mirobot_sketch/draw_executor.py` (`MirobotLink.wait_for_homing`에 `should_cancel` 인자)
- Test: `tests/test_virtual_robot.py`

**Interfaces:**
- Consumes: `de.ControllerError`, `cfg["paper_center_tcp_mm"]`, `cfg["timing"]["assumed_command_latency_s"]`
- Produces:
  - `VirtualMirobotLink(cfg, speed=1.0, fail_at=None, timeout_at=None, start_offset_mm=0.0, homing_fails=False, homing_s=1.0)`
    - `.wait_for_homing(timeout, progress=print, should_cancel=None) -> (state, tcp)`
    - `.query_status() -> (state, tcp, raw)`
    - `.send_and_ack(line, timeout, should_stop=None)`: 멈춤을 누르면 `StopRequested`를 던진다. 명령은 끝나지 않은 것으로 본다.
    - `.wait_idle(timeout) -> tcp`, `.close()`, `.sent: list[str]`
  - `de.StopRequested(Exception)`
  - `MirobotLink.wait_for_homing(timeout, progress=print, should_cancel=None)`: 취소하면 `("cancelled", None)`을 돌려준다.
  - `MirobotLink.send_and_ack(line, timeout, should_stop=None)`: 인자만 받고 동작은 지금과 같다. 실제 로봇에서는 보낸 명령을 도중에 멈추지 않는다.

- [ ] **Step 1: 실패하는 테스트**

```python
# tests/test_virtual_robot.py
"""로봇 없이 같은 실행 흐름을 주는 가상 시뮬레이션."""

import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch.virtual_robot import VirtualMirobotLink  # noqa: E402

CFG = de.load_config()
SQUARE = [[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0), (-10.0, -10.0)]]


class VirtualLinkTest(unittest.TestCase):
    def test_homing_reports_idle_at_paper_center(self):
        v = VirtualMirobotLink(CFG, homing_s=0.05)
        state, tcp = v.wait_for_homing(5, progress=lambda *_: None)
        c = CFG["paper_center_tcp_mm"]
        self.assertEqual(state, "Idle")
        self.assertAlmostEqual(tcp[0], c["x"])
        v2 = VirtualMirobotLink(CFG, homing_s=0.05, start_offset_mm=7.0)
        self.assertGreater(abs(v2.wait_for_homing(5, progress=lambda *_: None)[1][2] - c["z"]), 6.9)

    def test_homing_can_be_cancelled_and_can_fail(self):
        v = VirtualMirobotLink(CFG, homing_s=10)
        t0 = time.monotonic()
        self.assertEqual(v.wait_for_homing(20, lambda *_: None, should_cancel=lambda: True)[0], "cancelled")
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertEqual(VirtualMirobotLink(CFG, homing_s=0.05, homing_fails=True)
                         .wait_for_homing(0.3, lambda *_: None)[0], "Alarm")

    def test_ack_takes_expected_time_scaled_by_speed(self):
        cmds = de.Planner(CFG).plan(SQUARE)
        v = VirtualMirobotLink(CFG, speed=50)
        v.wait_for_homing(1, lambda *_: None)
        t0 = time.monotonic()
        result = de.execute(v, cmds, CFG, progress=lambda *_: None)
        took = time.monotonic() - t0
        lat = CFG["timing"]["assumed_command_latency_s"]      # 기대값 = 명령마다 (거리/속도 + 지연) / 배속
        pos, expected = VirtualMirobotLink(CFG).pos, 0.0
        for line, _ in cmds:
            xyz = tuple(float(v) for v in re.search(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)", line).groups())
            feed = float(re.search(r"F([\d.]+)", line).group(1))
            expected += (sum((a - b) ** 2 for a, b in zip(xyz, pos)) ** 0.5 / (feed / 60) + lat) / 50
            pos = xyz
        self.assertEqual(result["result"], "completed")
        self.assertEqual(len(v.sent), len(cmds))
        self.assertLess(abs(took - expected), max(0.5, expected * 0.5))

    def test_fail_and_timeout(self):
        cmds = de.Planner(CFG).plan(SQUARE)
        r = de.execute(VirtualMirobotLink(CFG, speed=100, fail_at=3), cmds, CFG, progress=lambda *_: None)
        self.assertEqual((r["result"], r["commands_sent"]), ("stopped_on_error", 2))
        cfg = {**CFG, "ack_timeout_s": 0.2}
        r = de.execute(VirtualMirobotLink(cfg, speed=100, timeout_at=2), cmds, cfg, progress=lambda *_: None)
        self.assertEqual((r["result"], r["commands_sent"]), ("stopped_on_error", 1))
        self.assertIn("시간 초과", r["error"])

    def test_stop_interrupts_a_long_move_quickly(self):
        cmds = de.Planner(CFG).plan(SQUARE)
        v = VirtualMirobotLink(CFG, speed=1)                  # 1배속: 한 명령이 몇 초
        stop = threading.Event()
        threading.Timer(0.3, stop.set).start()
        t0 = time.monotonic()
        r = de.execute(v, cmds, CFG, progress=lambda *_: None, should_stop=stop.is_set)
        self.assertEqual(r["result"], "stopped_by_user")
        self.assertLess(time.monotonic() - t0, 0.9)
        self.assertEqual(len(v.sent), r["commands_sent"] + 1)   # 멈춘 명령 뒤로는 안 보냄


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 실패 확인** — `cd tests && python -m unittest test_virtual_robot -v` → `ModuleNotFoundError: virtual_robot`

- [ ] **Step 3: 구현**

`draw_executor.py`의 `ControllerError` 아래:

```python
class StopRequested(Exception):
    """사용자가 [멈춤]을 눌러 다음 명령을 보내지 않음."""
```

`MirobotLink.wait_for_homing`의 시그니처를 `(self, timeout, progress=print, should_cancel=None)`으로 바꾸고, 반복문 맨 앞에 넣는다.

```python
            if should_cancel and should_cancel():
                return "cancelled", None
```

`MirobotLink.send_and_ack`의 시그니처는 `(self, line, timeout, should_stop=None)`이다(실제 로봇은 보낸 명령을 도중에 멈추지 않으므로 인자는 받기만 함).

`execute`는 Task 4에서 `should_stop`과 `on_ack`를 받도록 바꾼다. 이 작업의 테스트가 `execute(..., should_stop=)`를 쓰므로, **Task 4의 `execute` 변경을 이 작업에서 먼저 한다**(아래 코드). Task 4에서는 나머지 함수만 나눈다.

```python
def execute(link, cmds, cfg, progress=print, on_ack=None, should_stop=None):
    """계획된 명령을 순서대로 보냅니다. 반환: 실행 결과 dict. 오류·멈춤 시 즉시 멈춥니다(자동 복구 없음).
    on_ack(보낸 수, 전체 수): ok마다 호출 (GUI 진행 표시·RViz 따라가기). should_stop(): True면 다음 명령을 보내지 않음."""
    started = time.monotonic()
    sent = 0
    stop = should_stop or (lambda: False)
    try:
        for i, (line, label) in enumerate(cmds):
            if stop():
                raise StopRequested()
            link.send_and_ack(line, cfg["ack_timeout_s"], should_stop=stop) if should_stop \
                else link.send_and_ack(line, cfg["ack_timeout_s"])
            sent = i + 1
            if on_ack:
                on_ack(sent, len(cmds))
            if label.endswith("pen-down") or i == len(cmds) - 1:
                progress(f"  [{sent}/{len(cmds)}] {label}")
        link.wait_idle(cfg["idle_timeout_s"])
        return {"result": "completed", "commands_sent": sent, "elapsed_s": round(time.monotonic() - started, 1)}
    except StopRequested:
        return {"result": "stopped_by_user", "commands_sent": sent,
                "elapsed_s": round(time.monotonic() - started, 1)}
    except ControllerError as e:
        return {"result": "stopped_on_error", "error": str(e), "commands_sent": sent,
                "failed_command": cmds[sent][1] if sent < len(cmds) else None,
                "elapsed_s": round(time.monotonic() - started, 1)}
    except KeyboardInterrupt:
        return {"result": "interrupted_by_user", "commands_sent": sent,
                "elapsed_s": round(time.monotonic() - started, 1)}
```

`should_stop`이 없을 때 `send_and_ack`를 예전처럼 두 인자로 부르는 이유는, 기존 테스트의 가짜 연결 객체가 `should_stop` 인자를 모르기 때문이다(호환).

```python
# mirobot_sketch/virtual_robot.py
"""
가상 시뮬레이션 — 로봇 없이 실행 흐름(호밍 → 명령·ok → 멈춤·오류)을 그대로 시험
===================================================================================
MirobotLink와 같은 메서드를 가집니다. 명령마다 실제로 걸릴 시간(거리 / 속도 + 명령 지연 가정값)만큼
기다렸다가 ok를 돌려주며, speed(배속)로 줄일 수 있습니다. 오류 상황도 흉내 냅니다.
"""

import re
import time

from .draw_executor import ControllerError, StopRequested

_XYZ = re.compile(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)")
_F = re.compile(r"F([\d.]+)")
POLL_S = 0.02     # 기다리는 동안 멈춤 확인 간격


class VirtualMirobotLink:
    virtual = True

    def __init__(self, cfg, speed=1.0, fail_at=None, timeout_at=None, start_offset_mm=0.0,
                 homing_fails=False, homing_s=1.0):
        self.cfg, self.speed = cfg, max(float(speed), 1e-6)
        self.fail_at, self.timeout_at = fail_at, timeout_at
        self.homing_fails, self.homing_s = homing_fails, homing_s
        c = cfg["paper_center_tcp_mm"]
        self.pos = (c["x"], c["y"], c["z"] + float(start_offset_mm))
        self.state = "Alarm"
        self.sent = []
        self.latency = cfg.get("timing", {}).get("assumed_command_latency_s", 0.1)

    def _sleep(self, seconds, should_stop=None):
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return
            if should_stop and should_stop():
                raise StopRequested()
            time.sleep(min(POLL_S, left))

    def wait_for_homing(self, timeout, progress=print, should_cancel=None):
        progress("  컨트롤러 상태: Alarm (가상)")
        progress("  -> 가상 시뮬레이션: 잠시 뒤 자동으로 호밍됩니다.")
        deadline = time.monotonic() + timeout
        ready = time.monotonic() + self.homing_s
        while time.monotonic() < deadline:
            if should_cancel and should_cancel():
                return "cancelled", None
            if time.monotonic() >= ready and not self.homing_fails:
                self.state = "Idle"
                progress("  컨트롤러 상태: Idle (가상)")
                return "Idle", self.pos
            time.sleep(POLL_S)
        return self.state, None

    def query_status(self, timeout=2.0):
        return self.state, self.pos, f"<{self.state},virtual>"

    def send_and_ack(self, line, timeout, should_stop=None):
        self.sent.append(line)
        n = len(self.sent)
        if self.fail_at == n:
            raise ControllerError("컨트롤러 오류 응답: Alarm (가상 시뮬레이션 오류 흉내)")
        if self.timeout_at == n:
            self._sleep(timeout, should_stop)
            raise ControllerError(f"ok 응답 시간 초과: {line}")
        m, f = _XYZ.search(line), _F.search(line)
        if m:
            target = tuple(float(v) for v in m.groups())
            dist = sum((a - b) ** 2 for a, b in zip(target, self.pos)) ** 0.5
            feed = float(f.group(1)) if f else 1000.0
            self._sleep((dist / (feed / 60.0) + self.latency) / self.speed, should_stop)
            self.pos = target

    def wait_idle(self, timeout):
        return self.pos

    def close(self):
        self.state = "closed"
```

대기 시간은 명령마다 (직전 위치와의 거리 ÷ F + 명령 지연) ÷ 배속이다. 테스트의 기대값도 같은 식으로 직접 계산한다(`test_virtual_robot.py` 상단에 `import re`).

- [ ] **Step 4: 통과 확인** — `cd tests && python -m unittest test_virtual_robot test_draw_executor -v` → PASS (기존 실행기 테스트 포함)

- [ ] **Step 5: 커밋**

```bash
git add mirobot_sketch/virtual_robot.py mirobot_sketch/draw_executor.py tests/test_virtual_robot.py
git commit -m "Virtual simulation link; execute() gets on_ack/should_stop; cancellable homing

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: 실행기를 단계 함수로 나누기 (+ 명령줄 `--virtual`)

**Files:**
- Modify: `mirobot_sketch/draw_executor.py`
- Test: `tests/test_draw_executor.py` (새 테스트 추가만)

**Interfaces:**
- Consumes: `VirtualMirobotLink`, `execute`, `StopRequested`
- Produces:
  - `DRAW_STEPS = (("preflight", "사전 검사", "auto"), ("connect", "연결·호밍", "person"), ("start", "시작 위치 확인", "auto"), ("confirm", "최종 확인", "person"), ("drawing", "그리는 중", "auto"), ("done", "끝", "auto"))`
  - `class DrawError(Exception)`: `.step`, `.message`, `.hint`
  - `preflight(strokes, cfg, pending=False, air=False) -> {"cmds", "timing", "planner"}`: 범위를 넘으면 `DrawError("preflight")`
  - `open_link(cfg, virtual=False, virtual_speed=20.0, verbose=False)`
  - `connect_and_home(link, cfg, progress=print, should_cancel=None) -> tcp`: Idle이 아니거나 취소하면 `DrawError("connect")`
  - `check_start(tcp, cfg)`: 벗어나면 `DrawError("start")`
  - `write_run_record(meta: dict, result: dict, cfg) -> Path`
  - 명령줄 `--virtual`, `--virtual-speed`

- [ ] **Step 1: 실패하는 테스트** (`tests/test_draw_executor.py` 끝에 새 클래스)

```python
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

    def test_cli_virtual_run_writes_record(self):
        import tempfile
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            doc = Path(d) / "s.json"
            doc.write_text(json.dumps({"kind": "sketch_strokes", "units": "mm", "strokes": [
                {"points_xy_mm": [[-5, -5], [5, -5], [5, 5]]}]}), encoding="utf-8")
            with mock.patch.object(de.paths, "runs_dir", lambda: Path(d) / "runs"), \
                    mock.patch("builtins.input", return_value="yes"), \
                    mock.patch.object(sys, "argv", ["mirobot-draw", str(doc), "--execute", "--virtual",
                                                    "--virtual-speed", "200"]):
                self.assertEqual(de.main(), 0)
            rec = json.loads(next((Path(d) / "runs").glob("run-*.json")).read_text(encoding="utf-8"))
            self.assertTrue(rec["virtual"])
            self.assertEqual(rec["result"], "completed")
```

(`tests/test_draw_executor.py` 상단에 `import json`이 없으면 추가한다.)

- [ ] **Step 2: 실패 확인** — `cd tests && python -m unittest test_draw_executor.StepFunctionsTest -v` → `AttributeError: DRAW_STEPS`

- [ ] **Step 3: 구현** (`draw_executor.py`, `execute` 위에)

```python
# 실행 단계 (GUI 단계 표시줄과 명령줄 출력이 같은 이름을 씀): (id, 이름, 누가)
DRAW_STEPS = (("preflight", "사전 검사", "auto"), ("connect", "연결·호밍", "person"),
              ("start", "시작 위치 확인", "auto"), ("confirm", "최종 확인", "person"),
              ("drawing", "그리는 중", "auto"), ("done", "끝", "auto"))


class DrawError(Exception):
    """단계에서 멈춤: 어느 단계(step), 무엇이(message), 어떻게 하면 되는지(hint)."""

    def __init__(self, step, message, hint=""):
        super().__init__(message)
        self.step, self.message, self.hint = step, message, hint


def preflight(strokes, cfg, pending=False, air=False):
    """① 범위 검사 + 명령 계획 + 시간 추정. 로봇에 연결하기 전."""
    if not strokes:
        raise DrawError("preflight", "그릴 획이 없습니다.")
    bad = check_limits(strokes, cfg, pending)
    if bad:
        i, x, y = bad[0]
        hint = ("그림 크기를 줄이세요." if pending else
                "그림 크기를 줄이거나, 실물 확인 전 넓은 범위(±60mm)를 쓰려면 '넓은 범위 허용'을 켜세요.")
        raise DrawError("preflight", f"허용 범위를 벗어난 점 {len(bad)}개 (예: 획 {i}, x={x:.1f}, y={y:.1f} mm)", hint)
    planner = Planner(cfg, air=air)
    return {"cmds": planner.plan(strokes), "timing": estimate_time(strokes, cfg), "planner": planner}


def open_link(cfg, virtual=False, virtual_speed=20.0, verbose=False):
    if virtual:
        from .virtual_robot import VirtualMirobotLink
        return VirtualMirobotLink(cfg, speed=virtual_speed)
    try:
        return MirobotLink.open(cfg["port"], cfg["baud"], verbose)
    except Exception as e:  # 포트 없음·사용 중 (serial.SerialException 등)
        raise DrawError("connect", f"포트 {cfg['port']}를 열 수 없습니다: {e}",
                        "USB 연결과 포트 번호(drawing_config.json의 port)를 확인하고, 다른 프로그램이 쓰고 있지 않은지 보세요.")


def connect_and_home(link, cfg, progress=print, should_cancel=None):
    """② 호밍 대기 (자동 호밍 명령은 보내지 않음). 반환: TCP."""
    state, tcp = link.wait_for_homing(cfg["idle_timeout_s"], progress=progress, should_cancel=should_cancel)
    if state == "cancelled":
        raise DrawError("connect", "취소했습니다.")
    if state != "Idle":
        raise DrawError("connect", f"Idle이 되지 않았습니다 (상태: {state}).",
                        "로봇 가운데 네비게이션 버튼을 2초 눌러 호밍한 뒤 다시 시작하세요.")
    if tcp is None:
        raise DrawError("connect", "TCP 좌표를 읽지 못했습니다.", "연결을 다시 해 보세요.")
    return tcp


def check_start(tcp, cfg):
    """③ 현재 펜 끝이 설정된 종이 중심 근처인지."""
    c = cfg["paper_center_tcp_mm"]
    off = ((tcp[0] - c["x"]) ** 2 + (tcp[1] - c["y"]) ** 2 + (tcp[2] - c["z"]) ** 2) ** 0.5
    if off > cfg["max_start_offset_mm"]:
        raise DrawError("start", f"펜 끝이 종이 중심 설정에서 {off:.1f} mm 떨어져 있습니다 (허용 {cfg['max_start_offset_mm']} mm).",
                        "호밍 뒤 펜 끝이 종이 가운데에 오도록 종이 위치를 맞추세요.")
    return off


def write_run_record(meta, result, cfg):
    """⑥ runs/run-<시각>.json. meta: strokes_json, stroke_count, command_count, air_mode, pending_limits,
    estimated_time, source, virtual, virtual_speed 등."""
    run_dir = paths.runs_dir()
    run_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run = {"started_local": stamp, **meta, "config_snapshot": cfg, **result, "visual_verification": "pending"}
    path = run_dir / f"run-{stamp}.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(run, f, ensure_ascii=False, indent=1)
    return path
```

`main()`을 이 함수들로 바꾼다. 인자, 출력 문구, 반환 코드는 지금과 같게 유지하고 `--virtual`만 추가한다.

```python
    ap.add_argument("--virtual", action="store_true", help="로봇 없이 가상 시뮬레이션으로 실행 (--execute와 함께)")
    ap.add_argument("--virtual-speed", type=float, default=20.0, help="가상 시뮬레이션 배속")
    ...
    try:
        pre = preflight(strokes, cfg, args.pending_limits, args.air)
    except DrawError as e:
        print(f"거부: {e.message}")
        print(e.hint or "CV 단계에서 --box를 줄이거나 drawing_config.json의 limits를 확인하세요.")
        return 2
    planner, cmds, timing = pre["planner"], pre["cmds"], pre["timing"]
    ... (요약 출력은 그대로) ...
    if not args.execute: (dry-run 그대로)

    try:
        link = open_link(cfg, args.virtual, args.virtual_speed, args.verbose)
    except DrawError as e:
        print(e.message)
        return 3
    try:
        try:
            tcp = connect_and_home(link, cfg)
            print(f"컨트롤러 상태: Idle, TCP: {tcp}")
            check_start(tcp, cfg)
        except DrawError as e:
            print(f"{e.message} 중단합니다." + (f" ({e.hint})" if e.hint else ""))
            return 3
        if input("종이·펜·주변을 확인했으면 yes 입력: ").strip().lower() != "yes":
            print("취소했습니다.")
            return 0
        result = execute(link, cmds, cfg)
    finally:
        link.close()

    print(f"결과: {result}")
    if result["result"] != "completed":
        print("자동 복구를 하지 않았습니다. 펜과 로봇 상태를 확인한 뒤 수동으로 조치하세요.")
    path = write_run_record({"strokes_json": str(args.strokes_json), "stroke_count": len(strokes),
                             "command_count": len(cmds), "air_mode": args.air, "pending_limits": args.pending_limits,
                             "estimated_time": timing, "source": doc.get("source", {}),
                             "virtual": bool(args.virtual), "virtual_speed": args.virtual_speed if args.virtual else None},
                            result, cfg)
    print(f"실행 기록: {path} (종이 사진 확인 결과를 visual_verification에 적어두세요)")
    return 0 if result["result"] == "completed" else 4
```

- [ ] **Step 4: 통과 확인** — `python -m unittest discover -s tests` → OK (기존 실행기 테스트 그대로 통과)

- [ ] **Step 5: 커밋**

```bash
git add mirobot_sketch/draw_executor.py tests/test_draw_executor.py
git commit -m "Split executor into step functions (preflight/connect/start/execute/record) with DRAW_STEPS; CLI --virtual

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: `DrawJob` — 화면 없이 도는 실행 작업

**Files:**
- Create: `mirobot_sketch/draw_job.py`
- Test: `tests/test_draw_job.py`

**Interfaces:**
- Consumes:
  - `de.preflight / open_link / connect_and_home / check_start / execute / write_run_record / DRAW_STEPS / DrawError`
  - `live_progress.ProgressWriter`, `mirobot_sim.trajectory_doc`
  - 세션의 `result["strokes_mm"]`, `simulate()`, `sim`
- Produces:
  - `DrawJob(session, cfg, events, progress_path=None, launch_rviz=None)`
    - `events(kind, **data)`: 작업 스레드에서 불린다. kind는 `step`(id, status: active|done|failed, message, hint), `progress`(acked, total, stroke, strokes, elapsed_s, remaining_s), `finished`(result, record_path)다.
    - `.start(virtual=True, virtual_speed=20.0)`: ①②③을 진행한 뒤 ④에서 멈춰 기다린다.
    - `.confirm(air=True, pending=False, checked=True)`: ⑤를 시작한다. `checked`가 False면 거부한다.
    - `.stop()`, `.cancel()`: ②④에서는 취소, ⑤에서는 멈춤이다.
    - `.join(timeout)`, `.state`(단계 id), `.summary`(④에 보여 줄 요약 dict)
  - 이 파일은 설계 문서 §6의 파일 목록에 없는데, GUI 없이 테스트하려고 추가한다(ledger ruling).

- [ ] **Step 1: 실패하는 테스트**

```python
# tests/test_draw_job.py
"""실행 작업(가상 시뮬레이션): 단계 순서, 확인 전 대기, 멈춤, 기록, 진행 파일."""

import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import draw_executor as de  # noqa: E402
from mirobot_sketch import live_progress as lp  # noqa: E402
from mirobot_sketch.draw_job import DrawJob  # noqa: E402
from mirobot_sketch.session import SketchSession  # noqa: E402

import golden  # noqa: E402


class Recorder:
    def __init__(self):
        self.events, self.lock = [], threading.Lock()
        self.ready = threading.Event()
        self.done = threading.Event()

    def __call__(self, kind, **data):
        with self.lock:
            self.events.append((kind, data))
        if kind == "step" and data["id"] == "confirm" and data["status"] == "active":
            self.ready.set()
        if kind == "finished":
            self.done.set()

    def steps(self):
        return [(d["id"], d["status"]) for k, d in self.events if k == "step"]


class DrawJobTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        p = Path(cls.tmp.name) / "line.png"
        cv2.imwrite(str(p), golden.synthetic_images()["line"])
        cls.session = SketchSession()
        cls.session.set_image(p)
        cls.session.update_params({"box_mm": 60, "epsilon_px": 4.0})
        cls.session.run_current()
        cls.session.simulate()

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def job(self, rec, **kw):
        runs = Path(self.tmp.name) / "runs"
        self.patch = mock.patch.object(de.paths, "runs_dir", lambda: runs)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        return DrawJob(self.session, self.session.cfg, rec,
                       progress_path=Path(self.tmp.name) / "live.json", **kw)

    def test_waits_for_confirmation_then_runs_all_steps(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        self.assertEqual(job.state, "confirm")
        self.assertIn("command_count", job.summary)
        with self.assertRaises(ValueError):
            job.confirm(checked=False)                         # 확인 체크 없이는 시작 안 함
        job.confirm(air=True, checked=True)
        self.assertTrue(rec.done.wait(60))
        done_ids = [i for i, st in rec.steps() if st == "done"]
        self.assertEqual(done_ids, ["preflight", "connect", "start", "confirm", "drawing"])
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")
        self.assertTrue(Path(fin["record_path"]).exists())
        prog = [d for k, d in rec.events if k == "progress"]
        self.assertEqual(prog[-1]["acked"], prog[-1]["total"])
        live = lp.read_progress(Path(self.tmp.name) / "live.json")
        self.assertEqual((live["state"], live["acked"]), ("done", live["total"]))
        self.assertTrue(Path(lp.to_local_path(live["trajectory"])).exists())

    def test_stop_during_drawing(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=1)
        self.assertTrue(rec.ready.wait(10))
        job.confirm(checked=True)
        threading.Timer(0.5, job.stop).start()
        self.assertTrue(rec.done.wait(5))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "stopped_by_user")
        self.assertEqual(lp.read_progress(Path(self.tmp.name) / "live.json")["state"], "stopped")

    def test_cancel_at_confirmation_closes_without_record(self):
        rec = Recorder()
        job = self.job(rec)
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        job.cancel()
        self.assertTrue(rec.done.wait(5))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "cancelled")
        self.assertIsNone(fin["record_path"])

    def test_rviz_unavailable_does_not_block_drawing(self):
        rec = Recorder()

        def no_rviz(*a, **k):
            from mirobot_sketch.rviz_launch import RvizUnavailable
            raise RvizUnavailable("ROS 없음")

        job = self.job(rec, launch_rviz=no_rviz)
        job.start(virtual=True, virtual_speed=500)
        self.assertTrue(rec.ready.wait(10))
        job.confirm(checked=True)
        self.assertTrue(rec.done.wait(60))
        self.assertTrue(any(k == "rviz" and not d["ok"] for k, d in rec.events))
        fin = next(d for k, d in rec.events if k == "finished")
        self.assertEqual(fin["result"]["result"], "completed")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 실패 확인** — `cd tests && python -m unittest test_draw_job -v` → `ModuleNotFoundError: draw_job`

- [ ] **Step 3: 구현**

```python
# mirobot_sketch/draw_job.py
"""
로봇으로 그리기 작업 — 안전 절차 ①~⑥을 작업 스레드에서 진행 (화면과 무관, GUI 실행 창이 events로 표시)
======================================================================================================
①사전 검사 → ②연결·호밍 → ③시작 위치 → ④최종 확인(사람이 confirm 할 때까지 대기) → ⑤그리는 중 → ⑥끝.
ok 응답마다 진행 파일을 써서 RViz 따라가기가 같은 위치를 보여 줍니다. 자동 복구는 하지 않습니다.
"""

import json
import threading
import time
from datetime import datetime

from . import draw_executor as de
from . import live_progress as lp
from . import paths


class DrawJob:
    def __init__(self, session, cfg, events, progress_path=None, launch_rviz=None):
        self.session, self.cfg, self.events = session, cfg, events
        self.progress_path = progress_path or (paths.output_dir() / "live_progress.json")
        self.launch_rviz = launch_rviz
        self.state = "idle"
        self.summary = {}
        self._confirm = threading.Event()
        self._stop = threading.Event()
        self._choice = {}
        self._thread = None
        self.link = None

    # ---------------------------------------------------------------- 사람이 누르는 것
    def start(self, virtual=True, virtual_speed=20.0):
        self._virtual, self._speed = virtual, virtual_speed
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def confirm(self, air=True, pending=False, checked=False):
        if not checked:
            raise ValueError("종이·펜·주변 확인 체크가 필요합니다")
        if self.state != "confirm":
            raise ValueError("최종 확인 단계가 아닙니다")
        self._choice = {"air": bool(air), "pending": bool(pending)}
        self._confirm.set()

    def stop(self):
        """⑤에서는 다음 명령부터 보내지 않음, 그 전 단계에서는 취소."""
        self._stop.set()
        self._confirm.set()

    cancel = stop

    def join(self, timeout=None):
        if self._thread:
            self._thread.join(timeout)

    # ---------------------------------------------------------------- 작업 스레드
    def _step(self, sid, status, message="", hint=""):
        if status == "active":
            self.state = sid
        self.events("step", id=sid, status=status, message=message, hint=hint)

    def _run(self):
        result, record = {"result": "cancelled"}, None
        writer = lp.ProgressWriter(self.progress_path)
        run_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        try:
            # ① 사전 검사
            self._step("preflight", "active", "범위와 로봇 시뮬레이션을 확인합니다")
            s = self.session
            if s.sim is None:
                s.simulate()
            if s.sim["summary"]["verdict"].startswith("FAIL"):
                raise de.DrawError("preflight", f"로봇 시뮬레이션이 FAIL입니다: {s.sim['summary']['verdict']}",
                                   "그림 크기를 줄이거나 설정을 바꾼 뒤 다시 시뮬레이션하세요.")
            strokes = [[tuple(p) for p in st] for st in s.result["strokes_mm"]]
            pre = de.preflight(strokes, self.cfg, pending=False, air=True)
            self.summary = {"stroke_count": len(strokes), "command_count": len(pre["cmds"]),
                            "estimated_s": pre["timing"]["total_s"],
                            "drawing_mm": [s.result["placement"]["drawing_width_mm"],
                                           s.result["placement"]["drawing_height_mm"]],
                            "port": "가상 시뮬레이션" if self._virtual else self.cfg["port"],
                            "virtual": self._virtual, "virtual_speed": self._speed}
            self._step("preflight", "done", f"획 {len(strokes)}개 · 명령 {len(pre['cmds'])}줄 · 시뮬레이션 PASS")
            # ② 연결·호밍
            self._step("connect", "active", "로봇 가운데 버튼을 2초 눌러 호밍하세요. Idle이 되면 자동으로 넘어갑니다"
                       if not self._virtual else "가상 시뮬레이션에 연결합니다")
            self.link = de.open_link(self.cfg, self._virtual, self._speed)
            writer.write(run_id=run_id, state="homing", acked=0, total=len(pre["cmds"]), speed=self._speed if
                         self._virtual else 1.0, trajectory=None, message="호밍 대기")
            tcp = de.connect_and_home(self.link, self.cfg, progress=lambda *_: None, should_cancel=self._stop.is_set)
            self._step("connect", "done", "Idle")
            # ③ 시작 위치
            self._step("start", "active", "펜 끝 위치를 확인합니다")
            off = de.check_start(tcp, self.cfg)
            self._step("start", "done", f"종이 중심에서 {off:.1f} mm")
            # ④ 최종 확인 (사람)
            self._step("confirm", "active", "종이·펜·주변을 확인하고 시작하세요")
            self._confirm.wait()
            if self._stop.is_set():
                raise de.DrawError("confirm", "취소했습니다.")
            ch = self._choice
            pre = de.preflight(strokes, self.cfg, pending=ch["pending"], air=ch["air"])
            self._step("confirm", "done", "공중 모드" if ch["air"] else "펜으로 그림")
            # ⑤ 그리는 중
            traj = self._write_trajectory(run_id)
            writer.write(state="running", acked=0, total=len(pre["cmds"]), trajectory=str(traj), message="그리는 중")
            self._open_rviz(traj)
            self._step("drawing", "active", "그리는 중")
            started = time.monotonic()
            est = pre["timing"]["total_s"]
            starts = [i for i, (_, label) in enumerate(pre["cmds"]) if label.endswith("pen-down")]

            def on_ack(acked, total):
                writer.write(state="running", acked=acked)
                el = time.monotonic() - started
                stroke = sum(1 for i in starts if i < acked)
                ratio = el / max(est * acked / total, 1e-6) if acked else 1.0   # 실제/예상 속도 비로 보정
                remain = est * (1 - acked / total) * ratio
                self.events("progress", acked=acked, total=total, stroke=stroke, strokes=len(strokes),
                            elapsed_s=el, remaining_s=remain)

            result = de.execute(self.link, pre["cmds"], self.cfg, progress=lambda *_: None,
                                on_ack=on_ack, should_stop=self._stop.is_set)
            final = {"completed": "done", "stopped_by_user": "stopped"}.get(result["result"], "error")
            writer.write(state=final, message=result.get("error", result["result"]))
            self._step("drawing", "done" if final == "done" else "failed",
                       {"done": "완료", "stopped": "사용자 멈춤"}.get(final, result.get("error", "")),
                       "" if final == "done" else "자동 복구를 하지 않았습니다. 펜과 로봇 상태를 확인하세요.")
            record = de.write_run_record({
                "strokes_json": s.result["path"], "stroke_count": len(strokes), "command_count": len(pre["cmds"]),
                "air_mode": ch["air"], "pending_limits": ch["pending"], "estimated_time": pre["timing"],
                "source": {"image": s.result["path"], "params": s.result["params"]},
                "virtual": self._virtual, "virtual_speed": self._speed if self._virtual else None,
                "run_id": run_id, "trajectory": str(traj)}, result, self.cfg)
        except de.DrawError as e:
            self._step(e.step, "failed", e.message, e.hint)
            result = {"result": "cancelled" if e.message == "취소했습니다." else "not_started", "error": e.message}
            writer.write(state="stopped", message=e.message)
        except Exception as e:  # 예상 못 한 오류도 화면에 보이게 (포트는 아래에서 닫음)
            self._step(self.state if self.state != "idle" else "preflight", "failed", f"{type(e).__name__}: {e}")
            result = {"result": "error", "error": str(e)}
            writer.write(state="error", message=str(e))
        finally:
            if self.link is not None:
                self.link.close()
            self.state = "done"
            self._step("done", "done", result["result"])
            self.events("finished", result=result, record_path=str(record) if record else None)

    def _write_trajectory(self, run_id):
        from . import mirobot_sim as ms
        path = paths.output_dir() / f"live_traj_{run_id}.json"
        doc = ms.trajectory_doc(self.session.sim["raw"], self.cfg, self.session.result["path"])
        path.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
        return path

    def _open_rviz(self, traj):
        launch = self.launch_rviz
        if launch is None:
            from .rviz_launch import launch
        try:
            launch(traj, follow=self.progress_path)
            self.events("rviz", ok=True, message="RViz 따라가기를 열었습니다")
        except Exception as e:  # RvizUnavailable 포함: 드로잉은 계속
            self.events("rviz", ok=False, message=str(e))
```

`_write_trajectory`는 경로를 `self._traj_path = path`로도 저장한다(실행 창의 [RViz 3D로 따라보기]가 씀).

(`_write_trajectory`는 테스트에서 `paths.output_dir()`에 쓴다. 테스트 폴더를 오염시키지 않도록 `test_draw_job`의 `job()`에서 `paths.output_dir`도 임시 폴더로 바꾼다: `mock.patch.object(de.paths, "output_dir", lambda: Path(self.tmp.name))`. `draw_job`은 `from . import paths`로 같은 모듈 객체를 쓰므로 패치가 적용된다.)

- [ ] **Step 4: 통과 확인** — `cd tests && python -m unittest test_draw_job -v` → PASS. 이어서 전체 테스트 → OK

- [ ] **Step 5: 커밋**

```bash
git add mirobot_sketch/draw_job.py tests/test_draw_job.py
git commit -m "DrawJob: run safety steps 1-6 in a worker thread with confirm/stop, progress file and run record

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: RViz 따라가기 (`--follow`)

**Files:**
- Modify: `sim/rviz_playback.py`, `sim/run_rviz.sh`, `mirobot_sketch/rviz_launch.py`, `packaging/mirobot_sketch.spec`
- Test: `tests/test_rviz_launch.py`

**Interfaces:**
- Consumes: `live_progress.FollowTrack`, `read_progress`, `to_local_path`
- Produces:
  - `rviz_launch.launch(traj_path, speed=20, follow=None, run=..., popen=...)`. `follow`가 있으면 `run_rviz.sh <traj> <speed> <follow>`로 실행한다.
  - `run_rviz.sh`는 세 번째 인자가 있으면 `rviz_playback.py <traj> --follow <path>`로 실행하고, `PYTHONPATH`에 `$HERE:$HERE/..`를 넣는다.
  - 설치판 exe: `sim/mirobot_sketch/__init__.py`와 `sim/mirobot_sketch/live_progress.py`를 데이터로 함께 넣는다.

- [ ] **Step 1: 실패하는 테스트** (`tests/test_rviz_launch.py`)

```python
    def test_launch_follow_mode_passes_progress_path(self):
        popen = mock.Mock()
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(rl.sys, "platform", "win32"), mock.patch.object(rl, "_no_window", lambda: 0), \
                mock.patch.object(rl.paths, "output_dir", lambda: Path(d)), \
                mock.patch.object(rl, "to_wsl_path", lambda p: "/mnt/c/" + Path(p).name):
            rl.launch(Path(d) / "t.json", speed=1, follow=Path(d) / "live.json",
                      run=fake_run("Ubuntu-22.04"), popen=popen)
        self.assertEqual(popen.call_args.args[0][-1], "bash /mnt/c/run_rviz.sh /mnt/c/t.json 1 /mnt/c/live.json")

    def test_playback_script_supports_follow(self):
        src = (rl.scripts_dir() / "rviz_playback.py").read_text(encoding="utf-8")
        self.assertIn("--follow", src)
        self.assertIn("FollowTrack", src)
        sh = (rl.scripts_dir() / "run_rviz.sh").read_text(encoding="utf-8")
        self.assertIn("--follow", sh)
        self.assertIn("PYTHONPATH", sh)
        self.assertNotIn("\r\n", sh)
```

- [ ] **Step 2: 실패 확인** — `cd tests && python -m unittest test_rviz_launch -v` → 2 FAIL

- [ ] **Step 3: 구현**

`rviz_launch.launch`:

```python
def launch(traj_path, speed=20, follow=None, run=subprocess.run, popen=subprocess.Popen):
    """RViz 재생을 백그라운드로 시작하고 Popen을 돌려줌. follow=진행 파일이면 실시간 따라가기 모드."""
    ...(기존 검사 그대로)...
    cmd = f"bash {shlex.quote(to_wsl_path(sdir / 'run_rviz.sh'))} {shlex.quote(to_wsl_path(traj_path))} {float(speed):g}"
    if follow is not None:
        cmd += f" {shlex.quote(to_wsl_path(follow))}"
    ...(Popen 그대로)...
```

`run_rviz.sh` (마지막 재생 부분을 교체):

```bash
FOLLOW="${3:-}"
# rviz_playback.py가 mirobot_sketch.live_progress를 불러 쓰도록 (저장소: 상위 폴더, 설치판: sim/mirobot_sketch 복사본)
export PYTHONPATH="$HERE:$HERE/..${PYTHONPATH:+:$PYTHONPATH}"
if [ -n "$FOLLOW" ]; then
  python3 "$HERE/rviz_playback.py" "$TRAJ" --follow "$FOLLOW" &
else
  python3 "$HERE/rviz_playback.py" "$TRAJ" --speed "$SPEED" --loop &
fi
PLAY=$!
```

`rviz_playback.py`: `--follow` 인자를 추가하고, `Playback`을 확장한다(파일 머리 docstring에도 한 줄 추가).

```python
from mirobot_sketch.live_progress import FollowTrack, read_progress, to_local_path  # noqa: E402

FOLLOW_PERIOD_S = 0.05


class FollowPlayback(Playback):
    """진행 파일을 읽어 로봇의 명령 응답 위치를 따라감 (GUI '로봇으로 그리기')."""

    def __init__(self, traj, progress_path):
        super().__init__(traj, speed=1.0, loop=False)
        self.timer.cancel()
        self.progress_path = progress_path
        self.run_id, self.shown = None, 0
        self.track = FollowTrack(self.points, traj.get("cmd_feed_mm_min", []), traj.get("step_mm", 1.0))
        self.timer = self.create_timer(FOLLOW_PERIOD_S, self.follow_tick)

    def reload(self, path):
        with open(to_local_path(path), encoding="utf-8") as f:
            self.traj = json.load(f)
        self.points = self.traj["points"]
        self.track = FollowTrack(self.points, self.traj.get("cmd_feed_mm_min", []), self.traj.get("step_mm", 1.0))
        self.trail, self.shown = [], 0

    def follow_tick(self):
        prog = read_progress(self.progress_path)
        if prog and prog.get("run_id") != self.run_id and prog.get("trajectory"):
            self.run_id = prog["run_id"]
            self.reload(prog["trajectory"])
        idx = self.track.index(prog, time.time())
        while self.shown < idx:                     # 지나온 펜다운 구간만 자국으로
            a, b = self.points[self.shown], self.points[self.shown + 1]
            if a["pen_down"] and b["pen_down"]:
                self.trail.append((self.tip(a), self.tip(b)))
            self.shown += 1
        self.publish(self.points[idx], self.status_text(prog))

    @staticmethod
    def status_text(prog):
        if not prog:
            return "대기 중"
        st, a, n = prog.get("state"), prog.get("acked", 0), max(prog.get("total", 1), 1)
        return {"running": f"그리는 중 {100 * a // n}%", "stopped": "멈춤", "done": "완료",
                "homing": "호밍 대기"}.get(st, f"오류: {prog.get('message', '')}")
```

기존 `tick`에서 관절 상태와 마커를 보내는 부분을 `publish(self, p, text=None)` 메서드로 뽑는다. `text`가 있으면 종이 위쪽에 `Marker.TEXT_VIEW_FACING`(ns="status", 높이 0.012m)를 추가한다. 기존 `tick`은 `self.publish(p)`를 부른다. `main()`:

```python
    ap.add_argument("--follow", help="진행 파일 경로: 로봇 명령 응답을 실시간으로 따라감")
    ...
    node = FollowPlayback(traj, args.follow) if args.follow else Playback(traj, args.speed, args.loop)
```

`import time`을 추가한다. `sys.path`는 `run_rviz.sh`의 `PYTHONPATH`로 맞추므로 코드에서 바꾸지 않는다.

`packaging/mirobot_sketch.spec`의 datas에 추가한다.

```python
datas += [(str(ROOT / "mirobot_sketch" / "__init__.py"), "sim/mirobot_sketch"),
          (str(ROOT / "mirobot_sketch" / "live_progress.py"), "sim/mirobot_sketch")]
```

- [ ] **Step 4: 통과 확인** — `python -m unittest discover -s tests` → OK. WSL에서 문법을 확인한다: `wsl.exe -d Ubuntu-22.04 -- python3 -m py_compile /mnt/c/<사용자>/Desktop/Mirobot/sim/rviz_playback.py` → 출력 없음

- [ ] **Step 5: 커밋**

```bash
git add sim/rviz_playback.py sim/run_rviz.sh mirobot_sketch/rviz_launch.py packaging/mirobot_sketch.spec tests/test_rviz_launch.py
git commit -m "RViz follow mode: playback tracks the live progress file (status text, trail up to position)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: GUI 실행 창 (`draw_window.py`)과 메인 창 연결

**Files:**
- Create: `mirobot_sketch/draw_window.py`
- Modify: `mirobot_sketch/gui.py`
- Test: `tests/test_gui_smoke.py`

**Interfaces:**
- Consumes: `DrawJob`, `de.DRAW_STEPS`, `app.ui()`, `app.agent_busy()`, `rviz_launch`
- Produces:
  - `DrawWindow(app, session, cfg, font)`: 창(`CTkToplevel`)
    - 위젯: `.steps`(단계 id → 라벨), `.start_btn`, `.check_var`, `.air_var`, `.pending_var`, `.virtual_var`, `.speed_var`, `.stop_btn`, `.progress`, `.job`
    - 메서드: `.begin()`(①~④ 시작, [연결 시작] 버튼), `.on_start()`(④ 시작), `.on_stop()`, `.close()`
  - `SketchApp.open_draw_window()`, `SketchApp.drawing` (bool: 실행 중 잠금)

- [ ] **Step 1: 실패하는 스모크 테스트**

```python
    def test_draw_window_virtual_run(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d, \
                    mock.patch("mirobot_sketch.draw_executor.paths.runs_dir", lambda: Path(d) / "runs"), \
                    mock.patch("mirobot_sketch.draw_executor.paths.output_dir", lambda: Path(d)):
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                app.session.update_params({"box_mm": 60, "epsilon_px": 4.0})
                app._schedule_recompute(0)
                self.assertTrue(pump(root, app, lambda: app._workers == 0 and app.result["params"]["box_mm"] == 60))
                w = app.open_draw_window(launch_rviz=lambda *a, **k: None)
                w.virtual_var.set(True)
                w.speed_var.set("500×")
                w.begin()
                self.assertTrue(pump(root, app, lambda: w.job.state == "confirm", timeout=60))
                self.assertEqual(str(w.start_btn.cget("state")), "disabled")      # 체크 전에는 꺼짐
                w.check_var.set(True)
                w._on_check()
                self.assertEqual(str(w.start_btn.cget("state")), "normal")
                w.on_start()
                self.assertTrue(app.drawing)                                        # 실행 중 잠금
                self.assertTrue(pump(root, app, lambda: w.finished is not None, timeout=120))
                self.assertEqual(w.finished["result"]["result"], "completed")
                self.assertFalse(app.drawing)
                self.assertTrue(list((Path(d) / "runs").glob("run-*.json")))
                w.close()
        finally:
            app._on_close()

    def test_closing_app_while_drawing_stops_the_job(self):
        root, app = make_app()
        with tempfile.TemporaryDirectory() as d, \
                mock.patch("mirobot_sketch.draw_executor.paths.runs_dir", lambda: Path(d) / "runs"), \
                mock.patch("mirobot_sketch.draw_executor.paths.output_dir", lambda: Path(d)), \
                mock.patch("mirobot_sketch.gui.messagebox.askyesno", return_value=True):
            p = Path(d) / "line.png"
            cv2.imwrite(str(p), golden.synthetic_images()["line"])
            app.load_image(p)
            self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
            w = app.open_draw_window(launch_rviz=lambda *a, **k: None)
            w.virtual_var.set(True)
            w.speed_var.set("1×")
            w.begin()
            self.assertTrue(pump(root, app, lambda: w.job.state == "confirm", timeout=60))
            w.check_var.set(True)
            w._on_check()
            w.on_start()
            self.assertTrue(pump(root, app, lambda: w.job.state == "drawing", timeout=10))
            job = w.job
            app._on_close()                                   # 앱 종료: 묻고(예) 멈춘 뒤 닫음
            job.join(5)
            self.assertEqual(job.state, "done")
            self.assertEqual(job.link.state, "closed")        # 포트(가상) 닫힘
```

(`test_gui_smoke.py` 상단에 `from unittest import mock`을 추가한다.)

- [ ] **Step 2: 실패 확인** — `cd tests && python -m unittest test_gui_smoke -v` → `AttributeError: open_draw_window`

- [ ] **Step 3: `draw_window.py` 구현**

```python
# mirobot_sketch/draw_window.py
"""
'로봇으로 그리기' 창 — 단계 표시줄(①~⑥), 지금 할 일, 최종 확인, 진행 막대, 멈춤
===================================================================================
실행은 DrawJob(작업 스레드)이 하고, 이 창은 events를 app.ui()로 받아 표시만 합니다.
"""

import customtkinter as ctk

from . import draw_executor as de
from .draw_job import DrawJob

OK_C, BAD_C, ACT_C, IDLE_C = "#16a34a", "#dc2626", ("#2563eb", "#3b82f6"), ("#9aa3b2", "#6b7280")
SPEEDS = ["1×", "5×", "20×", "50×", "500×"]


def _fmt(sec):
    sec = int(max(0, sec))
    return f"{sec // 60}:{sec % 60:02d}"


class DrawWindow(ctk.CTkToplevel):
    def __init__(self, app, session, cfg, font, launch_rviz=None):
        super().__init__(app.root)
        self.app, self.session, self.cfg, self.font = app, session, cfg, font
        self.launch_rviz, self.job, self.finished = launch_rviz, None, None
        self.title("로봇으로 그리기")
        self.geometry("760x520")
        self.protocol("WM_DELETE_WINDOW", self.close)
        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.pack(fill="x", padx=16, pady=(14, 4))
        self.steps = {}
        for k, (sid, name, who) in enumerate(de.DRAW_STEPS):
            if k:
                ctk.CTkLabel(bar, text="─", text_color=IDLE_C).pack(side="left", padx=2)
            lb = ctk.CTkLabel(bar, text=f"○ {'①②③④⑤⑥'[k]} {name}", font=font(12), text_color=IDLE_C)
            lb.pack(side="left")
            self.steps[sid] = lb
        self.todo = ctk.CTkLabel(self, text="연결 방식을 고르고 [연결 시작]을 누르세요", font=font(13, "bold"),
                                 anchor="w", justify="left", wraplength=720)
        self.todo.pack(fill="x", padx=16, pady=(8, 2))
        self.detail = ctk.CTkLabel(self, text="", font=font(12), anchor="w", justify="left", wraplength=720)
        self.detail.pack(fill="x", padx=16)

        opts = ctk.CTkFrame(self, fg_color="transparent")
        opts.pack(fill="x", padx=16, pady=8)
        self.virtual_var = ctk.BooleanVar(value=True)
        ctk.CTkRadioButton(opts, text=f"로봇 ({cfg['port']})", variable=self.virtual_var, value=False,
                           font=font(12)).grid(row=0, column=0, sticky="w")
        ctk.CTkRadioButton(opts, text="가상 시뮬레이션 (로봇 없이)", variable=self.virtual_var, value=True,
                           font=font(12)).grid(row=0, column=1, sticky="w", padx=12)
        self.speed_var = ctk.StringVar(value="20×")
        ctk.CTkOptionMenu(opts, values=SPEEDS, variable=self.speed_var, width=80, font=font(12)
                          ).grid(row=0, column=2, sticky="w")
        self.air_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(opts, text="공중 모드 (펜을 대지 않고 경로만)", variable=self.air_var, font=font(12)
                        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=4)
        self.pending_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(opts, text="넓은 범위(±60mm) 허용", variable=self.pending_var, font=font(12)
                        ).grid(row=1, column=2, sticky="w", pady=4)
        self.check_var = ctk.BooleanVar(value=False)
        self.check_box = ctk.CTkCheckBox(opts, text="종이·펜·주변을 확인했습니다", variable=self.check_var,
                                         font=font(12, "bold"), command=self._on_check, state="disabled")
        self.check_box.grid(row=2, column=0, columnspan=3, sticky="w", pady=4)

        self.progress = ctk.CTkProgressBar(self, height=10)
        self.progress.pack(fill="x", padx=16, pady=(8, 2))
        self.progress.set(0)
        self.prog_text = ctk.CTkLabel(self, text="", font=font(12), anchor="w")
        self.prog_text.pack(fill="x", padx=16)

        btns = ctk.CTkFrame(self, fg_color="transparent")
        btns.pack(fill="x", padx=16, pady=12, side="bottom")
        self.begin_btn = ctk.CTkButton(btns, text="연결 시작", command=self.begin, font=font(13))
        self.begin_btn.pack(side="left")
        self.rviz_btn = ctk.CTkButton(btns, text="RViz 3D로 따라보기", command=self._open_rviz, font=font(12),
                                      fg_color="transparent", border_width=1, state="disabled")
        self.rviz_btn.pack(side="left", padx=8)
        self.stop_btn = ctk.CTkButton(btns, text="■ 멈춤", command=self.on_stop, font=font(13, "bold"),
                                      fg_color=BAD_C, state="disabled")
        self.stop_btn.pack(side="right")
        self.start_btn = ctk.CTkButton(btns, text="시작", command=self.on_start, font=font(13, "bold"),
                                       state="disabled")
        self.start_btn.pack(side="right", padx=8)

    # ---------------------------------------------------------------- 버튼
    def begin(self):
        self.begin_btn.configure(state="disabled")
        speed = float(self.speed_var.get().rstrip("×"))
        self.job = DrawJob(self.session, self.cfg, lambda kind, **d: self.app.ui(self._event, kind, d),
                           launch_rviz=self.launch_rviz)
        self.job.start(virtual=bool(self.virtual_var.get()), virtual_speed=speed)

    def _on_check(self):
        ready = self.job is not None and self.job.state == "confirm" and self.check_var.get()
        self.start_btn.configure(state="normal" if ready else "disabled")

    def on_start(self):
        self.job.confirm(air=self.air_var.get(), pending=self.pending_var.get(), checked=self.check_var.get())
        self.start_btn.configure(state="disabled")
        self.check_box.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.app.set_drawing(True)

    def on_stop(self):
        if self.job:
            self.job.stop()

    def _open_rviz(self):
        if self.job and self.job.progress_path:
            from .rviz_launch import launch
            try:
                (self.launch_rviz or launch)(self.job._traj_path, follow=self.job.progress_path)
            except Exception as e:
                self.detail.configure(text=f"RViz를 열 수 없습니다: {e}")

    def close(self):
        """그리는 중이면 묻고 멈춘 뒤 닫음."""
        if self.job and self.job.state in ("connect", "confirm", "drawing", "preflight", "start"):
            from tkinter import messagebox
            if not messagebox.askyesno("로봇으로 그리기", "진행 중입니다. 멈추고 닫을까요?", parent=self):
                return False
            self.job.stop()
            self.job.join(10)
        self.app.set_drawing(False)
        self.destroy()
        return True

    # ---------------------------------------------------------------- 작업 이벤트 (메인 스레드)
    def _event(self, kind, d):
        if not self.winfo_exists():
            return
        if kind == "step":
            k = [s[0] for s in de.DRAW_STEPS].index(d["id"])
            name = de.DRAW_STEPS[k][1]
            mark, color = {"active": ("●", ACT_C), "done": ("✓", OK_C), "failed": ("✕", BAD_C)}[d["status"]]
            self.steps[d["id"]].configure(text=f"{mark} {'①②③④⑤⑥'[k]} {name}", text_color=color)
            if d["status"] == "active":
                self.todo.configure(text=d["message"])
            elif d["status"] == "failed":
                self.todo.configure(text=f"{name}: {d['message']}")
                self.detail.configure(text=d.get("hint", ""))
            if d["id"] == "confirm" and d["status"] == "active":
                s = self.job.summary
                self.detail.configure(text=f"획 {s['stroke_count']} · 명령 {s['command_count']:,}줄 · "
                                           f"예상 {s['estimated_s'] / 60:.1f}분 · {s['drawing_mm'][0]:.0f}×"
                                           f"{s['drawing_mm'][1]:.0f}mm · {s['port']}")
                self.check_box.configure(state="normal")
                self._on_check()
        elif kind == "progress":
            self.progress.set(d["acked"] / max(d["total"], 1))
            self.prog_text.configure(text=f"명령 {d['acked']:,}/{d['total']:,} ({100 * d['acked'] // d['total']}%) · "
                                          f"획 {d['stroke']}/{d['strokes']} · 경과 {_fmt(d['elapsed_s'])} · "
                                          f"남은 시간 약 {_fmt(d['remaining_s'])}")
            self.app.show_draw_progress(d["acked"], d["total"])
        elif kind == "rviz":
            self.rviz_btn.configure(state="normal" if d["ok"] else "disabled")
            if not d["ok"]:
                self.detail.configure(text=f"RViz 없이 진행합니다: {d['message']}")
        elif kind == "finished":
            self.finished = d
            self.stop_btn.configure(state="disabled")
            r = d["result"]["result"]
            self.todo.configure(text={"completed": "완료했습니다", "stopped_by_user": "멈췄습니다",
                                      "cancelled": "취소했습니다"}.get(r, f"끝: {r}"))
            if d.get("record_path"):
                self.detail.configure(text=f"실행 기록: {d['record_path']}")
            self.app.set_drawing(False)
```

- `_open_rviz`가 쓰는 `self.job._traj_path`는 `DrawJob._write_trajectory`에서 `self._traj_path = path`로 저장한다(Task 5 코드에 한 줄 추가).
- `speed_var` 값 `"500×"`는 테스트용 빠른 배속이다. GUI 목록(`SPEEDS`)에도 두어 발표 전 빠른 확인에 쓴다.

- [ ] **Step 4: `gui.py` 연결**

"③ 실행" 카드의 시뮬레이션 버튼 아래:

```python
        self.draw_btn = ctk.CTkButton(c3, text="로봇으로 그리기", command=self.open_draw_window, font=font(13, "bold"),
                                      height=38, fg_color="#16a34a", hover_color="#15803d")
        self.draw_btn.pack(fill="x", padx=14, pady=3)
```

`SketchApp`의 메서드:

```python
    def open_draw_window(self, launch_rviz=None):
        if not self.result:
            messagebox.showwarning("알림", "먼저 이미지를 처리하세요.")
            return None
        if getattr(self, "draw_window", None) is not None and self.draw_window.winfo_exists():
            self.draw_window.focus()
            return self.draw_window
        from .draw_window import DrawWindow
        self.draw_window = DrawWindow(self, self.session, self.cfg, font, launch_rviz=launch_rviz)
        return self.draw_window

    def set_drawing(self, on):
        """실행 중에는 조절 칸·편집·재계산·에이전트 도구를 잠금 (그리는 획이 바뀌지 않게)."""
        self.drawing = on
        self.session.drawing_lock = on
        self.agent_busy(self._agent_busy)        # agent_busy가 busy or self.drawing으로 잠금을 적용
        self.draw_btn.configure(state="disabled" if on else "normal")

    def show_draw_progress(self, acked, total):
        self.st_time.value.configure(text=f"{100 * acked // max(total, 1)}% 진행")
```

- `__init__`에 `self.drawing = False`와 `self.draw_window = None`을 넣는다.
- `agent_busy(busy)`는 `self._agent_busy = busy`를 저장하되, 화면에 적용하는 잠금 값은 `locked = busy or self.drawing`으로 바꾼다(에이전트가 끝나도 그리는 중이면 잠금 유지). 조절 칸, 버튼, 종류·상세도 선택 모두 `locked`를 쓴다.
- 에이전트 도구도 막는다: `AgentToolbox.call` 맨 앞에서 `getattr(self.session, "drawing_lock", False)`가 참이면 `SessionError("로봇이 그리는 중이라 설정·편집을 바꿀 수 없습니다")`를 오류 부분으로 돌려준다. 단 `get_state`, `view`, `list_strokes`, `get_stroke`는 읽기 전용이라 허용한다. 세션 `__init__`에 `self.drawing_lock = False`를 넣는다. 테스트: `tests/test_agent.py`에 `drawing_lock = True`일 때 `set_params`는 오류이고 `get_state`는 되는지 확인하는 테스트를 Step 1에서 함께 추가한다.
- `_on_close` 맨 앞에 넣는다.

```python
        if getattr(self, "draw_window", None) is not None and self.draw_window.winfo_exists():
            if not self.draw_window.close():
                return
```

`draw_window.py`는 `from tkinter import messagebox`를 모듈 맨 위에서 가져온다(`close()` 안의 지역 import는 쓰지 않음). 스모크 테스트의 패치 대상은 `mirobot_sketch.draw_window.messagebox.askyesno`다(Step 1 테스트 코드의 `mirobot_sketch.gui.messagebox.askyesno`를 이 경로로 적는다).

- [ ] **Step 5: 통과 확인** — `python -m unittest discover -s tests && python -m pyflakes mirobot_sketch tests` → OK

- [ ] **Step 6: 커밋**

```bash
git add mirobot_sketch/draw_window.py mirobot_sketch/gui.py mirobot_sketch/draw_job.py mirobot_sketch/session.py mirobot_sketch/agent/tools.py tests/test_gui_smoke.py
git commit -m "GUI draw window: step bar, confirm check, progress, stop; locks editing while drawing; close stops safely

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: 이 PC에서 직접 확인과 기록

**Files:**
- Create: `LOG/2026-09-26-live-draw.md`
- Modify: `LOG/README.md`, `README.md`

- [ ] **Step 1: 명령줄 가상 시뮬레이션**

Run: `python -m mirobot_sketch.draw_executor trajectories/orientation-test-F.json --execute --virtual --virtual-speed 20` (확인 질문에 `yes`)
Expected: 단계 출력, "결과: {'result': 'completed' …}", `runs/run-*.json`에 `"virtual": true`

- [ ] **Step 2: GUI + RViz 따라가기 (가상 시뮬레이션 20×)**

스크래치 스크립트로 앱을 띄워 illust1을 처리한다. `open_draw_window()` → 가상 시뮬레이션 20× → 확인 → 시작 순서로 진행한다. 실행하는 동안 다음을 확인한다.
- `wsl.exe -d Ubuntu-22.04 -- bash -c "ps -eo args | grep -E '^(rviz2|python3 .*rviz_playback.*--follow)'"`로 따라가기 프로세스가 떠 있는지 본다.
- 진행 파일의 `acked`가 늘어나는지 5초 간격으로 3번 읽는다.
- 도중에 [멈춤]을 누르면 진행 파일이 `state: stopped`가 되고, RViz가 서는지 본다(`status_text` "멈춤").
- 사용자가 PC를 쓰는 중이면 화면 캡처는 하지 않는다.

- [ ] **Step 3: 기록**

`LOG/2026-09-26-live-draw.md`에 적는다(문제 → 해결 → 확인 → 로봇 연결 후 할 일).
- **확인한 수치:** 명령 수, 예상 시간과 실제 경과(가상), 따라가기 지연 느낌
- **로봇 연결 후 할 일:**
  - 공중 모드 → 실제 드로잉 순서로 시험
  - 화면이 실물보다 앞서는 정도를 재서 `display_lag_s` 보정 값을 정함
  - B안(실제 좌표 읽기) 검토

`LOG/README.md`에 링크를 추가한다. `README.md`의 GUI 사용법에 "로봇으로 그리기(가상 시뮬레이션 포함)와 RViz 따라가기"를 두세 줄로 적는다.

- [ ] **Step 4: 최종 확인과 커밋**

```bash
python -m unittest discover -s tests && python -m pyflakes mirobot_sketch tests
git add LOG/2026-09-26-live-draw.md LOG/README.md README.md
git commit -m "LOG and README: GUI draw-to-robot with live RViz follow

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
