"""GUI 스모크: 창을 띄워 이미지 열기 → 값 변경 → 자동 재계산이 끝나는지 (화면이 없으면 건너뜀)."""

import gc
import sys
import tempfile
import time
import unittest
from unittest import mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402

import golden  # noqa: E402


def make_app():
    gc.collect()      # 앞선 테스트가 남긴 tk 객체를 메인 스레드에서 정리 (작업 스레드의 GC가 닫힌 창을 기다리며 멈추는 것을 막음)
    try:
        import customtkinter as ctk
        root = ctk.CTk()
    except Exception as e:   # 화면(DISPLAY) 없음, Tk 없음
        raise unittest.SkipTest(f"GUI 없음: {e}")
    from mirobot_sketch import gui
    root.withdraw()
    return root, gui.SketchApp(root)


def close_quietly(app):
    """이미 닫혔으면 아무것도 안 함."""
    try:
        if app.root.winfo_exists():
            app._on_close()
    except Exception:  # TclError: 이미 파괴됨
        pass


def pump(root, app, until, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        root.update()
        if until():
            return True
        time.sleep(0.02)
    return False


class GuiResponsivenessTest(unittest.TestCase):
    def test_window_keeps_updating_while_robot_simulation_runs_in_a_thread(self):
        """시뮬레이션(작은 numpy 연산의 긴 반복)이 다른 스레드에서 도는 동안 화면 스레드가 오래 멈추지 않아야 한다."""
        import threading

        import numpy as np

        from mirobot_sketch import mirobot_sim as ms

        root, app = make_app()
        try:
            targets = [(np.array([198.668, 0.0, 230.477]), "start", False, 0.0)]
            for i in range(1, 60):     # 1 mm 간격 IK 약 2400회: 화면을 붙잡기에 충분한 CPU 작업
                targets.append((np.array([198.668, 0.0 + 2.0 * (i % 2) * 20.0, 230.477 + 40.0 * (i % 3)]),
                                f"move {i}", True, 300.0))
            done = threading.Event()
            worker = threading.Thread(target=lambda: (ms.simulate(targets), done.set()), daemon=True)
            gaps, last = [], time.perf_counter()
            worker.start()
            t0 = time.perf_counter()
            while not done.is_set() and time.perf_counter() - t0 < 120:
                root.update()
                now = time.perf_counter()
                gaps.append(now - last)
                last = now
                time.sleep(0.005)
            worker.join(5)
            self.assertTrue(done.is_set(), "시뮬레이션이 끝나지 않음")
            self.assertGreater(len(gaps), 10)
            self.assertLess(max(gaps), 1.0, f"화면이 {max(gaps):.1f}초 멈춤 ({len(gaps)}회 갱신)")
        finally:
            close_quietly(app)

    def test_app_shortens_the_thread_switch_interval(self):
        from mirobot_sketch import gui

        before = sys.getswitchinterval()
        root, app = make_app()
        try:
            self.assertLessEqual(sys.getswitchinterval(), gui.GIL_SWITCH_INTERVAL_S + 1e-9)
        finally:
            close_quietly(app)
            sys.setswitchinterval(before)


class GuiNextDrawingTest(unittest.TestCase):
    """그림이 끝나면 사진·작업을 비우고 새 그림을 바로 시작할 수 있는 처음 상태로."""

    def run_virtual_drawing(self, root, app, d, clear=True, air=True, stop=False):
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
        w.air_var.set(air)
        w.clear_var.set(clear)
        w.begin()
        self.assertTrue(pump(root, app, lambda: w.job.state == "confirm", timeout=90))
        if stop:
            w.on_stop()
        else:
            w.check_var.set(True)
            w._on_check()
            w.on_start()
        self.assertTrue(pump(root, app, lambda: w.finished is not None, timeout=120))
        return w, p

    def test_options_exist_and_default_to_on(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                cv2.imwrite(str(Path(d) / "line.png"), golden.synthetic_images()["line"])
                app.load_image(Path(d) / "line.png")
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                w = app.open_draw_window(launch_rviz=lambda *a, **k: None)
                self.assertTrue(w.return_var.get())
                self.assertTrue(w.clear_var.get())
                w.close()
        finally:
            close_quietly(app)

    def test_completed_drawing_clears_photo_and_work_and_a_new_photo_can_be_opened(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d, mock.patch("mirobot_sketch.draw_job.paths.output_dir",
                                                                 lambda: Path(d)), \
                    mock.patch("mirobot_sketch.draw_executor.paths.runs_dir", lambda: Path(d) / "runs"):
                w, path = self.run_virtual_drawing(root, app, d)
                self.assertEqual(w.finished["result"]["result"], "completed")
                self.assertIsNone(app.session.image_path)
                self.assertIsNone(app.session.result)
                self.assertIsNone(app.session.sim)
                self.assertEqual(app.session.edit_log, [])
                self.assertEqual(app.path_label.cget("text"), "선택된 파일 없음")
                self.assertEqual(str(app.traj_btn.cget("state")), "disabled")
                self.assertIsNone(app.view.image)                            # 큰 보기도 비움
                self.assertEqual(app.session.params["box_mm"], 60)          # 설정은 다음 그림에도 씀
                self.assertIn("새 사진", app.status.cget("text"))
                self.assertIn("비웠습니다", w.detail.cget("text"))
                self.assertFalse(app.drawing)
                self.assertEqual(str(app.open_btn.cget("state")), "normal")
                app.load_image(path)                                          # 새 그림을 바로 시작
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                self.assertGreater(app.result["timing"]["stroke_count"], 0)
                w.close()
        finally:
            close_quietly(app)

    def test_option_off_keeps_the_photo_and_work(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d, mock.patch("mirobot_sketch.draw_job.paths.output_dir",
                                                                 lambda: Path(d)), \
                    mock.patch("mirobot_sketch.draw_executor.paths.runs_dir", lambda: Path(d) / "runs"):
                w, path = self.run_virtual_drawing(root, app, d, clear=False)
                self.assertEqual(w.finished["result"]["result"], "completed")
                self.assertIsNotNone(app.session.result)
                self.assertEqual(app.path_label.cget("text"), "line.png")
                w.close()
        finally:
            close_quietly(app)

    def test_cancelled_drawing_keeps_the_photo_and_work(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d, mock.patch("mirobot_sketch.draw_job.paths.output_dir",
                                                                 lambda: Path(d)), \
                    mock.patch("mirobot_sketch.draw_executor.paths.runs_dir", lambda: Path(d) / "runs"):
                w, path = self.run_virtual_drawing(root, app, d, stop=True)
                self.assertEqual(w.finished["result"]["result"], "cancelled")
                self.assertIsNotNone(app.session.result)                      # 다시 시도할 수 있게 그대로
                self.assertEqual(app.path_label.cget("text"), "line.png")
                w.close()
        finally:
            close_quietly(app)

    def test_draw_window_asks_for_an_image_when_there_is_nothing_to_draw(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                cv2.imwrite(str(Path(d) / "line.png"), golden.synthetic_images()["line"])
                app.load_image(Path(d) / "line.png")
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                w = app.open_draw_window(launch_rviz=lambda *a, **k: None)
                app.reset_for_next_drawing()
                with mock.patch("mirobot_sketch.draw_window.messagebox.showwarning") as warn:
                    w.begin()
                warn.assert_called_once()
                self.assertIsNone(w.job)                                       # 작업을 시작하지 않음
                w.close()
        finally:
            close_quietly(app)


class GuiTextAndLayoutTest(unittest.TestCase):
    def test_rviz_buttons_are_folded_under_advanced(self):
        root, app = make_app()
        try:
            root.update()
            self.assertEqual(app.adv_frame.winfo_manager(), "")           # 처음에는 접혀 있음
            self.assertEqual(str(app.traj_btn.cget("state")), "disabled")  # 이미지 전에는 꺼짐 (접혀 있어도 상태는 유지)
            app._toggle_advanced()
            root.update()
            self.assertEqual(app.adv_frame.winfo_manager(), "pack")
            self.assertIn("▾", app.adv_btn.cget("text"))
            app._toggle_advanced()
            root.update()
            self.assertEqual(app.adv_frame.winfo_manager(), "")
        finally:
            close_quietly(app)

    def test_draw_window_hides_the_speed_menu_for_the_real_robot(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
            w = app.open_draw_window(launch_rviz=lambda *a, **k: None)     # 처리 결과가 없으면 경고창이 떠서 멈춤
            root.update()
            self.assertEqual(w.speed_menu.winfo_manager(), "grid")          # 가상 시뮬레이션(기본): 배속 선택 가능
            w.virtual_var.set(False)
            w._sync_speed_menu()
            root.update()
            self.assertEqual(w.speed_menu.winfo_manager(), "")              # 로봇: 항상 실제 속도라 숨김
            w.virtual_var.set(True)
            w._sync_speed_menu()
            root.update()
            self.assertEqual(w.speed_menu.winfo_manager(), "grid")
            w.close()
        finally:
            close_quietly(app)

    def test_no_stale_or_jargon_texts_in_the_screens(self):
        root_dir = Path(__file__).resolve().parent.parent / "mirobot_sketch"
        forbidden = ("처리 실행", "실행기 허용", "python robot/draw_executor.py", "1mm마다 역기구학")
        for name in ("gui.py", "draw_window.py", "stage_view.py"):
            text = (root_dir / name).read_text(encoding="utf-8")
            for bad in forbidden:
                self.assertNotIn(bad, text, f"{name}: {bad}")
        # GUI에는 명령줄 옵션 이름을 메시지로 노출하지 않음 (내보내기 완료창의 mirobot-draw 안내는 예외)
        gui_text = (root_dir / "gui.py").read_text(encoding="utf-8")
        self.assertNotIn("실행 시 --pending-limits", gui_text)


class GuiAgentUxTest(unittest.TestCase):
    def open_panel(self):
        root, app = make_app()
        app.toggle_agent()
        root.update()
        return root, app, app.agent_panel

    def test_thinking_indicator_shows_until_the_first_reply_and_counts_seconds(self):
        root, app, panel = self.open_panel()
        try:
            panel._show_thinking()
            self.assertIn("생각하는 중", panel._thinking.cget("text"))
            panel._thinking_since -= 5                                   # 5초 지난 것처럼
            panel._tick()
            self.assertRegex(panel._thinking.cget("text"), r"생각하는 중… [5-6]초")
            panel._handle({"type": "tool", "name": "get_state", "args": {}})   # 첫 응답이 오면 사라짐
            self.assertIsNone(panel._thinking)
            self.assertEqual(len(panel._current_tools), 1)
            panel._handle({"type": "done"})
        finally:
            close_quietly(app)

    def test_long_tool_chip_shows_hint_and_elapsed_time(self):
        root, app, panel = self.open_panel()
        try:
            panel.add_tool("simulate", {})
            chip = panel._current_tools[0][0]
            self.assertIn("10~30초", chip.cget("text"))
            panel._tool_started[chip] -= 12
            panel._tick()
            self.assertRegex(chip.cget("text"), r"… 1[2-3]초")
            panel.tool_done(True)
            self.assertRegex(chip.cget("text"), r"^✓ simulate .*1[2-3]초$")
        finally:
            close_quietly(app)

    def test_new_chat_clears_the_thinking_indicator(self):
        root, app, panel = self.open_panel()
        try:
            panel._show_thinking()
            panel.new_chat()
            self.assertIsNone(panel._thinking)
            panel._tick()                                                # 지워진 위젯을 건드려도 오류 없음
        finally:
            close_quietly(app)

    def test_screen_explains_why_settings_are_locked_while_the_agent_works(self):
        root, app, panel = self.open_panel()
        try:
            app.agent_busy(True)
            pump(root, app, lambda: "잠깁니다" in app.status.cget("text"), timeout=5)
            self.assertIn("잠깁니다", app.status.cget("text"))
            self.assertEqual(str(app.sim_btn.cget("state")), "disabled")
            app._set_status("x")
            app.agent_busy(False)
            self.assertEqual(str(app.sim_btn.cget("state")), "normal")
        finally:
            close_quietly(app)

    def test_empty_chat_mentions_the_robot_readiness_check(self):
        root, app, panel = self.open_panel()
        try:
            texts = [w.cget("text") for w in panel.chat.winfo_children()[0].winfo_children() if hasattr(w, "cget")
                     and isinstance(w.cget("text"), str)]
            self.assertTrue(any("로봇으로 그릴 준비" in t for t in texts), texts)
        finally:
            close_quietly(app)


class GuiSmokeTest(unittest.TestCase):
    def test_open_change_param_recomputes(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                n = app.result["timing"]["stroke_count"]
                for sid in ("source", "edges", "trace", "simplify", "edit", "paper"):
                    app.select_stage(sid)
                    root.update()
                app._on_param("epsilon_px", 4.0)
                self.assertTrue(pump(root, app, lambda: app._workers == 0 and
                                     app.result["params"]["epsilon_px"] == 4.0))
                self.assertEqual(app.result["timing"]["stroke_count"], n)   # 단순화는 획 수를 안 바꿈
        finally:
            app._on_close()

    def test_rviz_setup_window_shows_steps_and_installs(self):
        from mirobot_sketch import rviz_setup as rs
        from mirobot_sketch import rviz_setup_window as rsw

        def st(*states):
            return [rs.SetupStep(i, lab, s, hint="할 일 " + i if s != "ok" else "")
                    for (i, lab), s in zip((("wsl", "① WSL"), ("env", "② 환경"), ("verify", "③ 확인")), states)]

        fake = mock.Mock()
        fake.SetupError = rs.SetupError
        fake.check.return_value = st("ok", "missing", "blocked")

        def install(progress=None, should_cancel=None, **kw):
            progress("다운로드", 50, 100)
            return st("ok", "ok", "ok")
        fake.install.side_effect = install
        root, app = make_app()
        try:
            with mock.patch.object(rsw, "rs", fake):
                w = app.open_rviz_setup()
                self.assertTrue(pump(root, app, lambda: w.steps["env"].cget("text").startswith("✕")))
                self.assertTrue(w.steps["wsl"].cget("text").startswith("✓"))
                self.assertTrue(w.steps["verify"].cget("text").startswith("○"))
                self.assertIn("할 일 env", w.todo.cget("text"))
                self.assertEqual(w.install_btn.cget("text"), "설치")
                w.install()
                self.assertTrue(pump(root, app, lambda: w.steps["verify"].cget("text").startswith("✓")))
                fake.install.assert_called_once()
                self.assertIn("준비", w.todo.cget("text"))
                self.assertIs(app.open_rviz_setup(), w)                   # 이미 열려 있으면 그 창
                # 설치가 도는 동안 [다시 확인]·[설치]·[제거]를 눌러도 두 번째 작업이 시작되지 않아야 함
                import threading
                gate = threading.Event()
                fake.install.side_effect = lambda **kw: (gate.wait(10), st("ok", "ok", "ok"))[1]
                fake.check.return_value = st("ok", "missing", "blocked")
                w.busy = False
                w.install()
                checks = fake.check.call_count
                w.refresh()
                w.install()
                w.uninstall()
                root.update()
                self.assertEqual(fake.check.call_count, checks)
                self.assertEqual(fake.install.call_count, 2)
                fake.uninstall.assert_not_called()
                gate.set()
                self.assertTrue(pump(root, app, lambda: not w.busy))
                w.close()
        finally:
            app._on_close()

    def test_overlay_original_on_auto_cropped_photo(self):
        # 자동 구도로 자른 사진: "원본 겹치기"는 자른 작업 이미지를 겹쳐야 함 (전체 원본이면 크기가 달라 오류)
        import test_face_session as tfs
        from mirobot_sketch import session as session_mod
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d, \
                    mock.patch.object(session_mod.faces, "detect_faces", lambda img, **kw: [tfs.SMALL]):
                p = Path(d) / "photo.png"
                tfs.photo(p)
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                self.assertEqual(app.result["frame"]["kind"], "bust")
                app.view.alpha.set(0.5)
                for sid in ("edges", "face", "simplify", "edit"):
                    app.select_stage(sid)
                    root.update()
        finally:
            app._on_close()

    def test_edit_stage_shows_proposals_and_applies(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                s = app.session
                sid = next(i for i, e in s.table.items() if e["kind"] == "stroke")
                s.propose_edits([{"op": "delete", "ids": [sid]}])
                app.refresh_proposals()
                app.select_stage("edit")
                root.update()
                self.assertTrue(app.proposal_bar.winfo_manager())          # 제안이 있으면 바가 배치됨
                self.assertIn(sid, app.proposal_bar.chips)
                app.proposal_bar.apply_btn.invoke()                        # 적용은 작업 스레드에서
                self.assertTrue(pump(root, app, lambda: app._workers == 0 and s.table[sid]["kind"] == "candidate"))
                self.assertFalse(app.proposal_bar.winfo_manager())         # 적용하면 사라짐
        finally:
            app._on_close()

    def test_thousands_of_proposals_draw_fast_with_capped_labels(self):
        import numpy as np
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                s = app.session
                s.table = {i + 1: {"kind": "stroke", "reason": "",
                                   "poly": np.array([[10 + (i % 60) * 12, 10 + (i // 60) * 12],
                                                     [16 + (i % 60) * 12, 14 + (i // 60) * 12]], float)}
                           for i in range(3000)}
                s.next_id = 3001
                s.propose_edits([{"op": "delete", "ids": list(range(1, 200))}] +
                                [{"op": "delete", "ids": list(range(k, k + 200))} for k in range(200, 3000, 200)])
                app.refresh_proposals()
                t0 = time.time()
                app.select_stage("edit")
                app.view.canvas.draw()
                self.assertLess(time.time() - t0, 5.0)
                self.assertLessEqual(len(app.view.ax.texts), 450)
                self.assertLessEqual(len(app.view.ax.lines), 4)       # 제안마다 plot 하지 않음
        finally:
            app._on_close()


    def test_gui_minor_behaviours(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                s = app.session
                # 1) 뺀 표시는 번호를 새로 매기면 풀림
                sid = next(i for i, e in s.table.items() if e["kind"] == "stroke")
                s.propose_edits([{"op": "delete", "ids": [sid]}])
                app.refresh_proposals()
                app.proposal_bar._toggle(sid)
                app._on_param("canny_low", s.params["canny_low"] + 4)
                self.assertTrue(pump(root, app, lambda: app._workers == 0 and s.proposals == {}))
                sid = next(i for i, e in s.table.items() if e["kind"] == "stroke")
                s.propose_edits([{"op": "delete", "ids": [sid]}])
                app.refresh_proposals()
                self.assertEqual(app.proposal_bar.excluded, set())
                # 2) 적용은 작업 스레드에서 (화면이 멈추지 않게) 끝나면 반영
                app.proposal_bar.apply_btn.invoke()
                self.assertTrue(pump(root, app, lambda: s.table[sid]["kind"] == "candidate" and app._workers == 0))
                # 3) 에이전트 작업 중 프리셋을 바꾸면, 끝난 뒤 다시 계산
                app.agent_busy(True)
                app.detail_seg.set("낮음")
                app.apply_detail_preset()
                root.update()
                app.agent_busy(False)
                self.assertTrue(pump(root, app, lambda: app._workers == 0 and app.result["detail"] == "low"))
        finally:
            app._on_close()


    def _patched_dirs(self, d):
        return (mock.patch("mirobot_sketch.draw_executor.paths.runs_dir", lambda: Path(d) / "runs"),
                mock.patch("mirobot_sketch.draw_executor.paths.output_dir", lambda: Path(d)))

    def test_draw_window_virtual_run(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p1, p2 = self._patched_dirs(d)
                with p1, p2:
                    p = Path(d) / "line.png"
                    cv2.imwrite(str(p), golden.synthetic_images()["line"])
                    app.load_image(p)
                    self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                    app.session.update_params({"box_mm": 60, "epsilon_px": 4.0})
                    app._schedule_recompute(0)
                    self.assertTrue(pump(root, app, lambda: app._workers == 0 and
                                         app.result["params"]["box_mm"] == 60))
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
                    self.assertTrue(app.session.drawing_lock)
                    self.assertTrue(pump(root, app, lambda: w.finished is not None, timeout=120))
                    self.assertEqual(w.finished["result"]["result"], "completed")
                    self.assertFalse(app.drawing)
                    self.assertTrue(list((Path(d) / "runs").glob("run-*.json")))
                    w.close()
        finally:
            app._on_close()

    def test_draw_window_retries_after_start_position_failure(self):
        from mirobot_sketch import draw_executor as de

        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p1, p2 = self._patched_dirs(d)
                with p1, p2:
                    p = Path(d) / "line.png"
                    cv2.imwrite(str(p), golden.synthetic_images()["line"])
                    app.load_image(p)
                    self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                    app.session.update_params({"box_mm": 60, "epsilon_px": 4.0})
                    app._schedule_recompute(0)
                    self.assertTrue(pump(root, app, lambda: app._workers == 0 and
                                         app.result["params"]["box_mm"] == 60))
                    w = app.open_draw_window(launch_rviz=lambda *a, **k: None)
                    w.virtual_var.set(True)
                    with mock.patch.object(de, "check_start", side_effect=de.DrawError(
                            "start", "시작점이 8.0 mm 떨어져 있습니다", "종이 중심을 확인하세요")):
                        w.begin()
                        self.assertTrue(pump(root, app, lambda: w.finished is not None))
                    self.assertEqual(w.finished["result"]["result"], "not_started")
                    self.assertEqual(str(w.begin_btn.cget("state")), "normal")
                    self.assertIn("8.0 mm", w.todo.cget("text"))
                    w.check_var.set(True)
                    w.begin()
                    self.assertIsNone(w.finished)
                    self.assertFalse(w.check_var.get())
                    self.assertTrue(pump(root, app, lambda: w.job.state == "confirm"))
                    w.on_stop()
                    self.assertTrue(pump(root, app, lambda: w.finished is not None))
                    w.close()
        finally:
            close_quietly(app)

    def test_closing_app_while_drawing_stops_the_job(self):
        root, app = make_app()
        self.addCleanup(close_quietly, app)   # 실패해도 창을 닫아 다음 테스트에 안 번지게
        with tempfile.TemporaryDirectory() as d:
            p1, p2 = self._patched_dirs(d)
            with p1, p2, mock.patch("mirobot_sketch.draw_window.messagebox.askyesno", return_value=True):
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                app.session.update_params({"box_mm": 60})
                app._schedule_recompute(0)
                self.assertTrue(pump(root, app, lambda: app._workers == 0 and
                                     app.result["params"]["box_mm"] == 60))
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


    def test_lock_starts_at_begin_and_stuck_close_keeps_lock(self):
        root, app = make_app()
        self.addCleanup(close_quietly, app)
        with tempfile.TemporaryDirectory() as d:
            p1, p2 = self._patched_dirs(d)
            with p1, p2, mock.patch("mirobot_sketch.draw_window.messagebox.askyesno", return_value=True), \
                    mock.patch("mirobot_sketch.draw_window.messagebox.showwarning"):
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                app.session.update_params({"box_mm": 60})
                app._schedule_recompute(0)
                self.assertTrue(pump(root, app, lambda: app._workers == 0 and
                                     app.result["params"]["box_mm"] == 60))
                w = app.open_draw_window(launch_rviz=lambda *a, **k: None)
                w.begin()
                self.assertTrue(app.drawing)                          # ①부터 잠금 (호밍 중 편집 금지)
                self.assertEqual(str(app.open_btn.cget("state")), "disabled")
                self.assertEqual(str(app.traj_btn.cget("state")), "disabled")
                self.assertTrue(pump(root, app, lambda: w.job.state == "confirm", timeout=60))
                real_job = w.job

                class Stuck:                                          # 로봇 응답을 기다리느라 안 끝나는 작업
                    state = "drawing"

                    def stop(self):
                        pass

                    def join(self, timeout=None):
                        pass

                    def is_alive(self):
                        return True

                real_timeout = w.close_timeout
                w.job, w.close_timeout = Stuck(), 0.1
                self.assertFalse(w.close())                           # 안 닫고 잠금 유지
                self.assertTrue(w.winfo_exists())
                self.assertTrue(app.drawing)
                w.job, w.close_timeout = real_job, real_timeout   # 실제 작업은 멈출 시간을 줌 (느린 CI에서 0.1초는 모자람)
                self.assertTrue(w.close())
                self.assertFalse(app.drawing)


    def test_stage_strip_shows_numbers_results_and_compare(self):
        root, app = make_app()
        try:
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "line.png"
                cv2.imwrite(str(p), golden.synthetic_images()["line"])
                app.load_image(p)
                self.assertTrue(pump(root, app, lambda: app.result is not None and app._workers == 0))
                sm = app.session.stage_summaries()
                txt = app.strip.buttons["trace"].cget("text")
                self.assertTrue(txt.startswith("④ 뼈대·획"))
                self.assertIn(sm["trace"], txt)
                app.select_stage("dedupe")
                root.update()
                self.assertIn("→", app.view.subtitle.cget("text"))      # 설명 + 이전 대비 변화
                shown = app.view.image
                app.view.hold_compare(True)                             # 누르고 있는 동안 이전 단계
                self.assertIsNot(app.view.image, shown)
                self.assertTrue(app.view.title.cget("text").startswith("④"))   # ⑤의 이전 = ④
                app.view.hold_compare(False)
                self.assertIs(app.view.image, shown)
        finally:
            app._on_close()


if __name__ == "__main__":
    unittest.main()
