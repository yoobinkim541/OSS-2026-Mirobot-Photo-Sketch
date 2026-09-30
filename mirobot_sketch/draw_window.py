"""
'로봇으로 그리기' 창 — 단계 표시줄(①~⑥), 지금 할 일, 최종 확인, 진행 막대, 멈춤
===================================================================================
실행은 DrawJob(작업 스레드)이 하고, 이 창은 events를 app.ui()로 받아 표시만 합니다.
"""

from tkinter import messagebox, simpledialog

import customtkinter as ctk

from . import draw_executor as de
from .draw_job import DrawJob

OK_C, BAD_C, ACT_C, IDLE_C = "#16a34a", "#dc2626", ("#2563eb", "#3b82f6"), ("#9aa3b2", "#6b7280")
SPEEDS = ["1×", "5×", "20×", "50×", "500×"]
NUMS = "①②③④⑤⑥"
ACTIVE_STATES = ("preflight", "connect", "start", "confirm", "drawing")


def _fmt(sec):
    sec = int(max(0, sec))
    return f"{sec // 60}:{sec % 60:02d}"


class DrawWindow(ctk.CTkToplevel):
    def __init__(self, app, session, cfg, font, launch_rviz=None):
        super().__init__(app.root)
        self.app, self.session, self.cfg, self.font = app, session, cfg, font
        self.launch_rviz, self.job, self.finished = launch_rviz, None, None
        self.failure_message = ""
        # 닫을 때 작업이 끝나길 기다리는 시간: 로봇이 명령 응답을 기다리는 최대 시간 + 여유
        self.close_timeout = float(cfg.get("ack_timeout_s", 15)) + 5
        self.title("로봇으로 그리기")
        self.geometry("780x520")
        self.protocol("WM_DELETE_WINDOW", self.close)

        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.pack(fill="x", padx=16, pady=(14, 4))
        self.steps = {}
        for k, (sid, name, _who) in enumerate(de.DRAW_STEPS):
            if k:
                ctk.CTkLabel(bar, text="─", text_color=IDLE_C).pack(side="left", padx=2)
            lb = ctk.CTkLabel(bar, text=f"○ {NUMS[k]} {name}", font=font(12), text_color=IDLE_C)
            lb.pack(side="left")
            self.steps[sid] = lb
        self.todo = ctk.CTkLabel(self, text="연결 방식을 고르고 [연결 시작]을 누르세요", font=font(14, "bold"),
                                 anchor="w", justify="left", wraplength=740)
        self.todo.pack(fill="x", padx=16, pady=(10, 2))
        self.detail = ctk.CTkLabel(self, text="", font=font(12), anchor="w", justify="left", wraplength=740)
        self.detail.pack(fill="x", padx=16)

        opts = ctk.CTkFrame(self, fg_color="transparent")
        opts.pack(fill="x", padx=16, pady=8)
        self.virtual_var = ctk.BooleanVar(value=True)
        ctk.CTkRadioButton(opts, text=f"로봇 ({cfg['port']})", variable=self.virtual_var, value=False,
                           font=font(12), command=self._sync_speed_menu).grid(row=0, column=0, sticky="w")
        ctk.CTkRadioButton(opts, text="가상 시뮬레이션 (로봇 없이)", variable=self.virtual_var, value=True,
                           font=font(12), command=self._sync_speed_menu).grid(row=0, column=1, sticky="w", padx=12)
        self.speed_var = ctk.StringVar(value="20×")
        # 배속은 가상 시뮬레이션에만 적용되므로 로봇을 고르면 숨김 (실제 로봇은 항상 실제 속도)
        self.speed_menu = ctk.CTkOptionMenu(opts, values=SPEEDS, variable=self.speed_var, width=80, font=font(12))
        self.speed_menu.grid(row=0, column=2, sticky="w")
        self.air_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(opts, text="공중 모드 (펜을 대지 않고 경로만)", variable=self.air_var, font=font(12)
                        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=4)
        self.pending_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(opts, text="넓은 범위(공중 확인용)", variable=self.pending_var, font=font(12)
                        ).grid(row=1, column=2, sticky="w", pady=4)
        self.return_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(opts, text="끝나면 펜을 처음 위치(종이 중심)로 되돌리기", variable=self.return_var, font=font(12)
                        ).grid(row=4, column=0, columnspan=3, sticky="w", pady=4)
        self.clear_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(opts, text="그림이 끝나면 사진·작업 내용 비우기 (새 그림을 바로 시작)", variable=self.clear_var,
                        font=font(12)).grid(row=5, column=0, columnspan=3, sticky="w", pady=4)
        self.recalibrate_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(opts, text="종이·펜 위치 변경: 다시 보정", variable=self.recalibrate_var,
                        font=font(12)).grid(row=2, column=0, columnspan=3, sticky="w", pady=4)
        self.check_var = ctk.BooleanVar(value=False)
        self.check_box = ctk.CTkCheckBox(opts, text="펜 끝이 종이 가운데에 닿아 있고 주변을 확인했습니다", variable=self.check_var,
                                         font=font(12, "bold"), command=self._on_check, state="disabled")
        self.check_box.grid(row=3, column=0, columnspan=3, sticky="w", pady=4)

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
                                      fg_color=BAD_C, hover_color="#b91c1c", state="disabled")
        self.stop_btn.pack(side="right")
        self.start_btn = ctk.CTkButton(btns, text="시작", command=self.on_start, font=font(13, "bold"),
                                       state="disabled")
        self.start_btn.pack(side="right", padx=8)

    def _sync_speed_menu(self):
        if self.virtual_var.get():
            self.speed_menu.grid()
        else:
            self.speed_menu.grid_remove()

    # ---------------------------------------------------------------- 버튼
    def begin(self):
        """①~④: 사전 검사 → 연결·호밍 → 시작 위치 → 최종 확인에서 멈춰 기다림."""
        if not self.session.result:
            messagebox.showwarning("알림", "먼저 이미지를 열어 처리해 주세요.", parent=self)
            return
        self.finished = None
        self.failure_message = ""
        self.check_var.set(False)
        self.check_box.configure(state="disabled")
        self.start_btn.configure(state="disabled")
        self.rviz_btn.configure(state="disabled")
        self.rviz_note = ""
        self.progress.set(0)
        self.prog_text.configure(text="")
        self.detail.configure(text="")
        for k, (sid, name, _who) in enumerate(de.DRAW_STEPS):
            self.steps[sid].configure(text=f"○ {NUMS[k]} {name}", text_color=IDLE_C)
        self.begin_btn.configure(state="disabled")
        self.app.set_drawing(True)          # ①부터 잠금: 호밍 중 편집·이미지 열기로 그릴 그림이 바뀌지 않게
        speed = float(self.speed_var.get().rstrip("×"))
        self.job = DrawJob(self.session, self.cfg, lambda kind, **d: self.app.ui(self._event, kind, d),
                           launch_rviz=self.launch_rviz)
        self.stop_btn.configure(state="normal", text="취소")
        self.job.start(virtual=bool(self.virtual_var.get()), virtual_speed=speed,
                       pending=bool(self.pending_var.get()),
                       recalibrate=bool(self.recalibrate_var.get()),
                       return_to_origin=bool(self.return_var.get()))

    def _on_check(self):
        ready = self.job is not None and self.job.state == "confirm" and self.check_var.get()
        self.start_btn.configure(state="normal" if ready else "disabled")

    def on_start(self):
        self.job.confirm(air=self.air_var.get(), pending=self.pending_var.get(), checked=self.check_var.get())
        self.start_btn.configure(state="disabled")
        self.check_box.configure(state="disabled")
        self.stop_btn.configure(state="normal", text="■ 멈춤")

    def on_stop(self):
        if self.job:
            self.job.stop()

    def _open_rviz(self):
        if self.job is None or self.job._traj_path is None:
            return
        launch = self.launch_rviz
        if launch is None:
            from .rviz_launch import launch
        try:
            launch(self.job._traj_path, follow=self.job.progress_path)
        except Exception as e:  # RvizUnavailable 포함
            self.detail.configure(text=f"RViz를 열 수 없습니다: {e}")

    def close(self):
        """진행 중이면 묻고, 멈춘 뒤(포트가 닫힐 때까지) 닫음. 닫았으면 True.
        작업이 끝나지 않으면(로봇 응답 대기 등) 창과 잠금을 그대로 두고 알림 — 포트를 붙잡은 스레드를 남기지 않게."""
        if self.job and (self.job.state in ACTIVE_STATES or self.job.is_alive()):
            if not messagebox.askyesno("로봇으로 그리기", "진행 중입니다. 멈추고 닫을까요?", parent=self):
                return False
            self.job.stop()
            self.job.join(self.close_timeout)
            if self.job.is_alive():
                messagebox.showwarning("로봇으로 그리기", "로봇 응답을 기다리는 중이라 아직 닫을 수 없습니다. "
                                                   "잠시 뒤 다시 닫아 주세요.", parent=self)
                return False
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
            self.steps[d["id"]].configure(text=f"{mark} {NUMS[k]} {name}", text_color=color)
            if d["status"] == "active" and d["message"]:
                self.todo.configure(text=d["message"])
            elif d["status"] == "failed":
                self.failure_message = f"{name}: {d['message']}"
                self.todo.configure(text=self.failure_message)
                self.detail.configure(text=d.get("hint", ""))
            if d["id"] == "confirm" and d["status"] == "active":
                s = self.job.summary
                self.detail.configure(text=f"획 {s['stroke_count']} · 명령 {s['command_count']:,}줄 · "
                                           f"예상 {s['estimated_s'] / 60:.1f}분 · {s['drawing_mm'][0]:.0f}×"
                                           f"{s['drawing_mm'][1]:.0f}mm · {s['port']}"
                                           + ("" if s["virtual"] or s["plane_verified"] else
                                              " · 종이 보정 없음(설정에 저장된 값 사용)")
                                           + ("" if s["virtual"] or s.get("start_center_shift_mm", 0) < 0.05 else
                                              f" · 시작 자세를 종이 중심으로 사용(설정 파일 값과 {s['start_center_shift_mm']:g} mm 차이)"))
                self.check_box.configure(state="normal")
                self._on_check()
        elif kind == "robot_status":
            self.app.set_robot_controller_status(d.get("state"), recent=d.get("recent", False))
        elif kind == "calibration_status":
            self.todo.configure(text="종이 캘리브레이션")
            self.detail.configure(text=d.get("message", ""))
        elif kind == "calibration_input":
            answer = simpledialog.askstring("종이 캘리브레이션", d.get("prompt", "응답을 입력하세요"),
                                            parent=self)
            if self.job:
                self.job.answer_calibration_input("q" if answer is None else answer)
        elif kind == "progress":
            self.progress.set(d["acked"] / max(d["total"], 1))
            self.prog_text.configure(text=f"명령 {d['acked']:,}/{d['total']:,} ({100 * d['acked'] // d['total']}%) · "
                                          f"획 {d['stroke']}/{d['strokes']} · 경과 {_fmt(d['elapsed_s'])} · "
                                          f"남은 시간 약 {_fmt(d['remaining_s'])}")
            self.app.show_draw_progress(d["acked"], d["total"])
        elif kind == "rviz":
            self.rviz_btn.configure(state="normal" if d["ok"] else "disabled")
            if not d["ok"]:
                self.rviz_note = f"RViz 없이 진행합니다: {d['message']}"
                self.detail.configure(text=self.rviz_note)
        elif kind == "finished":
            self.finished = d
            self.stop_btn.configure(state="disabled")
            r = d["result"]["result"]
            returned = d["result"].get("returned_to_origin")      # True 돌아옴 / False 못 돌아옴 / None 해당 없음
            done_text = "완료했습니다" + {True: ". 펜이 처음 위치(종이 중심)로 돌아왔습니다.",
                                        False: " — 다만 펜이 처음 위치로 돌아가지 못했습니다.",
                                        None: ""}[returned]
            self.todo.configure(text=self.failure_message if r in ("not_started", "error") and self.failure_message
                                else {"completed": done_text, "stopped_by_user": "멈췄습니다 (자동 복구 없음)",
                                      "cancelled": "취소했습니다"}.get(r, f"끝: {r}"))
            note = getattr(self, "rviz_note", "")
            detail = ""
            if d.get("record_path"):
                detail = f"실행 기록: {d['record_path']}" + (f"\n{note}" if note else "")
            if returned is False:
                detail += f"\n{d['result'].get('return_error', '')} 다음 그림 전에 호밍하거나 펜 위치를 확인하세요."
            self.start_btn.configure(state="disabled")
            self.check_box.configure(state="disabled")
            self.app.set_drawing(False)
            self.begin_btn.configure(state="normal")
            # 끝까지 그렸고 펜도 처음 자리로 돌아왔으면 사진·작업을 비워 새 그림을 바로 시작할 수 있게 (옵션)
            if r == "completed" and returned is not False and self.clear_var.get():
                self.app.reset_for_next_drawing()
                detail += "\n사진과 작업 내용을 비웠습니다. 종이를 바꾸고 새 사진을 열어 주세요."
            if detail:
                self.detail.configure(text=detail.strip())
