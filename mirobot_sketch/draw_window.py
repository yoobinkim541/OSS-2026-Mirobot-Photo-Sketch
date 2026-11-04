"""
'로봇으로 그리기' 창 — 단계 표시줄(①~⑥), 지금 할 일, 최종 확인, 진행 막대, 멈춤
===================================================================================
실행은 DrawJob(작업 스레드)이 하고, 이 창은 events를 app.ui()로 받아 표시만 합니다.
"""

from tkinter import messagebox, simpledialog

import cv2
import customtkinter as ctk
from . import camera, draw_executor as de
from .draw_job import DrawJob

from .theme import ACCENT as ACT_C, BAD as BAD_C, BORDER, CARD, MUTED as IDLE_C, OK as OK_C, SURFACE, TEXT
SPEEDS = ["1×", "5×", "20×", "50×", "500×"]
NUMS = "①②③④⑤⑥"
ACTIVE_STATES = ("preflight", "connect", "start", "confirm", "drawing")
JOG_STEPS = ["0.5", "2", "10"]          # 조그 한 번 이동량(mm)
JOG_REPEAT_MS = 150                      # 버튼을 누르고 있는 동안 다시 보내는 간격 (로봇이 바쁘면 작업 쪽에서 버림)
# 키보드 조그: 로봇 좌표 방향 (X 벽 쪽, Y 종이 왼쪽, Z 위). 종이 오른쪽 = 로봇 -Y (paper_x_to_robot_y_sign = -1)
JOG_KEYS = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0),
            "Left": (0, 1, 0), "Right": (0, -1, 0), "Up": (0, 0, 1), "Down": (0, 0, -1)}


def _fmt(sec):
    sec = int(max(0, sec))
    return f"{sec // 60}:{sec % 60:02d}"


class DrawWindow(ctk.CTkToplevel):
    start_pose_in_virtual = False     # 가상 시뮬레이션에서도 시작 자세 맞추기를 할지 (화면 시험용)

    def __init__(self, app, session, cfg, font, launch_rviz=None):
        super().__init__(app.root)
        self.app, self.session, self.cfg, self.font = app, session, cfg, font
        self.launch_rviz, self.job, self.finished = launch_rviz, None, None
        self.failure_message = ""
        # 닫을 때 작업이 끝나길 기다리는 시간: 로봇이 명령 응답을 기다리는 최대 시간 + 여유
        self.close_timeout = float(cfg.get("ack_timeout_s", 15)) + 5
        self.title("로봇으로 그리기")
        self.geometry("1120x680")
        self.minsize(980, 680)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)
        self.grid_rowconfigure(0, weight=1)
        body = ctk.CTkFrame(self, fg_color="transparent")
        body.grid(row=0, column=0, sticky="nsew")
        self._build_camera()
        ctk.CTkLabel(body, text="드로잉 컨트롤", font=font(22, "bold"), anchor="w").pack(
            fill="x", padx=20, pady=(18, 0))
        ctk.CTkLabel(body, text="연결부터 완료까지, 안전 절차를 한 단계씩 확인합니다.",
                     font=font(12), text_color=IDLE_C, anchor="w").pack(fill="x", padx=20, pady=(2, 8))
        self.protocol("WM_DELETE_WINDOW", self.close)

        bar = ctk.CTkFrame(body, fg_color=CARD, border_width=1, border_color=BORDER)
        bar.pack(fill="x", padx=16, pady=(4, 8), ipady=8)
        self.steps = {}
        for k, (sid, name, _who) in enumerate(de.DRAW_STEPS):
            if k:
                ctk.CTkLabel(bar, text="›", text_color=IDLE_C).pack(side="left", padx=4)
            lb = ctk.CTkLabel(bar, text=f"○ {NUMS[k]} {name}", font=font(12), text_color=IDLE_C)
            lb.pack(side="left", padx=4)
            self.steps[sid] = lb
        self.todo = ctk.CTkLabel(body, text="연결 방식을 고르고 [연결 시작]을 누르세요", font=font(14, "bold"),
                                 anchor="w", justify="left", wraplength=740)
        self.todo.pack(fill="x", padx=16, pady=(10, 2))
        self.detail = ctk.CTkLabel(body, text="", font=font(12), anchor="w", justify="left", wraplength=740)
        self.detail.pack(fill="x", padx=16)

        opts = ctk.CTkFrame(body, fg_color="transparent")
        opts.pack(fill="x", padx=16, pady=8)
        self.virtual_var = ctk.BooleanVar(value=True)
        self.robot_radio = ctk.CTkRadioButton(opts, text=f"로봇 ({app.robot_port_label})", variable=self.virtual_var,
                                              value=False, font=font(12), command=self._sync_speed_menu)
        self.robot_radio.grid(row=0, column=0, sticky="w")
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
        self.start_pose_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(opts, text="호밍 뒤 시작 자세 맞추기 (기억한 자세로 자동 이동, 조그로 고치기)",
                        variable=self.start_pose_var, font=font(12)).grid(row=6, column=0, columnspan=3, sticky="w", pady=4)

        # ③ 시작 자세 맞추기 패널 (작업이 start_pose 이벤트를 보낼 때만 보임)
        self.jog_frame = ctk.CTkFrame(body)
        self._jog_vec, self._jog_after = None, None
        pad = ctk.CTkFrame(self.jog_frame, fg_color="transparent")
        pad.grid(row=0, column=0, rowspan=3, padx=(10, 16), pady=8)
        for r, c, text, vec in ((0, 0, "벽 쪽 (W)", (1, 0, 0)), (2, 0, "뒤로 (S)", (-1, 0, 0)),
                                (1, 1, "◀ 왼쪽 (A/←)", (0, 1, 0)), (1, 2, "오른쪽 ▶ (D/→)", (0, -1, 0)),
                                (0, 3, "▲ 위 (↑)", (0, 0, 1)), (2, 3, "▼ 아래 (↓)", (0, 0, -1))):
            b = ctk.CTkButton(pad, text=text, width=104, font=font(12))
            b.grid(row=r, column=c, padx=3, pady=3)
            b.bind("<ButtonPress-1>", lambda _e, v=vec: self._jog_press(v), add="+")
            b.bind("<ButtonRelease-1>", lambda _e: self._jog_release(), add="+")
        self.jog_step_var = ctk.StringVar(value=JOG_STEPS[1])
        ctk.CTkLabel(self.jog_frame, text="한 번에 (mm)", font=font(12)).grid(row=0, column=1, sticky="w")
        ctk.CTkSegmentedButton(self.jog_frame, values=JOG_STEPS, variable=self.jog_step_var, font=font(12)
                               ).grid(row=0, column=2, sticky="w", padx=6)
        self.jog_pos = ctk.CTkLabel(self.jog_frame, text="", font=font(12), anchor="w", justify="left")
        self.jog_pos.grid(row=1, column=1, columnspan=2, sticky="w")
        self.jog_accept_btn = ctk.CTkButton(self.jog_frame, text="이 위치 기억하고 계속", command=self.on_accept_start_pose,
                                            font=font(13, "bold"), fg_color=OK_C, hover_color="#15803d")
        self.jog_accept_btn.grid(row=2, column=1, columnspan=2, sticky="w", pady=(4, 8))
        self.jog_msg = ctk.CTkLabel(body, text="", font=font(12), anchor="w", justify="left", wraplength=740,
                                    text_color=BAD_C)
        self.bind("<KeyPress>", self._on_jog_key, add="+")
        self.clear_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(opts, text="그림이 끝나면 사진·작업 내용 비우기 (새 그림을 바로 시작)", variable=self.clear_var,
                        font=font(12)).grid(row=5, column=0, columnspan=3, sticky="w", pady=4)
        self.recalibrate_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(opts, text="종이·펜 위치 변경: 120×120 mm 다시 보정", variable=self.recalibrate_var,
                        font=font(12)).grid(row=2, column=0, columnspan=3, sticky="w", pady=4)
        self.check_var = ctk.BooleanVar(value=False)
        self.check_box = ctk.CTkCheckBox(opts, text="펜 끝이 종이 가운데에 닿아 있고 주변을 확인했습니다", variable=self.check_var,
                                         font=font(12, "bold"), command=self._on_check, state="disabled")
        self.check_box.grid(row=3, column=0, columnspan=3, sticky="w", pady=4)

        self.progress = ctk.CTkProgressBar(body, height=6)
        self.progress.pack(fill="x", padx=16, pady=(8, 2))
        self.progress.set(0)
        self.prog_text = ctk.CTkLabel(body, text="", font=font(12), anchor="w")
        self.prog_text.pack(fill="x", padx=16)

        btns = ctk.CTkFrame(body, fg_color="transparent")
        btns.pack(fill="x", padx=16, pady=12, side="bottom")
        self.begin_btn = ctk.CTkButton(btns, text="연결 시작", command=self.begin, font=font(13, "bold"), height=38)
        self.begin_btn.pack(side="left")
        self.rviz_btn = ctk.CTkButton(btns, text="RViz 3D로 따라보기", command=self._open_rviz, font=font(12),
                                      fg_color="transparent", border_width=1, text_color=TEXT,
                                      hover_color=SURFACE, state="disabled")
        self.rviz_btn.pack(side="left", padx=8)
        self.stop_btn = ctk.CTkButton(btns, text="■ 멈춤", command=self.on_stop, font=font(13, "bold"),
                                      fg_color=BAD_C, hover_color="#b91c1c", state="disabled")
        self.stop_btn.pack(side="right")
        self.start_btn = ctk.CTkButton(btns, text="시작", command=self.on_start, font=font(13, "bold"),
                                       state="disabled")
        self.start_btn.pack(side="right", padx=8)


    def _build_camera(self):
        """오른쪽 실시간 보기. 카메라를 못 열어도 왼쪽 그리기 절차는 그대로입니다."""
        side = ctk.CTkFrame(self, fg_color=CARD, border_width=1, border_color=BORDER, width=250)
        side.grid(row=0, column=1, sticky="ns", padx=(0, 16), pady=16)
        side.grid_propagate(False)
        ctk.CTkLabel(side, text="카메라", font=self.font(14, "bold"), anchor="w").pack(
            fill="x", padx=12, pady=(12, 4))
        self.camera_status = ctk.CTkLabel(side, text="꺼짐", font=self.font(11), text_color=IDLE_C, anchor="w")
        self.camera_status.pack(fill="x", padx=12)
        self.camera_on = ctk.BooleanVar(value=camera.preview_allowed())
        ctk.CTkSwitch(side, text="실시간 보기", variable=self.camera_on, font=self.font(12),
                      command=self._toggle_camera).pack(anchor="w", padx=12, pady=(6, 8))
        self.camera_label = ctk.CTkLabel(side, text="", width=220, height=160)
        self.camera_label.pack(padx=12, pady=(0, 8))
        ctk.CTkLabel(side, text="그림마다 모서리에 X를 그리면, 웹캠이 연결됐을 때 [준비 확인]이 나타납니다. 접촉은 보지 않습니다.",
                     font=self.font(11), text_color=IDLE_C, wraplength=220, justify="left", anchor="w").pack(
            fill="x", padx=12, pady=(0, 8))
        self.ready_btn = ctk.CTkButton(side, text="준비 확인", command=self._check_ready, font=self.font(12, "bold"),
                                       height=32)
        self.ready_result = ctk.CTkLabel(side, text="", font=self.font(11), text_color=TEXT, wraplength=220,
                                         justify="left", anchor="w")
        self._ready_shown = False
        self.camera = camera.CameraPreview()
        self._camera_photo = None
        self._camera_after = None
        if self.camera_on.get():
            self.camera.start()
            self.camera_status.configure(text="찾는 중")
        self._tick_camera()

    def _toggle_camera(self):
        if self.camera_on.get():
            self.camera.start()
            self.camera_status.configure(text="찾는 중", text_color=IDLE_C)
        else:
            self.camera.stop()
            self.camera_status.configure(text="꺼짐", text_color=IDLE_C)
            self._show_camera_frame(camera.blank_preview())

    def _tick_camera(self):
        if not self.winfo_exists():
            return
        frame = self.camera.latest()
        self._sync_ready_button(frame)
        if frame is not None:
            self._show_camera_frame(frame)
            name = f"카메라 {self.camera.index}" if self.camera.index is not None else "연결됨"
            self.camera_status.configure(text=name, text_color=OK_C)
        elif self.camera.error:
            self.camera_status.configure(text=self.camera.error, text_color=BAD_C)
        self._camera_after = self.after(80, self._tick_camera)

    def _show_camera_frame(self, frame):
        from PIL import Image
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        self._camera_photo = ctk.CTkImage(image, size=(220, max(80, int(220 * image.height / image.width))))
        self.camera_label.configure(image=self._camera_photo, text="")

    def _sync_ready_button(self, frame):
        from .camera_marks import ready_button_visible
        show = self.camera_on.get() and ready_button_visible(self.camera.index, frame)
        if show and not self._ready_shown:
            self.ready_btn.pack(fill="x", padx=12, pady=(0, 6))
            self.ready_result.pack(fill="x", padx=12, pady=(0, 8))
            self._ready_shown = True
        elif not show and self._ready_shown:
            self.ready_btn.pack_forget()
            self.ready_result.pack_forget()
            self._ready_shown = False

    def _check_ready(self):
        """기존 준비 안내에 USB·카메라 관찰만 얹어 보여 줍니다. 설정을 바꾸지 않습니다."""
        from . import robot_status
        from .camera_marks import attach_camera_check, observe_setup, ready_text
        frame = self.camera.latest()
        if frame is None:
            return
        try:
            choice = robot_status.detect_robot_port(self.cfg.get("port"))
        except Exception:
            choice = robot_status.PortChoice(None, "missing", ())
        report = attach_camera_check(self.session.robot_guide(), choice, observe_setup(frame))
        self.ready_result.configure(text=ready_text(report))

    def set_robot_port(self, label):
        """USB 감지 결과 (예: COM7, USB 미감지). 연결할 때 포트를 다시 찾으므로 표시용입니다."""
        self.robot_radio.configure(text=f"로봇 ({label})")

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
        if (not self.virtual_var.get() and self.start_pose_var.get() and self.cfg.get("requested_start_pose")
                and not self._requested_blocker()
                and not messagebox.askokcancel(
                    "로봇 이동 확인",
                    "호밍이 끝나면 로봇이 저장된 대기 자세(1번)로 바로 움직입니다. "
                    "옆으로 90° 돌아 손목을 꺾은 뒤 정면으로 돌아옵니다.\n\n"
                    "로봇 주변에 사람·물건이 없고, 이동 중 위험하면 로봇의 정지 버튼을 누를 수 있나요?\n"
                    "(앱의 취소는 이미 보낸 이동을 바로 멈추지 못합니다.)", parent=self)):
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
        self._hide_jog()
        virtual = bool(self.virtual_var.get())
        self.job.start(virtual=virtual, virtual_speed=speed,
                       pending=bool(self.pending_var.get()),
                       recalibrate=bool(self.recalibrate_var.get()),
                       return_to_origin=bool(self.return_var.get()),
                       start_pose=bool(self.start_pose_var.get()) and
                                  (not virtual or self.start_pose_in_virtual or bool(self.cfg.get("requested_start_pose"))))

    def _requested_blocker(self):
        """실물 검증이 안 됐으면 그 이유. 그때는 작업이 ① 사전 검사에서 이유를 보여 주고 멈추므로 묻지 않는다."""
        from . import start_pose as sp
        return sp.requested_motion_blocker(self.cfg)

    # ---------------------------------------------------------------- ③ 시작 자세 조그
    def _show_jog(self):
        if not self.jog_frame.winfo_ismapped():
            self.jog_frame.pack(fill="x", padx=16, pady=(4, 2), before=self.progress)
            self.jog_msg.pack(fill="x", padx=16, before=self.progress)
            self.jog_accept_btn.configure(state="normal")
            self.focus_set()                 # 키보드 조그를 바로 받게

    def _hide_jog(self):
        self._jog_release()
        self.jog_frame.pack_forget()
        self.jog_msg.pack_forget()
        self.jog_msg.configure(text="")

    def _jog_press(self, vec):
        """누르고 있는 동안 JOG_REPEAT_MS마다 같은 방향으로 한 번씩 (작업 쪽이 바쁘면 버림 → 손을 떼면 곧 멈춤)."""
        self._jog_release()
        self._jog_vec = vec
        self._jog_tick()

    def _jog_tick(self):
        if self._jog_vec is None or self.job is None:
            return
        step = float(self.jog_step_var.get())
        self.job.jog(*(v * step for v in self._jog_vec))
        self._jog_after = self.after(JOG_REPEAT_MS, self._jog_tick)

    def _jog_release(self):
        self._jog_vec = None
        if self._jog_after is not None:
            self.after_cancel(self._jog_after)
            self._jog_after = None

    def _on_jog_key(self, e):
        """키를 누르고 있으면 운영체제의 자동 반복으로 계속 들어옴 (한 번에 한 칸)."""
        vec = JOG_KEYS.get(e.keysym) or JOG_KEYS.get(e.keysym.lower())
        if vec is None or self.job is None or not self.jog_frame.winfo_ismapped():
            return
        step = float(self.jog_step_var.get())
        self.job.jog(*(v * step for v in vec))

    def on_accept_start_pose(self):
        if self.job:
            self.jog_accept_btn.configure(state="disabled")      # 검사(그리기 범위 계산)가 끝날 때까지
            self.jog_msg.configure(text="그리기 범위를 확인하는 중입니다…", text_color=IDLE_C)
            self.job.accept_start_pose()

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
        self._stop_camera()
        self.destroy()
        return True

    def _stop_camera(self):
        if self._camera_after is not None:
            try:
                self.after_cancel(self._camera_after)
            except Exception:
                pass
            self._camera_after = None
        self.camera.stop()

    # ---------------------------------------------------------------- 작업 이벤트 (메인 스레드)
    def _event(self, kind, d):
        if not self.winfo_exists():
            return
        if kind == "step":
            k = [s[0] for s in de.DRAW_STEPS].index(d["id"])
            name = de.DRAW_STEPS[k][1]
            mark, color = {"active": ("●", ACT_C), "done": ("✓", OK_C), "failed": ("✕", BAD_C)}[d["status"]]
            self.steps[d["id"]].configure(text=f"{mark} {NUMS[k]} {name}", text_color=color)
            if d["id"] == "start" and d["status"] in ("done", "failed"):
                self._hide_jog()
            if d["status"] == "active" and d["message"]:
                self.todo.configure(text=d["message"])
            elif d["status"] == "failed":
                self.failure_message = f"{name}: {d['message']}"
                self.todo.configure(text=self.failure_message)
                self.detail.configure(text=d.get("hint", ""))
            if d["id"] == "confirm" and d["status"] == "active":
                s = self.job.summary
                if self.cfg.get("requested_start_pose") and self.start_pose_var.get():
                    self.check_box.configure(text="종이와 주변을 확인했습니다 (시작하면 접촉 자세로 이동)")
                self.detail.configure(text=f"획 {s['stroke_count']} · 명령 {s['command_count']:,}줄 · "
                                           f"예상 {s['estimated_s'] / 60:.1f}분 · {s['drawing_mm'][0]:.0f}×"
                                           f"{s['drawing_mm'][1]:.0f}mm · {s['port']}"
                                           + ("" if s["virtual"] or s["plane_verified"] else
                                              " · 종이 보정 없음(설정에 저장된 값 사용)")
                                           + ("" if s["virtual"] or s.get("start_center_shift_mm", 0) < 0.05 else
                                              f" · 시작 자세를 종이 중심으로 사용(설정 파일 값과 {s['start_center_shift_mm']:g} mm 차이)"))
                self.check_box.configure(state="normal")
                self._on_check()
        elif kind == "start_pose":
            tcp = d.get("tcp")
            if d["state"] == "saved":
                self._hide_jog()
                self.detail.configure(text=d.get("message", ""))
                return
            self._show_jog()
            if tcp:
                self.jog_pos.configure(text=f"펜 위치(플랜지) X {tcp[0]:.1f} · Y {tcp[1]:.1f} · Z {tcp[2]:.1f} mm")
            if d["state"] == "moving":
                self.todo.configure(text=d.get("message", ""))
                self.jog_accept_btn.configure(state="disabled")
            else:
                self.todo.configure(text="조그로 펜 끝을 종이 가운데에 살짝 대고 [이 위치 기억하고 계속]을 누르세요")
                problems = d.get("problems") or []
                msg = d.get("message", "")
                self.jog_msg.configure(text="\n".join([msg, *problems]).strip() if (problems or msg) else "",
                                       text_color=BAD_C if problems or "넘어" in msg else IDLE_C)
                self.jog_accept_btn.configure(state="normal")   # 검사 중에는 작업이 다른 이벤트를 보내지 않음
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
            self._hide_jog()
            self.finished = d
            self.stop_btn.configure(state="disabled")
            r = d["result"]["result"]
            returned = d["result"].get("returned_to_origin")      # True 돌아옴 / False 못 돌아옴 / None 해당 없음
            return_name = "호밍 자세" if self.cfg.get("requested_start_pose") and self.start_pose_var.get() else "처음 위치(종이 중심)"
            done_text = "완료했습니다" + {True: f". 로봇이 {return_name}로 돌아왔습니다.",
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
