"""
'로봇으로 그리기' 창 — 단계 표시줄(①~⑥), 지금 할 일, 최종 확인, 진행 막대, 멈춤
===================================================================================
실행은 DrawJob(작업 스레드)이 하고, 이 창은 events를 app.ui()로 받아 표시만 합니다.
"""

import threading
import time
from tkinter import messagebox, simpledialog

import cv2
import customtkinter as ctk
import numpy as np
from . import camera, camera_marks, camera_share, draw_executor as de, limits, pen_correction, ui_scale, vision
from .draw_job import DrawJob

from .theme import (ACCENT as ACT_C, BAD as BAD_C, BORDER, CARD, MUTED as IDLE_C, OK as OK_C, SELECTED, SURFACE,
                    TEXT, FilledButton)
SPEEDS = ["1×", "5×", "20×", "50×", "500×"]
NUMS = "①②③④⑤⑥"
ACTIVE_STATES = ("preflight", "connect", "start", "confirm", "drawing", "paused")
JOG_STEPS = ["0.5", "2", "10"]          # 조그 한 번 이동량(mm)
JOG_REPEAT_MS = 150                      # 버튼을 누르고 있는 동안 다시 보내는 간격 (로봇이 바쁘면 작업 쪽에서 버림)
# 키보드 조그: 로봇 좌표 방향 (X 벽 쪽, Y 종이 왼쪽, Z 위). 종이 오른쪽 = 로봇 -Y (paper_x_to_robot_y_sign = -1)
JOG_KEYS = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0),
            "Left": (0, 1, 0), "Right": (0, -1, 0), "Up": (0, 0, 1), "Down": (0, 0, -1)}
CAMERA_PANEL_W = 300                     # 오른쪽 카메라 칸 폭 (배율 1.0 단위)
CAMERA_AUTO_LABEL = "자동 (처음 찾은 카메라)"
LINE_CHECK_EVERY_S = 8.0                 # 그리는 동안 선 확인 간격
LINE_CHECK_FRAMES = 6                    # 한 번 잴 때 평균하는 사진 수 (센서 잡음 줄이기)
LINE_CHECK_SKIP_LAST = 2                 # 그리는 동안은 방금 끝난 획 몇 개를 빼고 잼 (펜·팔 밑, 사진 지연)
MARK_CHECK_EVERY_S = 3.0                 # 그리지 않는 동안 모서리 표시(X·십자)를 찾는 간격
MARK_CONFIRM = 3                         # 이만큼 연달아 같은 자리에서 찾아야 종이 위치를 바꿈 (손·팔이 지나가는 순간 제외)
MARK_STABLE_PX = 4.0                     # 연달아 찾은 자리가 이 안이면 같은 자리
MARK_UPDATE_MM = 1.5                     # (자동 조절을 끄면) 종이가 펜 기준 자리에서 이보다 많이 벗어날 때 알림
MARK_ROT_WARN_DEG = 1.0                  # (자동 조절을 끄면) 종이가 이보다 많이 돌아가 있을 때 알림
FIT_CHANGE_MM = 0.5                      # 종이 맞춤을 이보다 많이 바뀔 때만 새로 적용 (표시를 잴 때마다 조금씩 흔들림)
FIT_CHANGE_DEG = 0.3
FIT_CHANGE_SCALE = 0.005
MARK_SEARCH_MM = 40.0                    # 저장된 위치가 있으면 표시가 있어야 할 자리에서 이 안만 봄


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
        ui_scale.place_window(self, 1180, 760, 1000, 640)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)
        self.grid_rowconfigure(0, weight=1)
        body = ctk.CTkFrame(self, fg_color="transparent")
        body.grid(row=0, column=0, sticky="nsew")
        self._build_camera()
        ctk.CTkLabel(body, text="드로잉 컨트롤", font=font(22, "bold"), anchor="w").pack(
            fill="x", padx=20, pady=(18, 0))
        ctk.CTkLabel(body, text="연결부터 완료까지, 안전 절차를 한 단계씩 확인합니다.",
                     font=font(13), text_color=IDLE_C, anchor="w").pack(fill="x", padx=20, pady=(2, 8))
        self.protocol("WM_DELETE_WINDOW", self.close)

        bar = ctk.CTkFrame(body, fg_color=CARD, corner_radius=12, border_width=1, border_color=BORDER)
        bar.pack(fill="x", padx=16, pady=(4, 8), ipady=8)
        self.steps = {}
        for k, (sid, name, _who) in enumerate(de.DRAW_STEPS):
            if k:
                ctk.CTkLabel(bar, text="›", font=font(14), text_color=IDLE_C).pack(side="left", padx=2)
            lb = ctk.CTkLabel(bar, text=f"○ {NUMS[k]} {name}", font=font(14), text_color=IDLE_C)
            lb.pack(side="left", padx=(10 if k == 0 else 4, 4))
            self.steps[sid] = lb
        self.todo = ctk.CTkLabel(body, text="연결 방식을 고르고 [연결 시작]을 누르세요", font=font(17, "bold"),
                                 anchor="w", justify="left", wraplength=740)
        self.todo.pack(fill="x", padx=16, pady=(10, 2))
        self.detail = ctk.CTkLabel(body, text="", font=font(14), anchor="w", justify="left", wraplength=740)
        self.detail.pack(fill="x", padx=16)
        ui_scale.follow_width(self.todo, body, 40)
        ui_scale.follow_width(self.detail, body, 40)

        # 옵션: 연결 방식 / 그리기 방식 / 끝난 뒤 로 묶어 왼쪽에 이름을 붙임
        opts = ctk.CTkFrame(body, fg_color=CARD, corner_radius=12, border_width=1, border_color=BORDER)
        opts.pack(fill="x", padx=16, pady=8)
        opts.grid_columnconfigure(1, weight=1)

        def group(row, title):
            ctk.CTkLabel(opts, text=title, font=font(14, "bold"), text_color=IDLE_C, anchor="w", width=96
                         ).grid(row=row, column=0, sticky="nw", padx=(16, 8), pady=(12 if row == 0 else 6, 6))
            frame = ctk.CTkFrame(opts, fg_color="transparent")
            frame.grid(row=row, column=1, sticky="ew", padx=(0, 16), pady=(10 if row == 0 else 4, 4))
            return frame

        check = dict(font=font(14), checkbox_width=22, checkbox_height=22)
        conn = group(0, "연결 방식")
        self.virtual_var = ctk.BooleanVar(value=True)
        self.robot_radio = ctk.CTkRadioButton(conn, text=f"로봇 ({app.robot_port_label})", variable=self.virtual_var,
                                              value=False, font=font(14), command=self._sync_speed_menu)
        self.robot_radio.grid(row=0, column=0, sticky="w", pady=3)
        ctk.CTkRadioButton(conn, text="가상 시뮬레이션 (로봇 없이)", variable=self.virtual_var, value=True,
                           font=font(14), command=self._sync_speed_menu).grid(row=0, column=1, sticky="w", padx=16)
        self.speed_var = ctk.StringVar(value="20×")
        # 배속은 가상 시뮬레이션에만 적용되므로 로봇을 고르면 숨김 (실제 로봇은 항상 실제 속도)
        self.speed_menu = ctk.CTkOptionMenu(conn, values=SPEEDS, variable=self.speed_var, width=90, height=32,
                                            font=font(14), dropdown_font=font(14))
        self.speed_menu.grid(row=0, column=2, sticky="w")
        how = group(1, "그리기 방식")
        self.air_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(how, text="공중 모드 (펜을 대지 않고 경로만)", variable=self.air_var,
                        command=self._on_air_changed, **check).grid(row=0, column=0, sticky="w", pady=3)
        self.pending_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(how, text="넓은 범위 (공중 확인용)", variable=self.pending_var, **check
                        ).grid(row=0, column=1, sticky="w", padx=16, pady=3)
        self.start_pose_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(how, text="호밍 뒤 시작 자세 맞추기 (기억한 자세로 이동, 조그로 고치기)",
                        variable=self.start_pose_var, **check).grid(row=1, column=0, columnspan=2, sticky="w", pady=3)
        self.recalibrate_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(how, text="종이·펜 위치가 바뀌었으면 120×120 mm 다시 보정", variable=self.recalibrate_var,
                        **check).grid(row=2, column=0, columnspan=2, sticky="w", pady=3)
        after = group(2, "끝난 뒤")
        self.return_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(after, text="펜을 처음 위치로 되돌리기 (저장된 자세로 시작했으면 멈춰도 호밍 자세로)",
                        variable=self.return_var, **check
                        ).grid(row=0, column=0, sticky="w", pady=3)
        self.clear_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(after, text="사진·작업 내용 비우기 (새 그림을 바로 시작)", variable=self.clear_var, **check
                        ).grid(row=1, column=0, sticky="w", pady=(3, 10))

        # ③ 시작 자세 맞추기 패널 (작업이 start_pose 이벤트를 보낼 때만 보임)
        self.jog_frame = ctk.CTkFrame(body, fg_color=CARD, corner_radius=12, border_width=1, border_color=BORDER)
        self._jog_vec, self._jog_after = None, None
        pad = ctk.CTkFrame(self.jog_frame, fg_color="transparent")
        pad.grid(row=0, column=0, rowspan=3, padx=(10, 16), pady=8)
        for r, c, text, vec in ((0, 0, "벽 쪽 (W)", (1, 0, 0)), (2, 0, "뒤로 (S)", (-1, 0, 0)),
                                (1, 1, "◀ 왼쪽 (A/←)", (0, 1, 0)), (1, 2, "오른쪽 ▶ (D/→)", (0, -1, 0)),
                                (0, 3, "▲ 위 (↑)", (0, 0, 1)), (2, 3, "▼ 아래 (↓)", (0, 0, -1))):
            b = ctk.CTkButton(pad, text=text, width=120, height=40, font=font(14))
            b.grid(row=r, column=c, padx=3, pady=3)
            b.bind("<ButtonPress-1>", lambda _e, v=vec: self._jog_press(v), add="+")
            b.bind("<ButtonRelease-1>", lambda _e: self._jog_release(), add="+")
        self.jog_step_var = ctk.StringVar(value=JOG_STEPS[1])
        ctk.CTkLabel(self.jog_frame, text="한 번에 (mm)", font=font(14)).grid(row=0, column=1, sticky="w")
        ctk.CTkSegmentedButton(self.jog_frame, values=JOG_STEPS, variable=self.jog_step_var, font=font(14), height=34
                               ).grid(row=0, column=2, sticky="w", padx=6)
        self.jog_pos = ctk.CTkLabel(self.jog_frame, text="", font=font(14), anchor="w", justify="left")
        self.jog_pos.grid(row=1, column=1, columnspan=2, sticky="w")
        self.jog_accept_btn = FilledButton(self.jog_frame, text="이 위치 기억하고 계속", command=self.on_accept_start_pose,
                                           font=font(14, "bold"), height=40, fg_color=OK_C, hover_color="#15803d")
        self.jog_accept_btn.grid(row=2, column=1, columnspan=2, sticky="w", pady=(4, 8))
        self.jog_msg = ctk.CTkLabel(body, text="", font=font(14), anchor="w", justify="left", wraplength=740,
                                    text_color=BAD_C)
        ui_scale.follow_width(self.jog_msg, body, 40)
        self.bind("<KeyPress>", self._on_jog_key, add="+")

        self.progress = ctk.CTkProgressBar(body, height=8)
        self.progress.pack(fill="x", padx=16, pady=(8, 2))
        self.progress.set(0)
        self.prog_text = ctk.CTkLabel(body, text="", font=font(14), anchor="w")
        self.prog_text.pack(fill="x", padx=16)

        # 맨 아래: 버튼 줄, 그 바로 위에 [시작]을 여는 최종 확인
        btns = ctk.CTkFrame(body, fg_color="transparent")
        btns.pack(fill="x", padx=16, pady=(8, 16), side="bottom")
        big = dict(height=46, font=font(15, "bold"))
        self.begin_btn = FilledButton(btns, text="연결 시작", command=self.begin, width=150, **big)
        self.begin_btn.pack(side="left")
        self.rviz_btn = ctk.CTkButton(btns, text="RViz 3D로 따라보기", command=self._open_rviz, font=font(14),
                                      height=46, fg_color="transparent", border_width=1, text_color=TEXT,
                                      hover_color=SURFACE, state="disabled")
        self.rviz_btn.pack(side="left", padx=8)
        self.stop_btn = FilledButton(btns, text="■ 멈춤", command=self.on_stop, width=130,
                                     fg_color=BAD_C, hover_color="#b91c1c", state="disabled", **big)
        self.stop_btn.pack(side="right")
        self.start_btn = FilledButton(btns, text="시작", command=self.on_start, width=150, state="disabled", **big)
        self.start_btn.pack(side="right", padx=8)
        confirm = ctk.CTkFrame(body, fg_color=SELECTED, corner_radius=12)
        confirm.pack(fill="x", padx=16, pady=(4, 0), side="bottom")
        self.check_var = ctk.BooleanVar(value=False)
        self.check_box = ctk.CTkCheckBox(confirm, text="펜 끝이 종이 가운데에 닿아 있고 주변을 확인했습니다",
                                         variable=self.check_var, font=font(14, "bold"), checkbox_width=24,
                                         checkbox_height=24, command=self._on_check, state="disabled")
        self.check_box.pack(anchor="w", padx=16, pady=12)


    def _build_camera(self):
        """오른쪽 실시간 보기. 카메라를 못 열어도 왼쪽 그리기 절차는 그대로입니다."""
        side = ctk.CTkFrame(self, fg_color=CARD, border_width=1, border_color=BORDER, width=CAMERA_PANEL_W)
        side.grid(row=0, column=1, sticky="ns", padx=(0, 16), pady=16)
        side.grid_propagate(False)
        side.pack_propagate(False)
        inner = CAMERA_PANEL_W - 28
        head = ctk.CTkFrame(side, fg_color="transparent")
        head.pack(fill="x", padx=12, pady=(12, 4))
        ctk.CTkLabel(head, text="카메라", font=self.font(16, "bold"), anchor="w").pack(side="left")
        self.camera_on = ctk.BooleanVar(value=camera.preview_allowed())
        ctk.CTkSwitch(head, text="보기", variable=self.camera_on, font=self.font(13), width=60,
                      command=self._toggle_camera).pack(side="right")
        # 카메라 고르기: 노트북 내장 카메라 대신 로봇·종이를 비추는 외장 카메라를 쓸 수 있게
        pick = ctk.CTkFrame(side, fg_color="transparent")
        pick.pack(fill="x", padx=12, pady=(4, 2))
        self.camera_menu = ctk.CTkOptionMenu(pick, values=[CAMERA_AUTO_LABEL], command=self._on_camera_pick,
                                             height=32, font=self.font(13), dropdown_font=self.font(13),
                                             dynamic_resizing=False)
        self.camera_menu.set(CAMERA_AUTO_LABEL)
        self.camera_menu.pack(side="left", fill="x", expand=True)
        self.camera_refresh_btn = ctk.CTkButton(pick, text="↻", width=34, height=32, font=self.font(15),
                                                fg_color="transparent", border_width=1, text_color=TEXT,
                                                hover_color=SURFACE, command=self._refresh_cameras)
        self.camera_refresh_btn.pack(side="left", padx=(6, 0))
        self.camera_status = ctk.CTkLabel(side, text="꺼짐", font=self.font(12), text_color=IDLE_C, anchor="w",
                                          justify="left", wraplength=inner)
        self.camera_status.pack(fill="x", padx=12, pady=(2, 6))
        self.camera_label = ctk.CTkLabel(side, text="", width=inner, height=int(inner * 0.75), fg_color=SURFACE,
                                         corner_radius=8)
        self.camera_label.pack(padx=12, pady=(0, 8))
        # 종이 위치: 사진에서 종이 귀퉁이 4개를 찍어 화면 ↔ 종이 mm를 맞추고, 계획한 그림을 화면에 겹침
        paper = ctk.CTkFrame(side, fg_color="transparent")
        paper.pack(fill="x", padx=12, pady=(0, 2))
        self.register_btn = ctk.CTkButton(paper, text="종이 위치 맞추기", command=self._open_register, height=34,
                                          font=self.font(13, "bold"), fg_color="transparent", border_width=1,
                                          text_color=TEXT, hover_color=SURFACE, state="disabled")
        self.register_btn.pack(side="left", fill="x", expand=True)
        # 그리는 중 펜 선을 원본 해상도로 크게 보기 (따로 창)
        self.zoom_btn = ctk.CTkButton(paper, text="크게 보기", command=self._open_zoom, height=34, width=90,
                                      font=self.font(13, "bold"), fg_color="transparent", border_width=1,
                                      text_color=TEXT, hover_color=SURFACE)
        self.zoom_btn.pack(side="left", padx=(6, 0))
        self.overlay_on = ctk.BooleanVar(value=True)
        self.overlay_switch = ctk.CTkSwitch(side, text="계획한 그림 겹쳐 보기", variable=self.overlay_on,
                                            font=self.font(13), state="disabled")
        self.overlay_switch.pack(anchor="w", padx=12, pady=(6, 0))
        self.vision_status = ctk.CTkLabel(side, text="", font=self.font(12), text_color=IDLE_C, anchor="w",
                                          justify="left", wraplength=inner)
        self.vision_status.pack(fill="x", padx=12, pady=(2, 0))
        # 사람이 종이 네 귀퉁이 근처에 X·십자를 그려 두면 그리지 않는 동안 찾음. 종이 위치가 아직 없으면 표시로 맞추고,
        # 있으면(로봇이 그리는 자리 기준) 덮어쓰지 않고, 표시로 잰 종이 자리·회전·크기에 맞춰 로봇이 그릴 좌표를 옮김.
        # 표시가 없으면 원래 위치로 그림
        self.marks_auto = ctk.BooleanVar(value=True)
        ctk.CTkSwitch(side, text="모서리 표시(X·십자)로 종이 위치 확인", variable=self.marks_auto,
                      font=self.font(13)).pack(anchor="w", padx=12, pady=(4, 0))
        self.fit_auto = ctk.BooleanVar(value=True)
        ctk.CTkSwitch(side, text="표시에 맞춰 그림 위치·크기 자동 조절", variable=self.fit_auto,
                      font=self.font(13), command=self._on_fit_toggle).pack(anchor="w", padx=12, pady=(2, 0))
        self.marks_status = ctk.CTkLabel(
            side, text="종이 네 귀퉁이에 붙여 X나 십자를 그리면, 종이가 옮겨지거나 "
                       "크기가 달라도 그 종이에 맞춰 그립니다. 없으면 원래 위치로 그립니다.",
            font=self.font(12), text_color=IDLE_C, anchor="w", justify="left", wraplength=inner)
        self.marks_status.pack(fill="x", padx=12, pady=(0, 2))
        self._marks_busy, self._marks_last, self._marks_hist, self._marks_miss = False, 0.0, [], 0
        # 그리는 동안 몇 초마다 다 그린 획을 잼: 선이 보인 획 초록, 안 보인 획 빨강, 아직 안 잰 획 보라
        self.line_status = ctk.CTkLabel(side, text="", font=self.font(12), text_color=IDLE_C, anchor="w",
                                        justify="left", wraplength=inner)
        self.line_status.pack(fill="x", padx=12, pady=(0, 2))
        # 끝점 세로 줄: 다 그린 획의 시작·끝점에서 세로로 뻗은 줄을 재고, 많으면 남은 획의 끝점을 종이에서 조금 뒤로 물림
        self.pen_auto = ctk.BooleanVar(value=True)
        ctk.CTkSwitch(side, text="끝점 세로 줄 자동 보정", variable=self.pen_auto, font=self.font(13)).pack(
            anchor="w", padx=12, pady=(4, 0))
        self.pen_status = ctk.CTkLabel(side, text=self._pen_text(), font=self.font(12), text_color=IDLE_C,
                                       anchor="w", justify="left", wraplength=inner)
        self.pen_status.pack(fill="x", padx=12, pady=(0, 8))
        self._overlay = vision.Overlay()
        self._registration = None         # 지금 카메라의 종이 위치 (vision.Registration)
        self._registration_camera = None  # 위 등록을 읽은 (카메라 이름, 사진 크기)
        self._regions = ((limits.executor_region(self.cfg), vision.PEN_RANGE_COLOR),
                         (limits.pending_region(self.cfg), vision.PENDING_RANGE_COLOR))
        self._run_strokes = None          # 이번 실행에서 그리는 획 (그리는 순서, plan 이벤트)
        self._drawn = 0                   # 앞에서부터 다 그린 획 수
        self._line_results = {}           # {획 번호: 선이 보인 경로 비율 또는 None(가려짐)}
        self._stroke_colors = None
        self._line_busy = False
        self._line_last = 0.0
        self._line_final_wanted = False   # 끝났을 때 확인 중이었으면 그 확인이 끝난 뒤 마지막 확인을 함
        self._tick_results = {}           # {획 번호: (시작점 세로 줄 mm|None, 끝점 mm|None)}
        self._tick_dirs = {}              # {획 번호: (시작점 (위 mm, 아래 mm)|None, 끝점 ...)} 비스듬히 대고 떼기 보정용
        self._tick_marks = ()             # 세로 줄이 난 끝점 (겹쳐 그릴 표시)
        self._pen_from = 0                # 이 획부터 지금 끝점 물림 값으로 그림 (보정 판단은 이후 획만)
        self._pen_prev_from = None        # 바꾸기 전 값으로 그린 구간의 시작 (선이 안 보인 비율을 비교할 기준)
        self._pen_note = ""               # 마지막 보정 내용
        self._pen_dirs = {}               # 이번 그림에서 값마다 마지막으로 바꾼 방향 (+1/−1)
        self._pen_locked = set()          # 판단이 오가 가운데 값으로 고정한 키 (이번 그림에서 더 안 바꿈)
        self._pen_start = None            # 이번 그림을 시작할 때 펜 값 (끝난 뒤 바뀐 값만 한 번 저장)
        self._zoom_win = None             # 크게 보기 창
        self._zoom_overlay = vision.Overlay()
        self._zoom_box_key = None
        self._share = camera_share.CameraShare()
        ctk.CTkLabel(side, text="모서리 X가 보이면 [준비 확인]도 나타납니다. 접촉은 보지 않습니다.",
                     font=self.font(12), text_color=IDLE_C, wraplength=inner, justify="left", anchor="w").pack(
            fill="x", padx=12, pady=(0, 8))
        self.ready_btn = ctk.CTkButton(side, text="준비 확인", command=self._check_ready, font=self.font(13, "bold"),
                                       height=36)
        self.ready_result = ctk.CTkLabel(side, text="", font=self.font(12), text_color=TEXT, wraplength=inner,
                                         justify="left", anchor="w")
        self._ready_shown = False
        self.cameras = []                 # 마지막으로 읽은 장치 목록 [camera.CameraInfo]
        self._listing = False
        self.camera = camera.CameraPreview()
        self._camera_photo = None
        self._camera_after = None
        if self.camera_on.get():
            self.camera_status.configure(text="카메라 목록을 읽는 중")
            self._refresh_cameras(start=True)
        self._tick_camera()

    def _refresh_cameras(self, start=False):
        """장치 목록을 작업 스레드에서 읽고(Windows 약 0.5초) 메뉴를 채움. start면 저장한 카메라로 미리보기를 켬."""
        if self._listing:
            return
        self._listing = True
        self.camera_refresh_btn.configure(state="disabled")

        def work():
            try:
                found = camera.list_cameras()
            except Exception:
                found = []
            self.app.ui(self._show_cameras, found, start)

        threading.Thread(target=work, name="camera-list", daemon=True).start()

    def _show_cameras(self, found, start):
        self._listing = False
        if not self.winfo_exists():
            return
        self.camera_refresh_btn.configure(state="normal")
        self.cameras = found
        self.camera_menu.configure(values=[CAMERA_AUTO_LABEL] + [c.label for c in found])
        wanted = camera.resolve_choice(camera.load_choice(), found)
        current = next((c for c in found if c.index == wanted), None)
        self.camera_menu.set(current.label if current else CAMERA_AUTO_LABEL)
        settings = camera.capture_settings(current.name) if current else None
        if wanted != self.camera.wanted or settings != self.camera.settings:
            self.camera.select(wanted, start=self.camera_on.get(), settings=settings)
        elif start and self.camera_on.get():
            self.camera.start()
        if not found:
            self.camera_status.configure(text="연결된 카메라가 없습니다", text_color=BAD_C)

    def _on_camera_pick(self, label):
        info = next((c for c in self.cameras if c.label == label), None)
        camera.save_choice(info)
        self.camera.select(None if info is None else info.index, start=self.camera_on.get(),
                           settings=camera.capture_settings(info.name) if info else None)
        self._show_camera_frame(camera.blank_preview("..."))
        if self.camera_on.get():
            self.camera_status.configure(text="여는 중", text_color=IDLE_C)

    def _toggle_camera(self):
        if self.camera_on.get():
            self.camera_status.configure(text="찾는 중", text_color=IDLE_C)
            if self.cameras:
                self.camera.start()
            else:
                self._refresh_cameras(start=True)
        else:
            self.camera.stop()
            self.camera_status.configure(text="꺼짐", text_color=IDLE_C)
            self._show_camera_frame(camera.blank_preview())

    def _camera_name(self, index):
        info = next((c for c in self.cameras if c.index == index), None)
        return info.name if info else f"카메라 {index}"

    def _current_camera_name(self):
        return None if self.camera.index is None else self._camera_name(self.camera.index)

    def _sync_registration(self):
        """지금 카메라에 저장된 종이 위치를 읽고 버튼·안내를 맞춤 (카메라나 사진 크기가 바뀔 때만 파일을 읽음).
        찍을 때와 화면 비율이 다른 사진(해상도가 바뀐 카메라)에는 쓰지 않음: 보이는 범위가 달라 계획이 엉뚱한 곳에 겹침."""
        name = self._current_camera_name()
        size = self._full_size()
        if (name, size) != self._registration_camera:
            self._registration_camera = (name, size)
            reg = vision.load(name) if name else None
            mismatch = reg is not None and size is not None and not reg.fits(size)
            self._registration = None if mismatch else reg
            if mismatch:
                w0, h0 = reg.frame_size
                self.vision_status.configure(
                    text=f"종이 위치를 {w0}×{h0} 사진에서 맞췄는데 지금 카메라는 {size[0]}×{size[1]}입니다. "
                         "해상도가 다르면 보이는 범위가 달라지니 다시 맞추세요.", text_color=BAD_C)
            elif reg is None:
                self.vision_status.configure(
                    text="종이 위치를 아직 맞추지 않았습니다. 카메라나 판을 옮겼다면 다시 맞추세요." if name else "",
                    text_color=IDLE_C)
            else:
                self.vision_status.configure(text=f"종이 위치 맞춤 · {reg.saved_at[:16].replace('T', ' ')}",
                                             text_color=OK_C)
        self.register_btn.configure(state="normal" if name else "disabled")
        self.overlay_switch.configure(state="normal" if self._registration else "disabled")

    def _plan(self):
        """겹쳐 그릴 계획: (획 mm 목록 또는 None, 펜 범위들). 그리는 중·뒤에는 이번 실행의 획(그리는 순서)."""
        if self._run_strokes:
            return self._run_strokes, self._regions
        result = self.session.result
        strokes = self.session.robot_strokes_mm() if result else None     # 종이 맞춤이 있으면 로봇이 그릴 자리
        return strokes, self._regions

    def _on_fit_toggle(self):
        """자동 조절을 끄면 바로 원래 위치로 (켜면 다음 표시 확인 때 다시 맞춤)."""
        if not self.fit_auto.get() and self.session.paper_fit_request is not None:
            self.session.set_paper_fit(None)
            self.marks_status.configure(text="자동 조절을 껐습니다. 원래 위치로 그립니다", text_color=IDLE_C)
        self._marks_hist = []

    def _open_register(self):
        name = self._current_camera_name()
        if name is None:
            return
        from .paper_register_window import PaperRegisterWindow

        def saved(reg):
            self._registration_camera = None           # 다음 틱에서 새 등록을 다시 읽음
            self.overlay_on.set(True)

        PaperRegisterWindow(self, self.font, name, self.camera.latest_full, self.cfg, saved)

    def _tick_camera(self):
        if not self.winfo_exists():
            return
        frame = self.camera.latest()
        self._sync_ready_button(frame)
        self._sync_registration()
        if frame is not None:
            if self._registration is not None and self.overlay_on.get():
                strokes, regions = self._plan()
                colors = self._stroke_colors if strokes is self._run_strokes else None
                frame = self._overlay.draw(frame, self._registration, strokes, regions, colors,
                                           self._tick_marks if strokes is self._run_strokes else ())
            self._show_camera_frame(frame)
            self._update_zoom()
            self._share_frame()
            name = self._camera_name(self.camera.index) if self.camera.index is not None else "연결됨"
            text = f"● {name}"
            if self._share.error:
                text = f"{text}\n{self._share.error}"
            self.camera_status.configure(text=text, text_color=BAD_C if self._share.error else OK_C)
        elif self.camera.error or self._share.error:
            text = self.camera.error
            if self._share.error:
                text = f"{text}\n{self._share.error}" if text else self._share.error
            self.camera_status.configure(text=text, text_color=BAD_C)
        self._maybe_check_lines()
        self._maybe_check_marks()
        self._camera_after = self.after(80, self._tick_camera)

    # ------------------------------------------------------------ 모서리 표시(X·십자)로 종이 위치 자동 보정
    def _maybe_check_marks(self):
        """그리지 않는 동안 MARK_CHECK_EVERY_S마다 원본 사진에서 표시를 찾음 (작업 스레드). 그리는 중에는 팔·펜이
        종이를 가리고 종이 위치를 바꾸면 선 확인 기준이 흔들리므로 보지 않음. 표시가 없으면 아무것도 바꾸지 않음."""
        if (not self.marks_auto.get() or self._marks_busy
                or time.monotonic() - self._marks_last < MARK_CHECK_EVERY_S
                or (self.job is not None and self.job.state in ACTIVE_STATES)):
            return
        name, size, full = self._current_camera_name(), self._full_size(), self.camera.latest_full()
        if name is None or size is None or full is None:
            return
        self._marks_busy, self._marks_last = True, time.monotonic()
        saved = vision.load(name)
        saved = saved if saved is not None and saved.fits(size) else None
        paper = saved.paper_mm if saved else vision.A4_LANDSCAPE_MM
        expected = radius = None
        if saved is not None:
            m = saved.matrix(size)
            expected = vision.project(m, camera_marks.mark_positions_mm(paper))
            c = vision.project(m, [(0, 0), (10, 0)])
            radius = MARK_SEARCH_MM * float(((c[1] - c[0]) ** 2).sum() ** 0.5) / 10

        def work():
            try:
                marks, _n = camera_marks.find_corner_marks(full, expected, radius)
                if marks is None and expected is not None:        # 저장된 위치가 많이 틀렸으면 자리 정보 없이 한 번 더
                    marks, _n = camera_marks.find_corner_marks(full)
            except Exception:
                marks = None
            self.app.ui(self._marks_found, name, size, paper, saved, marks)

        threading.Thread(target=work, name="corner-marks", daemon=True).start()

    def _marks_found(self, name, size, paper, saved, marks):
        self._marks_busy = False
        if not self.winfo_exists():
            return
        if marks is None or (self._marks_hist and self._marks_hist[-1][0] != (name, size)):
            self._marks_hist = []
        if marks is None:
            self._marks_miss += 1
            if self._marks_miss >= MARK_CONFIRM and self.session.paper_fit_request is not None:
                self.session.set_paper_fit(None)              # 표시가 없어졌으면 원래 위치로
                self.marks_status.configure(text="모서리 표시가 보이지 않아 원래 위치로 그립니다", text_color=IDLE_C)
            return
        self._marks_miss = 0
        self._marks_hist = (self._marks_hist + [((name, size), marks)])[-MARK_CONFIRM:]
        if len(self._marks_hist) < MARK_CONFIRM:
            return
        stack = [m for _key, m in self._marks_hist]
        if max(float(abs(a - stack[0]).max()) for a in stack) > MARK_STABLE_PX:
            return
        marks = sum(stack) / len(stack)
        stamp = time.strftime("%H:%M:%S")
        if saved is None:
            # 로봇 기준 등록이 없으면 표시로 만든 종이 위치가 유일한 기준 (종이가 호밍 직후 펜 끝 자리에 있다고 가정)
            vision.save(camera_marks.registration_from_marks(name, marks, size, paper))
            self._registration_camera = None           # 다음 틱에서 새 등록을 다시 읽음
            self.overlay_on.set(True)
            self.marks_status.configure(text=f"모서리 표시 4개로 종이 위치를 맞췄습니다 · {stamp}. 종이 중심이 호밍 직후 "
                                             "펜 끝 자리에 있다고 봅니다", text_color=OK_C)
            return
        # 저장된 등록은 로봇이 그리는 자리 기준이라 덮어쓰지 않음. 표시로 잰 실제 종이(자리·회전·크기)에 맞춰
        # 로봇이 그릴 좌표를 옮김(자동 조절을 끄면 벗어난 양만 알림)
        f = camera_marks.paper_frame_mm(saved, marks, size, paper)
        known = f" ({f['paper'][0]}로 봄)" if f["paper"] else ""
        where = (f"{'오른쪽' if f['cx'] >= 0 else '왼쪽'} {abs(f['cx']):.1f} mm, {'위' if f['cy'] >= 0 else '아래'} "
                 f"{abs(f['cy']):.1f} mm, {f['deg']:+.1f}°, 종이 약 {f['width_mm']:.0f}×{f['height_mm']:.0f} mm{known}")
        if not self.fit_auto.get():
            in_place = (f["cx"] ** 2 + f["cy"] ** 2) ** 0.5 <= MARK_UPDATE_MM and abs(f["deg"]) <= MARK_ROT_WARN_DEG
            self.marks_status.configure(text=f"모서리 표시 4개 확인 ({where}) · {stamp}. "
                                             + ("종이가 제자리에 있습니다" if in_place else
                                                "그림은 펜 기준 자리에 그려집니다(자동 조절 꺼짐)"),
                                        text_color=OK_C if in_place else BAD_C)
            return
        want = {"cx": f["cx"], "cy": f["cy"], "deg": f["deg"], "scale": f["scale"]}
        cur = self.session.paper_fit_request
        if cur is None or (abs(cur["cx"] - want["cx"]) > FIT_CHANGE_MM or abs(cur["cy"] - want["cy"]) > FIT_CHANGE_MM
                           or abs(cur["deg"] - want["deg"]) > FIT_CHANGE_DEG
                           or abs(cur["scale"] / want["scale"] - 1) > FIT_CHANGE_SCALE):
            self.session.set_paper_fit(want)              # 조금씩 흔들릴 때마다 시뮬레이션을 다시 하지 않게, 바뀐 만큼만
        used = self.session.paper_fit
        if used is None:
            self.marks_status.configure(text=f"{self.session.paper_fit_error} ({where}) · {stamp}", text_color=BAD_C)
            return
        shrink = f" · 펜 범위에 맞춰 ×{used['range_scale']:.2f}로 더 줄임" if "range_scale" in used else ""
        if f["paper"] is None:
            # 아는 종이 크기와 맞지 않으면 크기는 바꾸지 않고(배율 1) 자리·회전만 맞춤: 등록 배율이 틀렸을 가능성이 큼
            self.marks_status.configure(text=f"모서리 표시에 맞춰 자리·회전만 맞춥니다: {where} · {stamp}. 잰 종이 크기가 "
                                             "아는 크기와 맞지 않아 그림 크기는 그대로 둡니다. 저장된 종이 위치가 틀렸을 수 "
                                             "있으니 다시 맞추세요" + shrink, text_color=BAD_C)
            return
        self.marks_status.configure(text=f"모서리 표시에 맞춰 그립니다: {where}, 그림 ×{used['scale']:.2f}{shrink} · {stamp}",
                                    text_color=OK_C)

    # ------------------------------------------------------------ 크게 보기 · 다른 프로그램과 화면 공유
    def _open_zoom(self):
        if self._zoom_win is not None and self._zoom_win.winfo_exists():
            self._zoom_win.deiconify()
            self._zoom_win.lift()
            return
        from .camera_view_window import CameraViewWindow

        def closed():
            self._zoom_win = None
            self.camera.set_zoom(None)

        self._zoom_win = CameraViewWindow(self, self.font, overlay_var=self.overlay_on, on_close=closed)

    def _full_size(self):
        full = self.camera.full
        return None if full is None else (full.shape[1], full.shape[0])

    def _zoom_box(self, mode, size, strokes):
        """크게 보기에서 자를 원본 부분 (None이면 전체). 종이 위치를 안 맞췄으면 전체."""
        reg = self._registration
        if mode == "full" or reg is None or size is None:
            return None
        key = (mode, size, reg, id(strokes))
        if key != self._zoom_box_key:
            self._zoom_box_key = key
            self._zoom_box_value = vision.view_box(reg, size, strokes if mode == "drawing" else None)
        return self._zoom_box_value

    def _update_zoom(self):
        win = self._zoom_win
        if win is None or not win.winfo_exists():
            return
        strokes, regions = self._plan()
        size = self._full_size()
        self.camera.set_zoom(win.view_width(), self._zoom_box(win.mode, size, strokes))
        img, box = self.camera.latest_zoom()
        if img is not None and size is not None and self._registration is not None and self.overlay_on.get():
            reg = self._registration if box is None else vision.crop_registration(self._registration, size, box)
            colors = self._stroke_colors if strokes is self._run_strokes else None
            img = self._zoom_overlay.draw(img, reg, strokes, regions, colors, self._tick_marks)
        win.show(img, "   ".join(t for t in (self.line_status.cget("text"), self.pen_status.cget("text")) if t))

    def _share_frame(self):
        """다른 프로그램이 카메라를 열지 않고 볼 수 있게 out/camera_live/에 몇 초마다 사진을 씀."""
        name = self._current_camera_name()
        if name is None or not hasattr(self, "prog_text"):
            return
        strokes, regions = self._plan()
        colors = self._stroke_colors if strokes is self._run_strokes else None
        marks = self._tick_marks if strokes is self._run_strokes else ()
        self._share.maybe_write(self.camera, name, self._registration, strokes, colors, marks, regions,
                                {"progress": self.prog_text.cget("text"), "line_check": self.line_status.cget("text"),
                                 "pen_correction": self.pen_status.cget("text"),
                                 "state": self.job.state if self.job else "idle"})

    # ------------------------------------------------------------ 획마다 선 확인 (카메라 사진만, 로봇과 무관)
    def _maybe_check_lines(self):
        if (self._run_strokes and self._drawn > LINE_CHECK_SKIP_LAST and not self._line_busy
                and time.monotonic() - self._line_last >= LINE_CHECK_EVERY_S
                and self.job is not None and self.job.state == "drawing"):
            self._check_lines()

    def _check_lines(self, final=False):
        """작업 스레드에서 최신 원본 사진 몇 장을 평균해 다 그린 획마다 선이 보이는지 잼.
        그리는 동안은 방금 끝난 획 몇 개를 뺌 (아직 펜·팔 밑이거나 사진이 늦음). 가상 시뮬레이션은 종이에 선이 없어 재지 않음."""
        reg, strokes = self._registration, self._run_strokes
        if (reg is None or not strokes or self._line_busy or self.camera.index is None
                or (self.job is not None and self.job.summary.get("virtual"))):
            return
        if final:
            self._line_final_wanted = False
        upto = min(self._drawn - (0 if final else LINE_CHECK_SKIP_LAST), len(strokes))
        if upto <= 0:
            return
        self._line_busy = True
        self._line_last = time.monotonic()
        grab = self.camera.latest_full

        def work():
            result, align, size = None, None, None
            try:
                check = None
                for _ in range(LINE_CHECK_FRAMES):      # 원본(4K 약 25 MB)은 한 장씩만 들고 바로 펴서 더함
                    f = grab()
                    if f is not None:
                        h, w = f.shape[:2]
                        if check is None:
                            check = vision.LineCheck(reg, strokes, (w, h))
                        if (w, h) == check.frame_size:
                            check.add(f)
                    del f
                    time.sleep(0.15)
                if check is not None and check.frames:
                    # 종이를 다시 붙였거나 카메라가 움직여 계획과 실제 선이 어긋나면(이동·회전·배율) 먼저 맞춘 뒤 잰다
                    align = (check.align(upto), check.affine_mm, check.shift_mm, check.rotation_deg, check.scale)
                    size = check.frame_size
                    if align[0] >= vision.ALIGN_MIN_SCORE:
                        result = {i: (check.measure(i), check.end_ticks(i), check.end_tick_dirs(i)) for i in range(upto)}
                    else:
                        result = align
            except Exception:          # 확인은 보조 기능: 실패해도 그리기에는 영향 없음
                result = None
            self.app.ui(self._show_line_check, result, final, strokes, align, reg, size)

        threading.Thread(target=work, name="line-check", daemon=True).start()

    def _show_line_check(self, result, final, strokes, align=None, reg=None, size=None):
        self._line_busy = False
        if not self.winfo_exists() or strokes is not self._run_strokes:
            return
        if result is None:
            self.line_status.configure(text="선 확인: 카메라 사진을 읽지 못했습니다", text_color=IDLE_C)
        elif isinstance(result, tuple):      # 맞춰도 계획과 선이 거의 안 겹침: 판단하지 않음 (보정도 안 함)
            self.line_status.configure(
                text=f"선 확인: 계획과 종이의 선이 맞지 않습니다(겹침 {result[0]:.0%}). 종이 위치를 다시 맞춰 주세요. "
                     "끝점 자동 보정은 하지 않습니다.", text_color=BAD_C)
        else:
            merged = dict(self._line_results)
            ticks = dict(self._tick_results)
            dirs = dict(self._tick_dirs)
            # 이번에 팔에 가린 획은 앞서 보였을 때 잰 값을 그대로 둠 (끝나면 팔이 종이 가운데에 서 있어 가리기 쉬움)
            for i, (cov, tick, tdir) in result.items():
                if cov is not None or merged.get(i) is None:
                    merged[i] = cov
                old = ticks.get(i, (None, None))
                ticks[i] = tuple(new if new is not None else prev for new, prev in zip(tick, old))
                old = dirs.get(i, (None, None))
                dirs[i] = tuple(new if new is not None else prev for new, prev in zip(tdir, old))
            self._tick_results = ticks
            self._tick_dirs = dirs
            self._set_line_results(merged)
            self._correct_pen(final)
            if final:
                self._settle_pen()
                seen, missing, hidden = vision.line_summary(merged)
                note = f"카메라 선 확인: 획 {len(merged)}개 중 보임 {seen} · 안 보임 {missing} · 가려짐 {hidden}"
                t = vision.tick_summary(ticks, merged)
                note += (f"\n끝점 세로 줄: 시작 {t['start'][0]}/{t['start'][1]} · 끝 {t['end'][0]}/{t['end'][1]}"
                         + (f" · {self._pen_note}" if self._pen_note else ""))
                self.detail.configure(text=f"{self.detail.cget('text')}\n{note}".strip())
                self._auto_register(len(result), align, reg, size)
        if final and not isinstance(result, dict):    # 마지막 확인을 못 했어도 바뀐 값은 정리
            self._settle_pen()
        if self._line_final_wanted:           # 끝났을 때 그리는 중 확인이 돌고 있었음
            self._check_lines(final=True)

    def _auto_register(self, n_strokes, align, reg, size):
        """끝난 뒤 마지막 선 확인에서 계획과 실제 선이 확실히 맞았으면(겹침 ALIGN_SAVE_SCORE 이상, 획 ALIGN_SAVE_MIN_STROKES개 이상)
        그때 찾은 변환(이동·회전·배율)으로 종이 위치(등록)를 고쳐 저장한다. 등록은 종이 중심을 호밍 직후 펜 끝으로 보는데,
        종이를 그 자리에 딱 맞춰 붙이지 않거나 카메라가 틀어지면 겹쳐 보기·선 확인이 어긋난다(2026-11-16: +7, −6 mm;
        11-18: +16, −12 mm·−2°·1.06배). 다음 그림부터 맞게 보인다."""
        if (align is None or reg is None or size is None or reg is not self._registration
                or self.job is None or self.job.summary.get("virtual")):
            return
        score, affine, (dx, dy), rot, scale = align
        if score < vision.ALIGN_SAVE_SCORE or n_strokes < vision.ALIGN_SAVE_MIN_STROKES:
            return
        pts = np.vstack([np.asarray(s, float).reshape(-1, 2) for s in self._run_strokes[:n_strokes]])
        box = np.array([[x, y] for x in (pts[:, 0].min(), pts[:, 0].max()) for y in (pts[:, 1].min(), pts[:, 1].max())])
        if np.abs(box @ affine[:, :2].T + affine[:, 2] - box).max() < vision.ALIGN_SAVE_MIN_MM:
            return
        new = vision.corrected_registration(reg, size, affine)
        try:
            kept = vision.backup("autoregister")      # 잘못 맞춘 보정도 되돌릴 수 있게
            vision.save(new)
        except OSError as e:
            self.vision_status.configure(text=f"종이 위치 자동 보정을 저장하지 못했습니다: {e}", text_color=BAD_C)
            return
        self._registration = new
        self.vision_status.configure(
            text=f"종이 위치 자동 보정: 그린 선에 맞춰 {'오른쪽' if dx >= 0 else '왼쪽'} {abs(dx):.1f} mm, "
                 f"{'위' if dy >= 0 else '아래'} {abs(dy):.1f} mm, 회전 {rot:+.1f}°, 크기 ×{scale:.3f}로 고쳤습니다 "
                 f"(겹침 {score:.0%}). 다음 그림부터 겹쳐 보기가 맞습니다."
                 + (f" 예전 위치: {kept.name}" if kept else ""),
            text_color=OK_C)

    def _pen_text(self):
        b = pen_correction.current_backoff(self.cfg)
        rise = pen_correction.current_rise(self.cfg)
        low = pen_correction.current_rise_low(self.cfg)
        land = pen_correction.current_land(self.cfg)
        return (f"끝점 물림: 시작 {b['start']:g} mm · 끝 {b['end']:g} mm"
                + (f" · 비스듬히 {rise:g} mm" if rise else "") + (f" (종이 아래쪽 +{low:g})" if rise and low else "")
                + (f" · 댈 때 {abs(land):g} mm {'위' if land < 0 else '아래'}에서" if land else ""))

    def _correct_pen(self, final):
        """지금 끝점 물림·비스듬히 대고 떼기 값으로 그린 획만 보고 규칙(pen_correction)대로 한 단계 고침.
        그리는 중이면 남은 획에 바로 적용하고, 설정 파일 저장은 끝난 뒤 _settle_pen에서 한 번만 함."""
        if not self.pen_auto.get() or self.job is None or self.job.summary.get("virtual"):
            return
        ticks = {i: t for i, t in self._tick_results.items() if i >= self._pen_from}
        dirs = {i: t for i, t in self._tick_dirs.items() if i >= self._pen_from}
        current = pen_correction.current_backoff(self.cfg)
        baseline = (None if self._pen_prev_from is None else
                    pen_correction.missing_rate(self._line_results, range(self._pen_prev_from, self._pen_from)))
        rise, low = pen_correction.current_rise(self.cfg), pen_correction.current_rise_low(self.cfg)
        land = pen_correction.current_land(self.cfg)
        # 뗄 때 비스듬히가 켜져 있으면 끝점 줄은 뗄 때 값(edge_rise), 시작점 줄은 내릴 때 값(edge_land)이 맡고, 물림은 세로 줄로
        # 올리지 않음. 11-18 15:53 그림: 같은 끝점 줄로 끝점 물림(0.2 → 1.0)과 비스듬히가 함께 움직이다 엇갈림으로 멈췄다.
        new, why = pen_correction.next_backoff(current, ticks, self._line_results, baseline,
                                               kinds=() if rise > 0 else ("start", "end"))
        strokes = self._run_strokes or []
        new_rise, why_rise = pen_correction.next_edge_rise(rise, dirs, self._line_results, strokes)
        new_low, why_low = pen_correction.next_edge_rise_low(low, rise, dirs, self._line_results, strokes)
        new_land, why_land = pen_correction.next_edge_land(land, rise, dirs, self._line_results)
        old = {**current, "rise": rise, "rise_low": low, "land": land}
        values, locked, dirs, flipped = pen_correction.settle_changes(
            self._pen_locked, self._pen_dirs, old, {**new, "rise": new_rise, "rise_low": new_low, "land": new_land})
        if values == old:
            return
        names = {"start": "시작점 물림", "end": "끝점 물림", "rise": "뗄 때 비스듬히", "rise_low": "아래쪽 추가", "land": "댈 때"}
        why = " · ".join(w for w in (why, why_rise, why_low, why_land) if w)
        if flipped:
            why += " · 판단이 오가서 가운데 값으로 고정: " + ", ".join(f"{names[k]} {values[k]:g} mm" for k in flipped)
        if final:
            applied = len(self._run_strokes or ())
        else:
            applied = self.job.set_pen_correction(values)
            if applied is None:
                return
        self._pen_dirs, self._pen_locked = dirs, locked
        self.cfg["pen"]["start_backoff_mm"], self.cfg["pen"]["end_backoff_mm"] = values["start"], values["end"]
        self.cfg["pen"]["edge_rise_mm"], self.cfg["pen"]["edge_rise_low_mm"] = values["rise"], values["rise_low"]
        self.cfg["pen"]["edge_land_mm"] = values["land"]
        self._pen_prev_from, self._pen_from = self._pen_from, applied
        self._pen_note = why + ("" if final else f" (획 {applied + 1}부터)")
        self.pen_status.configure(text=f"{self._pen_text()}\n{self._pen_note}", text_color=ACT_C)

    def _settle_pen(self):
        """그림이 끝난 뒤 한 번: 자동 보정으로 바뀐 값을 설정 파일에 저장 (그리는 중에는 저장하지 않음)."""
        start = self._pen_start
        if start is None:
            return
        self._pen_start = None
        now = {k: self.cfg["pen"].get(k, 0.0) for k in start}
        if now == start:
            return
        try:
            pen_correction.save_pen({"start": now["start_backoff_mm"], "end": now["end_backoff_mm"],
                                     "rise": now["edge_rise_mm"], "rise_low": now["edge_rise_low_mm"],
                                     "land": now["edge_land_mm"]})
            note = "바뀐 값을 설정 파일에 저장함"
        except (OSError, ValueError, KeyError) as e:
            note = f"설정 저장 실패: {e}"
        self._pen_note = f"{self._pen_note} · {note}".strip(" ·")
        self.pen_status.configure(text=f"{self._pen_text()}\n{self._pen_note}", text_color=ACT_C)

    def _set_line_results(self, results):
        self._line_results = results
        n = len(self._run_strokes or ())
        if not results:
            self._stroke_colors = None
            self._tick_marks = ()
            self.line_status.configure(text="")
            return
        colors = [vision.STROKE_COLOR] * n
        for i, v in results.items():
            if v is not None and i < n:
                colors[i] = vision.LINE_SEEN_COLOR if v >= vision.LINE_SEEN else vision.LINE_MISSING_COLOR
        self._stroke_colors = colors
        marks = []
        for i, tick in self._tick_results.items():
            if i < n:
                for k, length in enumerate(tick):
                    if length is not None and length >= vision.TICK_MIN_MM:
                        marks.append((self._run_strokes[i][0 if k == 0 else -1], vision.TICK_COLOR))
        self._tick_marks = tuple(marks)
        seen, missing, hidden = vision.line_summary(results)
        t = vision.tick_summary(self._tick_results, results)
        self.line_status.configure(
            text=f"선 확인 (다 그린 획 {len(results)}개): 보임 {seen} · 안 보임 {missing} · 가려짐 {hidden}\n"
                 f"끝점 세로 줄: 시작 {t['start'][0]}/{t['start'][1]} · 끝 {t['end'][0]}/{t['end'][1]}",
            text_color=BAD_C if missing > seen else OK_C)

    def _save_vision_snapshot(self):
        """끝난 뒤 카메라 사진과 계획을 겹친 사진을 out/에 저장. 종이 위치를 안 맞췄거나 사진이 없으면 None."""
        frame = self.camera.latest_full()
        if frame is None or self._registration is None:
            return None
        from datetime import datetime

        from . import paths
        strokes, regions = self._plan()
        path = paths.output_dir() / f"vision_{datetime.now():%Y%m%d-%H%M%S}.png"
        try:
            return vision.save_snapshot(frame, self._registration, strokes, regions, path)
        except OSError:
            return None

    def _show_camera_frame(self, frame):
        from PIL import Image
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        width = CAMERA_PANEL_W - 28
        self._camera_photo = ctk.CTkImage(image, size=(width, max(80, int(width * image.height / image.width))))
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
        self._run_strokes = None
        self._drawn = 0
        self._line_final_wanted = False
        self._tick_results = {}
        self._tick_dirs = {}
        self._pen_from, self._pen_prev_from = 0, None
        self._pen_note = ""
        self._pen_dirs, self._pen_locked = {}, set()
        self.pen_status.configure(text=self._pen_text(), text_color=IDLE_C)
        self._set_line_results({})
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

    def _on_air_changed(self):
        if self.job is not None and self.job.state == "confirm":
            self._show_confirm_detail()

    def _show_confirm_detail(self):
        s = self.job.summary
        timing = self.job.timing_for(bool(self.air_var.get()))
        self.detail.configure(text=f"획 {s['stroke_count']} · 명령 {timing['command_count']:,}줄 · "
                                   f"예상 {timing['total_s'] / 60:.1f}분 · {s['drawing_mm'][0]:.0f}×"
                                   f"{s['drawing_mm'][1]:.0f}mm · {s['port']}"
                                   + ("" if s["virtual"] or s["plane_verified"] else
                                      " · 종이 보정 없음(설정에 저장된 값 사용)")
                                   + ("" if s["virtual"] or s.get("start_center_shift_mm", 0) < 0.05 else
                                      f" · 시작 자세를 종이 중심으로 사용(설정 파일 값과 {s['start_center_shift_mm']:g} mm 차이)"))

    def _on_check(self):
        ready = self.job is not None and self.job.state == "confirm" and self.check_var.get()
        self.start_btn.configure(state="normal" if ready else "disabled")

    def on_start(self):
        if self.job and self.job.state == "paused":          # 일시정지 중에는 이 버튼이 [▶ 이어서 그리기]
            self.job.resume()
            self.start_btn.configure(state="disabled")
            return
        self.job.confirm(air=self.air_var.get(), pending=self.pending_var.get(), checked=self.check_var.get())
        self.start_btn.configure(state="disabled")
        self.check_box.configure(state="disabled")
        self.stop_btn.configure(state="normal", text="■ 멈춤")

    def on_stop(self):
        if not self.job:
            return
        if self.job.state == "paused":                       # 일시정지 중에는 이 버튼이 [끝내기]
            self.job.finish()
            self.start_btn.configure(state="disabled", text="시작")
            if self.return_var.get() and self.cfg.get("requested_start_pose") and self.start_pose_var.get():
                self.stop_btn.configure(text="■ 그 자리에서 멈춤")   # 호밍 자세로 돌아가는 중에 누르면
            else:
                self.stop_btn.configure(state="disabled")
            return
        drawing = self.job.state == "drawing"
        self.job.stop()
        if drawing:
            self.stop_btn.configure(text="■ 그 자리에서 멈춤")      # 펜을 떼는 중에 한 번 더 누르면

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
            self.job.end()
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
        if self._zoom_win is not None and self._zoom_win.winfo_exists():
            self._zoom_win.close()
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
                self._show_confirm_detail()
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
        elif kind == "plan":
            self._run_strokes = [list(s) for s in d["strokes"]]
            self._drawn = 0
            self._tick_results = {}
            self._tick_dirs = {}
            self._pen_from, self._pen_prev_from = 0, None
            self._pen_dirs, self._pen_locked = {}, set()
            self._pen_start = {k: self.cfg["pen"].get(k, 0.0)
                               for k in ("start_backoff_mm", "end_backoff_mm", "edge_rise_mm", "edge_rise_low_mm",
                                         "edge_land_mm")}
            self._set_line_results({})
        elif kind == "progress":
            self.progress.set(d["acked"] / max(d["total"], 1))
            self.prog_text.configure(text=f"명령 {d['acked']:,}/{d['total']:,} ({100 * d['acked'] // d['total']}%) · "
                                          f"획 {d['stroke']}/{d['strokes']} · 경과 {_fmt(d['elapsed_s'])} · "
                                          f"남은 시간 약 {_fmt(d['remaining_s'])}")
            self._drawn = d.get("drawn", 0)
            self.app.show_draw_progress(d["acked"], d["total"])
        elif kind == "paused":
            self.start_btn.configure(state="normal", text="▶ 이어서 그리기")
            self.stop_btn.configure(state="normal", text="⌂ 끝내기")
            self.detail.configure(text=f"명령 {d['acked']:,}/{d['total']:,}에서 멈췄습니다. 이어서 그리면 그은 자리를 "
                                       "다시 긋지 않고 멈춘 점에 펜을 다시 대고 계속합니다.")
        elif kind == "resumed":
            self.start_btn.configure(state="disabled", text="시작")
            self.stop_btn.configure(state="normal", text="■ 멈춤")
            self.detail.configure(text="")
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
                                else {"completed": done_text,
                                      "stopped_by_user": "멈췄습니다" + {True: f". 로봇이 {return_name}(관절 0)로 돌아왔습니다.",
                                                                       False: f" — {return_name}로 돌아가지 못했습니다.",
                                                                       None: " — 펜을 종이에서 떼고 그 자리에 두었습니다."
                                                                       if d["result"].get("pen_lifted") else " (자동 복구 없음)"}[returned],
                                      "cancelled": "취소했습니다"}.get(r, f"끝: {r}"))
            note = getattr(self, "rviz_note", "")
            detail = ""
            if d.get("record_path"):
                detail = f"실행 기록: {d['record_path']}" + (f"\n{note}" if note else "")
            if returned is False:
                detail += f"\n{d['result'].get('return_error', '')} 다음 그림 전에 호밍하거나 펜 위치를 확인하세요."
            # 그리기를 시작했던 실행이면 카메라 사진 + 계획 겹침을 남김 (사진·작업을 비우기 전에)
            if r in ("completed", "stopped_by_user", "stopped_on_error", "interrupted_by_user"):
                shot = self._save_vision_snapshot()
                if shot is not None:
                    detail += f"\n카메라 기록: {shot}"
            self.start_btn.configure(state="disabled", text="시작")
            self.check_box.configure(state="disabled")
            self.app.set_drawing(False)
            self.begin_btn.configure(state="normal")
            # 끝까지 그렸고 펜도 처음 자리로 돌아왔으면 사진·작업을 비워 새 그림을 바로 시작할 수 있게 (옵션)
            if r == "completed" and returned is not False and self.clear_var.get():
                self.app.reset_for_next_drawing()
                detail += "\n사진과 작업 내용을 비웠습니다. 종이를 바꾸고 새 사진을 열어 주세요."
            if detail:
                self.detail.configure(text=detail.strip())
            if (self._run_strokes and r in ("completed", "stopped_by_user", "stopped_on_error", "interrupted_by_user")
                    and self._registration is not None and self.camera.index is not None):
                self._line_final_wanted = True   # 마지막으로 다 그린 획 전부를 한 번 더 잼 (결과는 위 안내 끝에 붙음)
                self._check_lines(final=True)    # 바뀐 펜 값은 그 확인이 끝난 뒤 _settle_pen에서 정리
            else:
                self._settle_pen()
