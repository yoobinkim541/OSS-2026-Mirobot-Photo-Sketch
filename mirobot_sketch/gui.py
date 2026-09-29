"""
Mirobot Sketch — 사진 -> 스케치 -> 로봇용 획(stroke) 데이터 GUI
===============================================================
사진을 열고 이미지 종류를 고르면 추천 설정이 들어갑니다. 결과는 A4 종이 위에
실제 크기와 펜 굵기로 미리 보여 주고, 예상 시간과 로봇 시뮬레이션(관절 한계)
결과를 함께 표시합니다. 내보낸 JSON은 그대로 robot/draw_executor.py에 넣습니다.

    mirobot-sketch                         (저장소에서는 python CV/gui_sketch.py)
    mirobot-draw <저장한.json>             (dry-run)
    mirobot-draw <저장한.json> --execute   (실제 드로잉)

화면: CustomTkinter (pip install customtkinter)
처리 로직: sketch_pipeline.py / paper_mapping.py / presets.py
명령줄 버전: make_strokes.py
"""

import gc
import json
import queue
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox

import matplotlib
import numpy as np
from matplotlib import font_manager
from matplotlib.collections import LineCollection

try:
    import customtkinter as ctk
except ImportError:  # 설치 안내 후 종료
    print("CustomTkinter가 필요합니다:  pip install customtkinter")
    raise

from . import draw_executor as de
from . import limits, robot_status
from . import paper_mapping as pm
from . import paths, presets, stages
from . import sketch_pipeline as sp
from .session import SketchSession
from .stage_view import BigView, ParamControls, ProposalBar, StageStrip

# 한글이 네모로 깨지지 않도록 설치된 한글 글꼴 사용 (Windows: 맑은 고딕)
FONT = "Malgun Gothic"
for _font in ("Malgun Gothic", "AppleGothic", "NanumGothic", "Noto Sans CJK KR"):
    if any(f.name == _font for f in font_manager.fontManager.ttflist):
        FONT = _font
        matplotlib.rcParams["font.family"] = _font
        break
matplotlib.rcParams["axes.unicode_minus"] = False

ACCENT = ("#2563eb", "#3b82f6")          # (라이트, 다크)
CARD = ("#ffffff", "#1f2430")
BG = ("#eef1f6", "#141821")
MUTED = ("#6b7280", "#9aa3b2")
TEXT = ("#1f2937", "#e5e7eb")
OK, WARN, BAD = "#16a34a", "#d97706", "#dc2626"

TYPE_KEYS = list(presets.IMAGE_TYPES)                          # photo / illustration / manga
TYPE_LABELS = [presets.IMAGE_TYPES[k]["label"] for k in TYPE_KEYS]
DETAIL_KEYS = ["low", "medium", "high"]
DETAIL_LABELS = ["낮음", "보통", "높음"]
LABEL_MAX = 400            # 편집 단계 번호 딱지 최대 개수 (화면을 덮지 않고 빠르게)
RECOMPUTE_DELAY_MS = 300   # 값을 바꾸고 이만큼 조용하면 다시 계산 (슬라이더를 끄는 동안 계속 계산하지 않게)


def font(size=13, weight="normal"):
    return ctk.CTkFont(family=FONT, size=size, weight=weight)


class Card(ctk.CTkFrame):
    """제목이 있는 둥근 카드."""

    def __init__(self, master, title=None, **kw):
        super().__init__(master, fg_color=CARD, corner_radius=14, **kw)
        if title:
            ctk.CTkLabel(self, text=title, font=font(14, "bold"), anchor="w").pack(fill="x", padx=14, pady=(12, 4))


class StatCard(ctk.CTkFrame):
    """아래쪽 요약 카드: 제목 / 큰 값 / 설명."""

    def __init__(self, master, title):
        super().__init__(master, fg_color=CARD, corner_radius=14)
        ctk.CTkLabel(self, text=title, font=font(12), text_color=MUTED, anchor="w").pack(fill="x", padx=14, pady=(10, 0))
        self.value = ctk.CTkLabel(self, text="—", font=font(24, "bold"), anchor="w")
        self.value.pack(fill="x", padx=14)
        self.sub = ctk.CTkLabel(self, text="", font=font(11), text_color=MUTED, anchor="w", justify="left",
                                wraplength=260)
        self.sub.pack(fill="x", padx=14, pady=(0, 10))

    def set(self, value, sub="", color=None):
        self.value.configure(text=value, text_color=color or ("#111827", "#f3f4f6"))
        self.sub.configure(text=sub)


class SketchApp:
    def __init__(self, root):
        self.root = root
        self._closing = False
        self._ui_after = self._usb_after = self._icon_after = None
        self.root.title("Mirobot Sketch")
        self.root.geometry("1560x940")
        self.root.minsize(1200, 760)
        self.root.configure(fg_color=BG)
        self._set_icon()

        self.cfg = de.load_config()
        self._robot_usb_scan_busy = False
        # 이미지·설정·결과·편집 기록은 세션 하나에 둠 (화면 버튼과 에이전트 도구가 함께 사용)
        self.session = SketchSession(self.cfg)
        self.busy = False
        self._agent_busy = False
        self.stage_id = "source"
        self._workers = 0              # 돌고 있는 재계산 스레드 수
        self._recompute_after = None   # 예약된 재계산 (after id)
        self._recompute_pending = False  # 에이전트 작업 중 들어온 재계산 요청 (끝나면 실행)
        self.drawing = False             # 로봇으로 그리는 중 (조절·편집·재계산 잠금)
        self.draw_window = None
        self.rviz_setup_window = None
        # 작업 스레드 -> 화면: Tkinter는 스레드에 안전하지 않으므로 작업 스레드는 큐에
        # 할 일만 넣고, 메인 스레드가 주기적으로 꺼내 실행한다.
        self._ui_queue = queue.Queue()

        self._build_ui()
        self.apply_type_preset()
        self._init_agent()
        self._poll_ui_queue()
        self._usb_after = self.root.after(0, self._poll_robot_usb)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # 세션에서 읽는 값
    @property
    def result(self):
        return self.session.result

    @property
    def sim_result(self):
        return self.session.sim["raw"] if self.session.sim else None

    @property
    def img_path(self):
        return self.session.image_path

    def ui(self, fn, *args):
        """에이전트 패널 등 다른 모듈이 화면 갱신을 요청할 때 사용 (스레드 안전)."""
        self._ui(fn, *args)

    def _ui(self, fn, *args):
        """작업 스레드에서 화면 갱신을 요청할 때 사용."""
        self._ui_queue.put((fn, args))

    def _poll_ui_queue(self):
        if self._closing:
            return
        try:
            while True:
                fn, args = self._ui_queue.get_nowait()
                fn(*args)
        except queue.Empty:
            pass
        self._ui_after = self.root.after(50, self._poll_ui_queue)

    # ------------------------------------------------------------------ 창
    def _set_icon(self):
        """창·작업표시줄 아이콘 (assets/make_icon.py로 생성). 없으면 기본 아이콘."""
        try:
            if sys.platform.startswith("win") and paths.asset("app_icon.ico").exists():
                # CustomTkinter가 시작 직후 자기 아이콘으로 덮어써서 조금 뒤에 다시 설정
                self._icon_after = self.root.after(
                    250, lambda: self.root.iconbitmap(str(paths.asset("app_icon.ico"))))
            if paths.asset("app_icon_256.png").exists():
                self._icon_img = tk.PhotoImage(file=str(paths.asset("app_icon_256.png")))
                self.root.iconphoto(True, self._icon_img)
        except tk.TclError:
            pass

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(1, weight=1)

        # --- 머리글
        header = ctk.CTkFrame(self.root, fg_color="transparent")
        header.grid(row=0, column=0, columnspan=3, sticky="ew", padx=18, pady=(14, 6))
        if paths.asset("app_icon_256.png").exists():
            from PIL import Image
            logo = ctk.CTkImage(Image.open(paths.asset("app_icon_256.png")), size=(40, 40))
            ctk.CTkLabel(header, image=logo, text="").pack(side="left", padx=(0, 10))
        title = ctk.CTkFrame(header, fg_color="transparent")
        title.pack(side="left")
        ctk.CTkLabel(title, text="Mirobot Sketch", font=font(22, "bold"), anchor="w").pack(anchor="w")
        ctk.CTkLabel(title, text="사진을 로봇 팔이 그릴 수 있는 선으로", font=font(12), text_color=MUTED,
                     anchor="w").pack(anchor="w")
        self.mode = ctk.CTkSegmentedButton(header, values=["라이트", "다크"], command=self.set_mode, font=font(12))
        self.mode.set("다크" if ctk.get_appearance_mode() == "Dark" else "라이트")
        self.mode.pack(side="right")
        self.agent_btn = ctk.CTkButton(header, text="✦ 에이전트", command=self.toggle_agent, font=font(13),
                                       height=32, width=110, fg_color="transparent", border_width=1,
                                       text_color=TEXT)
        self.agent_btn.pack(side="right", padx=(0, 10))
        self.robot_usb_status = ctk.CTkLabel(header, text="USB: 확인 중", font=font(11), text_color=MUTED)
        self.robot_usb_status.pack(side="right", padx=(0, 12))
        self.robot_controller_status = ctk.CTkLabel(header, text="제어기 상태: 미확인", font=font(11),
                                                    text_color=MUTED)
        self.robot_controller_status.pack(side="right", padx=(0, 10))

        # --- 왼쪽 설정 패널
        side = ctk.CTkScrollableFrame(self.root, width=340, fg_color="transparent")
        side.grid(row=1, column=0, sticky="ns", padx=(18, 8), pady=(0, 18))

        c1 = Card(side, "① 이미지")
        c1.pack(fill="x", pady=(0, 10))
        self.open_btn = ctk.CTkButton(c1, text="이미지 열기", command=self.open_image, font=font(13), height=36,
                                      fg_color=ACCENT)
        self.open_btn.pack(fill="x", padx=14)
        self.path_label = ctk.CTkLabel(c1, text="선택된 파일 없음", font=font(11), text_color=MUTED,
                                       wraplength=300, anchor="w", justify="left")
        self.path_label.pack(fill="x", padx=14, pady=(4, 8))
        ctk.CTkLabel(c1, text="이미지 종류 (추천 설정 적용)", font=font(12), anchor="w").pack(fill="x", padx=14)
        self.type_seg = ctk.CTkSegmentedButton(c1, values=TYPE_LABELS, command=lambda _: self.apply_type_preset(),
                                               font=font(12))
        self.type_seg.set(presets.IMAGE_TYPES["illustration"]["label"])
        self.type_seg.pack(fill="x", padx=14, pady=(4, 2))
        self.type_hint = ctk.CTkLabel(c1, text="", font=font(11), text_color=MUTED, wraplength=300,
                                      anchor="w", justify="left")
        self.type_hint.pack(fill="x", padx=14, pady=(2, 12))

        c2 = Card(side, "② 상세도")
        c2.pack(fill="x", pady=(0, 10))
        self.detail_seg = ctk.CTkSegmentedButton(c2, values=DETAIL_LABELS, command=lambda _: self.apply_detail_preset(),
                                                 font=font(12))
        self.detail_seg.set("높음")
        self.detail_seg.pack(fill="x", padx=14, pady=(2, 12))

        self.ctrl_card = Card(side)
        self.ctrl_card.pack(fill="x", pady=(0, 10))
        self.ctrl_title = ctk.CTkLabel(self.ctrl_card, text="", font=font(14, "bold"), anchor="w")
        self.ctrl_title.pack(fill="x", padx=14, pady=(12, 0))
        self.controls = None

        c3 = Card(side, "③ 실행")
        c3.pack(fill="x", pady=(0, 10))
        self.run_btn = ctk.CTkButton(c3, text="지금 다시 계산", command=lambda: self._schedule_recompute(0),
                                     font=font(13), height=36, fg_color=ACCENT)
        self.run_btn.pack(fill="x", padx=14, pady=(2, 6))
        self.sim_btn = ctk.CTkButton(c3, text="로봇 시뮬레이션 (관절 한계)", command=self.simulate, font=font(13),
                                     height=36, fg_color="transparent", border_width=2, text_color=TEXT)
        self.sim_btn.pack(fill="x", padx=14, pady=3)
        self.draw_btn = ctk.CTkButton(c3, text="로봇으로 그리기", command=self.open_draw_window, font=font(13, "bold"),
                                      height=38, fg_color="#16a34a", hover_color="#15803d")
        self.draw_btn.pack(fill="x", padx=14, pady=3)
        row = ctk.CTkFrame(c3, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(3, 6))
        ctk.CTkButton(row, text="JSON 내보내기", command=self.export_strokes, font=font(12), height=32,
                      fg_color="transparent", border_width=1, text_color=TEXT
                      ).pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.traj_btn = ctk.CTkButton(row, text="RViz 3D로 보기", command=self.view_rviz, font=font(12), height=32,
                                      fg_color="transparent", border_width=1, text_color=TEXT, state="disabled")
        self.traj_btn.pack(side="left", fill="x", expand=True, padx=(4, 0))
        ctk.CTkButton(c3, text="RViz 3D 환경 설치·확인…", command=self.open_rviz_setup, font=font(12), height=28,
                      fg_color="transparent", border_width=1, text_color=TEXT).pack(fill="x", padx=14, pady=(0, 3))
        self.progress = ctk.CTkProgressBar(c3, mode="indeterminate", height=6)
        self.progress.pack(fill="x", padx=14, pady=(6, 4))
        self.progress.set(0)
        self.status = ctk.CTkLabel(c3, text="이미지를 열어 주세요.", font=font(12), text_color=ACCENT,
                                   wraplength=300, anchor="w", justify="left")
        self.status.pack(fill="x", padx=14, pady=(0, 12))
        lim = limits.executor_region(self.cfg).x_max * 2
        plim = limits.pending_region(self.cfg).x_max * 2
        ctk.CTkLabel(c3, text=f"종이 미리보기: 파란 점선 {lim:.0f}mm = 실행기 허용 · 주황 점선(최대 폭 {plim:.0f}mm) = "
                              "넓은 범위(실물 확인 전, 위쪽이 낮은 지붕 모양)",
                     font=font(11), text_color=MUTED, wraplength=300, anchor="w", justify="left"
                     ).pack(fill="x", padx=14, pady=(0, 12))

        # --- 오른쪽: 단계 띠 + 큰 보기 + 요약 카드
        right = ctk.CTkFrame(self.root, fg_color="transparent")
        right.grid(row=1, column=1, sticky="nsew", padx=(8, 18), pady=(0, 18))
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)
        self.strip = StageStrip(right, stages.ALL_STAGES, self.select_stage, font,
                                titles={st.id: stages.stage_title(st.id) for st in stages.ALL_STAGES})
        self.strip.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.view_card = Card(right)
        self.view_card.grid(row=1, column=0, sticky="nsew")
        self.view = BigView(self.view_card, font, self._theme_colors)
        self.view.pack(fill="both", expand=True)
        self.show_numbers = tk.BooleanVar(value=True)
        self.show_cands = tk.BooleanVar(value=False)
        for text, var in (("번호", self.show_numbers), ("버린 선", self.show_cands)):
            ctk.CTkCheckBox(self.view.options_frame, text=text, variable=var, font=font(11), width=20,
                            command=self.view.redraw).pack(side="left", padx=4)
        self.view.on_view_change = self.view.redraw     # 확대·이동하면 보이는 획만 번호를 다시 붙임
        self.proposal_bar = ProposalBar(self.view_card, font, self._apply_proposals, self._discard_proposals)

        stats = ctk.CTkFrame(right, fg_color="transparent")
        stats.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        for i in range(4):
            stats.grid_columnconfigure(i, weight=1, uniform="stat")
        self.st_strokes = StatCard(stats, "획 / 명령")
        self.st_size = StatCard(stats, "그림 크기")
        self.st_time = StatCard(stats, "예상 시간")
        self.st_sim = StatCard(stats, "로봇 시뮬레이션")
        for i, w in enumerate((self.st_strokes, self.st_size, self.st_time, self.st_sim)):
            w.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 5, 0 if i == 3 else 5))
        self._build_controls()
        self.strip.select(self.stage_id)

    # ---------------------------------------------------------- 그림 테마
    def _theme_colors(self):
        dark = ctk.get_appearance_mode() == "Dark"
        return ("#1f2430", "#e5e7eb") if dark else ("#ffffff", "#111827")

    def set_mode(self, value):
        ctk.set_appearance_mode("Light" if value == "라이트" else "Dark")
        self.view.redraw()

    # -------------------------------------------------------------- 프리셋·조절 칸
    def _type_key(self):
        return TYPE_KEYS[TYPE_LABELS.index(self.type_seg.get())]

    def _detail_key(self):
        return DETAIL_KEYS[DETAIL_LABELS.index(self.detail_seg.get())]

    def _build_controls(self):
        if self.controls is not None:
            self.controls.destroy()
            self.controls = None
            # 버린 조절 칸의 tk 변수(순환 참조)를 메인 스레드에서 바로 정리. 두면 작업 스레드의 가비지 컬렉션이
            # 정리하다 "main thread is not in main loop" 경고를 냄
            gc.collect()
        st = stages.STAGE_BY_ID[self.stage_id]
        specs = [self.session.param_specs()[p.key] for p in st.params]
        self.ctrl_title.configure(text=f"조절: {st.label}" + ("" if specs else " (조절 항목 없음)"))
        self.controls = ParamControls(self.ctrl_card, specs, self.session.params, self._on_param, font)
        self.controls.pack(fill="x", pady=(0, 12))
        self.controls.set_enabled(not self._agent_busy)

    def apply_type_preset(self):
        t = presets.IMAGE_TYPES[self._type_key()]
        self.type_hint.configure(text=t["why"])
        self.session.apply_preset(self._type_key())
        self._sync_controls_from_session()
        self._schedule_recompute()

    def apply_detail_preset(self):
        self.session.apply_preset(detail=self._detail_key())
        self._sync_controls_from_session()
        self._schedule_recompute()

    def _on_param(self, key, value):
        if self._agent_busy or self.drawing:
            return
        self.session.update_params({key: value})
        self._schedule_recompute()

    # ---------------------------------------------------------------- 동작
    def open_image(self):
        path = filedialog.askopenfilename(
            initialdir=str(paths.input_dir()),
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.jfif *.bmp *.webp")])
        if path:
            self.load_image(path)

    def load_image(self, path):
        try:
            self.session.set_image(path)
        except ValueError as e:
            messagebox.showerror("오류", str(e))
            return
        self.traj_btn.configure(state="disabled")
        self.path_label.configure(text=Path(path).name)
        self.strip.set_thumbnail("source", self.session.color)
        self.select_stage("source")
        self._schedule_recompute(0)

    def select_stage(self, stage_id):
        self.stage_id = stage_id
        self.strip.select(stage_id)
        self._build_controls()
        self._show_stage()

    def _show_stage(self):
        s = self.session
        st = stages.STAGE_BY_ID[self.stage_id]
        title = stages.stage_title(self.stage_id)
        change = s.stage_change(self.stage_id)
        self.view.set_info(st.desc + (f"\n{change}" if change else ""), self._compare_target())
        if s.result is None:
            if s.color is not None:
                self.view.show(f"{title} (계산 전)", s.color)
            return
        overlay = None if self.stage_id in ("source", "paper") else s.result["color"]   # 구도를 자른 작업 이미지
        if self.stage_id == "edit":
            blank = np.full((*s.result["base"].shape, 3), 255, np.uint8)
            self.view.show(f"{title} (빨강=사라짐 · 초록=생김)", blank, overlay, draw_extra=self._draw_edit)
        else:
            self.view.show(title, s.render(self.stage_id), overlay)
        # 번호·버린 선 체크박스는 편집 단계에서만 보임
        if self.stage_id == "edit":
            self.view.options_frame.pack(side="right", padx=8)
        else:
            self.view.options_frame.pack_forget()

    def _compare_target(self):
        """이전 단계 (제목, 그림 함수). 그림 크기가 같은 단계끼리만 (원본·종이는 비교 없음)."""
        ids = [st.id for st in stages.ALL_STAGES]
        k = ids.index(self.stage_id)
        if self.session.result is None or k == 0 or self.stage_id == "paper":
            return None
        prev = ids[k - 1]
        return stages.stage_title(prev), lambda: self.session.render(prev)

    def _draw_edit(self, ax):
        """편집 단계 벡터 그림. 선은 색마다 LineCollection 하나로(제안이 수천 개여도 빠르게), 번호 딱지는
        보이는 범위 안에서 제안 먼저 최대 LABEL_MAX개."""
        s = self.session
        ink = "#374151"   # 편집 그림은 흰 바탕(종이)이라 테마와 상관없이 진한 회색
        x0, y0, x1, y1 = self.view.view_rect()
        px_per_unit = ax.get_window_extent().width / max(x1 - x0, 1e-6)
        views = s.proposal_views() if s.proposals else []
        proposed = {v["id"] for v in views}
        strokes, cands, labels = [], [], []
        for i, e in sorted(s.table.items()):
            if e["kind"] == "stroke":
                strokes.append(e["poly"])
                if i not in proposed:
                    labels.append((i, e["poly"], ink))
            elif e["kind"] == "candidate" and self.show_cands.get():
                cands.append(e["poly"])
                if i not in proposed:
                    labels.append((i, e["poly"], "#9ca3af"))
        ax.add_collection(LineCollection(strokes, colors=ink, linewidths=0.9))
        if cands:
            ax.add_collection(LineCollection(cands, colors="#9ca3af", linewidths=0.8, linestyles="dashed"))
        before = [v["before"] for v in views if v["before"] is not None]
        after = [v["after"] for v in views if v["after"] is not None]
        if before:
            ax.add_collection(LineCollection(before, colors="#dc2626", linewidths=2.6))
        if after:
            ax.add_collection(LineCollection(after, colors="#16a34a", linewidths=2.6))

        def in_view(poly):
            mid = poly[len(poly) // 2]
            return x0 <= mid[0] <= x1 and y0 <= mid[1] <= y1, mid

        n = 0
        for v in views:   # 제안 딱지 우선
            if n >= LABEL_MAX:
                break
            poly = v["after"] if v["after"] is not None else v["before"]
            ok, mid = in_view(poly)
            if ok:
                ax.text(mid[0], mid[1], str(v["id"]), fontsize=9, fontweight="bold", clip_on=True,
                        color="white", bbox={"boxstyle": "round,pad=0.2", "lw": 0,
                                             "fc": "#16a34a" if v["after"] is not None else "#dc2626"})
                n += 1
        if self.show_numbers.get():
            for i, poly, color in labels:
                if n >= LABEL_MAX:
                    break
                ok, mid = in_view(poly)
                if ok and np.hypot(*np.diff(poly, axis=0).T).sum() * px_per_unit >= 20:
                    ax.text(mid[0], mid[1], str(i), fontsize=8, color=color, clip_on=True)
                    n += 1

    def refresh_proposals(self):
        views = self.session.proposal_views() if self.session.proposals else []
        if views:
            self.proposal_bar.set_views(views, self.session.proposal_epoch)
            self.proposal_bar.pack(fill="x", padx=8, pady=(0, 8))
        else:
            self.proposal_bar.pack_forget()
        if self.stage_id == "edit":
            self.view.redraw()

    def _apply_proposals(self, exclude):
        """적용(순서 정하기·미리보기 다시 그리기)은 획이 많으면 오래 걸려 작업 스레드에서."""
        if self.drawing:
            messagebox.showinfo("알림", "로봇이 그리는 중에는 편집을 적용할 수 없습니다.")
            return
        if self._workers == 0:
            self.progress.configure(mode="indeterminate")
            self.progress.start()
        self._workers += 1
        self._set_status("편집 적용 중...")

        def work():
            try:
                out = self.session.apply_proposals(exclude=exclude)
                msg = f"편집 {len(out['applied'])}건 적용. {out['note']}".strip()
                self._ui(self._show_result, self.session.result, msg)
            except Exception as e:  # SessionError 포함 (예: 모든 획 삭제)
                self._ui(messagebox.showerror, "적용 실패", str(e))
                self._set_status("적용하지 못했습니다.")
            finally:
                self._ui(self._worker_done)

        threading.Thread(target=work, daemon=True).start()

    def _discard_proposals(self):
        self.session.discard_proposals()
        self.refresh_proposals()

    def _sync_controls_from_session(self):
        """에이전트·프리셋이 세션 설정을 바꿨을 때 화면 선택·조절 칸을 맞춤."""
        s = self.session
        self.type_seg.set(presets.IMAGE_TYPES[s.image_type]["label"])
        self.type_hint.configure(text=presets.IMAGE_TYPES[s.image_type]["why"])
        self.detail_seg.set(DETAIL_LABELS[DETAIL_KEYS.index(s.detail)])
        if self.controls is not None:
            self.controls.set_values(s.params)

    def _refresh_from_session(self, what="result"):
        if what == "proposals":
            self.refresh_proposals()
            return
        self._sync_controls_from_session()
        if self.session.result is not None:
            self._show_result(self.session.result, status="에이전트가 결과를 바꿨습니다.")
        if self.session.sim:
            self._show_sim(self.session.sim["summary"])

    def agent_busy(self, busy):
        """에이전트가 작업 중이거나 로봇이 그리는 중이면 처리·조절·시뮬레이션을 잠금 (같은 세션을 동시에 바꾸지 않게)."""
        self._agent_busy = busy
        locked = busy or self.drawing
        state = "disabled" if locked or self.busy else "normal"
        self.run_btn.configure(state=state)
        self.sim_btn.configure(state=state)
        if self.controls is not None:
            self.controls.set_enabled(not locked)
        for seg in (self.type_seg, self.detail_seg):
            seg.configure(state="disabled" if locked else "normal")
        if not locked and self._recompute_pending:
            self._recompute_pending = False
            self._schedule_recompute(0)

    def _start_busy(self, text):
        self.busy = True
        self.run_btn.configure(state="disabled")
        self.sim_btn.configure(state="disabled")
        self.progress.configure(mode="indeterminate")
        self.progress.start()
        self._set_status(text)

    def _end_busy(self):
        def done():
            self.busy = False
            self.progress.stop()
            self.progress.configure(mode="determinate")
            self.progress.set(0)
            state = "disabled" if self._agent_busy or self.drawing else "normal"
            self.run_btn.configure(state=state)
            self.sim_btn.configure(state=state)
        self._ui(done)

    def _schedule_recompute(self, delay_ms=RECOMPUTE_DELAY_MS):
        if self.img_path is None:
            return
        if self._recompute_after is not None:
            self.root.after_cancel(self._recompute_after)
        self._recompute_after = self.root.after(delay_ms, self._start_recompute)

    def _start_recompute(self):
        self._recompute_after = None
        if self.img_path is None:
            return
        if self._agent_busy or self.drawing:
            self._recompute_pending = True   # 에이전트·드로잉이 끝나면 다시 계산
            return
        self.strip.set_stale(self.session.dirty_stages())
        if self._workers == 0:
            self.progress.configure(mode="indeterminate")
            self.progress.start()
        self._workers += 1
        if self.session.needs_rembg():
            self._set_status("배경 제거 중 (rembg, 한 번만 오래 걸림)...")
        else:
            self._set_status("계산 중...")
        threading.Thread(target=self._recompute_worker, daemon=True).start()

    def _recompute_worker(self):
        try:
            r = self.session.run()          # None이면 더 새로운 설정의 계산이 뒤따름
            if r is not None:
                self._ui(self._show_result, r)
        except Exception as e:  # SessionError 포함: 화면에 보여 줌
            self._set_status(f"오류: {e}")
        finally:
            self._ui(self._worker_done)

    def _worker_done(self):
        self._workers -= 1
        if self._workers == 0:
            self.progress.stop()
            self.progress.configure(mode="determinate")
            self.progress.set(0)
            self.strip.set_stale(())

    def _show_result(self, r, status="완료. 로봇 시뮬레이션으로 관절 한계를 확인해 보세요."):
        self.traj_btn.configure(state="normal")
        for st in stages.ALL_STAGES:
            self.strip.set_thumbnail(st.id, self.session.render(st.id))
        self.strip.set_summaries(self.session.stage_summaries())
        self._show_stage()
        t, pl = r["timing"], r["placement"]
        self.st_strokes.set(f"{t['stroke_count']}획", f"명령 {t['command_count']}개")
        size_color = BAD if r["out_of_pending"] else (WARN if r["out_of_limits"] else None)
        size_note = ("실물 미확인 범위도 초과" if r["out_of_pending"]
                     else "실행 시 --pending-limits 필요" if r["out_of_limits"] else "실행기 허용 범위 안")
        self.st_size.set(f"{pl['drawing_width_mm']:.0f}×{pl['drawing_height_mm']:.0f} mm", size_note, size_color)
        self.st_time.set(f"{t['total_s'] / 60:.1f}분",
                         f"그리기 {t['draw_s'] / 60:.1f} · 이동 {t['travel_s'] / 60:.1f} · "
                         f"펜 올림/내림 {t['pen_lift_s'] / 60:.1f} · 지연 {t['latency_s'] / 60:.1f}(가정)")
        self.st_sim.set("—", "시뮬레이션 버튼으로 검사")
        edits = len(self.session.edit_log)
        notice, self.session.notice = self.session.notice, ""
        self._set_status(status + (f" (획 편집 {edits}건)" if edits else "") + (f"\n{notice}" if notice else ""))
        self.refresh_proposals()

    def simulate(self):
        if not self.result:
            messagebox.showwarning("알림", "먼저 '처리 실행'을 해주세요.")
            return
        if self.busy or self._agent_busy:
            return
        path_mm = self.result["timing"]["pen_down_mm"] + self.result["timing"]["pen_up_mm"]
        self._start_busy(f"시뮬레이션 중... (경로 약 {path_mm:.0f}mm, 1mm마다 역기구학)")
        self.st_sim.set("검사 중", "")
        threading.Thread(target=self._sim_worker, daemon=True).start()

    def _sim_worker(self):
        try:
            self._ui(self._show_sim, self.session.simulate())
        except Exception as e:
            self._ui(messagebox.showerror, "시뮬레이션 오류", str(e))
        finally:
            self._end_busy()

    def _show_sim(self, summary):
        head = summary["verdict"].split(":")[0]
        color = OK if head == "PASS" else (WARN if head == "WARN" else BAD)
        self.st_sim.set(head, f"한계까지 최소 여유 {summary['min_margin_deg']:.1f}° ({summary['axis']})", color)
        self.traj_btn.configure(state="normal")
        self._set_status("시뮬레이션 완료")

    def export_strokes(self):
        if not self.result:
            messagebox.showwarning("알림", "먼저 '처리 실행'을 해주세요.")
            return
        path = filedialog.asksaveasfilename(initialdir=str(paths.output_dir()), defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if not path:
            return
        r = self.result
        metrics = sp.stroke_metrics(r["strokes_mm"], start=(0.0, 0.0))
        metrics["estimated_time"] = r["timing"]
        source = {
            "image": r["path"],
            "image_size_px": [int(r["base"].shape[1]), int(r["base"].shape[0])],
            "params": {**r["params"], "type": self._type_key(), "detail": self._detail_key(), "tool": "gui_sketch"},
        }
        pm.save_strokes_json(path, pm.build_strokes_document(r["strokes_mm"], r["placement"], metrics, source))
        extra = " --pending-limits" if r["out_of_limits"] else ""
        messagebox.showinfo("완료", f"저장됨: {path}\n\n다음: python robot/draw_executor.py \"{path}\"{extra}")

    def view_rviz(self):
        """시뮬레이션(아직이면 먼저 실행) → 관절 궤적 저장 → WSL에서 RViz 재생 창 열기."""
        if not self.result:
            messagebox.showwarning("알림", "먼저 '처리 실행'을 해주세요.")
            return
        if self.busy or self._agent_busy:
            return
        self._start_busy("RViz 준비 중..." if self.sim_result else "시뮬레이션 후 RViz를 엽니다...")
        self.traj_btn.configure(state="disabled")
        threading.Thread(target=self._rviz_worker, daemon=True).start()

    def _rviz_worker(self):
        from . import mirobot_sim as ms
        from . import rviz_launch
        try:
            if not self.sim_result:
                self._ui(self._show_sim, self.session.simulate())
            traj_path = paths.output_dir() / "rviz_traj.json"
            doc = ms.trajectory_doc(self.sim_result, self.cfg, Path(self.result["path"]).name)
            traj_path.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
            self._set_status("WSL에서 RViz를 여는 중... (처음엔 10~20초 걸릴 수 있음)")
            self._rviz_proc = rviz_launch.launch(traj_path, speed=20)
            self._set_status(f"RViz 창에서 20배속으로 반복 재생합니다. 창을 닫으면 멈춥니다.\n궤적: {traj_path}")
        except rviz_launch.RvizUnavailable as e:
            self._set_status("RViz를 열 수 없습니다.")
            self._ui(self._offer_rviz_setup, str(e))
        except Exception as e:
            self._ui(messagebox.showerror, "RViz 오류", str(e))
        finally:
            self._end_busy()
            self._ui(lambda: self.traj_btn.configure(state="normal" if self.result else "disabled"))

    def _set_status(self, text):
        self._ui(lambda: self.status.configure(text=text))

    def _poll_robot_usb(self):
        """백그라운드에서 포트 목록만 확인합니다. 시리얼 포트는 열지 않습니다."""
        if self._closing:
            return
        if not self._robot_usb_scan_busy:
            self._robot_usb_scan_busy = True
            threading.Thread(target=self._query_robot_usb, daemon=True).start()
        self._usb_after = self.root.after(3000, self._poll_robot_usb)

    def _query_robot_usb(self):
        port = str(self.cfg.get("port") or "").strip()
        if not port:
            text, color = "USB: 포트 미설정", WARN
        else:
            try:
                from serial.tools import list_ports
                present = robot_status.port_is_present(port, list_ports.comports())
            except Exception:
                text, color = f"USB: {port} 확인 실패", WARN
            else:
                text = f"USB: {port} 감지" if present else f"USB: {port} 미감지"
                color = OK if present else BAD
        self._ui(self._show_robot_usb, text, color)
        self._ui(self._finish_robot_usb_scan)

    def _show_robot_usb(self, text, color):
        self.robot_usb_status.configure(text=text, text_color=color)

    def _finish_robot_usb_scan(self):
        self._robot_usb_scan_busy = False

    def set_robot_controller_status(self, state=None, recent=False):
        """실제 드로잉에서 이미 열린 링크로 확인한 컨트롤러 상태를 표시합니다."""
        text = robot_status.controller_status_text(state, recent=recent)
        if not state:
            color = MUTED
        elif state == "Idle":
            color = OK
        elif state in ("Alarm", "연결 실패", "응답 없음"):
            color = BAD
        elif state in ("연결 중", "그리는 중"):
            color = ACCENT
        else:
            color = WARN
        self.robot_controller_status.configure(text=text, text_color=color)

    # ---------------------------------------------------------------- 에이전트
    def _init_agent(self):
        """오른쪽 에이전트 패널과 CLI용 로컬 브리지. 실패해도 앱의 나머지 기능은 그대로 동작."""
        self.agent_panel = self.bridge = None
        self._agent_visible = False
        try:
            from .agent.bridge import BridgeServer
            from .agent.panel import AgentPanel
            from .agent.tools import AgentToolbox
        except ImportError as e:
            self.agent_btn.configure(state="disabled", text=f"에이전트 없음 ({e.name})")
            return
        self.toolbox = AgentToolbox(self.session, on_change=lambda what: self._ui(self._refresh_from_session, what))
        self.bridge = BridgeServer(self.toolbox).start()
        icon = None
        if paths.asset("app_icon_256.png").exists():
            from PIL import Image
            icon = ctk.CTkImage(Image.open(paths.asset("app_icon_256.png")), size=(26, 26))
        self.agent_panel = AgentPanel(self.root, self, self.toolbox, self.bridge, font, icon_image=icon,
                                      on_close=self.toggle_agent)

    def toggle_agent(self):
        if self.agent_panel is None:
            return
        self._agent_visible = not self._agent_visible
        if self._agent_visible:
            self.agent_panel.grid(row=1, column=2, sticky="ns", padx=(0, 18), pady=(0, 18))
            self.agent_btn.configure(fg_color=ACCENT, text_color="#ffffff")
        else:
            self.agent_panel.grid_remove()
            self.agent_btn.configure(fg_color="transparent", text_color=TEXT)

    # ---------------------------------------------------------------- RViz 3D 환경 설치 도우미
    def open_rviz_setup(self):
        if self.rviz_setup_window is not None and self.rviz_setup_window.winfo_exists():
            self.rviz_setup_window.focus()
            return self.rviz_setup_window
        from .rviz_setup_window import RvizSetupWindow
        self.rviz_setup_window = RvizSetupWindow(self, font)
        return self.rviz_setup_window

    def _offer_rviz_setup(self, message):
        if messagebox.askyesno("RViz 3D 보기", message + "\n\n지금 설치 도우미를 열까요?"):
            self.open_rviz_setup()

    # ---------------------------------------------------------------- 로봇으로 그리기
    def open_draw_window(self, launch_rviz=None):
        if not self.result:
            messagebox.showwarning("알림", "먼저 이미지를 처리하세요.")
            return None
        if self.draw_window is not None and self.draw_window.winfo_exists():
            self.draw_window.focus()
            return self.draw_window
        from .draw_window import DrawWindow
        self.draw_window = DrawWindow(self, self.session, self.cfg, font, launch_rviz=launch_rviz)
        return self.draw_window

    def set_drawing(self, on):
        """실행 중에는 조절 칸·편집·재계산·에이전트 편집 도구를 잠금 (그리는 획이 바뀌지 않게)."""
        self.drawing = bool(on)
        self.session.drawing_lock = self.drawing
        self.agent_busy(self._agent_busy)
        self.draw_btn.configure(state="disabled" if self.drawing else "normal")
        self.open_btn.configure(state="disabled" if self.drawing else "normal")
        # 메인 'RViz 3D로 보기'는 run_rviz.sh가 떠 있는 따라가기 화면을 정리(pkill)하므로 그리는 중엔 잠금
        self.traj_btn.configure(state="disabled" if self.drawing or not self.result else "normal")
        if not self.drawing:
            self.st_time.set(f"{self.result['timing']['total_s'] / 60:.1f}분" if self.result else "—",
                             self.st_time.sub.cget("text"))

    def show_draw_progress(self, acked, total):
        self.st_time.value.configure(text=f"{100 * acked // max(total, 1)}% 진행")

    def _on_close(self):
        if self.draw_window is not None and self.draw_window.winfo_exists():
            if not self.draw_window.close():
                return
        if self.agent_panel is not None:
            for b in self.agent_panel.backends.values():
                b.cancel()
        if self.bridge is not None:
            self.bridge.stop()
        self._closing = True
        for after_id in (self._ui_after, self._usb_after, self._icon_after, self._recompute_after):
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except tk.TclError:
                    pass
        self.root.destroy()


APP_ID = "YoobinKim.MirobotSketch"  # 작업표시줄 묶음·고정용 앱 ID (바로가기/설치 프로그램과 같은 값)


def main():
    if sys.platform.startswith("win"):
        # 파이썬으로 실행해도 작업표시줄에 python 아이콘이 아니라 이 앱 아이콘으로 따로 표시되게 함
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)
        except (AttributeError, OSError):
            pass
    ctk.set_appearance_mode("Light")
    ctk.set_default_color_theme("blue")
    root = ctk.CTk()
    app = SketchApp(root)
    if "--setup-rviz" in sys.argv[1:]:          # 설치 프로그램의 "RViz 3D 환경도 설치" 선택
        root.after(500, app.open_rviz_setup)
    root.mainloop()


if __name__ == "__main__":
    main()
