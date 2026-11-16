"""
'종이 위치 맞추기' 창 — 카메라 사진에서 종이 네 귀퉁이를 찍어 화면과 종이 좌표를 맞춤
=====================================================================================
사진을 한 장 멈춰 놓고 ① 왼쪽 위 → ② 오른쪽 위 → ③ 오른쪽 아래 → ④ 왼쪽 아래 순서로 누릅니다.
찍은 점은 끌어서 고칠 수 있고, 4개가 모이면 종이 테두리·펜 범위·종이 중심(호밍 직후 펜 끝)을 겹쳐 보여 줍니다.
펜 끝이 종이 중심 표시와 맞으면 저장합니다. 로봇은 움직이지 않습니다.
"""

import tkinter as tk

import customtkinter as ctk
import cv2
from PIL import Image, ImageTk

from . import camera_marks, limits, ui_scale, vision
from .paper_mapping import A4_LANDSCAPE_MM
from .theme import BAD, BG, CARD, MUTED, OK, SURFACE, TEXT, FilledButton

DISPLAY_W = 900              # 사진을 보여 줄 폭 (배율 1.0 단위)
GRAB_RADIUS = 14             # 이 거리(화면 px) 안을 누르면 이미 찍은 점을 끌어 옮김
DOT_COLORS = ("#e11d48", "#f59e0b", "#10b981", "#3b82f6")


class PaperRegisterWindow(ctk.CTkToplevel):
    def __init__(self, master, font, camera_name, grab_frame, cfg, on_saved):
        """grab_frame(): 카메라 원본 프레임(BGR) 또는 None. on_saved(Registration)."""
        super().__init__(master)
        self.font, self.camera_name, self.grab_frame, self.cfg = font, camera_name, grab_frame, cfg
        self.on_saved = on_saved
        self.title("종이 위치 맞추기")
        ui_scale.place_window(self, DISPLAY_W + 60, 860, 760, 640)
        self.frame = None            # 멈춰 놓은 원본 사진 (BGR)
        self.points = []             # 찍은 점, 원본 사진 px
        self._drag = None
        self._marks_note = ""        # 모서리 표시로 귀퉁이를 채웠을 때 안내 앞에 붙는 문구
        self._scale = 1.0            # 원본 px → 화면 px
        self._photo = None
        self.regions = ((limits.executor_region(cfg), vision.PEN_RANGE_COLOR),
                        (limits.pending_region(cfg), vision.PENDING_RANGE_COLOR))

        ctk.CTkLabel(self, text="종이 위치 맞추기", font=font(20, "bold"), anchor="w").pack(
            fill="x", padx=20, pady=(16, 0))
        ctk.CTkLabel(self, text=f"카메라: {camera_name}   ·   종이 중심은 호밍 직후 펜 끝 자리입니다.",
                     font=font(13), text_color=MUTED, anchor="w").pack(fill="x", padx=20, pady=(2, 6))
        self.todo = ctk.CTkLabel(self, text="", font=font(16, "bold"), anchor="w", justify="left")
        self.todo.pack(fill="x", padx=20, pady=(4, 8))

        holder = ctk.CTkFrame(self, fg_color=BG, corner_radius=10)
        holder.pack(fill="both", expand=True, padx=16)
        self.canvas = tk.Canvas(holder, highlightthickness=0, bg="#1f2430", cursor="crosshair")
        self.canvas.pack(fill="both", expand=True, padx=6, pady=6)
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._move)
        self.canvas.bind("<ButtonRelease-1>", self._release)
        self.canvas.bind("<Configure>", lambda _e: self._redraw())

        size = ctk.CTkFrame(self, fg_color=CARD, corner_radius=10)
        size.pack(fill="x", padx=16, pady=(10, 0))
        ctk.CTkLabel(size, text="종이 크기 (mm)", font=font(13, "bold")).pack(side="left", padx=(14, 8), pady=8)
        self.paper_w = tk.StringVar(value=f"{A4_LANDSCAPE_MM[0]:g}")
        self.paper_h = tk.StringVar(value=f"{A4_LANDSCAPE_MM[1]:g}")
        for var, label in ((self.paper_w, "가로"), (self.paper_h, "세로")):
            ctk.CTkLabel(size, text=label, font=font(13), text_color=MUTED).pack(side="left", padx=(6, 4))
            entry = ctk.CTkEntry(size, textvariable=var, width=70, height=32, font=font(13), justify="right")
            entry.pack(side="left")
            entry.bind("<KeyRelease>", lambda _e: self._redraw())
        ctk.CTkLabel(size, text="A4 가로 = 297 × 210", font=font(12), text_color=MUTED).pack(side="left", padx=12)

        btns = ctk.CTkFrame(self, fg_color="transparent")
        btns.pack(fill="x", padx=16, pady=(10, 16))
        plain = dict(height=42, font=font(14), fg_color="transparent", border_width=1, text_color=TEXT,
                     hover_color=SURFACE)
        ctk.CTkButton(btns, text="새 사진 찍기", command=self.retake, width=130, **plain).pack(side="left")
        ctk.CTkButton(btns, text="점 지우기", command=self.clear_points, width=110, **plain).pack(side="left", padx=8)
        ctk.CTkButton(btns, text="닫기", command=self.destroy, width=90, **plain).pack(side="right")
        self.save_btn = FilledButton(btns, text="이 위치로 저장", command=self.save, width=160, height=42,
                                     font=font(15, "bold"), state="disabled")
        self.save_btn.pack(side="right", padx=8)
        self.after(200, self.retake)
        self.grab_set()

    # ------------------------------------------------------------ 사진·점
    def retake(self):
        frame = self.grab_frame()
        if frame is None:
            self.todo.configure(text="카메라 사진을 받지 못했습니다. 카메라가 켜져 있는지 확인하고 [새 사진 찍기]를 누르세요.",
                                text_color=BAD)
            return
        if self.frame is not None and frame.shape[:2] != self.frame.shape[:2]:
            self.points = []                  # 해상도가 바뀌면 찍은 점의 뜻이 달라짐
        self.frame = frame
        self._marks_note = ""
        paper = self._paper_mm()
        if not self.points and paper is not None:
            # 종이 귀퉁이 근처에 그린 X·십자가 4개 보이면 귀퉁이를 계산해 채움 (끌어서 고칠 수 있음). 없으면 손으로 찍음
            marks, _n = camera_marks.find_corner_marks(frame, paper_mm=paper)
            if marks is not None:
                self.points = [(float(x), float(y)) for x, y in camera_marks.paper_corners_from_marks(marks, paper)]
                self._marks_note = ("모서리 표시 4개를 찾아 귀퉁이를 채웠습니다 "
                                    "(표시 중심을 종이 귀퉁이로 봄). ")
        self._redraw()

    def clear_points(self):
        self.points = []
        self._marks_note = ""
        self._redraw()

    def _paper_mm(self):
        try:
            w, h = float(self.paper_w.get()), float(self.paper_h.get())
        except ValueError:
            return None
        return (w, h) if 20 <= w <= 2000 and 20 <= h <= 2000 else None

    def _to_image(self, x, y):
        return x / self._scale, y / self._scale

    def _nearest(self, x, y):
        for i, (px, py) in enumerate(self.points):
            if (px * self._scale - x) ** 2 + (py * self._scale - y) ** 2 <= GRAB_RADIUS ** 2:
                return i
        return None

    def _press(self, event):
        if self.frame is None:
            return
        i = self._nearest(event.x, event.y)
        if i is None and len(self.points) < 4:
            h, w = self.frame.shape[:2]
            x, y = self._to_image(event.x, event.y)
            self.points.append((min(max(x, 0), w - 1), min(max(y, 0), h - 1)))
            i = len(self.points) - 1
        self._drag = i
        self._redraw()

    def _move(self, event):
        if self._drag is None:
            return
        h, w = self.frame.shape[:2]
        x, y = self._to_image(event.x, event.y)
        self.points[self._drag] = (min(max(x, 0), w - 1), min(max(y, 0), h - 1))
        self._redraw()

    def _release(self, _event):
        self._drag = None

    # ------------------------------------------------------------ 화면
    def _registration(self):
        paper = self._paper_mm()
        if self.frame is None or len(self.points) < 4 or paper is None:
            return None, "종이 크기를 숫자(mm)로 적어 주세요." if paper is None else None
        h, w = self.frame.shape[:2]
        problem = vision.check_corners(self.points, (w, h), paper)
        if problem:
            return None, problem
        return vision.Registration.from_pixels(self.camera_name, self.points, (w, h), paper), None

    def _redraw(self):
        if self.frame is None or not self.winfo_exists():
            return
        cw, ch = max(self.canvas.winfo_width(), 50), max(self.canvas.winfo_height(), 50)
        h, w = self.frame.shape[:2]
        self._scale = min(cw / w, ch / h)
        reg, problem = self._registration()
        shown = vision.Overlay().draw(self.frame, reg, None, self.regions) if reg else self.frame
        rgb = cv2.cvtColor(shown, cv2.COLOR_BGR2RGB)
        size = (max(1, int(w * self._scale)), max(1, int(h * self._scale)))
        self._photo = ImageTk.PhotoImage(Image.fromarray(rgb).resize(size, Image.LANCZOS))
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor="nw", image=self._photo)
        pts = [(x * self._scale, y * self._scale) for x, y in self.points]
        if 1 < len(pts) < 4:
            self.canvas.create_line(*[c for p in pts for c in p], fill="#f8fafc", width=2, dash=(4, 3))
        for i, (x, y) in enumerate(pts):
            r = 8
            self.canvas.create_oval(x - r, y - r, x + r, y + r, outline="white", width=2, fill=DOT_COLORS[i])
            self.canvas.create_text(x + 16, y - 14, text=f"{i + 1} {vision.CORNER_NAMES[i]}", fill="white",
                                    anchor="w", font=(self.font(13).cget("family"), 12, "bold"))
        if len(self.points) < 4:
            k = len(self.points)
            self.todo.configure(text=f"종이의 {k + 1}. {vision.CORNER_NAMES[k]} 귀퉁이를 누르세요"
                                     + ("  (찍은 점은 끌어서 옮길 수 있습니다)" if k else ""), text_color=TEXT)
        elif problem:
            self.todo.configure(text=problem, text_color=BAD)
        else:
            self.todo.configure(text=self._marks_note
                                + "초록 테두리가 종이와 맞고 + 표시가 종이 중심(호밍 직후 펜 끝)에 있으면 저장하세요. "
                                  "파랑: 펜 범위 · 주황: 공중 확인 범위", text_color=OK)
        self.save_btn.configure(state="normal" if reg else "disabled")

    def save(self):
        reg, _problem = self._registration()
        if reg is None:
            return
        vision.save(reg)
        self.on_saved(reg)
        self.destroy()
