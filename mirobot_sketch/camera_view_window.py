"""
'카메라 크게 보기' 창 — 그리기 창의 카메라 화면을 큰 창으로 봄 (그리는 중 펜 선을 원본 해상도로 확인)
=====================================================================================================
보기: 그림 부분(종이 위치를 맞췄으면 그림 둘레만 원본 해상도로 잘라 봄) / 종이 전체 / 사진 전체.
사진을 자르고 줄이는 일은 카메라 스레드가 하고(CameraPreview.set_zoom), 이 창은 받은 사진을 보여 주기만 합니다.
창 크기를 바꾸면 그 크기에 맞춰 다시 받습니다. 두 번 누르면 창을 화면 가득 / 원래 크기로 바꿉니다.
"""

import tkinter as tk

import customtkinter as ctk
import cv2
from PIL import Image, ImageTk

from . import ui_scale
from .theme import BG, MUTED, SURFACE, TEXT

MODES = {"그림 부분": "drawing", "종이 전체": "paper", "사진 전체": "full"}


class CameraViewWindow(ctk.CTkToplevel):
    def __init__(self, master, font, overlay_var=None, on_close=None):
        """overlay_var: 그리기 창의 '계획한 그림 겹쳐 보기' 값 (같이 켜고 끔, 겹침은 그리기 창이 그려서 보냄)."""
        super().__init__(master)
        self.font, self.on_close = font, on_close
        self.title("카메라 크게 보기")
        ui_scale.place_window(self, 1280, 860, 640, 480)
        self.mode = "drawing"
        self._photo = None
        top = ctk.CTkFrame(self, fg_color="transparent")
        top.pack(fill="x", padx=16, pady=(12, 6))
        self.mode_btn = ctk.CTkSegmentedButton(top, values=list(MODES), command=self._pick_mode, height=34,
                                               font=font(14))
        self.mode_btn.set("그림 부분")
        self.mode_btn.pack(side="left")
        ctk.CTkButton(top, text="닫기", command=self.close, width=80, height=34, font=font(14),
                      fg_color="transparent", border_width=1, text_color=TEXT, hover_color=SURFACE).pack(side="right")
        if overlay_var is not None:
            ctk.CTkSwitch(top, text="계획한 그림 겹쳐 보기", variable=overlay_var, font=font(14)).pack(side="right", padx=(0, 14))
        self.status = ctk.CTkLabel(top, text="", font=font(13), text_color=MUTED, anchor="w", justify="left")
        self.status.pack(side="left", fill="x", expand=True, padx=14)
        holder = ctk.CTkFrame(self, fg_color=BG, corner_radius=10)
        holder.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        self.canvas = tk.Canvas(holder, highlightthickness=0, bg="#1f2430")
        self.canvas.pack(fill="both", expand=True, padx=6, pady=6)
        self.canvas.bind("<Double-Button-1>", lambda _e: self._toggle_full())
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.bind("<Escape>", lambda _e: self.state("normal") if self.state() == "zoomed" else self.close())

    def _pick_mode(self, label):
        self.mode = MODES[label]

    def _toggle_full(self):
        self.state("normal" if self.state() == "zoomed" else "zoomed")

    def view_width(self):
        """카메라 스레드가 줄일 폭(화면 px). 창이 아직 그려지지 않았으면 1280."""
        w = self.canvas.winfo_width()
        return w if w > 50 else 1280

    def show(self, frame, status=""):
        """frame: BGR 사진 (None이면 안내 글자). 칸에 맞게 비율을 지켜 줄여 가운데 둠."""
        if status != self.status.cget("text"):
            self.status.configure(text=status)
        cw, ch = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        self.canvas.delete("all")
        if frame is None:
            self.canvas.create_text(cw // 2, ch // 2, text="카메라 사진을 기다리는 중", fill="#c8cfdd",
                                    font=(self.font(14).cget("family"), 14))
            return
        h, w = frame.shape[:2]
        scale = min(cw / w, ch / h)
        size = (max(1, int(w * scale)), max(1, int(h * scale)))
        if size != (w, h):
            frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
        self._photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
        self.canvas.create_image(cw // 2, ch // 2, image=self._photo)

    def close(self):
        if self.on_close:
            self.on_close()
        self.destroy()
