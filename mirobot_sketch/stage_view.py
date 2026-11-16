"""
단계 띠(썸네일) · 큰 보기(확대·이동·원본 겹치기) · 조절 칸 위젯 (CustomTkinter + matplotlib)
"""

import sys
import tkinter as tk

import customtkinter as ctk
import cv2
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from PIL import Image

from .theme import ACCENT, BG, BORDER, CARD, MUTED, SELECTED, SURFACE, TEXT, FilledButton
THUMB_H, THUMB_W = 56, 92
STRIP_GAP = 8                  # 단계 버튼 사이 간격 (양쪽 4씩)
MIN_BUTTON_W = THUMB_W + 40    # 이보다 좁아지면 띠를 가로로 넘겨 봄
ZOOM_STEP = 1.25               # 큰 보기 [+]/[−] 한 번에 확대·축소 비율


class StageStrip(ctk.CTkScrollableFrame):
    """단계 썸네일 띠. 누르면 on_select(stage_id). 다시 계산될 단계는 글자를 흐리게.
    공간이 넉넉하면 버튼이 띠 폭을 나눠 채우고, 모자라면 최소 폭을 지키며 휠·Shift+휠로 좌우로 넘긴다."""

    def __init__(self, master, stages, on_select, font, titles=None):
        super().__init__(master, orientation="horizontal", height=THUMB_H + 62,
                         fg_color=CARD, corner_radius=14, border_width=1, border_color=BORDER)
        self.buttons, self._imgs, self.selected = {}, {}, None
        self._button_w = MIN_BUTTON_W
        self.titles = titles or {st.id: st.label for st in stages}
        for st in stages:
            b = ctk.CTkButton(self, text=self.titles[st.id], compound="top", width=MIN_BUTTON_W,
                              height=THUMB_H + 54, corner_radius=10, font=font(12),
                              fg_color="transparent", border_width=1, border_color=BORDER,
                              text_color=TEXT, hover_color=SURFACE,
                              command=lambda sid=st.id: on_select(sid))
            b.pack(side="left", padx=STRIP_GAP // 2, pady=5)
            self.buttons[st.id] = b
        self._parent_canvas.bind("<Configure>", self._on_resize, add="+")

    def _on_resize(self, event):
        """띠 폭에 맞춰 버튼 폭을 나눔 (배율 1.0 단위)."""
        avail = event.width / self._get_widget_scaling() - 12
        width = max(MIN_BUTTON_W, int(avail / len(self.buttons)) - STRIP_GAP)
        if abs(width - self._button_w) >= 2:
            self._button_w = width
            for b in self.buttons.values():
                b.configure(width=width)
        self.after_idle(self._reveal_selected)

    def _mouse_wheel_all(self, event):
        """가로 띠라서 Shift 없이 휠만 굴려도 좌우로 넘김."""
        valid = getattr(self, "_check_if_valid_scroll", None) or self.check_if_master_is_canvas
        if not valid(event.widget) or self._parent_canvas.xview() == (0.0, 1.0):
            return
        if sys.platform.startswith("win"):
            step = -int(event.delta / 2)
        elif sys.platform == "darwin":
            step = -event.delta
        else:
            step = -1 if event.num == 4 else 1
        self._parent_canvas.xview("scroll", step, "units")

    def set_thumbnail(self, stage_id, img_bgr):
        h, w = img_bgr.shape[:2]
        s = min(THUMB_H / h, THUMB_W / w)
        small = cv2.resize(img_bgr, (max(1, int(w * s)), max(1, int(h * s))), interpolation=cv2.INTER_AREA)
        pil = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
        self._imgs[stage_id] = ctk.CTkImage(pil, size=pil.size)
        self.buttons[stage_id].configure(image=self._imgs[stage_id])

    def set_summaries(self, summaries):
        """썸네일 아래에 "번호 이름 / 결과 한 줄" (값을 바꾸면 어느 단계 숫자가 바뀌는지 보이게)."""
        for sid, b in self.buttons.items():
            line = summaries.get(sid, "")
            b.configure(text=self.titles[sid] + (f"\n{line}" if line else ""))

    def clear(self):
        """썸네일·결과 한 줄을 모두 지움 (새 그림을 시작하기 전 빈 상태)."""
        self._imgs.clear()
        for sid, b in self.buttons.items():
            b.configure(image=None, text=self.titles[sid], text_color=TEXT)

    def select(self, stage_id):
        self.selected = stage_id
        for sid, b in self.buttons.items():
            b.configure(border_color=ACCENT if sid == stage_id else BORDER,
                        fg_color=SELECTED if sid == stage_id else "transparent",
                        border_width=2 if sid == stage_id else 1)
        self._reveal_selected()

    def _reveal_selected(self):
        """Reveal selection after layout and when the workspace is resized."""
        if self.selected is None:
            return
        button = self.buttons[self.selected]
        canvas = self._parent_canvas
        width = max(self.winfo_width(), 1)
        left, right = canvas.xview()
        start, end = button.winfo_x() / width, (button.winfo_x() + button.winfo_width()) / width
        if start < left:
            canvas.xview_moveto(start)
        elif end > right:
            canvas.xview_moveto(end - (right - left))

    def set_stale(self, stage_ids):
        stale = set(stage_ids)
        for sid, b in self.buttons.items():
            b.configure(text_color=MUTED if sid in stale else TEXT)


class BigView(ctk.CTkFrame):
    """선택한 단계를 크게. 휠·[+]/[−]=확대·축소, 왼쪽 끌기=이동, 더블클릭·[화면에 맞춤]=맞춤. 원본 겹치기(투명도)."""

    def __init__(self, master, font, theme_colors):
        super().__init__(master, fg_color=CARD, corner_radius=14)
        self.theme_colors = theme_colors
        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.pack(fill="x", padx=16, pady=(12, 0))
        self.title = ctk.CTkLabel(bar, text="미리보기", font=font(16, "bold"))
        self.title.pack(side="left")
        self.fit_btn = ctk.CTkButton(bar, text="화면에 맞춤", command=self.fit, width=100, height=34,
                                     font=font(13), fg_color=SURFACE, hover_color=BORDER, text_color=TEXT,
                                     state="disabled")
        self.fit_btn.pack(side="right")
        self.zoom_btns = []
        for text, factor in (("+", 1 / ZOOM_STEP), ("−", ZOOM_STEP)):
            b = ctk.CTkButton(bar, text=text, command=lambda f=factor: self.zoom_by(f), width=36, height=34,
                              font=font(16, "bold"), fg_color=SURFACE, hover_color=BORDER, text_color=TEXT,
                              state="disabled")
            b.pack(side="right", padx=(0, 6))
            self.zoom_btns.append(b)
        self.subtitle = ctk.CTkLabel(self, text="사진을 선으로 바꾸고, 로봇으로 그려 보세요.",
                                     font=font(13), text_color=MUTED, anchor="w", justify="left")
        self.subtitle.pack(fill="x", padx=16, pady=(4, 8))
        self.note = ctk.CTkLabel(self, text="", font=font(13, "bold"), text_color=TEXT, anchor="w", justify="left")
        # 그림 바로 아래: 비교·겹치기 (그림을 보면서 손이 가는 자리)
        footer = ctk.CTkFrame(self, fg_color="transparent")
        footer.pack(side="bottom", fill="x", padx=16, pady=(0, 10))
        self._footer = footer
        self.compare_btn = ctk.CTkButton(footer, text="이전 단계와 비교 (누르고 있기)", width=220, height=34,
                                         font=font(13), fg_color="transparent", border_width=1,
                                         text_color=TEXT, hover_color=SURFACE, state="disabled")
        self.compare_btn.pack(side="left")
        self.compare_btn.bind("<ButtonPress-1>", lambda _e: self.hold_compare(True))
        self.compare_btn.bind("<ButtonRelease-1>", lambda _e: self.hold_compare(False))
        ctk.CTkLabel(footer, text="원본 겹치기", font=font(13), text_color=MUTED).pack(side="left", padx=(16, 8))
        self.alpha = tk.DoubleVar(value=0.0)
        self.overlay_slider = ctk.CTkSlider(footer, from_=0, to=1, variable=self.alpha, width=130, height=20,
                                            state="disabled", command=lambda _: self.redraw())
        self.overlay_slider.pack(side="left")
        self.hint = ctk.CTkLabel(footer, text="휠 확대 · 드래그 이동 · 더블클릭 맞춤", font=font(12),
                                 text_color=MUTED)
        self.hint.pack(side="right")
        # 편집 단계의 보기 옵션(번호·버린 선): 비교 줄 바로 위 한 줄 (좁은 창에서도 잘리지 않게 따로)
        self.options_frame = ctk.CTkFrame(self, fg_color="transparent", height=28)
        self._compare, self._held = None, None
        canvas_frame = ctk.CTkFrame(self, fg_color=BG, corner_radius=10)
        canvas_frame.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        self.fig = Figure(figsize=(8, 6))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.axis("off")
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)
        self.empty = ctk.CTkFrame(canvas_frame, fg_color=BG, corner_radius=0)
        ctk.CTkLabel(self.empty, text="사진 한 장, 새로운 드로잉", font=font(24, "bold"),
                     text_color=TEXT).pack(padx=24, pady=(8, 8))
        ctk.CTkLabel(self.empty, text="이미지를 열면 선 추출부터 종이 배치까지 자동으로 준비합니다.\n"
                     "로봇 없이도 미리보기와 시뮬레이션을 사용해 보세요.",
                     font=font(14), text_color=MUTED, justify="center", wraplength=420).pack(
                         padx=24, pady=(0, 16))
        self.empty_open_btn = FilledButton(self.empty, text="+  이미지 열기", width=200, height=46,
                                           font=font(16, "bold"))
        self.empty_open_btn.pack(pady=(0, 10))
        ctk.CTkLabel(self.empty, text="JPG · PNG · WEBP · BMP", font=font(12),
                     text_color=MUTED).pack(pady=(0, 8))
        self.empty.place(relx=0.5, rely=0.5, anchor="center")
        self.image = self.original = self.draw_extra = None
        self.on_view_change = None
        self._drag = None
        c = self.canvas
        c.mpl_connect("scroll_event", self._on_scroll)
        c.mpl_connect("button_press_event", self._on_press)
        c.mpl_connect("motion_notify_event", self._on_move)
        c.mpl_connect("button_release_event", self._on_release)
        self.redraw()

    def set_info(self, subtitle, compare=None):
        """큰 보기 위 설명 줄과 비교 대상. compare = (이전 단계 제목, 그림을 돌려주는 함수) 또는 None."""
        self.subtitle.configure(text=subtitle)
        self._compare = compare
        self.compare_btn.configure(state="normal" if compare else "disabled")

    def set_note(self, text):
        """설명 줄 아래 강조 한 줄 (예: 종이 단계의 가이드 선 색 설명). 빈 문자열이면 숨김."""
        self.note.configure(text=text)
        if text:
            self.note.pack(fill="x", padx=16, pady=(0, 8), after=self.subtitle)
        else:
            self.note.pack_forget()

    def show_options(self, on):
        """편집 단계의 보기 옵션 줄을 비교 줄 바로 위에 보이거나 숨김."""
        if on:
            self.options_frame.pack(side="bottom", fill="x", padx=12, pady=(0, 4), after=self._footer)
        else:
            self.options_frame.pack_forget()

    def clear(self):
        """큰 보기를 빈 화면으로."""
        self.image = self.original = self.draw_extra = self._held = self._compare = None
        self.title.configure(text="미리보기")
        self.subtitle.configure(text="사진을 선으로 바꾸고, 로봇으로 그려 보세요.")
        self.set_note("")
        self.show_options(False)
        self.compare_btn.configure(state="disabled")
        self._set_view_buttons("disabled")
        self.overlay_slider.configure(state="disabled")
        self.alpha.set(0)
        self._drag = None
        self.empty.place(relx=0.5, rely=0.5, anchor="center")
        self.redraw()

    def hold_compare(self, on):
        """누르고 있는 동안 이전 단계 그림 (같은 확대 위치), 떼면 원래대로."""
        if on and self._compare and self._held is None:
            self._held = (self.title.cget("text"), self.image, self.original, self.draw_extra)
            title, fn = self._compare
            self.show(f"{title} (이전 단계)", fn(), self.original)
        elif not on and self._held is not None:
            title, img, orig, extra = self._held
            self._held = None
            self.show(title, img, orig, extra)

    def show(self, title, img_bgr, original_bgr=None, draw_extra=None):
        """같은 크기의 그림이면 확대 위치를 유지."""
        same = self.image is not None and self.image.shape[:2] == img_bgr.shape[:2]
        lim = (self.ax.get_xlim(), self.ax.get_ylim()) if same else None
        self.image, self.original, self.draw_extra = img_bgr, original_bgr, draw_extra
        self.empty.place_forget()
        self._set_view_buttons("normal")
        self.overlay_slider.configure(state="normal" if original_bgr is not None else "disabled")
        self.title.configure(text=title)
        self.redraw(lim, keep=same)

    def redraw(self, lim=None, keep=True):
        bg, _ = self.theme_colors()
        self.fig.set_facecolor(bg)
        self.canvas.get_tk_widget().configure(bg=bg)
        if self.image is None:
            self.ax.clear()
            self.ax.axis("off")
            self.canvas.draw_idle()
            return
        if lim is None and keep and self.ax.images:
            lim = (self.ax.get_xlim(), self.ax.get_ylim())
        self.ax.clear()
        self.ax.axis("off")
        img = self.image
        a = float(self.alpha.get())
        if self.original is not None and a > 0:
            # 선(어두운 부분)은 그대로, 흰 바탕 자리에 흐린 원본이 비치게
            faded = (self.original.astype(np.float32) * a + 255 * (1 - a)).astype(np.uint8)
            img = np.minimum(img, faded)
        self.ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), interpolation="antialiased")
        if lim:
            self.ax.set_xlim(lim[0])
            self.ax.set_ylim(lim[1])
        if self.draw_extra:
            self.draw_extra(self.ax)
            if lim:   # 선 모음을 더하면 축 범위가 바뀔 수 있어 다시 맞춤
                self.ax.set_xlim(lim[0])
                self.ax.set_ylim(lim[1])
        self.canvas.draw_idle()

    def view_rect(self):
        (x0, x1), (y1, y0) = self.ax.get_xlim(), self.ax.get_ylim()
        return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)

    def _set_view_buttons(self, state):
        for b in (self.fit_btn, *self.zoom_btns):
            b.configure(state=state)

    def zoom_by(self, factor):
        """보이는 범위의 가운데를 기준으로 확대(factor<1)·축소(factor>1)."""
        if self.image is None:
            return
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.ax.set_xlim(cx + (x0 - cx) * factor, cx + (x1 - cx) * factor)
        self.ax.set_ylim(cy + (y0 - cy) * factor, cy + (y1 - cy) * factor)
        self._changed()

    def fit(self):
        if self.image is None:
            return
        h, w = self.image.shape[:2]
        self.ax.set_xlim(-0.5, w - 0.5)
        self.ax.set_ylim(h - 0.5, -0.5)
        self._changed()

    def _changed(self):
        if self.on_view_change:
            self.on_view_change()      # 번호 딱지를 보이는 범위에 맞게 다시 그림
        else:
            self.canvas.draw_idle()

    def _on_scroll(self, e):
        if e.xdata is None:
            return
        f = 0.8 if e.button == "up" else 1.25
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        self.ax.set_xlim(e.xdata + (x0 - e.xdata) * f, e.xdata + (x1 - e.xdata) * f)
        self.ax.set_ylim(e.ydata + (y0 - e.ydata) * f, e.ydata + (y1 - e.ydata) * f)
        self._changed()

    def _on_press(self, e):
        if e.dblclick:
            self.fit()
        elif e.button == 1 and e.inaxes is self.ax:
            self._drag = (e.x, e.y, self.ax.get_xlim(), self.ax.get_ylim())

    def _on_move(self, e):
        if not self._drag or e.x is None:
            return
        x, y, xl, yl = self._drag
        box = self.ax.get_window_extent()
        dx = (e.x - x) * (xl[1] - xl[0]) / box.width
        dy = (e.y - y) * (yl[1] - yl[0]) / box.height
        self.ax.set_xlim(xl[0] - dx, xl[1] - dx)
        self.ax.set_ylim(yl[0] - dy, yl[1] - dy)
        self.canvas.draw_idle()

    def _on_release(self, e):
        if self._drag:
            self._drag = None
            self._changed()


class ParamControls(ctk.CTkFrame):
    """ParamSpec 목록으로 조절 칸을 만듦. 값이 바뀌면 on_change(key, value)."""

    def __init__(self, master, specs, values, on_change, font):
        super().__init__(master, fg_color="transparent")
        self.specs, self.on_change, self._muted = {s.key: s for s in specs}, on_change, False
        self.vars, self.widgets = {}, []
        for s in specs:
            row = ctk.CTkFrame(self, fg_color="transparent")
            row.pack(fill="x", padx=14, pady=(10, 0))
            if s.kind == "bool":
                var = tk.BooleanVar(value=bool(values[s.key]))
                w = ctk.CTkSwitch(row, text=s.label, variable=var, font=font(13),
                                  command=lambda k=s.key, v=var: self._emit(k, v.get()))
                w.pack(anchor="w")
            elif s.kind == "choice":
                ctk.CTkLabel(row, text=s.label, font=font(13), anchor="w").pack(fill="x")
                names = [c[1] for c in s.choices]
                var = tk.StringVar(value=dict(s.choices)[values[s.key]])
                w = ctk.CTkSegmentedButton(self, values=names, variable=var, font=font(13), height=34,
                                           command=lambda name, sp=s: self._emit(
                                               sp.key, next(c[0] for c in sp.choices if c[1] == name)))
                w.pack(fill="x", padx=14, pady=(4, 0))
            else:
                ctk.CTkLabel(row, text=s.label, font=font(13), anchor="w").pack(side="left")
                var = tk.StringVar(value=self._fmt(s, values[s.key]))
                entry = ctk.CTkEntry(row, textvariable=var, width=70, height=32, font=font(13), justify="right")
                entry.pack(side="right")
                entry.bind("<Return>", lambda _e, sp=s, v=var: self._from_entry(sp, v))
                entry.bind("<FocusOut>", lambda _e, sp=s, v=var: self._from_entry(sp, v))
                entry.bind("<Up>", lambda _e, sp=s, v=var: self._nudge(sp, v, 1))     # ↑/↓ 한 칸씩
                entry.bind("<Down>", lambda _e, sp=s, v=var: self._nudge(sp, v, -1))
                steps = max(1, int(round((s.hi - s.lo) / (s.step or 1))))
                w = ctk.CTkSlider(self, from_=s.lo, to=s.hi, number_of_steps=steps, height=20,
                                  button_color=ACCENT, progress_color=ACCENT,
                                  command=lambda val, sp=s, v=var: self._from_slider(sp, v, val))
                w.set(values[s.key])
                w.pack(fill="x", padx=10, pady=(6, 0))
                self.widgets.append(entry)
                self.vars[s.key + ":slider"] = w
            if s.help:
                ctk.CTkLabel(self, text=s.help, font=font(12), text_color=MUTED, anchor="w", justify="left",
                             wraplength=300).pack(fill="x", padx=14, pady=(4, 4))
            self.vars[s.key] = var
            self.widgets.append(w)

    @staticmethod
    def _fmt(spec, v):
        if spec.kind != "float":
            return str(int(v))
        step = spec.step or 1
        text = f"{float(step):.10f}".rstrip("0").rstrip(".")
        decimals = len(text.split(".")[1]) if "." in text else 0
        return f"{float(v):.{decimals}f}"

    def _nudge(self, spec, var, direction):
        """입력 칸에서 ↑/↓: 슬라이더 한 칸만큼 올리고 내림."""
        try:
            value = float(var.get())
        except ValueError:
            return "break"
        var.set(self._fmt(spec, spec.clamp(value + direction * (spec.step or 1))))
        self._from_entry(spec, var)
        return "break"

    def _emit(self, key, value):
        if not self._muted:
            self.on_change(key, value)

    def _from_slider(self, spec, var, val):
        v = spec.clamp(val)
        var.set(self._fmt(spec, v))
        self._emit(spec.key, v)

    def _from_entry(self, spec, var):
        try:
            v = spec.clamp(float(var.get()))
        except ValueError:
            return
        var.set(self._fmt(spec, v))
        self.vars[spec.key + ":slider"].set(v)
        self._emit(spec.key, v)

    def set_values(self, values):
        """에이전트·프리셋이 바꾼 값을 화면에 반영 (on_change는 부르지 않음)."""
        self._muted = True
        try:
            for key, s in self.specs.items():
                v = values[key]
                if s.kind == "bool":
                    self.vars[key].set(bool(v))
                elif s.kind == "choice":
                    self.vars[key].set(dict(s.choices)[v])
                else:
                    self.vars[key].set(self._fmt(s, v))
                    self.vars[key + ":slider"].set(v)
        finally:
            self._muted = False

    def set_enabled(self, enabled):
        for w in self.widgets:
            w.configure(state="normal" if enabled else "disabled")


GREEN_TXT, RED_TXT = "#16a34a", "#dc2626"
MAX_CHIPS = 40


class ProposalBar(ctk.CTkFrame):
    """제안 n건 · 초록 a · 빨강 b [적용] [취소] + 번호 딱지(눌러서 빼기/넣기)."""

    def __init__(self, master, font, on_apply, on_discard):
        super().__init__(master, fg_color=SELECTED, corner_radius=12, border_width=1, border_color=BORDER)
        self.font, self.excluded, self.chips, self.epoch = font, set(), {}, None
        top = ctk.CTkFrame(self, fg_color="transparent")
        top.pack(fill="x", padx=10, pady=(8, 2))
        self.summary = ctk.CTkLabel(top, text="", font=font(14, "bold"))
        self.summary.pack(side="left")
        self.discard_btn = ctk.CTkButton(top, text="취소", width=60, height=28, font=font(13),
                                         fg_color="transparent", border_width=1, text_color=TEXT,
                                         command=on_discard)
        self.discard_btn.pack(side="right", padx=(6, 0))
        self.apply_btn = ctk.CTkButton(top, text="적용", width=80, height=28, font=font(13, "bold"),
                                       fg_color=ACCENT, command=lambda: on_apply(sorted(self.excluded)))
        self.apply_btn.pack(side="right")
        self.chip_row = ctk.CTkScrollableFrame(self, orientation="horizontal", height=34, fg_color="transparent")
        self.chip_row.pack(fill="x", padx=6, pady=(0, 6))

    def set_views(self, views, epoch=None):
        if epoch != self.epoch:            # 번호를 새로 매겼으면 예전 '뺌' 표시는 다른 획의 것
            self.excluded, self.epoch = set(), epoch
        for w in self.chip_row.winfo_children():
            w.destroy()
        self.chips = {}
        ids = {v["id"] for v in views}
        self.excluded &= ids
        adds = sum(1 for v in views if v["after"] is not None)
        dels = sum(1 for v in views if v["after"] is None)
        # Tk 글꼴은 컬러 이모지를 못 그려(빗금 원) 색 이름으로 표시
        self.summary.configure(text=f"제안 {len(views)}건 · 초록(생김) {adds} · 빨강(사라짐) {dels}   딱지를 눌러 빼기")
        for v in views[:MAX_CHIPS]:
            color = GREEN_TXT if v["after"] is not None else RED_TXT
            b = ctk.CTkButton(self.chip_row, text=f"#{v['id']}", width=52, height=26, font=self.font(13, "bold"),
                              fg_color="transparent", border_width=2, border_color=color, text_color=color,
                              command=lambda i=v["id"]: self._toggle(i))
            b.pack(side="left", padx=2)
            self.chips[v["id"]] = b
        if len(views) > MAX_CHIPS:
            ctk.CTkLabel(self.chip_row, text=f"… 외 {len(views) - MAX_CHIPS}건", font=self.font(12),
                         text_color=MUTED).pack(side="left", padx=4)
        for i in self.excluded:
            self._style(i)

    def _toggle(self, i):
        self.excluded ^= {i}
        self._style(i)

    def _style(self, i):
        b = self.chips.get(i)
        if b is not None:
            b.configure(fg_color=("#e5e7eb", "#374151") if i in self.excluded else "transparent",
                        text=f"#{i}" + (" 뺌" if i in self.excluded else ""))
