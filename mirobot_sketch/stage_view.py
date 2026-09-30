"""
단계 띠(썸네일) · 큰 보기(확대·이동·원본 겹치기) · 조절 칸 위젯 (CustomTkinter + matplotlib)
"""

import tkinter as tk

import customtkinter as ctk
import cv2
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from PIL import Image

ACCENT = ("#2563eb", "#3b82f6")
BORDER = ("#dde2ea", "#303644")
TEXT = ("#1f2937", "#e5e7eb")
MUTED = ("#9aa3b2", "#6b7280")
THUMB_H, THUMB_W = 60, 88


class StageStrip(ctk.CTkScrollableFrame):
    """단계 썸네일 띠. 누르면 on_select(stage_id). 다시 계산될 단계는 글자를 흐리게."""

    def __init__(self, master, stages, on_select, font, titles=None):
        super().__init__(master, orientation="horizontal", height=THUMB_H + 60, fg_color="transparent")
        self.buttons, self._imgs, self.selected = {}, {}, None
        self.titles = titles or {st.id: st.label for st in stages}
        for i, st in enumerate(stages):
            if i:
                ctk.CTkLabel(self, text="→", font=font(12), text_color=MUTED).pack(side="left", padx=1)
            b = ctk.CTkButton(self, text=self.titles[st.id], compound="top", width=THUMB_W + 34, height=THUMB_H + 50,
                              font=font(11), fg_color="transparent", border_width=2, border_color=BORDER,
                              text_color=TEXT, hover_color=("#e8ecf3", "#2a2f3a"),
                              command=lambda sid=st.id: on_select(sid))
            b.pack(side="left", pady=2)
            self.buttons[st.id] = b

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
            b.configure(border_color=ACCENT if sid == stage_id else BORDER)

    def set_stale(self, stage_ids):
        stale = set(stage_ids)
        for sid, b in self.buttons.items():
            b.configure(text_color=MUTED if sid in stale else TEXT)


class BigView(ctk.CTkFrame):
    """선택한 단계를 크게. 휠=확대, 왼쪽 끌기=이동, 더블클릭=맞춤. 원본 겹치기(투명도)."""

    def __init__(self, master, font, theme_colors):
        super().__init__(master, fg_color="transparent")
        self.theme_colors = theme_colors
        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.pack(fill="x", padx=8, pady=(6, 0))
        self.title = ctk.CTkLabel(bar, text="", font=font(14, "bold"))
        self.title.pack(side="left")
        self.alpha = tk.DoubleVar(value=0.0)
        ctk.CTkSlider(bar, from_=0, to=1, variable=self.alpha, width=110,
                      command=lambda _: self.redraw()).pack(side="right")
        ctk.CTkLabel(bar, text="원본 겹치기", font=font(11)).pack(side="right", padx=4)
        self.compare_btn = ctk.CTkButton(bar, text="이전 단계와 비교 (누르고 있기)", width=180, height=26,
                                         font=font(11), fg_color="transparent", border_width=1, text_color=TEXT)
        self.compare_btn.pack(side="right", padx=(8, 0))
        self.compare_btn.bind("<ButtonPress-1>", lambda _e: self.hold_compare(True))
        self.compare_btn.bind("<ButtonRelease-1>", lambda _e: self.hold_compare(False))
        self.options_frame = ctk.CTkFrame(bar, fg_color="transparent", height=28)   # 편집 단계 옵션 자리 (빈 프레임 기본 높이 200px 방지)
        self.options_frame.pack(side="right", padx=8)
        self.subtitle = ctk.CTkLabel(self, text="", font=font(12), text_color=MUTED, anchor="w", justify="left")
        self.subtitle.pack(fill="x", padx=10)
        self._compare, self._held = None, None
        self.fig = Figure(figsize=(8, 6))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.axis("off")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)
        self.image = self.original = self.draw_extra = None
        self.on_view_change = None
        self._drag = None
        c = self.canvas
        c.mpl_connect("scroll_event", self._on_scroll)
        c.mpl_connect("button_press_event", self._on_press)
        c.mpl_connect("motion_notify_event", self._on_move)
        c.mpl_connect("button_release_event", self._on_release)

    def set_info(self, subtitle, compare=None):
        """큰 보기 위 설명 줄과 비교 대상. compare = (이전 단계 제목, 그림을 돌려주는 함수) 또는 None."""
        self.subtitle.configure(text=subtitle)
        self._compare = compare
        self.compare_btn.configure(state="normal" if compare else "disabled")

    def clear(self):
        """큰 보기를 빈 화면으로."""
        self.image = self.original = self.draw_extra = self._held = self._compare = None
        self.title.configure(text="")
        self.subtitle.configure(text="")
        self.compare_btn.configure(state="disabled")
        self.ax.clear()
        self.ax.axis("off")
        self.canvas.draw_idle()

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
        self.title.configure(text=title)
        self.redraw(lim, keep=same)

    def redraw(self, lim=None, keep=True):
        if self.image is None:
            return
        if lim is None and keep and self.ax.images:
            lim = (self.ax.get_xlim(), self.ax.get_ylim())
        bg, _ = self.theme_colors()
        self.fig.set_facecolor(bg)
        self.canvas.get_tk_widget().configure(bg=bg)
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
            row.pack(fill="x", padx=14, pady=(6, 0))
            if s.kind == "bool":
                var = tk.BooleanVar(value=bool(values[s.key]))
                w = ctk.CTkSwitch(row, text=s.label, variable=var, font=font(12),
                                  command=lambda k=s.key, v=var: self._emit(k, v.get()))
                w.pack(anchor="w")
            elif s.kind == "choice":
                ctk.CTkLabel(row, text=s.label, font=font(12), anchor="w").pack(fill="x")
                names = [c[1] for c in s.choices]
                var = tk.StringVar(value=dict(s.choices)[values[s.key]])
                w = ctk.CTkSegmentedButton(self, values=names, variable=var, font=font(12),
                                           command=lambda name, sp=s: self._emit(
                                               sp.key, next(c[0] for c in sp.choices if c[1] == name)))
                w.pack(fill="x", padx=14, pady=(2, 0))
            else:
                ctk.CTkLabel(row, text=s.label, font=font(12), anchor="w").pack(side="left")
                var = tk.StringVar(value=self._fmt(s, values[s.key]))
                entry = ctk.CTkEntry(row, textvariable=var, width=58, font=font(12), justify="right")
                entry.pack(side="right")
                entry.bind("<Return>", lambda _e, sp=s, v=var: self._from_entry(sp, v))
                entry.bind("<FocusOut>", lambda _e, sp=s, v=var: self._from_entry(sp, v))
                steps = max(1, int(round((s.hi - s.lo) / (s.step or 1))))
                w = ctk.CTkSlider(self, from_=s.lo, to=s.hi, number_of_steps=steps,
                                  button_color=ACCENT, progress_color=ACCENT,
                                  command=lambda val, sp=s, v=var: self._from_slider(sp, v, val))
                w.set(values[s.key])
                w.pack(fill="x", padx=10, pady=(2, 0))
                self.widgets.append(entry)
                self.vars[s.key + ":slider"] = w
            if s.help:
                ctk.CTkLabel(self, text=s.help, font=font(10), text_color=MUTED, anchor="w", justify="left",
                             wraplength=290).pack(fill="x", padx=14)
            self.vars[s.key] = var
            self.widgets.append(w)

    @staticmethod
    def _fmt(spec, v):
        return f"{v:.1f}" if spec.kind == "float" else str(int(v))

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
        super().__init__(master, fg_color=("#f3f6fb", "#1b2029"), corner_radius=12)
        self.font, self.excluded, self.chips, self.epoch = font, set(), {}, None
        top = ctk.CTkFrame(self, fg_color="transparent")
        top.pack(fill="x", padx=10, pady=(8, 2))
        self.summary = ctk.CTkLabel(top, text="", font=font(13, "bold"))
        self.summary.pack(side="left")
        self.discard_btn = ctk.CTkButton(top, text="취소", width=60, height=28, font=font(12),
                                         fg_color="transparent", border_width=1, text_color=TEXT,
                                         command=on_discard)
        self.discard_btn.pack(side="right", padx=(6, 0))
        self.apply_btn = ctk.CTkButton(top, text="적용", width=80, height=28, font=font(12, "bold"),
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
            b = ctk.CTkButton(self.chip_row, text=f"#{v['id']}", width=52, height=26, font=self.font(12, "bold"),
                              fg_color="transparent", border_width=2, border_color=color, text_color=color,
                              command=lambda i=v["id"]: self._toggle(i))
            b.pack(side="left", padx=2)
            self.chips[v["id"]] = b
        if len(views) > MAX_CHIPS:
            ctk.CTkLabel(self.chip_row, text=f"… 외 {len(views) - MAX_CHIPS}건", font=self.font(11),
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
