"""
화면에 맞춘 글자·버튼 배율
==========================
CustomTkinter 위젯 배율(글자·버튼·여백 전부)을 창 크기에 맞춰 바꿉니다. 창이 BASE_W×BASE_H(DPI를 반영한 단위)일 때
1.0이고, 창을 키우면 같이 커지고 줄이면 MIN_ZOOM까지 작아집니다. 배치는 항상 기준 크기 이상의 공간을 받으므로
어느 화면에서도 같은 모양으로 보입니다. 사용자가 머리글의 [−]/[+](Ctrl+- / Ctrl+= / Ctrl+0)로 고른 배율을 여기에
곱하고, 그 값은 다음 실행에도 쓰도록 사용자 폴더에 저장합니다.
"""

import json
import sys

import customtkinter as ctk

from . import paths

BASE_W, BASE_H = 1280, 800        # 배율 1.0에서 메인 화면 배치가 딱 맞는 창 크기 (DPI 반영 단위)
MIN_ZOOM, MAX_ZOOM = 0.8, 1.6     # 창 크기로 정하는 자동 배율의 범위
ZOOM_STEP = 0.05                  # 이 단위로만 바꿈 (창을 끄는 동안 조금씩 계속 다시 그리지 않게)
USER_MIN, USER_MAX, USER_STEP = 0.7, 1.5, 0.1
RESIZE_DELAY_MS = 200             # 창 크기 변경이 이만큼 멈추면 배율을 다시 계산
SETTINGS_FILE = "ui.json"


def work_area(win):
    """작업 표시줄을 뺀 화면 (x, y, 너비, 높이), 실제 픽셀.

    Windows 고해상도(예: 2880×1800, 200%)에서 Tk의 winfo_screenwidth는 DPI 인식 전 값(1440)을 돌려준다.
    그 값으로 계산하면 화면이 절반 크기라고 판단해 위젯을 반으로 줄였다."""
    if sys.platform.startswith("win"):
        try:
            import ctypes
            from ctypes import wintypes
            rect = wintypes.RECT()
            if ctypes.windll.user32.SystemParametersInfoW(0x0030, 0, ctypes.byref(rect), 0):   # SPI_GETWORKAREA
                return rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top
        except (AttributeError, OSError):
            pass
    return 0, 0, win.winfo_screenwidth(), win.winfo_screenheight()


def window_scaling(win):
    """창 크기 단위 1을 실제 픽셀로 바꾸는 배율 (Windows 표시 배율, 예: 200% -> 2.0)."""
    return ctk.ScalingTracker.get_window_scaling(win)


def available_units(win):
    """작업 영역을 창 크기 단위로 (너비, 높이)."""
    _x, _y, w, h = work_area(win)
    s = window_scaling(win)
    return w / s, h / s


def fit_zoom(width_units, height_units, user=1.0):
    """창 크기에 맞춘 위젯 배율 × 사용자 배율, ZOOM_STEP 단위로 반올림."""
    auto = min(MAX_ZOOM, max(MIN_ZOOM, min(width_units / BASE_W, height_units / BASE_H)))
    return round(round(auto * user / ZOOM_STEP) * ZOOM_STEP, 2)


def clamp_user(value):
    return round(min(USER_MAX, max(USER_MIN, value)), 2)


def load_user_zoom():
    try:
        value = float(json.loads((paths.user_dir() / SETTINGS_FILE).read_text(encoding="utf-8"))["zoom"])
    except (OSError, ValueError, KeyError, TypeError):
        return 1.0
    return clamp_user(value)


def save_user_zoom(value):
    try:
        (paths.user_dir() / SETTINGS_FILE).write_text(json.dumps({"zoom": value}), encoding="utf-8")
    except OSError:
        pass


def place_window(win, width, height, min_width=None, min_height=None):
    """보조 창: 배율 1.0 기준 크기에 현재 위젯 배율을 곱하고, 화면 작업 영역 안에서 가운데 둔다."""
    zoom = ctk.ScalingTracker.widget_scaling
    s = window_scaling(win)
    x, y, aw, ah = work_area(win)
    max_w, max_h = aw / s - 24, ah / s - 64           # 테두리·제목 표시줄 여유
    w, h = round(min(width * zoom, max_w)), round(min(height * zoom, max_h))
    left = x + max(0, (aw - round(w * s)) // 2)
    top = y + max(0, (ah - round(h * s)) // 3)
    win.geometry(f"{w}x{h}+{left}+{top}")
    if min_width and min_height:
        win.minsize(round(min(min_width * zoom, max_w)), round(min(min_height * zoom, max_h)))


def follow_width(label, container, pad=0):
    """label의 줄바꿈 폭을 container 폭에 맞춤 (창 크기·배율이 바뀌어도 글이 잘리거나 한 줄로 길게 늘어지지 않게)."""
    def resize(event):
        width = int(event.width / label._get_widget_scaling()) - pad
        if width > 40:
            label.configure(wraplength=width)
    container.bind("<Configure>", resize, add="+")


class Zoom:
    """메인 창 크기를 따라가는 위젯 배율. 값이 바뀌면 on_change()."""

    def __init__(self, root, on_change=None):
        self.root, self.on_change = root, on_change
        self.user = load_user_zoom()
        self.value = None
        self._size = None
        self._after = None
        root.bind("<Configure>", self._on_configure, add="+")

    def apply_for_size(self, width_units, height_units):
        """배율이 바뀌었으면 True."""
        self._size = (width_units, height_units)
        zoom = fit_zoom(width_units, height_units, self.user)
        if zoom == self.value:
            return False
        self.value = zoom
        ctk.set_widget_scaling(zoom)
        if self.on_change:
            self.on_change()
        return True

    def _on_configure(self, event):
        if event.widget is not self.root:
            return
        if self._after is not None:
            self.root.after_cancel(self._after)
        self._after = self.root.after(RESIZE_DELAY_MS, self.refresh)

    def refresh(self):
        self._after = None
        if self.root.state() in ("withdrawn", "iconic"):
            return
        s = window_scaling(self.root)
        width, height = self.root.winfo_width() / s, self.root.winfo_height() / s
        if width >= 200 and height >= 200:
            self.apply_for_size(width, height)

    def step(self, direction):
        """사용자 배율: +1 크게, -1 작게, 0 처음 값(100%)."""
        self.user = 1.0 if direction == 0 else clamp_user(self.user + direction * USER_STEP)
        save_user_zoom(self.user)
        changed = self._size is not None and self.apply_for_size(*self._size)
        if not changed and self.on_change:
            self.on_change()

    def cancel(self):
        if self._after is not None:
            try:
                self.root.after_cancel(self._after)
            except Exception:
                pass
            self._after = None
