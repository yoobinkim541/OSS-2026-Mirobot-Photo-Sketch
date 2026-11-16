"""Shared light/dark palette and native widget defaults for the sketch studio."""

import customtkinter as ctk

BG = ("#f4f5f9", "#10131c")
CARD = ("#ffffff", "#1a1e2b")
SURFACE = ("#eceef5", "#222838")
BORDER = ("#dce0eb", "#30384b")
TEXT = ("#20263b", "#eef1fa")
MUTED = ("#626d83", "#a4aec4")
ACCENT = ("#6254df", "#9585ff")
ACCENT_HOVER = ("#5143c7", "#7e6deb")
SELECTED = ("#eeebff", "#302b4b")
OK, WARN, BAD = ("#15804d", "#54d39a"), ("#a5640b", "#f3bd65"), ("#c33549", "#ff8393")
DISABLED_FILL = ("#e3e6ee", "#2a3042")
DISABLED_TEXT = ("#9aa3b5", "#6b7489")


def configure_widgets():
    """Set defaults before constructing widgets, including secondary windows."""
    ctk.set_default_color_theme("blue")
    theme = ctk.ThemeManager.theme
    updates = {
        "CTk": {"fg_color": BG},
        "CTkToplevel": {"fg_color": BG},
        "CTkFrame": {"fg_color": CARD, "top_fg_color": SURFACE, "border_color": BORDER,
                     "corner_radius": 14},
        "CTkLabel": {"text_color": TEXT},
        "CTkButton": {"fg_color": ACCENT, "hover_color": ACCENT_HOVER, "border_color": BORDER,
                      "text_color": ("#ffffff", "#10131c"), "corner_radius": 9},
        "CTkSegmentedButton": {"fg_color": SURFACE, "selected_color": SELECTED,
                               "selected_hover_color": BORDER, "unselected_color": SURFACE,
                               "unselected_hover_color": BORDER, "text_color": TEXT,
                               "corner_radius": 8},
        "CTkEntry": {"fg_color": BG, "border_color": BORDER, "text_color": TEXT,
                     "placeholder_text_color": MUTED, "corner_radius": 8},
        "CTkTextbox": {"fg_color": BG, "border_color": BORDER, "text_color": TEXT,
                       "corner_radius": 10},
        "CTkSlider": {"fg_color": SURFACE, "progress_color": ACCENT, "button_color": ACCENT,
                      "button_hover_color": ACCENT_HOVER},
        "CTkProgressBar": {"fg_color": SURFACE, "progress_color": ACCENT},
        "CTkSwitch": {"fg_color": BORDER, "progress_color": ACCENT, "text_color": TEXT},
        "CTkCheckBox": {"fg_color": ACCENT, "hover_color": ACCENT_HOVER,
                        "border_color": MUTED, "text_color": TEXT},
        "CTkRadioButton": {"fg_color": ACCENT, "hover_color": ACCENT_HOVER,
                           "border_color": MUTED, "text_color": TEXT},
        "CTkOptionMenu": {"fg_color": SURFACE, "button_color": SURFACE,
                          "button_hover_color": BORDER, "text_color": TEXT},
        "CTkScrollbar": {"button_color": BORDER, "button_hover_color": MUTED},
    }
    for widget, colors in updates.items():
        theme[widget].update(colors)


class FilledButton(ctk.CTkButton):
    """색을 채운 주요 버튼. CustomTkinter는 꺼진 버튼도 같은 색으로 칠해 눌러도 될 것처럼 보여서,
    꺼지면 회색 바탕·흐린 글자로 바꾼다."""

    def __init__(self, master, fg_color=ACCENT, hover_color=ACCENT_HOVER, **kwargs):
        self._on_colors = (fg_color, hover_color)
        super().__init__(master, fg_color=fg_color, hover_color=hover_color,
                         text_color_disabled=DISABLED_TEXT, **kwargs)
        self._sync_fill()

    def configure(self, require_redraw=False, **kwargs):
        if "fg_color" in kwargs or "hover_color" in kwargs:
            self._on_colors = (kwargs.pop("fg_color", self._on_colors[0]),
                               kwargs.pop("hover_color", self._on_colors[1]))
            require_redraw = True
        super().configure(require_redraw=require_redraw, **kwargs)
        self._sync_fill()

    def _sync_fill(self):
        fill, hover = (DISABLED_FILL, DISABLED_FILL) if self._state == "disabled" else self._on_colors
        if self._fg_color != fill or self._hover_color != hover:
            super().configure(fg_color=fill, hover_color=hover)