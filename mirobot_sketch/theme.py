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