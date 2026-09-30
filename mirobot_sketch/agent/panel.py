"""
오른쪽 에이전트 채팅 패널 (CustomTkinter) — 앱의 라이트/다크 테마를 그대로 따름
===========================================================================
머리글(제목·새 대화·설정·닫기) / 대화 영역(빈 화면 안내, 말풍선, 도구 칩, 오류 카드) /
입력창(여러 줄, Enter 전송, Shift+Enter 줄바꿈, 빠른 요청 +, 연결 방식 선택, 전송·중지 버튼).

백엔드 호출은 작업 스레드에서 하고, 화면 갱신은 app.ui(fn, *args)로 메인 스레드에 넘깁니다.
"""

import threading
import time
import tkinter as tk

import customtkinter as ctk

from . import backends as bk

PANEL_BG = ("#f7f8fa", "#171a21")
BUBBLE = ("#e8ecf3", "#2a2f3a")
CARD = ("#ffffff", "#1f2430")
BORDER = ("#dde2ea", "#303644")
TEXT = ("#111827", "#e5e7eb")
MUTED = ("#6b7280", "#9aa3b2")
ACCENT = ("#2563eb", "#3b82f6")
ERR_BG = ("#fdecec", "#3a2226")
ERR_FG = ("#b42318", "#f7a8a8")

BACKENDS = ["Claude Code", "Codex", "OpenRouter"]
QUICK_PROMPTS = [
    "지금 결과를 보고 어떻게 개선하면 좋을지 제안해줘",
    "배경이나 테두리에 있는 잡음 획을 찾아서 지워줘",
    "주요 윤곽은 유지하면서 15분 안에 그릴 수 있게 줄여줘",
    "얼굴 디테일을 더 살려줘",
    "사진과 최대한 비슷하고 예쁘게 다듬어줘 (필요하면 명암 빗금도 써줘)",
    "로봇 시뮬레이션으로 관절 한계를 확인해줘",
    "이 그림을 로봇으로 그릴 준비가 됐는지 확인하고 순서를 알려줘",
]


class AgentPanel(ctk.CTkFrame):
    def __init__(self, master, app, toolbox, bridge, font, icon_image=None, on_close=None):
        super().__init__(master, fg_color=PANEL_BG, corner_radius=14, width=420)
        self.app, self.toolbox, self.bridge, self.font = app, toolbox, bridge, font
        self.on_close = on_close
        self.settings = bk.load_settings()
        self.backend_name = bk.default_backend_name(self.settings)   # 설치 안 된 방식이면 쓸 수 있는 방식으로
        self.backends = {}
        self.busy = False
        self._current_tools = []
        self._tool_started = {}        # 도구 칩 -> 시작 시각 (오래 걸리는 도구의 경과 시간 표시)
        self._thinking = None          # "생각하는 중…" 표시 (첫 응답이 오기 전)
        self._thinking_since = 0.0
        self._tick_after = None
        self.grid_propagate(False)
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self._build(icon_image)
        self._show_empty()

    # ------------------------------------------------------------ 화면
    def _build(self, icon_image):
        head = ctk.CTkFrame(self, fg_color="transparent")
        head.grid(row=0, column=0, sticky="ew", padx=14, pady=(12, 4))
        if icon_image is not None:
            ctk.CTkLabel(head, image=icon_image, text="").pack(side="left", padx=(0, 8))
        ctk.CTkLabel(head, text="에이전트", font=self.font(15, "bold")).pack(side="left")
        for txt, cmd, tip in (("✕", self._close, "닫기"), ("⚙", self.open_settings, "설정"),
                              ("＋ 새 대화", self.new_chat, "새 대화")):
            ctk.CTkButton(head, text=txt, command=cmd, width=34 if len(txt) < 3 else 80, height=28,
                          fg_color="transparent", hover_color=BUBBLE, text_color=TEXT,
                          font=self.font(13)).pack(side="right", padx=2)

        self.chat = ctk.CTkScrollableFrame(self, fg_color="transparent")
        self.chat.grid(row=1, column=0, sticky="nsew", padx=6, pady=4)
        self.chat.grid_columnconfigure(0, weight=1)
        self.icon_image = icon_image

        comp = ctk.CTkFrame(self, fg_color=CARD, corner_radius=18, border_width=1, border_color=BORDER)
        comp.grid(row=2, column=0, sticky="ew", padx=12, pady=(4, 12))
        comp.grid_columnconfigure(0, weight=1)
        self.input = ctk.CTkTextbox(comp, height=64, fg_color="transparent", font=self.font(13), wrap="word",
                                    border_width=0, activate_scrollbars=False)
        self.input.grid(row=0, column=0, columnspan=4, sticky="ew", padx=10, pady=(8, 0))
        self._placeholder_on()
        self.input.bind("<FocusIn>", lambda e: self._placeholder_off())
        self.input.bind("<FocusOut>", lambda e: self._placeholder_on() if not self._text() else None)
        self.input.bind("<Return>", self._on_enter)

        row = ctk.CTkFrame(comp, fg_color="transparent")
        row.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        self.plus = ctk.CTkButton(row, text="＋", width=32, height=32, corner_radius=16, fg_color="transparent",
                                  hover_color=BUBBLE, text_color=TEXT, font=self.font(16), command=self._quick_menu)
        self.plus.pack(side="left")
        self.backend_menu = ctk.CTkOptionMenu(row, values=BACKENDS, command=self._on_backend, width=120, height=28,
                                              font=self.font(12), dropdown_font=self.font(12),
                                              fg_color=BUBBLE, button_color=BUBBLE, button_hover_color=BORDER,
                                              text_color=TEXT)
        self.backend_menu.set(self.backend_name)
        self.backend_menu.pack(side="left", padx=(6, 0))
        self.model_label = ctk.CTkLabel(row, text="", font=self.font(11), text_color=MUTED)
        self.model_label.pack(side="left", padx=6)
        self.send_btn = ctk.CTkButton(row, text="↑", width=34, height=34, corner_radius=17, fg_color=ACCENT,
                                      font=self.font(16, "bold"), command=self._send_or_stop)
        self.send_btn.pack(side="right")
        self._update_model_label()

    def _clear_chat(self):
        for w in self.chat.winfo_children():
            w.destroy()

    def _show_empty(self):
        self._clear_chat()
        box = ctk.CTkFrame(self.chat, fg_color="transparent")
        box.grid(row=0, column=0, pady=(120, 0))
        if self.icon_image is not None:
            ctk.CTkLabel(box, image=self.icon_image, text="").pack()
        ctk.CTkLabel(box, text="무엇을 도와드릴까요?", font=self.font(16, "bold")).pack(pady=(10, 4))
        ctk.CTkLabel(box, text="대화로 처리 설정을 바꾸고 획을 정리하고, 로봇으로 그릴 준비가 됐는지 확인합니다.\n"
                               "예: “배경 잡음을 지워줘”, “10분 안에 끝나게 해줘”, “로봇으로 그려도 되는지 봐줘”",
                     font=self.font(12), text_color=MUTED, justify="center", wraplength=340).pack()
        ctk.CTkLabel(box, text="로봇 연결·시작은 사용자가 '로봇으로 그리기' 창에서 직접 합니다.", font=self.font(11), text_color=MUTED).pack(pady=(8, 0))
        self._empty = True

    def _row(self):
        if getattr(self, "_empty", False):
            self._clear_chat()
            self._empty = False
        return len(self.chat.winfo_children())

    def add_user(self, text):
        r = self._row()
        wrap = ctk.CTkFrame(self.chat, fg_color="transparent")
        wrap.grid(row=r, column=0, sticky="e", padx=(60, 8), pady=6)
        ctk.CTkLabel(wrap, text=text, font=self.font(13), fg_color=BUBBLE, corner_radius=14, text_color=TEXT,
                     wraplength=280, justify="left", padx=12, pady=8).pack(anchor="e")
        self._scroll_end()

    def add_assistant(self, text):
        r = self._row()
        ctk.CTkLabel(self.chat, text=text.strip(), font=self.font(13), text_color=TEXT, wraplength=360,
                     justify="left", anchor="w").grid(row=r, column=0, sticky="w", padx=10, pady=6)
        self._scroll_end()

    def add_tool(self, name, args):
        r = self._row()
        brief = ", ".join(f"{k}={v}" for k, v in list(args.items())[:4]) if isinstance(args, dict) else ""
        if len(brief) > 60:
            brief = brief[:57] + "…"
        if name == "simulate":
            brief = (brief + "  " if brief else "") + "(그림에 따라 10~30초)"
        chip = ctk.CTkLabel(self.chat, text=f"⚙ {name}  {brief}  …", font=self.font(11), text_color=MUTED,
                            fg_color=BUBBLE, corner_radius=10, padx=10, pady=3, anchor="w")
        chip.grid(row=r, column=0, sticky="w", padx=10, pady=2)
        self._current_tools.append((chip, name, brief))
        self._tool_started[chip] = time.monotonic()
        self._start_ticker()
        self._scroll_end()

    def tool_done(self, ok):
        if not self._current_tools:
            return
        chip, name, brief = self._current_tools.pop(0)
        started = self._tool_started.pop(chip, None)
        took = f"  {time.monotonic() - started:.0f}초" if started and time.monotonic() - started >= 3 else ""
        chip.configure(text=f"{'✓' if ok else '✕'} {name}  {brief}{took}", text_color=MUTED if ok else ERR_FG)

    def add_error(self, text, action=None):
        r = self._row()
        card = ctk.CTkFrame(self.chat, fg_color=ERR_BG, corner_radius=14)
        card.grid(row=r, column=0, sticky="ew", padx=8, pady=6)
        ctk.CTkLabel(card, text="⚠  " + text, font=self.font(12), text_color=ERR_FG, wraplength=360,
                     justify="left", anchor="w").pack(fill="x", padx=12, pady=(10, 4 if action else 10))
        if action == "login":
            ctk.CTkButton(card, text="로그인 창 열기", font=self.font(12, "bold"), height=30, corner_radius=15,
                          fg_color=ACCENT, command=self.open_login).pack(anchor="w", padx=12, pady=(0, 10))
        self._scroll_end()

    def open_login(self):
        """현재 백엔드의 로그인 명령을 새 콘솔 창에서 실행 (브라우저 로그인은 사용자가 직접)."""
        b = self._backend()
        argv = b.login_argv() if hasattr(b, "login_argv") else None
        if not argv:
            self.add_error("이 연결 방식은 로그인 명령이 없습니다. OpenRouter는 ⚙ 설정에서 API 키를 넣으세요."
                           if self.backend_name == "OpenRouter" else f"{self.backend_name} 명령을 찾을 수 없습니다.")
            return
        if bk.open_login_console(argv):
            self.add_info(f"새 창에서 '{' '.join(['claude' if 'claude' in argv[0].lower() else 'codex', *argv[1:]])}'"
                          " 실행 중 — 브라우저에서 로그인을 마친 뒤 다시 보내세요")
        else:
            self.add_error("로그인 창을 열지 못했습니다. 명령 프롬프트에서 직접 실행하세요: " + " ".join(argv))

    # ------------------------------------------------------------ 기다리는 동안의 표시
    def _show_thinking(self):
        """응답이 오기 전에 '생각하는 중… N초'를 보여 멈춘 것처럼 보이지 않게 함."""
        r = self._row()
        self._thinking = ctk.CTkLabel(self.chat, text="생각하는 중… 0초", font=self.font(11), text_color=MUTED,
                                      anchor="w")
        self._thinking.grid(row=r, column=0, sticky="w", padx=12, pady=(2, 6))
        self._thinking_since = time.monotonic()
        self._start_ticker()
        self._scroll_end()

    def _hide_thinking(self):
        if self._thinking is not None:
            try:
                self._thinking.destroy()
            except tk.TclError:
                pass
            self._thinking = None

    def _start_ticker(self):
        if self._tick_after is None:
            self._tick_after = self.after(1000, self._tick)

    def _tick(self):
        self._tick_after = None
        now = time.monotonic()
        try:
            if self._thinking is not None:
                self._thinking.configure(text=f"생각하는 중… {now - self._thinking_since:.0f}초")
            for chip, name, brief in self._current_tools:
                t0 = self._tool_started.get(chip)
                if t0 is not None and now - t0 >= 3:
                    chip.configure(text=f"⚙ {name}  {brief}  … {now - t0:.0f}초")
        except tk.TclError:      # 새 대화로 위젯이 지워진 경우
            self._thinking = None
        if self._thinking is not None or self._current_tools:
            self._tick_after = self.after(1000, self._tick)

    def add_info(self, text):
        r = self._row()
        ctk.CTkLabel(self.chat, text=text, font=self.font(10), text_color=MUTED).grid(row=r, column=0, sticky="w",
                                                                                       padx=12, pady=(0, 6))
        self._scroll_end()

    def _scroll_end(self):
        self.after(30, lambda: self.chat._parent_canvas.yview_moveto(1.0))

    # ------------------------------------------------------------ 입력
    PLACEHOLDER = "무엇이든 요청하세요"

    def _placeholder_on(self):
        if not self._text():
            self.input.delete("1.0", "end")
            self.input.insert("1.0", self.PLACEHOLDER)
            self.input.configure(text_color=MUTED)
            self._ph = True

    def _placeholder_off(self):
        if getattr(self, "_ph", False):
            self.input.delete("1.0", "end")
            self.input.configure(text_color=TEXT)
            self._ph = False

    def _text(self):
        if getattr(self, "_ph", False):
            return ""
        return self.input.get("1.0", "end").strip()

    def _on_enter(self, event):
        if event.state & 0x0001:   # Shift+Enter는 줄바꿈
            return None
        self._send_or_stop()
        return "break"

    def _quick_menu(self):
        menu = tk.Menu(self, tearoff=0)
        for p in QUICK_PROMPTS:
            menu.add_command(label=p, command=lambda p=p: self._fill(p))
        menu.tk_popup(self.plus.winfo_rootx(), self.plus.winfo_rooty() - 10 - 24 * len(QUICK_PROMPTS))

    def _fill(self, text):
        self._placeholder_off()
        self.input.delete("1.0", "end")
        self.input.insert("1.0", text)
        self.input.focus_set()

    # ------------------------------------------------------------ 백엔드
    def _on_backend(self, name):
        self.backend_name = name
        self.settings["backend"] = name
        bk.save_settings(self.settings)
        self._update_model_label()

    def _update_model_label(self):
        name = self.backend_name
        if name == "OpenRouter":
            m = self.settings.get("openrouter_model", bk.DEFAULT_OPENROUTER_MODEL)
            key = "" if bk.get_openrouter_key() else " · 키 없음"
            self.model_label.configure(text=m.split("/")[-1] + key)
        else:
            m = self.settings.get("claude_model" if name == "Claude Code" else "codex_model") or "기본 모델"
            ok = (bk.ClaudeCodeBackend if name == "Claude Code" else bk.CodexBackend).available()
            self.model_label.configure(text=m if ok else "설치 안 됨")

    def _backend(self):
        name = self.backend_name
        b = self.backends.get(name)
        if name == "OpenRouter":
            model = self.settings.get("openrouter_model", bk.DEFAULT_OPENROUTER_MODEL)
            if b is None:
                b = bk.OpenRouterBackend(self.toolbox, bk.get_openrouter_key, model)
            b.model = model
        elif name == "Claude Code":
            b = b or bk.ClaudeCodeBackend(self.bridge)
            b.model = self.settings.get("claude_model") or None
        else:
            b = b or bk.CodexBackend(self.bridge)
            b.model = self.settings.get("codex_model") or None
        self.backends[name] = b
        return b

    def new_chat(self):
        if self.busy:
            return
        for b in self.backends.values():
            b.reset()
        self._current_tools = []
        self._tool_started = {}
        self._thinking = None
        self._show_empty()

    def _send_or_stop(self):
        if self.busy:
            b = self.backends.get(self.backend_name)
            if b:
                b.cancel()
            return
        text = self._text()
        if not text:
            return
        self.input.delete("1.0", "end")
        if text.strip().lower() in ("/login", "/로그인"):   # CLI -p 모드에는 /login이 없으므로 패널에서 처리
            self.add_user(text)
            self.open_login()
            return
        self.add_user(text)
        self._set_busy(True)
        self._show_thinking()
        backend = self._backend()
        threading.Thread(target=self._worker, args=(backend, text), daemon=True).start()

    def _worker(self, backend, text):
        backend.send(text, lambda ev: self.app.ui(self._handle, ev))

    def _handle(self, ev):
        t = ev["type"]
        if t in ("text", "tool", "error", "done"):
            self._hide_thinking()
        if t == "text":
            self.add_assistant(ev["text"])
        elif t == "tool":
            self.add_tool(ev.get("name", ""), ev.get("args") or {})
        elif t == "tool_done":
            self.tool_done(ev.get("ok", True))
        elif t == "error":
            self.add_error(ev["text"], ev.get("action"))
        elif t == "done":
            for chip, name, brief in self._current_tools:
                chip.configure(text=f"· {name}  {brief}")
            self._current_tools = []
            if ev.get("info"):
                self.add_info(ev["info"])
            self._set_busy(False)

    def _set_busy(self, busy):
        self.busy = busy
        self.send_btn.configure(text="■" if busy else "↑")
        self.backend_menu.configure(state="disabled" if busy else "normal")
        self.app.agent_busy(busy)

    def _close(self):
        if self.on_close:
            self.on_close()

    # ------------------------------------------------------------ 설정 창
    def open_settings(self):
        win = ctk.CTkToplevel(self)
        win.title("에이전트 설정")
        win.geometry("460x420")
        win.transient(self.winfo_toplevel())
        win.after(100, win.grab_set)
        pad = {"padx": 16, "anchor": "w"}

        ctk.CTkLabel(win, text="OpenRouter", font=self.font(14, "bold")).pack(pady=(14, 2), **pad)
        ctk.CTkLabel(win, text="API 키 (Windows 자격 증명 관리자에 저장, 화면·파일에 남기지 않음)",
                     font=self.font(11), text_color=MUTED).pack(**pad)
        key_entry = ctk.CTkEntry(win, show="•", width=420, font=self.font(12),
                                 placeholder_text="sk-or-… (비워 두면 기존 키 유지)")
        key_entry.pack(padx=16, pady=(2, 6))
        has_key = bool(bk.get_openrouter_key())
        ctk.CTkLabel(win, text="저장된 키: " + ("있음" if has_key else "없음"), font=self.font(11),
                     text_color=MUTED).pack(**pad)
        ctk.CTkLabel(win, text="모델 (도구 호출 + 이미지 입력 지원 모델만 표시)", font=self.font(12)).pack(pady=(8, 0), **pad)
        models = [self.settings.get("openrouter_model", bk.DEFAULT_OPENROUTER_MODEL)]
        model_box = ctk.CTkComboBox(win, values=models, width=420, font=self.font(12))
        model_box.set(models[0])
        model_box.pack(padx=16, pady=2)

        def load_models():
            try:
                ids = bk.fetch_openrouter_models()
                self.app.ui(lambda: model_box.configure(values=ids))
            except Exception as e:
                msg = f"(목록을 불러오지 못함: {e})"   # except 블록 밖 람다에서 e는 사라지므로 미리 문자열로
                self.app.ui(lambda: model_box.configure(values=models + [msg]))
        threading.Thread(target=load_models, daemon=True).start()

        ctk.CTkLabel(win, text="로컬 CLI 모델 (비워 두면 CLI 기본값)", font=self.font(14, "bold")).pack(pady=(16, 2), **pad)
        cl = ctk.CTkEntry(win, width=420, font=self.font(12), placeholder_text="Claude Code 예: opus, sonnet")
        cl.insert(0, self.settings.get("claude_model", ""))
        cl.pack(padx=16, pady=2)
        cx = ctk.CTkEntry(win, width=420, font=self.font(12), placeholder_text="Codex 예: gpt-5.6-luna")
        cx.insert(0, self.settings.get("codex_model", ""))
        cx.pack(padx=16, pady=2)

        def save():
            key = key_entry.get().strip()
            if key:   # 비워 두면 기존 키 유지 (삭제는 "키 삭제" 버튼)
                bk.set_openrouter_key(key)
            m = model_box.get().strip()
            if m and not m.startswith("("):
                self.settings["openrouter_model"] = m
            self.settings["claude_model"] = cl.get().strip()
            self.settings["codex_model"] = cx.get().strip()
            bk.save_settings(self.settings)
            self._update_model_label()
            win.destroy()

        def delete_key():
            bk.set_openrouter_key("")
            self._update_model_label()
            win.destroy()

        row = ctk.CTkFrame(win, fg_color="transparent")
        row.pack(fill="x", padx=16, pady=16)
        ctk.CTkButton(row, text="저장", command=save, fg_color=ACCENT, font=self.font(13)).pack(side="right")
        ctk.CTkButton(row, text="키 삭제", command=delete_key, fg_color="transparent", border_width=1,
                      text_color=TEXT, font=self.font(12)).pack(side="left")
