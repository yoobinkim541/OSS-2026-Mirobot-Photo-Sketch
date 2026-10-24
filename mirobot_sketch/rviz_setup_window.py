"""
'RViz 3D 환경' 설치 도우미 창 — 단계(① WSL ② RViz 환경 ③ 동작 확인), 진행 막대, 지금 할 일
==========================================================================================
확인·설치는 작업 스레드에서 rviz_setup으로 하고, 화면은 app.ui()로 갱신합니다.
비밀번호는 다루지 않습니다. 관리자 승인은 [WSL 설치(관리자)]의 Windows 승인 창에서 사용자가 합니다.
"""

import os
import sys
import threading
from tkinter import messagebox

import customtkinter as ctk

from . import paths, rviz_launch
from . import rviz_setup as rs

from .theme import ACCENT as ACT_C, BAD as BAD_C, MUTED as IDLE_C, OK as OK_C, SURFACE, TEXT
MARK = {"ok": ("✓", OK_C), "missing": ("✕", BAD_C), "blocked": ("○", IDLE_C)}


class RvizSetupWindow(ctk.CTkToplevel):
    def __init__(self, app, font):
        super().__init__(app.root)
        self.app, self.font = app, font
        self.busy, self.cancel = False, False
        self.title("RViz 3D 환경")
        self.geometry("620x360")
        self.protocol("WM_DELETE_WINDOW", self.close)

        ctk.CTkLabel(self, text="RViz 3D 환경 (WSL2 + ROS 2 Humble + Mirobot 모델)", font=font(15, "bold"),
                     anchor="w").pack(fill="x", padx=16, pady=(14, 6))
        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.pack(fill="x", padx=16)
        self.steps = {}
        for k, (sid, label) in enumerate((("wsl", "① WSL 기능"), ("env", "② RViz 환경"), ("verify", "③ 동작 확인"))):
            if k:
                ctk.CTkLabel(bar, text="─", text_color=IDLE_C).pack(side="left", padx=4)
            lb = ctk.CTkLabel(bar, text=f"○ {label}", font=font(13), text_color=IDLE_C)
            lb.pack(side="left")
            self.steps[sid] = lb
        self.todo = ctk.CTkLabel(self, text="상태를 확인하는 중...", font=font(13, "bold"), anchor="w",
                                 justify="left", wraplength=580)
        self.todo.pack(fill="x", padx=16, pady=(14, 4))
        self.detail = ctk.CTkLabel(self, text="", font=font(12), anchor="w", justify="left", wraplength=580)
        self.detail.pack(fill="x", padx=16)
        self.bar = ctk.CTkProgressBar(self, height=8)
        self.bar.pack(fill="x", padx=16, pady=10)
        self.bar.set(0)

        row = ctk.CTkFrame(self, fg_color="transparent")
        row.pack(fill="x", padx=16, pady=(4, 4))
        self.install_btn = ctk.CTkButton(row, text="설치", command=self.install, font=font(13, "bold"), width=120)
        self.install_btn.pack(side="left")
        self.wsl_btn = ctk.CTkButton(row, text="WSL 설치(관리자)", command=self.install_wsl, font=font(12), width=130)
        self.manual_btn = ctk.CTkButton(row, text="직접 설치(예비)", command=self.manual, font=font(12), width=120,
                                        fg_color="transparent", border_width=1, text_color=TEXT, hover_color=SURFACE)
        self.manual_btn.pack(side="left", padx=6)
        row2 = ctk.CTkFrame(self, fg_color="transparent")
        row2.pack(fill="x", padx=16, pady=(4, 12))
        for text, cmd in (("다시 확인", self.refresh), ("제거", self.uninstall), ("로그 열기", self.open_log),
                          ("닫기", self.close)):
            ctk.CTkButton(row2, text=text, command=cmd, font=font(12), width=90, fg_color="transparent",
                          border_width=1, text_color=TEXT, hover_color=SURFACE).pack(side="left", padx=(0, 6))
        self.refresh()

    # ------------------------------------------------------------ 표시
    def _alive(self):
        try:
            return bool(self.winfo_exists())
        except Exception:
            return False

    def _show(self, steps):
        if not self._alive():
            return
        for s in steps:
            mark, color = MARK[s.state]
            self.steps[s.id].configure(text=f"{mark} {self.steps[s.id].cget('text')[2:]}", text_color=color)
        todo = next((s for s in steps if s.state == "missing"), None)
        if todo is None:
            self.todo.configure(text="RViz 3D 환경이 준비되었습니다. 'RViz 3D로 보기'와 실시간 따라가기를 쓸 수 있습니다.",
                                text_color=OK_C)
            self.detail.configure(text="")
            self.bar.set(1)
        else:
            self.todo.configure(text=f"지금 할 일: {todo.hint}", text_color=("#111827", "#f3f4f6"))
        wsl_missing = steps[0].state == "missing"
        if wsl_missing:
            self.wsl_btn.pack(side="left", padx=6, after=self.install_btn)
        else:
            self.wsl_btn.pack_forget()
        try:                                  # 받다 만 파일이 있으면 "이어서 설치"
            started = any(True for _ in rs.install_dir().glob("*.part"))
        except Exception:
            started = False
        self.install_btn.configure(text="이어서 설치" if started else "설치",
                                   state="disabled" if (self.busy or todo is None or wsl_missing) else "normal")
        # 예비 경로도 WSL 기능이 있어야 함 (없을 때 누르면 관리자 승인이 한 번 더 뜨는 경로를 막음)
        self.manual_btn.configure(state="disabled" if (self.busy or wsl_missing) else "normal")

    def _progress(self, stage, done, total):
        if not self._alive():
            return
        if total:
            self.bar.set(done / total)
            self.detail.configure(text=f"{stage} {done / 2**20:,.0f} / {total / 2**20:,.0f} MB")
        else:
            self.bar.configure(mode="indeterminate")
            self.bar.start()
            self.detail.configure(text=f"{stage}... (몇 분 걸릴 수 있음)")

    def _error(self, e):
        if not self._alive():
            return
        self.busy = False
        self.bar.stop()
        self.bar.configure(mode="determinate")
        self.todo.configure(text=f"실패: {e.message}", text_color=BAD_C)
        self.detail.configure(text=e.hint)
        self.install_btn.configure(state="normal", text="이어서 설치")

    # ------------------------------------------------------------ 동작
    def _work(self, fn, *args):
        def run():
            try:
                steps = fn(*args)
            except rs.SetupError as e:
                rs.log(f"실패 ({e.step}): {e.message}")
                self.app.ui(self._error, e)
                return
            except Exception as e:        # 예상 못 한 오류도 창에 표시
                self.app.ui(self._error, rs.SetupError("env", str(e), "로그를 확인하세요."))
                return
            self.app.ui(self._done, steps)
        threading.Thread(target=run, daemon=True).start()

    def _done(self, steps):
        self.busy = False
        rviz_launch._distro_cache.clear()     # 설치·제거 뒤 'RViz 3D로 보기'가 새 배포판을 다시 찾게
        if self._alive():
            self.bar.stop()
            self.bar.configure(mode="determinate")
            self._show(steps)

    def refresh(self):
        if self.busy:                          # 설치·제거가 도는 중에는 확인하지 않음 (버튼이 다시 켜지지 않게)
            return
        self.busy = True
        self._work(rs.check)

    def install(self):
        if self.busy:
            return
        self.busy, self.cancel = True, False
        self.install_btn.configure(state="disabled")
        self.todo.configure(text="설치 중... 창을 닫으면 멈추고, 다음에 이어서 받습니다.")
        rs.log("설치 시작 (GUI)")
        self._work(lambda: rs.install(progress=lambda st, d, t: self.app.ui(self._progress, st, d, t),
                                      should_cancel=lambda: self.cancel))

    def install_wsl(self):
        if messagebox.askokcancel("WSL 설치", "Windows 관리자 승인 창이 뜹니다. [예]를 누르고, 설치가 끝나면 PC를 "
                                              "재부팅한 뒤 이 창에서 [설치]를 누르세요.", parent=self):
            rs.install_wsl_feature()
            rs.log("WSL 기능 설치 요청")

    def manual(self):
        if self.busy:
            return
        if not messagebox.askokcancel(
                "직접 설치(예비)", f"Ubuntu 공식 22.04 루트 파일(약 230MB)을 받아 전용 배포판 {rs.DISTRO}를 만들고,\n"
                                "새 콘솔에서 ROS 2와 Mirobot 모델을 설치합니다(약 10분, 비밀번호 필요 없음).\n"
                                "기존 WSL 배포판은 건드리지 않습니다. 콘솔이 끝나면 [다시 확인]을 누르세요.", parent=self):
            return
        self.busy, self.cancel = True, False
        self.todo.configure(text="직접 설치 준비 중... (Ubuntu 22.04 받기 → 가져오기 → 새 콘솔에서 설치)")
        rs.log("직접 설치 시작 (GUI)")

        def work():
            rs.manual_install(progress=lambda st, d, t: self.app.ui(self._progress, st, d, t),
                              should_cancel=lambda: self.cancel)
            return rs.check()
        self._work(work)

    def uninstall(self):
        if self.busy:
            return
        if messagebox.askyesno("제거", f"WSL 배포판 {rs.DISTRO}를 지웁니다(다른 배포판은 그대로). 계속할까요?",
                               parent=self):
            self.busy = True
            rs.log("제거")
            self._work(lambda: (rs.uninstall(), rs.check())[1])

    def open_log(self):
        p = paths.user_dir() / "rviz_setup.log"
        if not p.exists():
            p.write_text("", encoding="utf-8")
        if sys.platform == "win32":
            os.startfile(str(p))   # noqa: S606 - 사용자가 누른 로그 열기

    def close(self):
        self.cancel = True
        self.destroy()
