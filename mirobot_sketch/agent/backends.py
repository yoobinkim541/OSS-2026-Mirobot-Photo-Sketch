"""
대화 백엔드 — OpenRouter(API 키) / Claude Code / Codex(로컬 CLI)
=================================================================
모두 같은 인터페이스를 가집니다.
    backend.send(user_text, on_event)   # 작업 스레드에서 호출, 끝날 때까지 블록
    backend.reset()                     # 새 대화
    backend.cancel()                    # 진행 중인 요청 중단

on_event(dict)로 화면에 알리는 이벤트:
    {"type": "text", "text": ...}                 에이전트의 답변
    {"type": "tool", "name": ..., "args": {...}}  도구 호출 시작
    {"type": "tool_done", "name": ..., "ok": bool}
    {"type": "error", "text": ...}
    {"type": "done", "info": "..."}               한 번의 요청 처리 끝 (비용 등)
"""

import json
import os
import shutil
import subprocess
import sys
import threading
from pathlib import Path

from .. import paths
from .tools import SYSTEM_PROMPT, TOOLS

OPENROUTER_URL = "https://openrouter.ai/api/v1"
DEFAULT_OPENROUTER_MODEL = "anthropic/claude-opus-5"
MAX_TOOL_ROUNDS = 16


class Cancelled(Exception):
    pass


# ---------------------------------------------------------------------------
# OpenRouter (앱이 직접 API 호출 + 도구 실행)
# ---------------------------------------------------------------------------

def fetch_openrouter_models(timeout=20):
    """도구 호출과 이미지 입력을 둘 다 지원하는 모델 id 목록 (일괄 처리 전용 :batch 제외)."""
    import requests

    data = requests.get(f"{OPENROUTER_URL}/models", timeout=timeout).json()["data"]
    ids = [m["id"] for m in data
           if "tools" in (m.get("supported_parameters") or [])
           and "image" in (m.get("architecture", {}).get("input_modalities") or [])
           and not m["id"].endswith(":batch")]
    return sorted(ids)


class OpenRouterBackend:
    label = "OpenRouter"

    def __init__(self, toolbox, api_key_getter, model=DEFAULT_OPENROUTER_MODEL):
        self.toolbox = toolbox
        self.api_key_getter = api_key_getter
        self.model = model
        self._cancel = threading.Event()
        self.reset()

    def reset(self):
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    def cancel(self):
        self._cancel.set()

    def _post(self, body):
        import requests

        key = self.api_key_getter()
        if not key:
            raise RuntimeError("OpenRouter API 키가 없습니다. 패널의 설정(⚙)에서 입력하세요.")
        r = requests.post(
            f"{OPENROUTER_URL}/chat/completions", json=body, timeout=300,
            headers={"Authorization": f"Bearer {key}",
                     "HTTP-Referer": "https://github.com/yoobinkim541/OSS-2026-Mirobot-Photo-Sketch",
                     "X-Title": "Mirobot Sketch"})
        try:
            data = r.json()
        except ValueError:
            raise RuntimeError(f"OpenRouter 응답을 읽을 수 없습니다 (HTTP {r.status_code})")
        if r.status_code != 200 or "error" in data:
            msg = (data.get("error") or {}).get("message", r.text[:300])
            hint = {401: " (API 키 확인)", 402: " (크레딧 부족)", 429: " (요청 한도, 잠시 후 다시)"}.get(r.status_code, "")
            raise RuntimeError(f"OpenRouter 오류 {r.status_code}: {msg}{hint}")
        return data

    def send(self, user_text, on_event):
        self._cancel.clear()
        self.messages.append({"role": "user", "content": user_text})
        tools = [{"type": "function", "function": {"name": t["name"], "description": t["description"],
                                                   "parameters": t["parameters"]}} for t in TOOLS]
        cost = 0.0
        try:
            for _ in range(MAX_TOOL_ROUNDS):
                if self._cancel.is_set():
                    raise Cancelled()
                data = self._post({"model": self.model, "messages": self.messages, "tools": tools,
                                   "tool_choice": "auto", "usage": {"include": True}})
                cost += float((data.get("usage") or {}).get("cost") or 0)
                msg = data["choices"][0]["message"]
                calls = msg.get("tool_calls") or []
                self.messages.append({"role": "assistant", "content": msg.get("content") or "",
                                      **({"tool_calls": calls} if calls else {})})
                if msg.get("content"):
                    on_event({"type": "text", "text": msg["content"]})
                if not calls:
                    break
                images = []
                for call in calls:
                    name = call["function"]["name"]
                    try:
                        args = json.loads(call["function"].get("arguments") or "{}")
                    except json.JSONDecodeError:
                        args, parts, is_error = {}, [{"type": "text", "text": "오류: 인자 JSON을 읽을 수 없습니다."}], True
                    else:
                        on_event({"type": "tool", "name": name, "args": args})
                        parts, is_error = self.toolbox.call(name, args)
                    on_event({"type": "tool_done", "name": name, "ok": not is_error})
                    text = "\n".join(p["text"] for p in parts if p["type"] == "text") or "(그림)"
                    self.messages.append({"role": "tool", "tool_call_id": call["id"], "content": text})
                    images += [p for p in parts if p["type"] == "image"]
                if images:
                    # 도구 결과 메시지는 대부분의 모델에서 글자만 받으므로, 그림은 사용자 메시지로 붙임
                    self.messages.append({"role": "user", "content": [
                        {"type": "text", "text": "(방금 호출한 view 도구의 결과 그림입니다)"},
                        *[{"type": "image_url", "image_url": {"url": f"data:{p['mime']};base64,{p['data']}"}}
                          for p in images]]})
            else:
                on_event({"type": "error", "text": f"도구 호출이 {MAX_TOOL_ROUNDS}번을 넘어 멈췄습니다."})
        except Cancelled:
            on_event({"type": "error", "text": "중단했습니다."})
        except Exception as e:
            on_event({"type": "error", "text": str(e)})
        on_event({"type": "done", "info": f"{self.model} · ${cost:.4f}" if cost else self.model})


# ---------------------------------------------------------------------------
# 로컬 CLI 공통 (MCP 서버로 앱 도구를 연결)
# ---------------------------------------------------------------------------

def mcp_server_command():
    """CLI가 띄울 MCP 서버 명령. .exe 배포판이면 mirobot.exe mcp, 아니면 python -m ..."""
    if paths.FROZEN:
        return str(Path(sys.executable).with_name("mirobot.exe")), ["mcp"]
    py = Path(sys.executable)
    if py.name.lower() == "pythonw.exe" and py.with_name("python.exe").exists():
        py = py.with_name("python.exe")   # pythonw에는 콘솔 표준 입출력이 없음
    return str(py), ["-m", "mirobot_sketch.agent.mcp_server"]


def mcp_env(bridge):
    env = dict(bridge.env())
    if not paths.FROZEN:
        env["PYTHONPATH"] = str(paths.PACKAGE_DIR.parent)
    env["PYTHONIOENCODING"] = "utf-8"
    return env


def _agent_workdir():
    d = paths.user_dir() / "agent"   # 저장소 밖 빈 폴더: 프로젝트 설정·파일을 읽지 않게
    d.mkdir(parents=True, exist_ok=True)
    return d


def _no_window():
    return subprocess.CREATE_NO_WINDOW if sys.platform.startswith("win") else 0


# 앱이 다른 Claude Code 세션(예: 데스크톱 앱) 안에서 실행되면, 그 세션 전용 환경 변수가
# 자식 CLI로 넘어가 "로그인 갱신은 호스트가 한다"로 동작해 인증이 실패함. 걸러낸다.
_HOST_MARKERS = ("CLAUDECODE", "CLAUDE_CODE_ENTRYPOINT", "CLAUDE_CODE_SDK_HAS_HOST_AUTH_REFRESH")
_HOST_PREFIXES = ("CLAUDE_CODE_", "CLAUDE_AGENT_SDK", "CLAUDE_PREVIEW_")
_HOST_EXACT = ("CLAUDECODE", "CLAUDE_PID", "CLAUDE_EFFORT")
_HOST_INJECTED_WHEN_NESTED = ("ANTHROPIC_BASE_URL", "ANTHROPIC_AUTH_TOKEN")


def clean_cli_env(environ=None):
    env = dict(os.environ if environ is None else environ)
    nested = any(k in env for k in _HOST_MARKERS)
    for k in list(env):
        if k in _HOST_EXACT or k.startswith(_HOST_PREFIXES) or (nested and k in _HOST_INJECTED_WHEN_NESTED):
            del env[k]
    return env


class _CliBackend:
    def __init__(self, bridge, model=None):
        self.bridge = bridge
        self.model = model or None
        self.session_id = None
        self.proc = None

    def reset(self):
        self.session_id = None

    def cancel(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()

    def _run(self, cmd, stdin_text, on_line):
        self.proc = subprocess.Popen(
            cmd, cwd=_agent_workdir(), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", errors="replace", creationflags=_no_window(), env=clean_cli_env())
        err_lines = []
        t = threading.Thread(target=lambda: err_lines.extend(self.proc.stderr), daemon=True)
        t.start()
        self.proc.stdin.write(stdin_text)
        self.proc.stdin.close()
        for line in self.proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            on_line(ev)
        code = self.proc.wait()
        t.join(timeout=2)
        return code, "".join(err_lines).strip()


def open_login_console(argv):
    """로그인 명령을 새 콘솔 창에서 실행 (브라우저 로그인은 사용자가 직접). 성공하면 True."""
    env = clean_cli_env()
    try:
        if sys.platform == "win32":
            # cmd /k: 로그인 후에도 창이 남아 결과를 확인할 수 있게
            subprocess.Popen(["cmd", "/k", subprocess.list2cmdline(argv)], env=env,
                             creationflags=subprocess.CREATE_NEW_CONSOLE)
        else:
            subprocess.Popen(argv, env=env, start_new_session=True)
        return True
    except OSError:
        return False


# ---------------------------------------------------------------------------
# Claude Code (claude -p --output-format stream-json)
# ---------------------------------------------------------------------------

class ClaudeCodeBackend(_CliBackend):
    label = "Claude Code"

    @staticmethod
    def available():
        return shutil.which("claude")

    def _mcp_config_file(self):
        cmd, args = mcp_server_command()
        cfg = {"mcpServers": {"mirobot": {"command": cmd, "args": args, "env": mcp_env(self.bridge)}}}
        p = _agent_workdir() / "claude_mcp.json"
        p.write_text(json.dumps(cfg, ensure_ascii=False), encoding="utf-8")
        return p

    LOGIN_HINT = ("Claude Code에 로그인되어 있지 않습니다.\n"
                  "→ 아래 [로그인 창 열기](= claude auth login)를 눌러 브라우저로 로그인한 뒤 다시 보내세요.\n"
                  "  (Claude 데스크톱 앱의 로그인과 claude 명령의 로그인은 따로 관리됩니다)")

    def login_argv(self):
        exe = self.available()
        return [exe, "auth", "login"] if exe else None

    @staticmethod
    def logged_in(exe):
        """claude auth status로 로그인 여부 확인 (모델 호출 없음). 확인할 수 없으면 None."""
        try:
            r = subprocess.run([exe, "auth", "status"], capture_output=True, text=True, encoding="utf-8",
                               errors="replace", env=clean_cli_env(), timeout=30, creationflags=_no_window())
            return bool(json.loads(r.stdout).get("loggedIn"))
        except (OSError, ValueError, subprocess.SubprocessError):
            return None

    def send(self, user_text, on_event):
        exe = self.available()
        if not exe:
            on_event({"type": "error", "text": "claude 명령을 찾을 수 없습니다. Claude Code를 설치하고 로그인하세요."})
            on_event({"type": "done", "info": ""})
            return
        if self.logged_in(exe) is False:   # 요청을 보내 15초쯤 기다렸다 실패하기 전에 미리 안내
            on_event({"type": "error", "text": self.LOGIN_HINT, "action": "login"})
            on_event({"type": "done", "info": ""})
            return
        cmd = [exe, "-p", "--output-format", "stream-json", "--verbose",
               "--mcp-config", str(self._mcp_config_file()), "--strict-mcp-config",
               "--tools", "", "--allowedTools", "mcp__mirobot", "--permission-mode", "dontAsk",
               "--append-system-prompt", SYSTEM_PROMPT]
        if self.session_id:
            cmd += ["--resume", self.session_id]
        if self.model:
            cmd += ["--model", self.model]
        info = {"text": "", "failed": False, "last_text": ""}

        def on_line(ev):
            t = ev.get("type")
            if t == "system" and ev.get("subtype") == "init":
                self.session_id = ev.get("session_id") or self.session_id
                servers = {s.get("name"): s.get("status") for s in ev.get("mcp_servers") or []}
                if servers.get("mirobot") not in (None, "connected"):
                    on_event({"type": "error", "text": f"앱 도구(MCP) 연결 실패: {servers.get('mirobot')}"})
            elif t == "assistant":
                for block in (ev.get("message") or {}).get("content") or []:
                    if block.get("type") == "text" and block.get("text", "").strip():
                        info["last_text"] = block["text"].strip()
                        on_event({"type": "text", "text": block["text"]})
                    elif block.get("type") == "tool_use":
                        on_event({"type": "tool", "name": block.get("name", "").replace("mcp__mirobot__", ""),
                                  "args": block.get("input") or {}})
            elif t == "user":
                for block in (ev.get("message") or {}).get("content") or []:
                    if isinstance(block, dict) and block.get("type") == "tool_result":
                        on_event({"type": "tool_done", "name": "", "ok": not block.get("is_error")})
            elif t == "result":
                self.session_id = ev.get("session_id") or self.session_id
                cost = ev.get("total_cost_usd")
                info["text"] = "Claude Code" + (f" · ${cost:.4f}" if cost else "")
                if ev.get("is_error"):
                    info["failed"] = True
                    msg = str(ev.get("result") or ev.get("subtype") or "실패").strip()
                    hint, action = "", None
                    if "authenticate" in msg.lower() or "login" in msg.lower():
                        hint, action = "\n→ 아래 [로그인 창 열기](= claude auth login)로 다시 로그인하세요.", "login"
                    if msg == info["last_text"]:  # 같은 문구가 이미 답변으로 나갔으면 반복하지 않음
                        msg = "위 오류로 요청을 처리하지 못했습니다."
                    on_event({"type": "error", "text": msg + hint, "action": action})

        try:
            code, err = self._run(cmd, user_text, on_line)
            if code not in (0, None) and not info["failed"]:
                on_event({"type": "error", "text": err[-500:] or f"claude 종료 코드 {code}"})
        except Exception as e:
            on_event({"type": "error", "text": str(e)})
        on_event({"type": "done", "info": info["text"]})


# ---------------------------------------------------------------------------
# Codex (codex exec --json)
# ---------------------------------------------------------------------------

def _codex_executable():
    """npm이 만든 codex.CMD 대신 실제 codex.exe를 찾음 (cmd.exe의 인자 해석 문제 회피)."""
    found = shutil.which("codex")
    if not found:
        return None
    if found.lower().endswith((".cmd", ".bat")):
        root = Path(found).parent / "node_modules" / "@openai" / "codex"
        for exe in root.rglob("codex.exe"):
            if exe.parent.name == "bin":
                return str(exe)
    return found


def _toml_str(s):
    return "'" + str(s).replace("'", "") + "'"   # TOML 리터럴 문자열 (역슬래시 그대로)


class CodexBackend(_CliBackend):
    label = "Codex"

    @staticmethod
    def available():
        return _codex_executable()

    def login_argv(self):
        exe = self.available()
        return [exe, "login"] if exe else None

    def _config_args(self):
        cmd, args = mcp_server_command()
        env = ", ".join(f"{k} = {_toml_str(v)}" for k, v in mcp_env(self.bridge).items())
        return ["-c", f"mcp_servers.mirobot.command={_toml_str(cmd)}",
                "-c", "mcp_servers.mirobot.args=[" + ", ".join(_toml_str(a) for a in args) + "]",
                "-c", "mcp_servers.mirobot.env={" + env + "}",
                "-c", 'sandbox_mode="read-only"',
                "-c", 'approval_policy="never"']

    def send(self, user_text, on_event):
        exe = self.available()
        if not exe:
            on_event({"type": "error", "text": "codex 명령을 찾을 수 없습니다. Codex CLI를 설치하고 로그인하세요."})
            on_event({"type": "done", "info": ""})
            return
        if self.session_id:
            cmd = [exe, "exec", "resume", self.session_id, "--json", "--skip-git-repo-check", *self._config_args(), "-"]
            prompt = user_text
        else:
            cmd = [exe, "exec", "--json", "--skip-git-repo-check", *self._config_args(), "-"]
            prompt = f"{SYSTEM_PROMPT}\n\n--- 사용자 요청 ---\n{user_text}"
        if self.model:
            cmd[2:2] = ["-m", self.model] if not self.session_id else []
        info = {"failed": False, "usage": ""}

        def on_line(ev):
            t = ev.get("type", "")
            item = ev.get("item") or {}
            it = item.get("type") or item.get("item_type")
            if t == "thread.started":
                self.session_id = ev.get("thread_id") or self.session_id
            elif t == "item.started" and it == "mcp_tool_call":
                on_event({"type": "tool", "name": item.get("tool", ""), "args": item.get("arguments") or {}})
            elif t == "item.completed" and it == "mcp_tool_call":
                on_event({"type": "tool_done", "name": item.get("tool", ""),
                          "ok": item.get("status") not in ("failed", "error") and not item.get("error")})
            elif t == "item.completed" and it in ("agent_message", "assistant_message"):
                if item.get("text", "").strip():
                    on_event({"type": "text", "text": item["text"]})
            elif t == "turn.completed":
                u = ev.get("usage") or {}
                info["usage"] = f"Codex · 입력 {u.get('input_tokens', '?')} / 출력 {u.get('output_tokens', '?')} 토큰" if u else "Codex"
            elif t in ("turn.failed", "error"):
                info["failed"] = True
                msg = ev.get("message") or (ev.get("error") or {}).get("message") or json.dumps(ev, ensure_ascii=False)[:300]
                on_event({"type": "error", "text": msg})

        try:
            code, err = self._run(cmd, prompt, on_line)
            if code not in (0, None) and not info["failed"]:
                on_event({"type": "error", "text": err[-500:] or f"codex 종료 코드 {code}"})
        except Exception as e:
            on_event({"type": "error", "text": str(e)})
        on_event({"type": "done", "info": info["usage"] or "Codex"})


# ---------------------------------------------------------------------------
# 설정·키 저장
# ---------------------------------------------------------------------------

KEYRING_SERVICE = "MirobotSketch"


BACKEND_ORDER = ("Claude Code", "Codex", "OpenRouter")


def backend_ready(name):
    """이 연결 방식이 지금 바로 쓸 수 있는 상태인지 (CLI 설치 / OpenRouter 키 저장)."""
    if name == "Claude Code":
        return bool(ClaudeCodeBackend.available())
    if name == "Codex":
        return bool(CodexBackend.available())
    return bool(get_openrouter_key())


def default_backend_name(settings):
    """패널이 시작할 때 고를 연결 방식: 저장한 선택이 지금 쓸 수 있으면 그대로, 아니면 쓸 수 있는 것 중 첫 번째
    (설치 안 된 방식에서 첫 메시지를 보내고서야 오류가 나는 것을 막음). 하나도 준비되지 않았으면 저장값(없으면 Claude Code).
    설정 파일은 바꾸지 않는다 (사용자가 직접 고를 때만 저장)."""
    saved = settings.get("backend")
    if saved in BACKEND_ORDER and backend_ready(saved):
        return saved
    ready = next((n for n in BACKEND_ORDER if backend_ready(n)), None)
    if ready:
        return ready
    return saved if saved in BACKEND_ORDER else "Claude Code"


def get_openrouter_key():
    """OS 자격 증명 저장소(Windows 자격 증명 관리자) > 환경 변수 OPENROUTER_API_KEY."""
    try:
        import keyring
        key = keyring.get_password(KEYRING_SERVICE, "openrouter")
        if key:
            return key
    except Exception:
        pass
    return os.environ.get("OPENROUTER_API_KEY")


def set_openrouter_key(key):
    import keyring

    if key:
        keyring.set_password(KEYRING_SERVICE, "openrouter", key)
    else:
        try:
            keyring.delete_password(KEYRING_SERVICE, "openrouter")
        except Exception:
            pass


def settings_path():
    return paths.user_dir() / "agent_settings.json"


def load_settings():
    try:
        return json.loads(settings_path().read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_settings(s):
    settings_path().write_text(json.dumps(s, ensure_ascii=False, indent=1), encoding="utf-8")
