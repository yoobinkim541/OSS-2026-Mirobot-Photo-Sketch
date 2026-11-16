"""
로컬 브리지 — CLI(Claude Code / Codex)의 MCP 서버가 실행 중인 앱의 도구를 부르는 통로
======================================================================================
앱(GUI)이 127.0.0.1의 임의 포트에 작은 HTTP 서버를 띄우고, MCP 서버 프로세스는
환경 변수(MIROBOT_BRIDGE_URL / MIROBOT_BRIDGE_TOKEN)로 받은 주소와 토큰으로 도구를
호출합니다. 외부에서 접속할 수 없도록 루프백에만 묶고, 매 요청에 토큰을 확인합니다.
"""

import hmac
import json
import secrets
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .tools import tools_for

TOKEN_HEADER = "X-Mirobot-Token"


class BridgeServer:
    def __init__(self, toolbox):
        self.toolbox = toolbox
        self.token = secrets.token_urlsafe(24)
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # 콘솔에 요청 로그를 찍지 않음
                pass

            def _reply(self, code, obj):
                body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _authorized(self):
                return hmac.compare_digest(self.headers.get(TOKEN_HEADER, ""), bridge.token)

            def do_GET(self):
                if not self._authorized():
                    return self._reply(403, {"error": "forbidden"})
                if self.path == "/tools":   # 세션의 실제 조절 범위(예: box_mm 상한)로 만든 목록
                    return self._reply(200, {"tools": tools_for(bridge.toolbox.session)})
                self._reply(404, {"error": "not found"})

            def do_POST(self):
                if not self._authorized():
                    return self._reply(403, {"error": "forbidden"})
                if self.path != "/call":
                    return self._reply(404, {"error": "not found"})
                length = int(self.headers.get("Content-Length", "0"))
                req = json.loads(self.rfile.read(length) or b"{}")
                parts, is_error = bridge.toolbox.call(req.get("name"), req.get("args") or {})
                self._reply(200, {"parts": parts, "is_error": is_error})

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()

    def env(self):
        """MCP 서버 프로세스에 넘길 환경 변수."""
        return {"MIROBOT_BRIDGE_URL": self.url, "MIROBOT_BRIDGE_TOKEN": self.token}


class BridgeClient:
    def __init__(self, url, token):
        self.url, self.token = url.rstrip("/"), token

    def _request(self, path, payload=None, timeout=600):
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(self.url + path, data=data, method="POST" if data else "GET",
                                     headers={TOKEN_HEADER: self.token, "Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def tools(self):
        return self._request("/tools")["tools"]

    def call(self, name, args):
        r = self._request("/call", {"name": name, "args": args})
        return r["parts"], r["is_error"]
