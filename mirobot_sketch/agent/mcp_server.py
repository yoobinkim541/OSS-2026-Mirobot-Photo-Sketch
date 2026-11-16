"""
MCP 서버 (stdio) — Claude Code / Codex가 앱 도구를 쓰게 해 주는 어댑터
=====================================================================
CLI가 이 프로세스를 띄우면, 도구 목록과 호출을 로컬 브리지(bridge.py)를 통해 실행 중인
앱으로 전달합니다. 앱 없이 단독으로는 동작하지 않습니다(MIROBOT_BRIDGE_URL 필요).

    python -m mirobot_sketch.agent.mcp_server      (앱이 CLI 설정에 넣어 자동으로 실행)
    mirobot.exe mcp                                (.exe 배포판)
"""

import os
import sys

import anyio
import mcp.types as types
from mcp.server import Server
from mcp.server.stdio import stdio_server

from .bridge import BridgeClient
from .tools import TOOLS


def build_server(client):
    async def list_tools(ctx, params):
        try:   # 앱 세션의 실제 조절 범위가 담긴 목록. 브리지가 없으면 기본 목록
            tools = await anyio.to_thread.run_sync(client.tools)
        except OSError:
            tools = TOOLS
        return types.ListToolsResult(tools=[
            types.Tool(name=t["name"], description=t["description"], input_schema=t["parameters"])
            for t in tools
        ])

    async def call_tool(ctx, params):
        parts, is_error = await anyio.to_thread.run_sync(client.call, params.name, params.arguments or {})
        content = []
        for p in parts:
            if p["type"] == "image":
                content.append(types.ImageContent(data=p["data"], mime_type=p["mime"]))
            else:
                content.append(types.TextContent(text=p["text"]))
        return types.CallToolResult(content=content, is_error=is_error)

    return Server("mirobot", on_list_tools=list_tools, on_call_tool=call_tool)


def main():
    url, token = os.environ.get("MIROBOT_BRIDGE_URL"), os.environ.get("MIROBOT_BRIDGE_TOKEN")
    if not url or not token:
        print("MIROBOT_BRIDGE_URL / MIROBOT_BRIDGE_TOKEN이 없습니다. Mirobot Sketch 앱에서 실행해야 합니다.",
              file=sys.stderr)
        return 2
    server = build_server(BridgeClient(url, token))

    async def run():
        async with stdio_server() as (read, write):
            await server.run(read, write, server.create_initialization_options())

    anyio.run(run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
