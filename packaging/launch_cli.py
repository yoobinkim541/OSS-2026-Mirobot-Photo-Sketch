"""PyInstaller용 명령줄 진입점 (mirobot.exe).

    mirobot draw <strokes.json> [--execute ...]    = mirobot-draw
    mirobot strokes <image> [--type photo ...]     = mirobot-strokes
    mirobot svg <strokes.json> [-o out.svg]        = mirobot-svg (펜 플로터·GRBL용 SVG)
    mirobot sim <strokes.json> [--gif ...]         = mirobot-sim
    mirobot mcp                                    = 에이전트 MCP 서버 (앱이 CLI에 넘겨 자동 실행)
    mirobot setup-rviz [--check / --install ...]    = RViz 3D 환경 설치 도우미
"""
import sys

COMMANDS = {
    "draw": "mirobot_sketch.draw_executor",
    "strokes": "mirobot_sketch.make_strokes",
    "svg": "mirobot_sketch.svg_export",
    "sim": "mirobot_sketch.mirobot_sim",
    "mcp": "mirobot_sketch.agent.mcp_server",
    "setup-rviz": "mirobot_sketch.rviz_setup",
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in COMMANDS:
        print(__doc__)
        return 1
    import importlib

    module = importlib.import_module(COMMANDS[sys.argv[1]])
    sys.argv = [f"mirobot {sys.argv[1]}"] + sys.argv[2:]
    return module.main()


if __name__ == "__main__":
    sys.exit(main())
