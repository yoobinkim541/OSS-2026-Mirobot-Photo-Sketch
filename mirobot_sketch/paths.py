"""
파일 위치 규칙 — 저장소에서 실행할 때와 설치(.exe/pip)해서 실행할 때를 모두 지원
===============================================================================
- 저장소 체크아웃에서 실행: 지금까지처럼 robot/drawing_config.json, LOG/runs/,
  out/cache/, input/ 을 그대로 사용합니다.
- pip 설치나 .exe로 실행: 사용자 폴더(Windows: %APPDATA%/MirobotSketch,
  그 외: ~/.mirobot-sketch)를 씁니다. 설정 파일이 없으면 패키지에 들어 있는
  기본 설정을 복사해 두므로, 사용자가 그 파일을 고쳐 포트·보정값을 바꿉니다.
- 환경 변수 MIROBOT_CONFIG로 설정 파일을 직접 지정할 수 있습니다.
"""

import os
import re
import shutil
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
DATA_DIR = PACKAGE_DIR / "data"          # 패키지에 함께 들어가는 기본 설정·아이콘
FROZEN = getattr(sys, "frozen", False)   # PyInstaller로 만든 .exe에서 실행 중인지


def repo_root():
    """저장소 체크아웃이면 그 루트, 아니면 None."""
    if FROZEN:
        return None
    root = PACKAGE_DIR.parent
    if (root / "robot" / "drawing_config.json").exists() and (root / "LOG").is_dir():
        return root
    return None


def user_dir():
    base = os.environ.get("APPDATA")
    d = Path(base) / "MirobotSketch" if base else Path.home() / ".mirobot-sketch"
    d.mkdir(parents=True, exist_ok=True)
    return d


def config_path():
    """로봇 설정 파일 위치 (환경 변수 > 저장소 > 사용자 폴더)."""
    env = os.environ.get("MIROBOT_CONFIG")
    if env:
        return Path(env)
    root = repo_root()
    if root:
        return root / "robot" / "drawing_config.json"
    p = user_dir() / "drawing_config.json"
    if not p.exists():
        shutil.copyfile(DATA_DIR / "drawing_config.json", p)
    return p


def runs_dir():
    root = repo_root()
    return root / "LOG" / "runs" if root else user_dir() / "runs"


def cache_dir():
    root = repo_root()
    return root / "out" / "cache" if root else user_dir() / "cache"


def output_dir():
    root = repo_root()
    d = root / "out" if root else user_dir() / "out"
    d.mkdir(parents=True, exist_ok=True)
    return d


def input_dir():
    root = repo_root()
    return root / "input" if root and (root / "input").is_dir() else Path.home()


def trajectories_dir():
    root = repo_root()
    return root / "trajectories" if root else None


_ABS_PATH = re.compile(r"^(?:[A-Za-z]:[\\/]|/(?:home|Users|mnt|root)/)")


def portable_path(value):
    """기록(LOG)에 남기는 경로에서 사용자 폴더 이름을 뺀다. 절대 경로가 아니면 그대로.
    저장소 안이면 저장소 기준 상대 경로(슬래시), 밖이면 파일 이름만. (실행·보정 기록을 공개 저장소에 올려도
    컴퓨터 사용자 이름이나 사진 폴더 이름이 남지 않게)"""
    text = str(value)
    if not _ABS_PATH.match(text):
        return value
    norm = text.replace("\\", "/")
    root = repo_root()
    if root is not None:
        prefix = str(root).replace("\\", "/").rstrip("/") + "/"
        if norm.lower().startswith(prefix.lower()):
            return norm[len(prefix):]
    return norm.rsplit("/", 1)[-1]


def scrub_paths(obj):
    """dict·list 안의 절대 경로 문자열을 모두 portable_path로 바꾼 사본."""
    if isinstance(obj, dict):
        return {k: scrub_paths(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [scrub_paths(v) for v in obj]
    if isinstance(obj, str):
        return portable_path(obj)
    return obj


def safe_console():
    """콘솔이 한글을 못 쓰는 환경(예: 영문 Windows의 cp1252)에서 print가 멈추지 않도록,
    인코딩할 수 없는 글자는 ?로 바꿔 출력합니다. 명령줄 도구 main() 시작에서 호출."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass


def asset(name):
    """아이콘 등 패키지 데이터 파일 경로."""
    return DATA_DIR / name
