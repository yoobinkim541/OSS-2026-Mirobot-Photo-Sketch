"""
RViz 3D 환경 설치 도우미 (WSL2 + ROS 2 Humble + WLKATA Mirobot 모델)
====================================================================
세 단계를 매번 처음부터 확인하고, 끝난 단계는 건너뜁니다.
  ① WSL 기능      — 없으면 관리자 창에서 `wsl --install --no-distribution` (사용자가 승인 후 재부팅)
  ② RViz 환경     — 릴리스의 미리 만든 이미지를 받아(이어받기) SHA256 확인 → 압축 풀기 → `wsl --import`
  ③ 동작 확인     — ROS·모델 패키지와 WSLg 화면 소켓 확인
예비 경로: Ubuntu 공식 22.04 WSL 루트 파일을 전용 배포판으로 가져와 그 안에서 packaging/wsl/setup_ros_env.sh 실행.
비밀번호는 받지도 저장하지도 않습니다. 사용자의 기존 배포판은 건드리지 않습니다.
"""

import fnmatch
import gzip
import errno
import hashlib
import os
import shutil
import subprocess
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path

from . import __version__, paths
from .rviz_launch import ROS_CHECK, _no_window, list_distros, to_wsl_path

DISTRO = "MirobotSketch-ROS"
IMAGE_NAME = "MirobotSketch-ROS-humble-{version}.tar.gz"
REPO = "yoobinkim541/OSS-2026-Mirobot-Photo-Sketch"
MIN_FREE_BYTES = 5 * 2**30               # 가져오기 직전에 필요한 여유 공간
PRE_DOWNLOAD_FREE_BYTES = 8 * 2**30      # 받기 전: 압축 파일 + 풀린 tar + 배포판 디스크
# 예비 경로: Ubuntu 공식 WSL 루트 파일을 전용 배포판으로 가져와 그 안에서 설치 스크립트를 실행
UBUNTU_ROOTFS_DIR = "https://cloud-images.ubuntu.com/wsl/releases/jammy/current/"
UBUNTU_ROOTFS_NAME = "ubuntu-jammy-wsl-amd64-wsl.rootfs.tar.gz"
VERIFY_CMD = ("source /opt/ros/humble/setup.bash && source ~/mirobot_ws/install/setup.bash"
              " && ros2 pkg prefix wlkata_mirobot_description >/dev/null && test -S /tmp/.X11-unix/X0")
CHUNK = 1 << 20

WSL_HINT = ("WSL 기능이 없습니다. [WSL 설치(관리자)]를 누르고 관리자 승인 → PC 재부팅 후 [이어서 설치]를 누르세요.\n"
            "승인 뒤 '가상화를 사용할 수 없음(0x80370102)'이 나오면 BIOS에서 가상화(VT-x/SVM)를 켜야 합니다.")
ENV_HINT = "[설치]를 누르면 RViz 환경 이미지(약 400MB, 설치 후 약 2GB)를 받아 자동으로 설치합니다."
VERIFY_HINT = ("ROS·Mirobot 모델 또는 WSLg 화면이 확인되지 않습니다. `wsl --update`로 WSL을 최신으로 한 뒤"
               " 다시 확인하고, 그래도 안 되면 [제거] 후 [설치]하세요.")


@dataclass
class SetupStep:
    id: str
    label: str
    state: str          # "ok" | "missing" | "blocked"(앞 단계가 안 돼서 확인 못 함)
    detail: str = ""
    hint: str = ""

    @property
    def ok(self):
        return self.state == "ok"


class SetupError(RuntimeError):
    def __init__(self, step, message, hint=""):
        super().__init__(message)
        self.step, self.message, self.hint = step, message, hint


def install_dir():
    base = os.environ.get("LOCALAPPDATA")
    return Path(base) / "MirobotSketch" / "wsl" if base else paths.user_dir() / "wsl"


def script_path():
    """예비 설치 스크립트 (저장소 packaging/wsl/ 또는 설치판 _internal/wsl/)."""
    root = paths.repo_root()
    if root is not None:
        return root / "packaging" / "wsl" / "setup_ros_env.sh"
    return Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent)) / "wsl" / "setup_ros_env.sh"


def _wsl(run, *args, timeout=120):
    return run(["wsl.exe", *args], capture_output=True, timeout=timeout, creationflags=_no_window())


def _in_distro(run, distro, script, timeout=120):
    try:
        return _wsl(run, "-d", distro, "--", "bash", "-lc", script, timeout=timeout).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def check(run=subprocess.run, distro=DISTRO):
    """[① wsl, ② env, ③ verify] 상태."""
    try:
        wsl_ok = _wsl(run, "--status", timeout=30).returncode == 0
    except (OSError, subprocess.SubprocessError):
        wsl_ok = False
    steps = [SetupStep("wsl", "① WSL 기능", "ok" if wsl_ok else "missing", hint="" if wsl_ok else WSL_HINT)]
    if not wsl_ok:
        return steps + [SetupStep("env", "② RViz 환경", "blocked"), SetupStep("verify", "③ 동작 확인", "blocked")]
    env_ok = distro in list_distros(run) and _in_distro(run, distro, ROS_CHECK)
    steps.append(SetupStep("env", "② RViz 환경", "ok" if env_ok else "missing",
                           detail=f"WSL 배포판 {distro}", hint="" if env_ok else ENV_HINT))
    if not env_ok:
        return steps + [SetupStep("verify", "③ 동작 확인", "blocked")]
    v_ok = _in_distro(run, distro, VERIFY_CMD)
    steps.append(SetupStep("verify", "③ 동작 확인", "ok" if v_ok else "missing", hint="" if v_ok else VERIFY_HINT))
    return steps


def _requests_get():
    import requests
    return requests.get


def image_urls(version=__version__, get=None):
    """(이미지 주소, .sha256 주소). 같은 버전 릴리스를 먼저, 없으면 이미지가 있는 최신 릴리스."""
    get = get or _requests_get()
    name = IMAGE_NAME.format(version=version)
    url = f"https://github.com/{REPO}/releases/download/v{version}/{name}"
    try:
        if get(url + ".sha256", timeout=30, allow_redirects=True).status_code == 200:
            return url, url + ".sha256"
    except Exception:
        pass
    try:
        r = get(f"https://api.github.com/repos/{REPO}/releases", timeout=30)
        releases = r.json() if r.status_code == 200 else []
    except Exception as e:
        raise SetupError("env", f"릴리스 목록을 받지 못했습니다: {e}", "인터넷 연결을 확인하세요.") from e
    for rel in releases:
        assets = {a["name"]: a["browser_download_url"] for a in rel.get("assets", [])}
        for n, u in assets.items():
            if fnmatch.fnmatch(n, IMAGE_NAME.format(version="*")) and n + ".sha256" in assets:
                return u, assets[n + ".sha256"]
    raise SetupError("env", "릴리스에서 RViz 환경 이미지를 찾지 못했습니다.", "[직접 설치(예비)]를 사용하세요.")


def download(url, dest, progress=None, should_cancel=None, get=None):
    """dest.part로 받으며 이어받기. 끝나면 dest로 바꿔 돌려줌."""
    get = get or _requests_get()
    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = Path(str(dest) + ".part")
    have = part.stat().st_size if part.exists() else 0
    try:
        r = get(url, headers={"Range": f"bytes={have}-"} if have else {}, stream=True, timeout=60)
    except Exception as e:
        raise SetupError("env", f"다운로드 실패: {e}", "인터넷 연결을 확인하고 [이어서 설치]를 누르세요.") from e
    if r.status_code == 206:
        mode = "ab"
    elif r.status_code == 200:
        mode, have = "wb", 0          # 서버가 Range를 무시 → 처음부터
    else:
        raise SetupError("env", f"다운로드 실패 (HTTP {r.status_code})", "잠시 뒤 [이어서 설치]를 누르세요.")
    total = have + int(r.headers.get("Content-Length") or 0)
    done = have
    with open(part, mode) as f:
        for chunk in r.iter_content(CHUNK):
            if should_cancel and should_cancel():
                raise SetupError("env", "다운로드를 멈췄습니다.", "[이어서 설치]를 누르면 받던 곳부터 이어받습니다.")
            f.write(chunk)
            done += len(chunk)
            if progress:
                progress(done, total)
    os.replace(part, dest)
    return dest


def verify_sha256(path, expected):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(CHUNK), b""):
            h.update(b)
    return h.hexdigest().lower() == expected.strip().split()[0].lower()


def gunzip(src, dst, progress=None, should_cancel=None):
    total = Path(src).stat().st_size
    try:
        with open(src, "rb") as raw, gzip.GzipFile(fileobj=raw) as g, open(dst, "wb") as out:
            for b in iter(lambda: g.read(CHUNK), b""):
                if should_cancel and should_cancel():
                    raise SetupError("env", "압축 풀기를 멈췄습니다.")
                out.write(b)
                if progress:
                    progress(raw.tell(), total)
    except OSError as e:
        Path(dst).unlink(missing_ok=True)
        if e.errno == errno.ENOSPC:              # 디스크가 가득 참: 파일은 멀쩡하므로 "손상"이라고 하지 않음
            raise SetupError("env", "디스크 공간이 부족해 압축을 풀지 못했습니다.",
                             f"{install_dir().drive or install_dir()} 드라이브의 공간을 비운 뒤 [이어서 설치]를 누르세요."
                             ) from e
        raise SetupError("env", f"이미지 파일이 손상되었습니다: {e}", "[설치]를 다시 누르면 새로 받습니다.") from e
    except (EOFError, zlib.error) as e:
        Path(dst).unlink(missing_ok=True)
        raise SetupError("env", f"이미지 파일이 손상되었습니다: {e}", "[설치]를 다시 누르면 새로 받습니다.") from e


def _check_space(need, when):
    base = install_dir()
    base.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(base).free
    if free < need:
        raise SetupError("env", f"디스크 공간이 부족합니다 (남은 공간 {free / 2**30:.1f}GB, {when} {need / 2**30:.0f}GB 필요).",
                         f"{base.drive or base} 드라이브의 공간을 비운 뒤 다시 시도하세요.")


def _fetch_verified(url, sha_text, dest, progress, should_cancel, get):
    """SHA256이 맞는 파일 (이미 받아 둔 파일이 맞으면 다시 받지 않음). 틀리면 지우고 SetupError."""
    dest = Path(dest)
    if dest.exists() and verify_sha256(dest, sha_text):
        return dest
    gz = download(url, dest, progress, should_cancel, get)
    if not verify_sha256(gz, sha_text):
        gz.unlink(missing_ok=True)
        raise SetupError("env", "받은 파일의 SHA256이 맞지 않습니다.",
                         "[설치]를 다시 누르면 새로 받습니다. 계속되면 [직접 설치(예비)]를 쓰세요.")
    return gz


def _unpack_and_import(gz, name, run, progress, should_cancel):
    report = (lambda stage: (lambda d, t: progress(stage, d, t))) if progress else (lambda stage: None)
    tar = Path(str(gz)[:-3]) if str(gz).endswith(".gz") else Path(str(gz) + ".tar")
    try:
        gunzip(gz, tar, report("압축 풀기"), should_cancel)
        if progress:
            progress("가져오기", 0, 0)
        import_distro(tar, name, run)
    finally:
        tar.unlink(missing_ok=True)


def _wsl_text(raw):
    raw = raw or b""
    if b"\x00" in raw:
        text = raw.decode("utf-16-le", errors="ignore")
    else:
        text = raw.decode(errors="ignore")
    return text.replace("\x00", "").strip()


def import_distro(tar_path, name=DISTRO, run=subprocess.run):
    _check_space(MIN_FREE_BYTES, "가져오기에")
    base = install_dir()
    r = _wsl(run, "--import", name, str(base / name), str(tar_path), "--version", "2", timeout=1800)
    if r.returncode != 0:
        out = _wsl_text(r.stdout)
        raise SetupError("env", f"WSL 가져오기 실패: {out}", "`wsl --update` 후 다시 시도하세요.")


def install(progress=None, should_cancel=None, run=subprocess.run, get=None, version=__version__,
            name=DISTRO, image=None):
    """②를 설치하고 세 단계 상태를 돌려줌. progress(단계 이름, 완료, 전체). image=로컬 .tar.gz(시험용)."""
    steps = check(run, name)
    if not steps[0].ok:
        raise SetupError("wsl", "WSL 기능이 없습니다.", WSL_HINT)
    if steps[1].ok:
        return steps
    if name in list_distros(run):          # 등록은 됐는데 ROS 확인 실패: 다시 가져오면 "이미 있음"으로 실패함
        raise SetupError("env", f"{name} 배포판이 있지만 ROS·모델 확인에 실패했습니다.",
                         "[다시 확인]을 눌러 보고, 계속되면 [직접 설치(예비)]로 이어서 설치하거나 [제거] 후 [설치]하세요.")
    if image is not None:
        _unpack_and_import(Path(image), name, run, progress, should_cancel)
        return check(run, name)
    _check_space(PRE_DOWNLOAD_FREE_BYTES, "설치에")
    get = get or _requests_get()
    url, sha_url = image_urls(version, get)
    try:
        sha = get(sha_url, timeout=30).text
    except Exception as e:
        raise SetupError("env", f"SHA256 파일을 받지 못했습니다: {e}", "인터넷 연결을 확인하세요.") from e
    report = (lambda d, t: progress("다운로드", d, t)) if progress else None
    gz = _fetch_verified(url, sha, install_dir() / IMAGE_NAME.format(version=version), report, should_cancel, get)
    _unpack_and_import(gz, name, run, progress, should_cancel)
    gz.unlink(missing_ok=True)
    return check(run, name)


def install_wsl_feature(popen=subprocess.Popen):
    """관리자 창에서 WSL 기능만 설치 (UAC 승인은 사용자가 함). 끝나면 재부팅이 필요."""
    return popen(["powershell.exe", "-NoProfile", "-Command",
                  "Start-Process wsl.exe -ArgumentList '--install','--no-distribution' -Verb RunAs"],
                 creationflags=_no_window())


def manual_install(progress=None, should_cancel=None, run=subprocess.run, get=None, popen=subprocess.Popen):
    """예비 경로 (미리 만든 이미지를 받을 수 없을 때): Ubuntu 공식 22.04 WSL 루트 파일을 전용 배포판 DISTRO로
    가져온 뒤, 새 콘솔에서 그 안의 root로 설치 스크립트(--image)를 실행. 이미 있으면 스크립트만 다시 실행
    (끝난 단계는 건너뜀). 사용자의 기존 배포판은 건드리지 않고, 비밀번호·추가 관리자 승인이 없음."""
    if not check(run, DISTRO)[0].ok:
        raise SetupError("wsl", "WSL 기능이 없습니다.", WSL_HINT)
    if DISTRO not in list_distros(run):
        _check_space(PRE_DOWNLOAD_FREE_BYTES, "설치에")
        get = get or _requests_get()
        try:
            sums = get(UBUNTU_ROOTFS_DIR + "SHA256SUMS", timeout=30).text
        except Exception as e:
            raise SetupError("env", f"Ubuntu 체크섬을 받지 못했습니다: {e}", "인터넷 연결을 확인하세요.") from e
        sha = next((ln.split()[0] for ln in sums.splitlines()
                    if ln.split() and ln.split()[-1].lstrip("*") == UBUNTU_ROOTFS_NAME), None)
        if sha is None:
            raise SetupError("env", "Ubuntu 루트 파일 체크섬을 찾지 못했습니다.", "잠시 뒤 다시 시도하세요.")
        report = (lambda d, t: progress("Ubuntu 22.04 받기", d, t)) if progress else None
        gz = _fetch_verified(UBUNTU_ROOTFS_DIR + UBUNTU_ROOTFS_NAME, sha, install_dir() / UBUNTU_ROOTFS_NAME,
                             report, should_cancel, get)
        _unpack_and_import(gz, DISTRO, run, progress, should_cancel)
        gz.unlink(missing_ok=True)
    flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
    return popen(["wsl.exe", "-d", DISTRO, "-u", "root", "--", "bash", to_wsl_path(script_path()), "--image"],
                 creationflags=flags)


def uninstall(run=subprocess.run, name=DISTRO):
    """이 도우미가 만든 배포판만 지움."""
    if name not in list_distros(run):
        return False
    _wsl(run, "--unregister", name, timeout=300)
    return True


def log(msg):
    """설치 기록 (사용자 폴더의 rviz_setup.log). 기록 실패는 무시."""
    import datetime
    try:
        with open(paths.user_dir() / "rviz_setup.log", "a", encoding="utf-8") as f:
            f.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {msg}\n")
    except OSError:
        pass


MARK = {"ok": "✓", "missing": "✕", "blocked": "○"}


def _print_steps(steps):
    for s in steps:
        print(f"{MARK[s.state]} {s.label}" + (f" — {s.detail}" if s.detail else ""))
    todo = next((s for s in steps if s.state == "missing"), None)
    print("RViz 3D 환경 준비 완료" if todo is None else f"할 일: {todo.hint}")


def _progress_printer():
    """단계마다 5% 단위로 한 줄씩 출력하는 progress(stage, done, total)."""
    last = {}

    def progress(stage, done, total):
        pct = int(100 * done / total) if total else 0
        if last.get(stage) != pct // 5:
            last[stage] = pct // 5
            print(f"{stage} {pct}%" if total else stage, flush=True)
    return progress


def _print_error(e):
    log(f"실패 ({e.step}): {e.message}")
    print(f"실패: {e.message}")
    if e.hint:
        print(e.hint)
    if e.step == "wsl":
        print("WSL 기능 설치: mirobot setup-rviz --wsl")


def main(argv=None):
    """mirobot setup-rviz --check | --install | --uninstall | --manual | --wsl"""
    paths.safe_console()
    import argparse
    ap = argparse.ArgumentParser(prog="mirobot setup-rviz", description="RViz 3D 환경(WSL2 + ROS 2 Humble) 설치 도우미")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--check", action="store_true", help="상태 확인 (준비되면 종료 코드 0)")
    g.add_argument("--install", action="store_true", help="RViz 환경 이미지를 받아 설치 (이어받기)")
    g.add_argument("--uninstall", action="store_true", help=f"{DISTRO} 배포판 제거")
    g.add_argument("--manual", action="store_true",
                   help="예비: Ubuntu 공식 루트 파일로 전용 배포판을 만들고 설치 스크립트 실행 (새 콘솔)")
    g.add_argument("--wsl", action="store_true", help="WSL 기능 설치 (관리자 승인 창, 끝나면 재부팅)")
    a = ap.parse_args(argv)
    if a.install:
        try:
            log("설치 시작")
            steps = install(progress=_progress_printer())
        except SetupError as e:
            _print_error(e)
            return 1
        log("설치 끝: " + ", ".join(f"{s.id}={s.state}" for s in steps))
        _print_steps(steps)
        return 0 if all(s.ok for s in steps) else 1
    if a.uninstall:
        print("제거했습니다." if uninstall() else f"{DISTRO} 배포판이 없습니다.")
        log("제거")
        return 0
    if a.manual:
        try:
            log("직접 설치 시작")
            manual_install(progress=_progress_printer())
        except SetupError as e:
            _print_error(e)
            return 1
        print("새 콘솔에서 설치 스크립트가 진행됩니다(약 10분). 끝나면 mirobot setup-rviz --check로 확인하세요.")
        log("직접 설치: 스크립트 실행")
        return 0
    if a.wsl:
        install_wsl_feature()
        print("관리자 승인 창에서 [예]를 누르고, 끝나면 PC를 재부팅한 뒤 다시 설치하세요.")
        return 0
    steps = check()
    _print_steps(steps)
    return 0 if all(s.ok for s in steps) else 1


if __name__ == "__main__":
    sys.exit(main())
