"""RViz 3D 환경 설치 도우미 (WSL·네트워크는 가짜로 대체 — CI에서도 동작)."""

import gzip
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mirobot_sketch import rviz_setup as rs  # noqa: E402

SCRIPT = ROOT / "packaging" / "wsl" / "setup_ros_env.sh"


def cp(rc=0, out=b""):
    return subprocess.CompletedProcess([], rc, out, b"")


class FakeWsl:
    """wsl.exe 흉내: 배포판 목록(UTF-16), 상태, 배포판 안 확인 명령의 성공 여부."""

    def __init__(self, wsl=True, distros=("Ubuntu-22.04",), env_ok=(), verify_ok=()):
        self.wsl, self.distros = wsl, list(distros)
        self.env_ok, self.verify_ok = set(env_ok), set(verify_ok)
        self.calls = []

    def __call__(self, cmd, **kw):
        self.calls.append(cmd)
        if not self.wsl:
            raise FileNotFoundError("wsl.exe")
        if cmd[1:2] == ["--status"]:
            return cp(0)
        if cmd[1:3] == ["-l", "-q"]:
            return cp(0, ("\r\n".join(self.distros) + "\r\n").encode("utf-16-le"))
        if cmd[1] == "--import":
            self.distros.append(cmd[2])
            self.env_ok.add(cmd[2])
            self.verify_ok.add(cmd[2])
            return cp(0)
        if cmd[1] == "--unregister":
            self.distros.remove(cmd[2])
            return cp(0)
        if cmd[1] == "-d":
            name, script = cmd[2], cmd[-1]
            ok = name in (self.verify_ok if "X11" in script else self.env_ok)
            return cp(0 if ok else 1)
        return cp(1)


class FakeResp:
    def __init__(self, status, body=b"", headers=None):
        self.status_code, self.body = status, body
        self.headers = {"Content-Length": str(len(body)), **(headers or {})}
        self.text = body.decode("utf-8", "replace")

    def iter_content(self, n):
        for i in range(0, len(self.body), 3):
            yield self.body[i:i + 3]

    def json(self):
        return json.loads(self.body)


class SetupScriptTest(unittest.TestCase):
    def test_script_contents(self):
        src = SCRIPT.read_bytes()
        self.assertNotIn(b"\r\n", src)                       # WSL bash는 CRLF를 못 읽음
        text = src.decode("utf-8")
        for needle in ("set -euo pipefail", "c0a7ad4", "--packages-select wlkata_mirobot_description",
                       "textures", "ros-humble-rviz2", "ros-humble-robot-state-publisher", "SETUP_OK",
                       "--image", "default=mirobot", " g++",
                       "share/wlkata_mirobot_description/package.xml", "IMAGE_PKGS=\"sudo\"",
                       "mkdir -p /etc/sudoers.d"):
            self.assertIn(needle, text, needle)

    def test_script_syntax(self):
        bash = shutil.which("bash")
        if not bash or sys.platform == "win32":
            self.skipTest("리눅스 bash 없음 (Windows에서는 CI Ubuntu에서 검사)")
        r = subprocess.run([bash, "-n", str(SCRIPT)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)


class CheckTest(unittest.TestCase):
    def test_no_wsl(self):
        st = rs.check(run=FakeWsl(wsl=False))
        self.assertEqual([s.state for s in st], ["missing", "blocked", "blocked"])
        self.assertIn("관리자", st[0].hint)

    def test_no_distro_then_ready(self):
        self.assertEqual([s.state for s in rs.check(run=FakeWsl())], ["ok", "missing", "blocked"])
        w = FakeWsl(distros=["Ubuntu", rs.DISTRO], env_ok=[rs.DISTRO], verify_ok=[rs.DISTRO])
        self.assertEqual([s.state for s in rs.check(run=w)], ["ok", "ok", "ok"])

    def test_verify_failure_is_reported(self):
        w = FakeWsl(distros=[rs.DISTRO], env_ok=[rs.DISTRO])
        st = rs.check(run=w)
        self.assertEqual([s.state for s in st], ["ok", "ok", "missing"])
        self.assertTrue(st[2].hint)


class DownloadTest(unittest.TestCase):
    def test_resume_with_range_and_restart_when_ignored(self):
        body = b"0123456789abcdef"
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / "img.tar.gz"
            part = Path(str(dest) + ".part")
            part.write_bytes(body[:5])
            seen = []

            def get206(url, headers=None, **kw):
                seen.append(headers)
                return FakeResp(206, body[5:])

            rs.download("u", dest, get=get206)
            self.assertEqual(seen[0], {"Range": "bytes=5-"})
            self.assertEqual(dest.read_bytes(), body)
            part.write_bytes(b"garbage")                  # 서버가 Range를 무시하고 200 -> 처음부터
            dest.unlink()
            rs.download("u", dest, get=lambda url, headers=None, **kw: FakeResp(200, body))
            self.assertEqual(dest.read_bytes(), body)

    def test_cancel_keeps_partial_file(self):
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / "img.tar.gz"
            with self.assertRaises(rs.SetupError):
                rs.download("u", dest, get=lambda *a, **k: FakeResp(200, b"x" * 30), should_cancel=lambda: True)
            self.assertFalse(dest.exists())

    def test_http_error(self):
        with tempfile.TemporaryDirectory() as d, self.assertRaises(rs.SetupError):
            rs.download("u", Path(d) / "a", get=lambda *a, **k: FakeResp(404))


class InstallTest(unittest.TestCase):
    def make_image(self, d):
        gz = Path(d) / "img.tar.gz"
        with gzip.open(gz, "wb") as g:
            g.write(b"rootfs" * 100)
        return gz, hashlib.sha256(gz.read_bytes()).hexdigest()

    def fake_get(self, gz, sha):
        def get(url, headers=None, **kw):
            if url.endswith(".sha256"):
                return FakeResp(200, f"{sha}  img.tar.gz\n".encode())
            return FakeResp(200, gz.read_bytes())
        return get

    def patches(self, d, free=100 * 2**30):
        return (mock.patch.object(rs, "install_dir", lambda: Path(d) / "wsl"),
                mock.patch.object(rs.shutil, "disk_usage", lambda p: mock.Mock(free=free)))

    def test_full_install_imports_and_cleans_up(self):
        with tempfile.TemporaryDirectory() as d:
            p1, p2 = self.patches(d)
            with p1, p2:
                gz, sha = self.make_image(d)
                w = FakeWsl()
                st = rs.install(run=w, get=self.fake_get(gz, sha), version="9.9.9")
                self.assertEqual([s.state for s in st], ["ok", "ok", "ok"])
                imp = next(c for c in w.calls if c[1] == "--import")
                self.assertEqual(imp[2], rs.DISTRO)
                self.assertEqual(imp[-2:], ["--version", "2"])
                self.assertFalse(list((Path(d) / "wsl").glob("*.tar*")))      # 받은 파일은 지움

    def test_already_installed_does_not_download(self):
        w = FakeWsl(distros=[rs.DISTRO], env_ok=[rs.DISTRO], verify_ok=[rs.DISTRO])
        get = mock.Mock()
        rs.install(run=w, get=get, version="9.9.9")
        get.assert_not_called()

    def test_sha_mismatch_and_bad_gzip_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p1, p2 = self.patches(d)
            with p1, p2:
                gz, sha = self.make_image(d)
                with self.assertRaises(rs.SetupError) as cm:
                    rs.install(run=FakeWsl(), get=self.fake_get(gz, "0" * 64), version="9.9.9")
                self.assertIn("SHA256", cm.exception.message)
                self.assertFalse(list((Path(d) / "wsl").glob("*.tar.gz")))    # 다음엔 새로 받게 지움
            bad = Path(d) / "bad.gz"
            bad.write_bytes(b"not gzip")
            with self.assertRaises(rs.SetupError):
                rs.gunzip(bad, Path(d) / "out.tar")

    def test_low_disk_space(self):
        with tempfile.TemporaryDirectory() as d:
            p1, p2 = self.patches(d, free=1 * 2**30)
            with p1, p2, self.assertRaises(rs.SetupError) as cm:
                rs.import_distro(Path(d) / "x.tar", run=FakeWsl())
            self.assertIn("공간", cm.exception.message)
    def test_import_failure_decodes_utf16_message(self):
        msg = "매개 변수가 틀립니다\r\n오류: Wsl/E_INVALIDARG\r\n"
        raw = msg.encode("utf-16-le")

        def run(cmd, **kw):
            return subprocess.CompletedProcess(cmd, 1, raw, b"")

        with tempfile.TemporaryDirectory() as d:
            p1, p2 = self.patches(d)
            with p1, p2, self.assertRaises(rs.SetupError) as cm:
                rs.import_distro(Path(d) / "x.tar", run=run)
        self.assertIn("매개 변수가 틀립니다", cm.exception.message)
        self.assertIn("Wsl/E_INVALIDARG", cm.exception.message)


    def test_wsl_missing_stops_before_download(self):
        with self.assertRaises(rs.SetupError) as cm:
            rs.install(run=FakeWsl(wsl=False), get=mock.Mock(), version="9.9.9")
        self.assertEqual(cm.exception.step, "wsl")


class UrlAndCommandsTest(unittest.TestCase):
    def test_image_urls_prefer_same_version_then_latest_release(self):
        url, sha = rs.image_urls("0.3.0", get=lambda url, **kw: FakeResp(200, b"abc  x\n"))
        self.assertTrue(url.startswith("https://github.com/yoobinkim541/OSS-2026-Mirobot-Photo-Sketch/releases/"))
        self.assertTrue(url.endswith("/v0.3.0/MirobotSketch-ROS-humble-0.3.0.tar.gz"))
        self.assertEqual(sha, url + ".sha256")

        def get_api(url, **kw):
            if "api.github.com" in url:
                return FakeResp(200, json.dumps([
                    {"tag_name": "v0.4.0", "assets": []},
                    {"tag_name": "v0.3.0", "assets": [
                        {"name": "MirobotSketch-ROS-humble-0.3.0.tar.gz", "browser_download_url": "U"},
                        {"name": "MirobotSketch-ROS-humble-0.3.0.tar.gz.sha256", "browser_download_url": "S"}]},
                ]).encode())
            return FakeResp(404)

        self.assertEqual(rs.image_urls("0.5.0", get=get_api), ("U", "S"))

    def test_admin_command(self):
        popen = mock.Mock()
        rs.install_wsl_feature(popen=popen)
        cmd = " ".join(popen.call_args.args[0])
        self.assertIn("--no-distribution", cmd)
        self.assertIn("RunAs", cmd)
        self.assertTrue(rs.script_path().exists())


class ManualInstallTest(unittest.TestCase):
    """예비 경로: 공식 Ubuntu 22.04 WSL 루트 파일을 전용 배포판으로 가져와 그 안에서 스크립트 실행.
    사용자의 기존 배포판(Ubuntu-22.04 등)은 건드리지 않고, 비밀번호·추가 관리자 승인이 없음."""

    def rootfs_get(self, d):
        gz = Path(d) / "rootfs.tar.gz"
        with gzip.open(gz, "wb") as g:
            g.write(b"ubuntu" * 50)
        sha = hashlib.sha256(gz.read_bytes()).hexdigest()
        urls = []

        def get(url, headers=None, **kw):
            urls.append(url)
            if url.endswith("SHA256SUMS"):
                return FakeResp(200, f"0000  other.tar.gz\n{sha}  {rs.UBUNTU_ROOTFS_NAME}\n".encode())
            return FakeResp(200, gz.read_bytes())
        return get, urls

    def test_new_distro_from_official_rootfs(self):
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(rs, "install_dir", lambda: Path(d) / "wsl"), \
                mock.patch.object(rs.shutil, "disk_usage", lambda p: mock.Mock(free=100 * 2**30)):
            get, urls = self.rootfs_get(d)
            w, popen = FakeWsl(distros=["Ubuntu-22.04"]), mock.Mock()
            rs.manual_install(run=w, get=get, popen=popen)
            self.assertTrue(any(u.startswith("https://cloud-images.ubuntu.com/wsl/") for u in urls))
            imp = next(c for c in w.calls if c[1] == "--import")
            self.assertEqual(imp[2], rs.DISTRO)
            cmd = popen.call_args.args[0]
            self.assertEqual(cmd[:5], ["wsl.exe", "-d", rs.DISTRO, "-u", "root"])
            self.assertTrue(cmd[-2].endswith("setup_ros_env.sh"))
            self.assertEqual(cmd[-1], "--image")
            everything = " ".join(" ".join(c) for c in w.calls + [cmd])
            self.assertNotIn("Ubuntu-22.04", everything.replace(rs.UBUNTU_ROOTFS_NAME, ""))   # 기존 배포판 안 건드림
            self.assertNotIn("--install", everything)
            self.assertFalse(list((Path(d) / "wsl").glob("*.tar*")))

    def test_existing_distro_just_reruns_script(self):
        w, popen, get = FakeWsl(distros=[rs.DISTRO]), mock.Mock(), mock.Mock()
        rs.manual_install(run=w, get=get, popen=popen)
        get.assert_not_called()
        self.assertFalse(any(c[1] == "--import" for c in w.calls))
        self.assertEqual(popen.call_args.args[0][-1], "--image")

    def test_needs_wsl_and_rejects_bad_checksum(self):
        with self.assertRaises(rs.SetupError) as cm:
            rs.manual_install(run=FakeWsl(wsl=False), get=mock.Mock(), popen=mock.Mock())
        self.assertEqual(cm.exception.step, "wsl")
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(rs, "install_dir", lambda: Path(d) / "wsl"), \
                mock.patch.object(rs.shutil, "disk_usage", lambda p: mock.Mock(free=100 * 2**30)):
            def get(url, headers=None, **kw):
                if url.endswith("SHA256SUMS"):
                    return FakeResp(200, f"{'0' * 64}  {rs.UBUNTU_ROOTFS_NAME}\n".encode())
                return FakeResp(200, b"x" * 100)
            with self.assertRaises(rs.SetupError) as cm:
                rs.manual_install(run=FakeWsl(), get=get, popen=mock.Mock())
            self.assertIn("SHA256", cm.exception.message)


class ReviewFixTest(unittest.TestCase):
    def test_disk_space_checked_before_download(self):
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(rs, "install_dir", lambda: Path(d) / "wsl"), \
                mock.patch.object(rs.shutil, "disk_usage", lambda p: mock.Mock(free=3 * 2**30)):
            get = mock.Mock()
            with self.assertRaises(rs.SetupError) as cm:
                rs.install(run=FakeWsl(), get=get, version="9.9.9")
            self.assertIn("공간", cm.exception.message)
            get.assert_not_called()

    def test_disk_full_while_unpacking_is_not_called_corrupt(self):
        import errno
        with tempfile.TemporaryDirectory() as d:
            src = Path(d) / "a.gz"
            with gzip.open(src, "wb") as g:
                g.write(b"x" * 1000)
            real_open = open

            def full_open(path, mode="r", *a, **k):
                f = real_open(path, mode, *a, **k)
                if "w" in mode:
                    def write(b):
                        raise OSError(errno.ENOSPC, "No space left on device")
                    f.write = write
                return f
            with mock.patch("builtins.open", full_open), self.assertRaises(rs.SetupError) as cm:
                rs.gunzip(src, Path(d) / "a.tar")
            self.assertIn("공간", cm.exception.message)
            self.assertNotIn("손상", cm.exception.message)

    def test_verified_download_is_reused(self):
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(rs, "install_dir", lambda: Path(d) / "wsl"), \
                mock.patch.object(rs.shutil, "disk_usage", lambda p: mock.Mock(free=100 * 2**30)):
            gz, sha = InstallTest().make_image(d)
            dest = Path(d) / "wsl" / rs.IMAGE_NAME.format(version="9.9.9")
            dest.parent.mkdir(parents=True)
            dest.write_bytes(gz.read_bytes())                  # 지난번에 받아 둔 파일 (가져오기에서 실패했던 경우)
            urls = []

            def get(url, headers=None, **kw):
                urls.append(url)
                return FakeResp(200, f"{sha}  x\n".encode()) if url.endswith(".sha256") else FakeResp(500)
            steps = rs.install(run=FakeWsl(), get=get, version="9.9.9")
            self.assertEqual([s.state for s in steps], ["ok", "ok", "ok"])
            self.assertFalse(any(u.endswith(".tar.gz") for u in urls))

    def test_registered_but_broken_distro_is_not_imported_again(self):
        w = FakeWsl(distros=[rs.DISTRO])                       # 등록은 됐지만 ROS 확인 실패
        with self.assertRaises(rs.SetupError) as cm:
            rs.install(run=w, get=mock.Mock(), version="9.9.9")
        self.assertIn("제거", cm.exception.hint)
        self.assertFalse(any(c[1] == "--import" for c in w.calls))

    def test_release_uploads_are_not_racing(self):
        y = (ROOT / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")   # yaml 모듈 없이
        job = y[y.index("\n  wsl-image:"):]
        self.assertIn("\n    needs: windows\n", job[:job.index("runs-on:")])


def steps(*states):
    ids = ("wsl", "env", "verify")
    return [rs.SetupStep(i, i, s, hint="hint-" + i if s != "ok" else "") for i, s in zip(ids, states)]


class PackagingTest(unittest.TestCase):
    def test_release_builds_wsl_image(self):
        y = (ROOT / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")
        for needle in ("wsl-image:", "ubuntu:22.04", "setup_ros_env.sh --image", "SETUP_OK",
                       "ros2 pkg prefix wlkata_mirobot_description", "robot_state_publisher",
                       "docker export", "gzip", "sha256sum", "MirobotSketch-ROS-humble-", "1900",
                       "_internal\\wsl\\setup_ros_env.sh"):
            self.assertIn(needle, y, needle)

    def test_installer_offers_rviz_env_and_cleans_up(self):
        iss = (ROOT / "packaging" / "installer.iss").read_text(encoding="utf-8-sig")
        for needle in ('Name: "rvizenv"', "--setup-rviz", "Tasks: rvizenv", "--unregister MirobotSketch-ROS",
                       "CurUninstallStepChanged", "SuppressibleMsgBox", "IDNO"):
            self.assertIn(needle, iss, needle)
        self.assertNotIn("OSS-2026\n", iss)

    def test_exe_bundles_setup_script(self):
        spec = (ROOT / "packaging" / "mirobot_sketch.spec").read_text(encoding="utf-8")
        self.assertIn('"wsl" / "setup_ros_env.sh"), "wsl")', spec)
        with mock.patch.object(rs.paths, "repo_root", lambda: None), \
                mock.patch.object(rs.sys, "_MEIPASS", "C:/app/_internal", create=True):
            self.assertEqual(rs.script_path(), Path("C:/app/_internal/wsl/setup_ros_env.sh"))


class CliTest(unittest.TestCase):
    def test_check_exit_code(self):
        with mock.patch.object(rs, "check", return_value=steps("ok", "ok", "ok")):
            self.assertEqual(rs.main(["--check"]), 0)
        with mock.patch.object(rs, "check", return_value=steps("ok", "missing", "blocked")):
            self.assertEqual(rs.main(["--check"]), 1)

    def test_install_reports_error_hint(self):
        err = rs.SetupError("wsl", "WSL 기능이 없습니다.", "관리자 승인")
        with mock.patch.object(rs, "install", side_effect=err), mock.patch("builtins.print") as pr:
            self.assertEqual(rs.main(["--install"]), 1)
        text = " ".join(str(c.args[0]) for c in pr.call_args_list)
        self.assertIn("관리자 승인", text)
        self.assertIn("--wsl", text)                           # WSL 기능 설치 명령을 안내
        with mock.patch.object(rs, "install", return_value=steps("ok", "ok", "ok")):
            self.assertEqual(rs.main(["--install"]), 0)

    def test_other_commands(self):
        with mock.patch.object(rs, "uninstall", return_value=True) as un:
            self.assertEqual(rs.main(["--uninstall"]), 0)
        un.assert_called_once()
        with mock.patch.object(rs, "manual_install") as man:
            self.assertEqual(rs.main(["--manual"]), 0)
        man.assert_called_once()
        with mock.patch.object(rs, "install_wsl_feature") as feat:
            self.assertEqual(rs.main(["--wsl"]), 0)
        feat.assert_called_once()

    def test_log_is_written(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.object(rs.paths, "user_dir", lambda: Path(d)):
            rs.log("hello")
            self.assertIn("hello", (Path(d) / "rviz_setup.log").read_text(encoding="utf-8"))

    def test_cli_entry_points(self):
        sys.path.insert(0, str(ROOT / "packaging"))
        import launch_cli
        self.assertEqual(launch_cli.COMMANDS["setup-rviz"], "mirobot_sketch.rviz_setup")
        self.assertIn('mirobot-setup-rviz = "mirobot_sketch.rviz_setup:main"',
                      (ROOT / "pyproject.toml").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
