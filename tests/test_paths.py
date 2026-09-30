"""파일 위치 규칙(paths.py) 테스트.

    python -m unittest discover -s tests -v
"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mirobot_sketch import paths  # noqa: E402


def keys(d, prefix=""):
    out = set()
    for k, v in d.items():
        if k.startswith("_"):
            continue  # 설명용 항목
        out.add(prefix + k)
        if isinstance(v, dict) and k != "limits":     # limits는 대칭 사각형과 영역 형식이 모두 유효 (보정·수동 확대로 바뀜)
            out |= keys(v, prefix + k + ".")
    return out


class PathsTest(unittest.TestCase):
    def test_repo_checkout_uses_repo_files(self):
        self.assertEqual(paths.repo_root(), ROOT)
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("MIROBOT_CONFIG", None)
            self.assertEqual(paths.config_path(), ROOT / "robot" / "drawing_config.json")
        self.assertEqual(paths.runs_dir(), ROOT / "LOG" / "runs")

    def test_installed_mode_copies_default_config_to_user_dir(self):
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(paths, "repo_root", return_value=None), \
                mock.patch.dict(os.environ, {"APPDATA": d}):
            os.environ.pop("MIROBOT_CONFIG", None)
            p = paths.config_path()
            self.assertEqual(p, Path(d) / "MirobotSketch" / "drawing_config.json")
            self.assertTrue(p.exists())
            self.assertEqual(paths.runs_dir(), Path(d) / "MirobotSketch" / "runs")

    def test_env_var_overrides_config(self):
        with mock.patch.dict(os.environ, {"MIROBOT_CONFIG": "C:/x/my.json"}):
            self.assertEqual(paths.config_path(), Path("C:/x/my.json"))

    def test_packaged_default_config_has_same_fields_as_repo_config(self):
        # 패키지 기본값(설치·exe용)과 저장소 설정의 항목 구조가 어긋나지 않게
        repo = json.loads((ROOT / "robot" / "drawing_config.json").read_text(encoding="utf-8"))
        default = json.loads(paths.asset("drawing_config.json").read_text(encoding="utf-8"))
        self.assertEqual(keys(repo), keys(default))

    def test_icons_are_packaged(self):
        self.assertTrue(paths.asset("app_icon.ico").exists())
        self.assertTrue(paths.asset("app_icon_256.png").exists())


class PortablePathTest(unittest.TestCase):
    """기록(LOG)에 사용자 폴더 이름이 남지 않게 경로를 정리."""

    def test_paths_inside_the_repo_become_relative_and_outside_only_the_file_name(self):
        self.assertEqual(paths.portable_path(str(ROOT / "out" / "run_strokes_1.json")), "out/run_strokes_1.json")
        self.assertEqual(paths.portable_path(str(ROOT / "input" / "a.jpg").replace("\\", "/")), "input/a.jpg")
        self.assertEqual(paths.portable_path(r"C:\Users\someone\Pictures\Saved Pictures\IU.jfif"), "IU.jfif")
        self.assertEqual(paths.portable_path("D:/photos/family/pic.png"), "pic.png")
        self.assertEqual(paths.portable_path("/mnt/c/Users/someone/x/y.json"), "y.json")
        self.assertEqual(paths.portable_path("/home/someone/z.txt"), "z.txt")
        for posix in ("/tmp/tmpa4w3jork/line.png", "/var/folders/ab/cd/line.png", "/opt/app/data/line.png",
                      "/srv/data/line.png", "/root/x/line.png"):             # 임시 폴더 등 어떤 절대 경로든
            self.assertEqual(paths.portable_path(posix), "line.png", posix)

    def test_things_that_are_not_absolute_paths_are_left_alone(self):
        for value in ("COM9", "out/x.json", "input/a.jpg", "C:", "https://example.com/a", "펜 끝이 종이에 닿음", "", 3.5, None,
                      "/", "/ 또는 \\", "약 5/6"):
            self.assertEqual(paths.portable_path(value), value)

    def test_installed_mode_keeps_only_the_file_name(self):
        with mock.patch.object(paths, "repo_root", return_value=None):
            self.assertEqual(paths.portable_path(r"C:\Users\someone\AppData\Roaming\MirobotSketch\out\a.json"), "a.json")

    def test_scrub_paths_walks_dicts_and_lists_without_changing_the_input(self):
        data = {"a": [str(ROOT / "out" / "a.json"), 3, {"b": r"C:\Users\someone\x\b.jpg", "c": "ok"}], "n": None}
        clean = paths.scrub_paths(data)
        self.assertEqual(clean, {"a": ["out/a.json", 3, {"b": "b.jpg", "c": "ok"}], "n": None})
        self.assertIn("someone", data["a"][2]["b"])                    # 원본은 그대로

    def test_run_record_and_calibration_report_carry_no_user_folder(self):
        from mirobot_sketch import calibration, draw_executor as de
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(paths, "runs_dir", lambda: Path(d) / "runs"):
                cfg = de.load_config(paths.asset("drawing_config.json"))
                path = de.write_run_record(
                    {"strokes_json": str(ROOT / "out" / "run_strokes_1.json"),
                     "source": {"image": r"C:\Users\someone\Pictures\Saved Pictures\Waguri.webp"},
                     "trajectory": r"C:\Users\someone\Desktop\Mirobot\out\live_traj_1.json"},
                    {"result": "completed"}, cfg)
                text = path.read_text(encoding="utf-8")
                run = json.loads(text)
                self.assertNotIn("someone", text)
                self.assertNotIn("Users", text)
                self.assertEqual(run["strokes_json"], "out/run_strokes_1.json")
                self.assertEqual(run["source"]["image"], "Waguri.webp")
                self.assertEqual(run["result"], "completed")
                report = calibration.save_report({"result": "aborted", "samples": []},
                                                 r"C:\Users\someone\Desktop\Mirobot\robot\drawing_config.json")
                text = report.read_text(encoding="utf-8")
                self.assertNotIn("someone", text)
                self.assertEqual(json.loads(text)["config_path"], "drawing_config.json")

    def test_committed_logs_do_not_contain_the_user_folder(self):
        """저장소에 올라가는 기록·문서에 컴퓨터 사용자 폴더 이름이 남아 있지 않은지."""
        import re
        pattern = re.compile(r"Users.{1,2}(?!someone)[A-Za-z0-9_.-]+.{1,2}(?:Desktop|Pictures|Documents|AppData)", re.I)
        offenders = []
        for folder in ("LOG/runs", "LOG/calibrations"):
            for f in (ROOT / folder).glob("*.json"):
                if pattern.search(f.read_text(encoding="utf-8")):
                    offenders.append(f.relative_to(ROOT).as_posix())
        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
