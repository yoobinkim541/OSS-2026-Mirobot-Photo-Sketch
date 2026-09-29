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


if __name__ == "__main__":
    unittest.main()
