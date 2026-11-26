"""카메라 미리보기: 장치를 열지 않고 가짜 캡처로 확인합니다."""

import sys
import time
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import camera  # noqa: E402


def fake_opener(frames, fail_indexes=()):
    calls = []

    def open_capture(index, backend=None):
        calls.append(index)
        if index in fail_indexes:
            return None
        frame = frames[index]
        state = {"n": 0}

        class Cap:
            def read(self):
                state["n"] += 1
                return True, frame

            def release(self):
                state["released"] = True

        cap = Cap()
        cap.state = state
        return cap, frame

    open_capture.calls = calls
    return open_capture


class CameraFindTest(unittest.TestCase):
    def test_skips_closed_indexes_and_keeps_first_frame(self):
        frames = {1: np.zeros((100, 200, 3), np.uint8)}
        opener = fake_opener(frames, fail_indexes=(0,))
        found = camera.find_camera(opener=opener)
        self.assertEqual(found[0], 1)
        self.assertEqual(opener.calls, [0, 1])

    def test_none_when_every_index_fails(self):
        self.assertIsNone(camera.find_camera(opener=lambda index, backend=None: None))

    def test_preview_shrinks_wide_frames(self):
        frame = np.zeros((720, 1280, 3), np.uint8)
        small = camera.fit_preview(frame, width=480)
        self.assertEqual(small.shape[1], 480)
        self.assertLess(small.shape[0], 720)


class CameraPreviewTest(unittest.TestCase):
    def test_start_publishes_latest_frame_and_stop_releases(self):
        frame = np.full((80, 120, 3), 40, np.uint8)
        opener = fake_opener({0: frame})
        preview = camera.CameraPreview(opener=opener)
        preview.start()
        deadline = time.time() + 2
        latest = None
        while time.time() < deadline and latest is None:
            latest = preview.latest()
            time.sleep(0.02)
        self.assertEqual(preview.index, 0)
        preview.stop()
        self.assertIsNotNone(latest)
        self.assertFalse(preview.running)
        self.assertIsNone(preview.latest())         # 끈 뒤에는 옛 프레임을 보여 주지 않음

    def test_missing_camera_sets_error_without_raising(self):
        preview = camera.CameraPreview(opener=lambda index, backend=None: None)
        preview.start()
        preview._thread.join(2)
        self.assertEqual(preview.error, "카메라를 찾지 못했습니다")
        self.assertIsNone(preview.latest())

    def test_chosen_camera_opens_only_that_index(self):
        frames = {0: np.zeros((40, 60, 3), np.uint8), 2: np.full((40, 60, 3), 90, np.uint8)}
        opener = fake_opener(frames)
        preview = camera.CameraPreview(opener=opener, wanted=2)
        preview.start()
        preview._thread.join(2)          # 가짜 캡처는 계속 읽으므로 잠깐 뒤 멈춤
        deadline = time.time() + 2
        while time.time() < deadline and preview.latest() is None:
            time.sleep(0.02)
        self.assertEqual(int(preview.latest().mean()), 90)
        self.assertEqual(preview.index, 2)
        preview.stop()
        self.assertEqual(opener.calls, [2])

    def test_chosen_camera_that_fails_reports_it_instead_of_falling_back(self):
        opener = fake_opener({0: np.zeros((4, 4, 3), np.uint8)}, fail_indexes=(1,))
        preview = camera.CameraPreview(opener=opener, wanted=1)
        preview.start()
        preview._thread.join(2)
        self.assertEqual(preview.error, "고른 카메라를 열 수 없습니다")
        self.assertEqual(opener.calls, [1])


    def test_select_opens_the_next_device_only_after_release(self):
        import threading
        from unittest import mock

        class FakeCap:
            live = 0
            overlap = False
            events = []
            reading = threading.Event()

            def __init__(self, index, backend=None):
                self.index = index
                self.released = False
                if FakeCap.live:
                    FakeCap.overlap = True
                FakeCap.live += 1
                FakeCap.events.append(("open", index))

            def isOpened(self):
                return not self.released

            def read(self):
                FakeCap.reading.set()
                time.sleep(0.3)
                if self.released:
                    return False, None
                return True, np.full((40, 60, 3), 40 + int(self.index), np.uint8)

            def release(self):
                if not self.released:
                    self.released = True
                    FakeCap.live -= 1
                    FakeCap.events.append(("release", self.index))

            def set(self, *_args, **_kwargs):
                return True

        FakeCap.live = 0
        FakeCap.overlap = False
        FakeCap.events = []
        FakeCap.reading = threading.Event()
        with mock.patch("mirobot_sketch.camera.cv2.VideoCapture", FakeCap):
            preview = camera.CameraPreview(wanted=0)
            preview.start()
            try:
                deadline = time.time() + 3
                while time.time() < deadline and preview.latest() is None:
                    time.sleep(0.02)
                self.assertIsNotNone(preview.latest())
                FakeCap.reading.clear()
                deadline = time.time() + 2
                while time.time() < deadline and not FakeCap.reading.is_set():
                    time.sleep(0.01)
                preview.select(1)
                deadline = time.time() + 3
                while time.time() < deadline and preview.index != 1:
                    time.sleep(0.02)
                self.assertEqual(preview.index, 1)
            finally:
                preview.stop()
        self.assertFalse(FakeCap.overlap)
        self.assertLess(FakeCap.events.index(("release", 0)), FakeCap.events.index(("open", 1)))

class SizeLockTest(unittest.TestCase):
    """해상도를 바꾸는 카메라: 4:3 기본 프레임은 4K 16:9 화면의 가운데만 보여 배율이 달라 보인다."""
    SMALL, BIG = np.full((48, 64, 3), 10, np.uint8), np.full((72, 128, 3), 200, np.uint8)
    SETTINGS = {"width": 128, "height": 72}

    def opener(self, honours_size=True, switch_after=3):
        small, big = self.SMALL, self.BIG

        def open_capture(index, backend=None):
            state = {"asked": set(), "reads_after": 0}

            class Cap:
                def set(self, prop, value):
                    if honours_size and prop in (camera.cv2.CAP_PROP_FRAME_WIDTH, camera.cv2.CAP_PROP_FRAME_HEIGHT):
                        state["asked"].add(prop)
                    return True

                def read(self):
                    if len(state["asked"]) == 2:
                        state["reads_after"] += 1
                        if state["reads_after"] > switch_after:      # 드라이버가 몇 프레임 뒤에 바뀜
                            return True, big
                    return True, small

                def release(self):
                    pass

            return Cap(), small
        return open_capture

    def frames_seen(self, preview, seconds=1.0):
        seen, deadline = set(), time.time() + seconds
        while time.time() < deadline:
            f = preview.latest_full()
            if f is not None:
                seen.add(f.shape[:2])
            time.sleep(0.01)
        return seen

    def test_frames_before_the_requested_size_are_never_shown(self):
        preview = camera.CameraPreview(opener=self.opener(), wanted=0, settings=self.SETTINGS)
        preview.start()
        seen = self.frames_seen(preview)
        preview.stop()
        self.assertEqual(seen, {(72, 128)})
        self.assertEqual(preview.error, "")

    def test_camera_that_ignores_the_size_reports_it_and_shows_nothing(self):
        old = camera.SIZE_SETTLE_S
        camera.SIZE_SETTLE_S = 0.2
        try:
            preview = camera.CameraPreview(opener=self.opener(honours_size=False), wanted=0, settings=self.SETTINGS)
            preview.start()
            preview._thread.join(3)
        finally:
            camera.SIZE_SETTLE_S = old
        self.assertIsNone(preview.latest())
        self.assertIn("128×72", preview.error)
        self.assertIn("64×48", preview.error)

    def test_auto_picked_camera_still_gets_its_saved_size(self):
        # 자동 선택이라도 그 장치의 촬영 설정을 써야 한다 (빼면 BRIO가 640×480 4:3으로 열려 배율이 달라 보임)
        from unittest import mock
        with mock.patch.object(camera, "capture_settings", lambda name: self.SETTINGS if name == "Logitech BRIO" else None):
            preview = camera.CameraPreview(opener=self.opener(), names_reader=lambda: ["Logitech BRIO"])
            preview.start()
            seen = self.frames_seen(preview)
            preview.stop()
        self.assertEqual(seen, {(72, 128)})


class CameraChoiceTest(unittest.TestCase):
    CAMS = [camera.CameraInfo(0, "USB2.0 HD UVC WebCam"), camera.CameraInfo(1, "OBS Virtual Camera"),
            camera.CameraInfo(2, "Logitech BRIO")]

    def test_saved_name_wins_when_the_index_moved(self):
        self.assertEqual(camera.resolve_choice({"name": "Logitech BRIO", "index": 0}, self.CAMS), 2)

    def test_same_name_twice_prefers_the_saved_index(self):
        twins = [camera.CameraInfo(0, "USB Camera"), camera.CameraInfo(1, "USB Camera")]
        self.assertEqual(camera.resolve_choice({"name": "USB Camera", "index": 1}, twins), 1)
        self.assertEqual(camera.resolve_choice({"name": "USB Camera", "index": 5}, twins), 0)

    def test_unplugged_or_no_choice_means_auto(self):
        self.assertIsNone(camera.resolve_choice({"name": "Logitech BRIO", "index": 2}, self.CAMS[:2]))
        self.assertIsNone(camera.resolve_choice(None, self.CAMS))

    def test_listing_uses_device_names_without_opening(self):
        opener = fake_opener({})
        found = camera.list_cameras(opener=opener, names_reader=lambda: ["Built-in", "BRIO"])
        self.assertEqual([(c.index, c.name) for c in found], [(0, "Built-in"), (1, "BRIO")])
        self.assertEqual(opener.calls, [])

    def test_listing_probes_indexes_when_names_are_unavailable(self):
        frames = {i: np.zeros((4, 4, 3), np.uint8) for i in range(4)}
        opener = fake_opener(frames, fail_indexes=(0, 2))
        found = camera.list_cameras(opener=opener, names_reader=lambda: None)
        self.assertEqual([c.index for c in found], [1, 3])

    def test_picking_another_camera_keeps_capture_settings(self):
        import json
        import tempfile
        from unittest import mock
        with tempfile.TemporaryDirectory() as d, mock.patch.object(camera.paths, "user_dir", lambda: Path(d)):
            (Path(d) / camera.SETTINGS_FILE).write_text(json.dumps(
                {"name": "Logitech BRIO", "index": 2,
                 "capture": {"Logitech BRIO": {"width": 3840, "height": 2160, "focus": 15}}}), encoding="utf-8")
            camera.save_choice(self.CAMS[0])
            self.assertEqual(camera.load_choice()["name"], "USB2.0 HD UVC WebCam")
            self.assertEqual(camera.capture_settings("Logitech BRIO"), {"width": 3840, "height": 2160, "focus": 15})
            self.assertIsNone(camera.capture_settings("USB2.0 HD UVC WebCam"))
            camera.save_choice(None)                     # 자동으로 되돌려도 촬영 설정은 남음
            self.assertIs(camera.load_choice(), camera.AUTO)
            self.assertEqual(camera.capture_settings("Logitech BRIO")["focus"], 15)


class PreviewPolicyTest(unittest.TestCase):
    def test_unittest_does_not_auto_open_camera(self):
        self.assertFalse(camera.preview_allowed())

    def test_app_that_merely_imports_unittest_opens_camera(self):
        """앱에서 matplotlib → pyparsing.testing이 unittest를 불러와도 테스트 실행은 아님."""
        import os
        import subprocess
        env = {k: v for k, v in os.environ.items() if k != "MIROBOT_CAMERA"}
        env["PYTHONPATH"] = str(Path(__file__).resolve().parent.parent)
        out = subprocess.run(
            [sys.executable, "-c", "import unittest; from mirobot_sketch import camera; print(camera.preview_allowed())"],
            capture_output=True, text=True, env=env, timeout=120, check=True).stdout.strip()
        self.assertEqual(out, "True")
