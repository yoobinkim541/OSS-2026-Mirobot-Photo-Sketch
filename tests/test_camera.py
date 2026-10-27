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
        preview.stop()
        self.assertIsNotNone(latest)
        self.assertEqual(preview.index, 0)
        self.assertFalse(preview.running)

    def test_missing_camera_sets_error_without_raising(self):
        preview = camera.CameraPreview(opener=lambda index, backend=None: None)
        preview.start()
        preview._thread.join(2)
        self.assertEqual(preview.error, "카메라를 찾지 못했습니다")
        self.assertIsNone(preview.latest())


class PreviewPolicyTest(unittest.TestCase):
    def test_unittest_does_not_auto_open_camera(self):
        self.assertFalse(camera.preview_allowed())
