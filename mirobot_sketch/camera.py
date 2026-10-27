"""노트북 카메라 미리보기. 포트를 열지 않고, 최신 프레임만 보관합니다.

그리기 스레드와 분리된 작업 스레드에서 캡처하고, 화면은 메인 스레드가 주기적으로 가져갑니다.
카메라가 없거나 사용자가 꺼도 로봇 그리기는 그대로 진행됩니다.
"""

import os
import threading
import time

import cv2
import numpy as np

PREVIEW_FPS = 12
PREVIEW_WIDTH = 480


def preview_allowed():
    """테스트·CI는 카메라를 열지 않습니다. MIROBOT_CAMERA=1이면 강제로 엽니다."""
    flag = os.environ.get("MIROBOT_CAMERA", "").strip()
    if flag == "1":
        return True
    if flag == "0" or "unittest" in __import__("sys").modules:
        return False
    return True


def open_capture(index, backend=None):
    """인덱스 하나의 VideoCapture. 실패하면 None."""
    cap = cv2.VideoCapture(index) if backend is None else cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap.release()
        return None
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap, frame


def find_camera(indices=(0, 1), opener=open_capture):
    """첫 번째로 프레임이 나오는 카메라. (index, capture, frame) 또는 None."""
    for index in indices:
        opened = opener(index)
        if opened is not None:
            return (index, *opened)
    return None


def fit_preview(frame, width=PREVIEW_WIDTH):
    """가로 width로 줄인 BGR 복사본. 화면 갱신용이라 원본을 보관하지 않습니다."""
    h, w = frame.shape[:2]
    if w <= width:
        return frame.copy()
    scale = width / w
    return cv2.resize(frame, (width, max(1, int(h * scale))), interpolation=cv2.INTER_AREA)


class CameraPreview:
    """최신 프레임 하나. start() 전에는 장치를 열지 않습니다."""

    def __init__(self, opener=open_capture):
        self.opener = opener
        self.frame = None
        self.index = None
        self.error = ""
        self._cap = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.running:
            return
        self._stop.clear()
        self.error = ""
        self._thread = threading.Thread(target=self._run, name="camera-preview", daemon=True)
        self._thread.start()

    def _run(self):
        found = find_camera(opener=self.opener)
        if found is None:
            self.error = "카메라를 찾지 못했습니다"
            return
        index, cap, frame = found
        self.index = index
        self._cap = cap
        with self._lock:
            self.frame = fit_preview(frame)
        interval = 1.0 / PREVIEW_FPS
        while not self._stop.is_set():
            t0 = time.perf_counter()
            ok, nxt = cap.read()
            if ok and nxt is not None:
                small = fit_preview(nxt)
                with self._lock:
                    self.frame = small
            else:
                self.error = "카메라 프레임을 읽지 못했습니다"
                break
            time.sleep(max(0.0, interval - (time.perf_counter() - t0)))
        cap.release()
        self._cap = None

    def latest(self):
        with self._lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(2)
        self._thread = None


def blank_preview(text="카메라 꺼짐"):
    """카메라가 없을 때 보여줄 단색 프레임."""
    img = np.full((270, PREVIEW_WIDTH, 3), 28, np.uint8)
    cv2.putText(img, text, (24, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1, cv2.LINE_AA)
    return img
