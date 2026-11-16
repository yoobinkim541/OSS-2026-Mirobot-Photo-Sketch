"""
그리기 창 카메라 화면을 파일로 공유 — 카메라는 앱 하나만 열 수 있으므로, 다른 프로그램(터미널·원격 도우미)은
카메라를 열지 않고 이 파일들을 읽어 지금 종이를 봅니다. 로컬 파일만 쓰고 네트워크로 보내지 않습니다.

out/camera_live/
  view.jpg     그림 부분(종이 위치를 맞췄으면), 계획·선 확인 겹침        EVERY_S마다
  raw.jpg      같은 부분, 겹침 없는 원본 (펜 선을 그대로 볼 때)          EVERY_S마다
  full.jpg     사진 전체 원본                                         FULL_EVERY_S마다
  status.json  시각·카메라·자른 위치·그리기 창 상태 문구
파일은 임시 파일에 다 쓴 뒤 바꿔 넣으므로 읽는 쪽이 반쯤 쓴 사진을 보지 않습니다.
"""

import json
import os
import threading
import time
from datetime import datetime

import cv2

from . import paths, vision
from .camera import fit_preview

DIR_NAME = "camera_live"
EVERY_S = 2.0
FULL_EVERY_S = 10.0
MAX_WIDTH = 1920
JPEG_QUALITY = 90


def share_dir():
    return paths.output_dir() / DIR_NAME


def _write_jpg(path, image):
    ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise OSError(f"사진을 만들지 못했습니다: {path.name}")
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(buf.tobytes())        # 한글 경로도 그대로 (cv2.imwrite는 Windows 한글 경로에서 실패)
    os.replace(tmp, path)


class CameraShare:
    """maybe_write()를 화면 갱신마다 불러도 EVERY_S마다 한 번, 작업 스레드에서 씀 (화면을 멈추지 않음)."""

    def __init__(self, directory=None):
        self.directory = directory
        self._busy = False
        self._last = -EVERY_S
        self._last_full = -FULL_EVERY_S
        self._overlay = vision.Overlay()
        self.error = ""

    def maybe_write(self, camera, camera_name, registration, strokes, colors=None, marks=(), regions=(), status=None):
        now = time.monotonic()
        if self._busy or now - self._last < EVERY_S:
            return False
        self._busy, self._last = True, now
        full_due = now - self._last_full >= FULL_EVERY_S
        if full_due:
            self._last_full = now
        threading.Thread(target=self._work, name="camera-share", daemon=True,
                         args=(camera, camera_name, registration, strokes, colors, marks, regions, status,
                               full_due)).start()
        return True

    def _work(self, camera, camera_name, registration, strokes, colors, marks, regions, status, full_due):
        try:
            self.write(camera.latest_full(), camera_name, registration, strokes, colors, marks, regions, status,
                       full_due)
            self.error = ""
        except Exception as e:      # 공유는 보조 기능: 실패해도 그리기·화면에는 영향 없음
            self.error = f"{type(e).__name__}: {e}"
        finally:
            self._busy = False

    def write(self, frame, camera_name, registration, strokes, colors=None, marks=(), regions=(), status=None,
              full=True):
        if frame is None:
            return None
        directory = self.directory or share_dir()
        directory.mkdir(parents=True, exist_ok=True)
        h, w = frame.shape[:2]
        box = None
        if registration is not None:
            box = vision.view_box(registration, (w, h), strokes)
            x0, y0, x1, y1 = box
            raw = fit_preview(frame[y0:y1, x0:x1], MAX_WIDTH)
            view = self._overlay.draw(raw, vision.crop_registration(registration, (w, h), box), strokes, regions,
                                      colors, marks)
        else:
            raw = view = fit_preview(frame, MAX_WIDTH)
        _write_jpg(directory / "view.jpg", view)
        _write_jpg(directory / "raw.jpg", raw)
        if full:
            _write_jpg(directory / "full.jpg", frame)
        info = {"time": datetime.now().astimezone().isoformat(timespec="seconds"), "camera": camera_name,
                "frame_size": [w, h], "view_box_px": list(box) if box else None,
                "view_size": [raw.shape[1], raw.shape[0]], **(status or {})}
        tmp = directory / "status.json.tmp"
        tmp.write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(tmp, directory / "status.json")
        return directory
