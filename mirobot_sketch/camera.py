"""카메라 미리보기. 로봇 포트를 열지 않고, 최신 프레임만 보관합니다.

그리기 스레드와 분리된 작업 스레드에서 캡처하고, 화면은 메인 스레드가 주기적으로 가져갑니다.
카메라가 없거나 사용자가 꺼도 로봇 그리기는 그대로 진행됩니다.

장치 고르기: Windows에서는 DirectShow로 장치 이름을 읽고, OpenCV도 같은 DirectShow 순서로 엽니다
(그래야 목록의 n번째 이름과 VideoCapture(n)이 같은 장치). 고른 장치는 이름으로 저장해 USB를 다른 구멍에
꽂아 번호가 바뀌어도 같은 카메라를 다시 찾습니다.
"""

import json
import os
import sys
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from . import paths

PREVIEW_FPS = 12
PREVIEW_WIDTH = 480
PROBE_INDICES = range(4)          # 이름을 못 읽는 OS에서 열어 볼 번호
SETTINGS_FILE = "camera.json"
AUTO = None                        # 저장된 선택 없음: 열리는 첫 카메라
FOCUS_AFTER_FRAMES = 25           # 켜자마자 초점을 정하면 카메라가 자동 초점으로 되돌리는 경우가 있어 조금 뒤에 정함
SIZE_SETTLE_S = 4.0               # 해상도를 바꾼 뒤 그 크기 프레임이 나올 때까지 기다리는 시간


@dataclass(frozen=True)
class CameraInfo:
    index: int
    name: str

    @property
    def label(self):
        """목록에 보일 이름. 같은 이름의 카메라가 둘이면 번호로 구분."""
        return f"{self.name} ({self.index})"


def preview_allowed():
    """테스트·CI는 카메라를 열지 않습니다. MIROBOT_CAMERA=1이면 강제로 엽니다."""
    flag = os.environ.get("MIROBOT_CAMERA", "").strip()
    if flag == "1":
        return True
    if flag == "0" or "unittest" in __import__("sys").modules:
        return False
    return True


def default_backend():
    """Windows는 DirectShow: 장치 이름 목록과 번호가 같은 순서이고, MSMF보다 여는 시간이 짧습니다."""
    return cv2.CAP_DSHOW if sys.platform.startswith("win") else None


def open_capture(index, backend=None):
    """인덱스 하나의 VideoCapture. 실패하면 None."""
    backend = default_backend() if backend is None else backend
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


def _dshow_device_names():
    """DirectShow 비디오 입력 장치 이름 (OpenCV CAP_DSHOW 번호 순서). 읽지 못하면 None.
    COM 인터페이스를 ctypes vtable로 직접 부릅니다 (추가 패키지 없이)."""
    import ctypes
    from ctypes import POINTER, byref, c_void_p, c_wchar_p, wintypes

    class GUID(ctypes.Structure):
        _fields_ = [("d1", ctypes.c_uint32), ("d2", ctypes.c_uint16), ("d3", ctypes.c_uint16),
                    ("d4", ctypes.c_ubyte * 8)]

        def __init__(self, text):
            super().__init__()
            ole32.CLSIDFromString(c_wchar_p(text), byref(self))

    class VARIANT(ctypes.Structure):
        _fields_ = [("vt", ctypes.c_ushort), ("r1", ctypes.c_ushort), ("r2", ctypes.c_ushort),
                    ("r3", ctypes.c_ushort), ("val", c_void_p), ("pad", c_void_p)]

    def call(obj, slot, restype, *args):
        """COM 메서드 호출: obj의 vtable slot번 함수."""
        vtbl = ctypes.cast(obj, POINTER(POINTER(c_void_p))).contents
        proto = ctypes.WINFUNCTYPE(restype, c_void_p,
                                   *[type(a) if isinstance(a, ctypes._SimpleCData) else c_void_p for a in args])
        return proto(vtbl[slot])(obj, *args)

    def release(obj):
        if obj:
            call(obj, 2, ctypes.c_ulong)

    ole32, oleaut32 = ctypes.windll.ole32, ctypes.windll.oleaut32
    oleaut32.SysFreeString.argtypes = [c_void_p]
    hr = ole32.CoInitializeEx(None, 0)            # 이 스레드에서 COM 사용 (이미 다른 방식이면 그대로)
    initialized = hr in (0, 1)
    names = []
    dev_enum = class_enum = None
    try:
        dev_enum = c_void_p()
        if ole32.CoCreateInstance(byref(GUID("{62BE5D10-60EB-11d0-BD3B-00A0C911CE86}")), None, 1,
                                  byref(GUID("{29840822-5B84-11D0-BD3B-00A0C911CE86}")), byref(dev_enum)) != 0:
            return None
        class_enum = c_void_p()
        hr = call(dev_enum, 3, ctypes.HRESULT, byref(GUID("{860BB310-5D01-11d0-BD3B-00A0C911CE86}")),
                  byref(class_enum), wintypes.DWORD(0))
        if hr != 0 or not class_enum:              # S_FALSE(1): 비디오 장치 없음
            return names
        bag_iid = GUID("{55272A00-42CB-11CE-8135-00AA004BB851}")
        while True:
            moniker, fetched = c_void_p(), wintypes.ULONG(0)
            if call(class_enum, 3, ctypes.HRESULT, wintypes.ULONG(1), byref(moniker), byref(fetched)) != 0 \
                    or not fetched.value:
                break
            name = f"카메라 {len(names)}"
            bag = c_void_p()
            try:
                if call(moniker, 9, ctypes.HRESULT, c_void_p(), c_void_p(), byref(bag_iid), byref(bag)) == 0:
                    var = VARIANT()
                    if call(bag, 3, ctypes.HRESULT, c_wchar_p("FriendlyName"), byref(var), c_void_p()) == 0 \
                            and var.vt == 8 and var.val:      # VT_BSTR
                        name = ctypes.wstring_at(var.val)
                        oleaut32.SysFreeString(var.val)
            finally:
                release(bag)
                release(moniker)
            names.append(name)
        return names
    except OSError:
        return None
    finally:
        release(class_enum)
        release(dev_enum)
        if initialized:
            ole32.CoUninitialize()


def list_cameras(opener=open_capture, names_reader=None):
    """연결된 카메라 목록 [CameraInfo]. Windows는 이름을 읽기만 하고(빠름), 그 밖에는 번호를 열어 봅니다."""
    reader = names_reader or (_dshow_device_names if sys.platform.startswith("win") else lambda: None)
    names = reader()
    if names is not None:
        return [CameraInfo(i, n) for i, n in enumerate(names)]
    found = []
    for index in PROBE_INDICES:
        opened = opener(index)
        if opened is not None:
            opened[0].release()
            found.append(CameraInfo(index, f"카메라 {index}"))
    return found


def auto_settings(index, names_reader=None):
    """자동으로 고른 번호의 장치 이름으로 저장된 촬영 설정. 이름을 읽을 수 없으면 None.
    자동 선택이라고 설정을 빼면 BRIO가 기본 640×480(4K 화면의 가운데 4:3)으로 열려 화면 배율이 달라진다."""
    reader = names_reader or (_dshow_device_names if sys.platform.startswith("win") else lambda: None)
    try:
        names = reader()
    except Exception:
        return None
    if not names or not 0 <= index < len(names):
        return None
    return capture_settings(names[index])


def resolve_choice(choice, cameras):
    """저장된 선택 {"name", "index"}을 지금 목록의 번호로. 이름이 같은 장치를 우선(번호가 바뀌어도 같은 카메라),
    없으면 None(= 자동)."""
    if not choice or not cameras:
        return None
    same_name = [c for c in cameras if c.name == choice.get("name")]
    if not same_name:
        return None
    exact = [c for c in same_name if c.index == choice.get("index")]
    return (exact or same_name)[0].index


def _load_settings():
    try:
        data = json.loads((paths.user_dir() / SETTINGS_FILE).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def load_choice():
    data = _load_settings()
    return data if isinstance(data.get("name"), str) else AUTO


def capture_settings(name):
    """장치 이름별 촬영 설정 {"width", "height", "focus"} (camera.json의 "capture"), 없으면 None.
    선이 가늘어 종이를 크게 찍어야 할 때 쓰고, 초점은 카메라와 판 거리가 고정일 때만 정해 둡니다."""
    capture = _load_settings().get("capture")
    found = capture.get(name) if isinstance(capture, dict) else None
    return dict(found) if isinstance(found, dict) and found else None


def save_choice(info):
    """CameraInfo를 저장, None이면 자동으로 되돌림. 장치별 촬영 설정("capture")은 그대로 둠."""
    path = paths.user_dir() / SETTINGS_FILE
    data = {k: v for k, v in _load_settings().items() if k == "capture"}
    if info is not None:
        data.update(name=info.name, index=info.index)
    try:
        if data:
            path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
        else:
            path.unlink(missing_ok=True)
    except OSError:
        pass


def apply_size(cap, settings):
    """요청한 해상도로 바꿈 (MJPG: 4K도 USB 대역 안에서 받음). 카메라가 못 하면 가까운 크기로 줌."""
    if settings and settings.get("width") and settings.get("height"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(settings["width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(settings["height"]))


def wanted_size(settings):
    """촬영 설정에 적은 (가로, 세로), 없으면 None."""
    if settings and settings.get("width") and settings.get("height"):
        return int(settings["width"]), int(settings["height"])
    return None


def settle_size(cap, settings, stop, timeout=None):
    """요청한 크기의 프레임이 나올 때까지 읽어 버림. 반환: (그 크기의 첫 프레임 또는 None, 마지막으로 본 (가로, 세로)).
    해상도에 따라 화면 범위가 달라지는 카메라가 있어(BRIO: 640×480은 4K 화면의 가운데 4:3만 보여 1.33배 확대된 것처럼 보임),
    바꾸기 전 프레임을 보여 주거나 등록에 쓰면 켤 때마다 배율이 달라진다."""
    want = wanted_size(settings)
    deadline = time.monotonic() + (SIZE_SETTLE_S if timeout is None else timeout)
    last = None
    while not stop.is_set() and time.monotonic() < deadline:
        ok, frame = cap.read()
        if ok and frame is not None:
            last = (frame.shape[1], frame.shape[0])
            if last == want:
                return frame, last
    return None, last


def apply_focus(cap, settings):
    if settings and settings.get("focus") is not None:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, float(settings["focus"]))


def fit_preview(frame, width=PREVIEW_WIDTH):
    """가로 width로 줄인 BGR 복사본. 화면 갱신용이라 원본을 보관하지 않습니다."""
    h, w = frame.shape[:2]
    if w <= width:
        return frame.copy()
    scale = width / w
    return cv2.resize(frame, (width, max(1, int(h * scale))), interpolation=cv2.INTER_AREA)


class CameraPreview:
    """최신 프레임 하나. start() 전에는 장치를 열지 않습니다. wanted가 None이면 열리는 첫 카메라."""

    def __init__(self, opener=open_capture, wanted=AUTO, settings=None, names_reader=None):
        self.opener = opener
        self.wanted = wanted
        self.settings = settings         # 고른 카메라의 촬영 설정 (capture_settings)
        # 자동 선택일 때 번호 → 장치 이름(촬영 설정 찾기). 장치 이름은 실제 장치를 열 때만 뜻이 있어,
        # 가짜 opener에는 기본으로 이름을 읽지 않음
        self.names_reader = names_reader or (None if opener is open_capture else (lambda: None))
        self.frame = None
        self.full = None                 # 줄이지 않은 원본 (종이 위치 찍기·기록 사진용, 복사하지 않고 참조만 보관)
        self.zoom = None                 # 크게 보기: 원본의 zoom_box 부분을 zoom_width로 줄인 것 (켰을 때만 만듦)
        self.zoom_box = None             # (x0, y0, x1, y1) 원본 px, None이면 사진 전체
        self.zoom_width = None           # None이면 크게 보기 꺼짐 (원본을 줄이는 계산을 하지 않음)
        self.index = None
        self.error = ""
        self._stop = threading.Event()
        self._thread = None
        self._generation = 0
        self._lock = threading.Lock()

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.running:
            return
        # 실행마다 새 멈춤 신호: 장치를 여느라 늦게 끝나는 이전 스레드가 다시 살아나지 않게
        self._stop = threading.Event()
        self.error = ""
        gen = self._generation
        self._thread = threading.Thread(target=self._run, args=(self._stop, self.wanted, self.settings, gen),
                                        name="camera-preview", daemon=True)
        self._thread.start()

    def _retire(self):
        """이전 스레드가 캡처를 놓은 뒤에만 돌아온다. 세대가 바뀌면 늦은 쓰기는 무시된다."""
        self._generation += 1
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join()
        if self._thread is thread:
            self._thread = None

    def select(self, wanted, start=True, settings=None):
        """다른 카메라로 바꿈. 이전 장치가 닫힌 뒤에만 다음 장치를 연다."""
        self._retire()
        with self._lock:
            self.frame = self.full = self.zoom = None
        self.index = None
        self.wanted = wanted
        self.settings = settings
        if start:
            self.start()

    def _run(self, stop, wanted, settings, gen):
        def current():
            return gen == self._generation

        if wanted is AUTO:
            found = find_camera(opener=self.opener)
        else:
            opened = self.opener(wanted)
            found = None if opened is None else (wanted, *opened)
        if found is None:
            if current() and not stop.is_set():
                self.error = "카메라를 찾지 못했습니다" if wanted is AUTO else "고른 카메라를 열 수 없습니다"
            return
        index, cap, frame = found
        if not current() or stop.is_set():              # 여는 동안 다른 카메라를 골랐거나 껐음
            cap.release()
            return
        self.index = index
        if wanted is AUTO:
            settings = auto_settings(index, self.names_reader)
        apply_size(cap, settings)
        want = wanted_size(settings)
        if want is not None and (frame.shape[1], frame.shape[0]) != want:
            frame, got = settle_size(cap, settings, stop)
            if frame is None and current() and not stop.is_set():
                apply_size(cap, settings)                 # 한 번 더 요청 (드라이버가 첫 요청을 무시하는 경우)
                frame, got = settle_size(cap, settings, stop)
            if frame is None:
                if current() and not stop.is_set():
                    self.error = (f"카메라가 설정한 {want[0]}×{want[1]}로 열리지 않았습니다"
                                  + (f" (지금 {got[0]}×{got[1]})" if got else "")
                                  + ". 해상도가 다르면 화면 범위가 달라 종이 위치가 맞지 않습니다")
                cap.release()
                return
        if not current():
            cap.release()
            return
        with self._lock:
            if current():
                self.frame, self.full = fit_preview(frame), frame
        interval = 1.0 / PREVIEW_FPS
        frames = 0
        while not stop.is_set() and current():
            t0 = time.perf_counter()
            ok, nxt = cap.read()
            if stop.is_set() or not current():
                break
            if ok and nxt is not None:
                if want is not None and (nxt.shape[1], nxt.shape[0]) != want:
                    if current():
                        self.error = f"카메라 해상도가 {nxt.shape[1]}×{nxt.shape[0]}로 바뀌었습니다. 카메라를 다시 켜 주세요"
                    break
                small = fit_preview(nxt)
                zoom = self._make_zoom(nxt)
                with self._lock:
                    if current():
                        self.frame, self.full, self.zoom = small, nxt, zoom
                frames += 1
                if frames == FOCUS_AFTER_FRAMES:
                    apply_focus(cap, settings)
            else:
                if current():
                    self.error = "카메라 프레임을 읽지 못했습니다"
                break
            time.sleep(max(0.0, interval - (time.perf_counter() - t0)))
        cap.release()

    def latest(self):
        with self._lock:
            return None if self.frame is None else self.frame.copy()

    def latest_full(self):
        """줄이지 않은 최신 프레임 복사본."""
        with self._lock:
            return None if self.full is None else self.full.copy()

    def set_zoom(self, width, box=None):
        """크게 보기 켬(width px로 줄인 box 부분) / 끔(width=None). 줄이는 계산은 카메라 스레드가 함."""
        self.zoom_box, self.zoom_width = box, width
        if width is None:
            with self._lock:
                self.zoom = None

    def _make_zoom(self, frame):
        width, box = self.zoom_width, self.zoom_box
        if not width:
            return None
        if box is not None:
            x0, y0, x1, y1 = box
            frame = frame[y0:y1, x0:x1]
        if frame.size == 0:
            return None
        return fit_preview(frame, width), box

    def latest_zoom(self):
        """(크게 보기 사진, 그 사진을 자른 box 또는 None=전체) — 꺼져 있거나 아직 없으면 (None, None)."""
        with self._lock:
            return (None, None) if self.zoom is None else self.zoom

    def stop(self):
        self._retire()
        with self._lock:                             # 다음에 켜거나 다른 카메라를 고르면 새 프레임부터
            self.frame = self.full = self.zoom = None
        self.index = None


def blank_preview(text="카메라 꺼짐"):
    """카메라가 없을 때 보여줄 단색 프레임."""
    img = np.full((270, PREVIEW_WIDTH, 3), 28, np.uint8)
    cv2.putText(img, text, (24, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1, cv2.LINE_AA)
    return img
