"""실사 사진을 선화로 바꿉니다. Informative Drawings (Chan, Durand, Isola, CVPR 2022).

로봇 경로는 만들지 않습니다. 나온 흑백 선화를 기존 어두운 선 단계가 획으로 추적합니다.
가중치: https://huggingface.co/rocca/informative-drawings-line-art-onnx (model.onnx)
"""

from pathlib import Path

import cv2
import numpy as np

MODEL_NAME = "informative_drawings.onnx"
MAX_SIDE = 768
INK_THRESHOLD = 170   # 얼굴 피부의 옅은 잡선은 버림

_session = None


def model_path():
    from . import paths
    bundled = paths.asset(MODEL_NAME) if hasattr(paths, "asset") else None
    here = Path(__file__).resolve().parent / "data" / MODEL_NAME
    if here.exists():
        return here
    if bundled is not None and Path(bundled).exists():
        return Path(bundled)
    return here


def available():
    return model_path().exists()


def _load():
    global _session
    if _session is None:
        import onnxruntime as ort
        path = model_path()
        if not path.exists():
            raise FileNotFoundError(f"선화 모델이 없습니다: {path}")
        _session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    return _session


def line_drawing(bgr, max_side=MAX_SIDE):
    """BGR 사진을 흰 바탕의 어두운 선화(BGR)로. 입력과 같은 크기."""
    h, w = bgr.shape[:2]
    scale = min(1.0, max_side / max(h, w))
    small = bgr if scale == 1.0 else cv2.resize(
        bgr, (max(1, round(w * scale)), max(1, round(h * scale))), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    tensor = np.transpose(rgb, (2, 0, 1))[None]
    out = _load().run(None, {"input": tensor})[0]
    art = np.asarray(out)[0]
    if art.ndim == 3:
        art = art[0] if art.shape[0] == 1 else art.mean(axis=0)
    art = np.clip(art, 0.0, 1.0)
    gray = (art * 255.0).astype(np.uint8)
    if float(gray.mean()) < 127.0:   # 모델은 검은 바탕에 흰 선을 냄
        gray = 255 - gray
    if gray.shape[:2] != (h, w):
        gray = cv2.resize(gray, (w, h), interpolation=cv2.INTER_CUBIC)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
