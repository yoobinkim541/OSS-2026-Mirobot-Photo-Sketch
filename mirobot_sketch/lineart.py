"""실사 사진을 선화로 바꿉니다. Informative Drawings (Chan, Durand, Isola, CVPR 2022).

로봇 경로는 만들지 않습니다. 나온 흑백 선화를 기존 어두운 선 단계가 획으로 추적합니다.
가중치(17 MB)는 저장소에 넣지 않고, 처음 쓸 때 이 프로젝트의 GitHub Releases에서 내려받아
SHA256을 확인한 뒤 캐시 폴더(paths.cache_dir()/models)에 둡니다.
원본: https://huggingface.co/rocca/informative-drawings-line-art-onnx (model.onnx)
"""

import hashlib
import os
import urllib.request

import cv2
import numpy as np

MODEL_NAME = "informative_drawings.onnx"
MODEL_URL = ("https://github.com/yoobinkim541/OSS-2026-Mirobot-Photo-Sketch/releases/download/"
             "models-v1/informative_drawings.onnx")
MODEL_SHA256 = "1fef40b8f7126d827e30fbebccf95ae9b0b391795df926bf9366a821bad4f498"
DOWNLOAD_TIMEOUT_S = 60
MAX_SIDE = 768
STRONG_INK = 165      # 이 이상 진한 점이 있는 선만 남기고 (잡결 제거)
WEAK_INK = 225        # 그 선을 따라 옅은 끝부분까지 이어 남김 (속눈썹·입술처럼 가는 선이 끊기지 않게)
HOLLOW_PX = 2.0       # 이보다 두꺼운 먹 덩어리(눈동자 등)는 속을 비워 테두리로 그림 (뼈대로 따면 짧은 토막이 되어 사라짐)

_session = None


class LineartUnavailable(RuntimeError):
    """선화 모델을 쓸 수 없음 (onnxruntime 없음, 내려받기 실패, 파일 손상)."""


def model_path():
    from . import paths
    return paths.cache_dir() / "models" / MODEL_NAME


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def ensure_model(url=MODEL_URL, timeout=DOWNLOAD_TIMEOUT_S):
    """모델 파일 경로. 없으면 내려받고, 받은 파일의 SHA256이 다르면 버리고 LineartUnavailable."""
    path = model_path()
    if path.exists():
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_suffix(".part")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp, open(part, "wb") as out:
            for block in iter(lambda: resp.read(1 << 20), b""):
                out.write(block)
        if _sha256(part) != MODEL_SHA256:
            raise LineartUnavailable("내려받은 선화 모델이 손상됐습니다 (SHA256 불일치).")
        os.replace(part, path)
    except OSError as e:          # 인터넷 없음, 시간 초과, 디스크 오류 (URLError도 OSError)
        raise LineartUnavailable(f"선화 모델을 내려받지 못했습니다: {e}") from None
    finally:
        if part.exists():
            part.unlink()
    return path


def _load():
    global _session
    if _session is None:
        try:
            import onnxruntime as ort
        except ImportError:
            raise LineartUnavailable("onnxruntime이 설치되어 있지 않습니다.") from None
        _session = ort.InferenceSession(str(ensure_model()), providers=["CPUExecutionProvider"])
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



def hysteresis(gray, strong=STRONG_INK, weak=WEAK_INK):
    """옅은 선(weak 미만) 중 진한 점(strong 미만)과 이어진 덩어리만 남긴 이진 마스크 (0/255)."""
    weak_mask = (gray < weak).astype(np.uint8)
    _, labels = cv2.connectedComponents(weak_mask, connectivity=8)
    seeds = np.unique(labels[(gray < strong) & (weak_mask > 0)])
    return np.isin(labels, seeds[seeds > 0]).astype(np.uint8) * 255


def hollow(mask, keep_px=HOLLOW_PX):
    """두꺼운 덩어리의 속을 비움: 경계에서 keep_px 안쪽까지만 남김. 가는 선은 그대로."""
    inside = (cv2.distanceTransform((mask > 0).astype(np.uint8), cv2.DIST_L2, 3) > keep_px).astype(np.uint8)
    inside = cv2.morphologyEx(inside, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    out = mask.copy()
    out[inside > 0] = 0
    return out


def ink_mask(bgr):
    """(선화 BGR, 선 마스크) — 둘 다 입력과 같은 크기. 선화는 사진을 키우지 않고 긴 변 MAX_SIDE로 그린 것을 씀
    (키워 넣으면 니트 결 같은 잔선이 더 촘촘해져 펜으로 그리면 덩어리가 됨)."""
    art = line_drawing(bgr)
    return art, hollow(hysteresis(cv2.cvtColor(art, cv2.COLOR_BGR2GRAY)))