"""얼굴 부위 분할 (BiSeNet, CelebAMask-HQ 19부위). 모자·머리카락·눈·눈썹·귀·귀걸이·입술·옷을 나눠,
부위마다 사람이 그리는 방식으로 선을 고르게 합니다.

가중치: https://github.com/yakhyo/face-parsing (MIT) releases/weights/resnet18.onnx
"""

from pathlib import Path

import cv2
import numpy as np

MODEL_NAME = "face_parsing_resnet18.onnx"
SIZE = 512
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)

PARTS = ("background", "skin", "l_brow", "r_brow", "l_eye", "r_eye", "eye_g", "l_ear", "r_ear", "earring",
         "nose", "mouth", "u_lip", "l_lip", "neck", "necklace", "cloth", "hair", "hat")
GROUPS = {
    "skin": ("skin", "nose", "neck"),
    "brow": ("l_brow", "r_brow"),
    "eye": ("l_eye", "r_eye"),
    "ear": ("l_ear", "r_ear"),
    "earring": ("earring",),
    "lips": ("u_lip", "l_lip", "mouth"),
    "cloth": ("cloth", "necklace"),
    "hair": ("hair",),
    "hat": ("hat",),
}

_session = None


def model_path():
    return Path(__file__).resolve().parent / "data" / MODEL_NAME


def available():
    return model_path().exists()


def _load():
    global _session
    if _session is None:
        import onnxruntime as ort
        _session = ort.InferenceSession(str(model_path()), providers=["CPUExecutionProvider"])
    return _session


def head_square(face, img_w, img_h, scale=2.6):
    """얼굴 중심의 정사각형 (모델이 학습한 얼굴 사진과 비슷한 구도). 이미지 밖은 잘림."""
    x, y, w, h = [float(v) for v in face.box]
    side = scale * max(w, h)
    cx, cy = x + w / 2, y + h / 2 - 0.15 * h
    x0, y0 = int(round(cx - side / 2)), int(round(cy - side / 2))
    return x0, y0, int(round(x0 + side)), int(round(y0 + side))


def _parse_once(bgr, faces, scale):
    h, w = bgr.shape[:2]
    labels = np.zeros((h, w), np.uint8)
    for face in faces:
        x0, y0, x1, y1 = head_square(face, w, h, scale)
        pad = [max(0, -y0), max(0, y1 - h), max(0, -x0), max(0, x1 - w)]
        crop = bgr[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
        crop = cv2.copyMakeBorder(crop, *pad, cv2.BORDER_REPLICATE)
        rgb = cv2.cvtColor(cv2.resize(crop, (SIZE, SIZE), interpolation=cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB)
        x = ((rgb.astype(np.float32) / 255.0 - MEAN) / STD).transpose(2, 0, 1)[None]
        out = _load().run(None, {"input": x})[0][0].argmax(0).astype(np.uint8)
        out = cv2.resize(out, (crop.shape[1], crop.shape[0]), interpolation=cv2.INTER_NEAREST)
        out = out[pad[0]:out.shape[0] - pad[1], pad[2]:out.shape[1] - pad[3]]
        region = labels[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
        region[out > 0] = out[out > 0]
    return labels


SMALL_PARTS = ("l_brow", "r_brow", "l_eye", "r_eye", "l_ear", "r_ear", "earring", "nose", "mouth", "u_lip", "l_lip")


_last = None   # (사진·얼굴 열쇠, 결과): 같은 사진에서 다른 설정만 바꿀 때 다시 돌리지 않음


def parse(bgr, faces):
    """부위 번호 지도 (입력과 같은 크기, 0 = 배경).
    넓게 자른 결과(모자·머리카락 전체)에, 가깝게 자른 결과의 눈·눈썹·귀·입술과 앞머리를 덮어씀.
    입 아래로 내려온 '모자'는 옷(레이스 옷깃 등)으로 바꿈."""
    global _last
    key = (bgr.shape, bgr.tobytes()[::4099], tuple(tuple(np.round(f.box, 1)) for f in faces))
    if _last is not None and _last[0] == key:
        return _last[1].copy()
    labels = _parse_raw(bgr, faces)
    _last = (key, labels)
    return labels.copy()


def _parse_raw(bgr, faces):
    labels = _parse_once(bgr, faces, 2.6)
    if not faces:
        return labels
    tight = _parse_once(bgr, faces, 1.3)
    small = np.isin(tight, [PARTS.index(n) for n in SMALL_PARTS])
    labels[small] = tight[small]
    rows = np.arange(labels.shape[0])[:, None]
    hat, hair, cloth = PARTS.index("hat"), PARTS.index("hair"), PARTS.index("cloth")
    for face in faces:
        lm = np.asarray(face.landmarks, np.float64)
        eye_y, mouth_y, fh = float(lm[:2, 1].min()), float(lm[3:, 1].max()), float(face.box[3])
        bangs = (tight == hair) & (labels == hat) & (rows > eye_y - 0.25 * fh)
        labels[bangs] = hair
        labels[(labels == hat) & (rows > mouth_y)] = cloth
    return labels


def coverage(shape, faces):
    """분할 모델이 실제로 본 영역(넓은 사각형들). 이 경계는 사진의 경계가 아니므로 윤곽으로 긋지 않음."""
    h, w = shape[:2]
    seen = np.zeros((h, w), np.uint8)
    for face in faces:
        x0, y0, x1, y1 = head_square(face, w, h, 2.6)
        seen[max(0, y0):min(h, y1), max(0, x0):min(w, x1)] = 255
    return seen


def mask(labels, group):
    ids = [PARTS.index(n) for n in GROUPS[group]]
    return np.isin(labels, ids).astype(np.uint8) * 255
