"""얼굴 점 478개로 눈·눈썹·다문 입을 다시 그리고, 끊긴 턱선을 잇습니다.

YuNet은 눈·코·입꼬리 다섯 점만 줘서 눈꺼풀·눈동자·입술 모양을 알 수 없었습니다. 그래서 선화의 눈 속 짧은 선을
그대로 두면 안경 반사광·속눈썹이 얼룩이 되고, 지우면 눈이 통째로 사라졌으며, 다문 입술의 위·아래 윤곽이
벌린 입처럼 그려졌습니다. 얼굴 점이 있으면:
- 눈: 선화의 눈 속 선을 지우고 윗눈꺼풀 곡선, 아래 눈꺼풀 눈꼬리~가운데 너머, 눈꺼풀 안쪽으로 잘린 홍채 원과 펜 굵기
  점 하나(눈동자)로 그립니다. 감은 눈은 윗눈꺼풀만 그립니다.
- 눈썹: 선화의 끊긴 눈썹 조각 대신 위·아래 윤곽 사이 가운데 한 줄(두꺼우면 두 줄)로 그립니다.
- 다문 입: 입술 사이 선(입술이 맞닿은 선)과 위·아래 입술 바깥선의 가운데로 그립니다. 벌린 입은 기존 정리
  (declutter.tidy_mouth)를 그대로 씁니다.
- 턱선: 선화가 놓친 구간 중 사진에 경계가 보이는 곳만 얼굴 점 곡선으로 채웁니다(fill_jaw).
정면에서 많이 돌아간 얼굴(두 눈 폭 비율이 낮음)은 가려진 쪽 점이 추정값이라 다시 그리지 않습니다.

모델: MediaPipe Face Landmarker(Apache-2.0)의 face_landmarks_detector.tflite(face_landmarker.task float16 v1 안)를
tf2onnx로 ONNX로 바꾼 것(4.9 MB). 같은 입력에서 TFLite와 출력 차이는 256px 기준 0.0002px. 이미 쓰는 onnxruntime으로
돌리므로 mediapipe 패키지(opencv-contrib-python을 끌어와 opencv-python의 cv2를 덮어씀)가 필요 없습니다.
MediaPipe가 하던 앞뒤 처리는 여기서 합니다: YuNet 얼굴 상자와 두 눈으로 기울어진 정사각 영역(긴 변 1.5배)을 256px로
잘라 한 번 돌리고, 나온 점으로 영역을 다시 잡아(눈꼬리 33·263으로 기울기, 점 범위의 1.5배) 한 번 더 돌립니다.
정면 사진 7장(두 눈 폭 비 0.85 이상)에서 MediaPipe 결과와 평균 차이는 두 눈 사이 거리의 0.5~1.6%였고, 머리를 기울인
사진 한 장은 MediaPipe가 한쪽 눈을 볼에 놓았는데 이 방식은 눈 위에 놓았습니다(차이 10.6%).
처음 쓸 때 이 프로젝트의 GitHub Releases에서 내려받아 SHA256을 확인합니다. 못 받으면 예전처럼 그립니다.
"""

import cv2
import numpy as np

from . import models
from . import sketch_pipeline as sp

MODEL_NAME = "face_landmarks_478.onnx"
MODEL_URL = ("https://github.com/yoobinkim541/OSS-2026-Mirobot-Photo-Sketch/releases/download/"
             "models-v1/face_landmarks_478.onnx")
MODEL_SHA256 = "2d7005f71d33a9e754ef6e22a70de016d9130fbdab385ff41bee426d8bab05ec"
INPUT_SIDE = 256          # 모델 입력 (RGB, 0~1)
ROI_SCALE = 1.5           # 얼굴 영역 → 모델 입력: 긴 변의 이 배수 정사각 (이 값으로 MediaPipe 결과와 위 차이)
FRONTAL_EYE_RATIO = 0.6   # 두 눈 폭의 비(작은/큰)가 이 미만이면 돌아간 얼굴: 다시 그리지 않음
REDRAWN = "눈·입을 얼굴 점으로 다시 그림"

# MediaPipe Face Mesh 번호 (눈꼬리 → 눈앞머리 순)
EYES = ({"upper": (33, 246, 161, 160, 159, 158, 157, 173, 133), "lower": (33, 7, 163, 144, 145, 153, 154, 155, 133)},
        {"upper": (263, 466, 388, 387, 386, 385, 384, 398, 362), "lower": (263, 249, 390, 373, 374, 380, 381, 382, 362)})
IRISES = ((468, 469, 470, 471, 472), (473, 474, 475, 476, 477))
LIP_OUTER_UP = (61, 185, 40, 39, 37, 0, 267, 269, 270, 409, 291)
LIP_OUTER_LOW = (61, 146, 91, 181, 84, 17, 314, 405, 321, 375, 291)
LIP_INNER_UP = (78, 191, 80, 81, 82, 13, 312, 311, 310, 415, 308)
LIP_INNER_LOW = (78, 95, 88, 178, 87, 14, 317, 402, 318, 324, 308)
BROWS = (((70, 63, 105, 66, 107), (46, 53, 52, 65, 55)),          # (위 윤곽, 아래 윤곽) 꼬리 → 머리
         ((300, 293, 334, 296, 336), (276, 283, 282, 295, 285)))
JAW = (234, 93, 132, 58, 172, 136, 150, 149, 176, 148, 152, 377, 400, 378, 379, 365, 397, 288, 361, 323, 454)
JAW_COVER_PEN = 2.0      # 턱선 점에서 이 배수(펜 굵기) 안에 선이 있으면 이미 그려진 곳
JAW_MIN_RUN_PEN = 4.0    # 이보다 짧은 빈 구간은 채우지 않음 (점처럼 보임)
JAW_MIN_CONTRAST = 40.0  # 사진에서 턱선 자리의 색·밝기 대비가 이보다 약하면 실제로 안 보이는 경계: 채우지 않음

_session = None


class MeshUnavailable(RuntimeError):
    """얼굴 점을 쓸 수 없음 (onnxruntime 없음, 모델 내려받기 실패)."""


def _load():
    global _session
    if _session is None:
        try:
            import onnxruntime as ort
        except ImportError:
            raise MeshUnavailable("onnxruntime이 없어 눈·입을 얼굴 점으로 다시 그리지 않습니다.") from None
        try:
            path = models.fetch(MODEL_NAME, MODEL_URL, MODEL_SHA256, "얼굴 점 모델")
        except models.ModelUnavailable as e:
            raise MeshUnavailable(f"{e} 눈·입은 얼굴 점 없이 그렸습니다.") from None
        _session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    return _session


def _run(img, center, size, angle):
    """기울어진 정사각 영역(중심, 한 변, 각도 라디안)을 모델에 넣어 (478×2 원본 좌표, 얼굴 점수 로짓)."""
    sess = _load()
    ca, sa, k = np.cos(angle), np.sin(angle), size / INPUT_SIDE
    half = INPUT_SIDE / 2
    fwd = np.array([[ca * k, -sa * k, center[0] - (ca - sa) * half * k],     # 입력 (u, v) → 원본
                    [sa * k, ca * k, center[1] - (sa + ca) * half * k]])
    crop = cv2.warpAffine(img, cv2.invertAffineTransform(fwd), (INPUT_SIDE, INPUT_SIDE), flags=cv2.INTER_LINEAR)
    x = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).astype(np.float32)[None] / 255.0
    pts, score = sess.run(None, {sess.get_inputs()[0].name: x})[:2]
    pts = pts.reshape(-1, 3)[:, :2].astype(np.float64)
    return pts @ fwd[:, :2].T + fwd[:, 2], float(np.ravel(score)[0])


def _roi(pts, eye_a, eye_b):
    """점들을 감싸는 영역: 두 눈을 이은 기울기로 돌린 틀에서 점 범위의 긴 변 × ROI_SCALE 정사각 (중심, 한 변, 각도)."""
    ang = float(np.arctan2(eye_b[1] - eye_a[1], eye_b[0] - eye_a[0]))
    ca, sa = np.cos(ang), np.sin(ang)
    rot = np.array([[ca, -sa], [sa, ca]])                                        # 틀 → 원본
    local = (np.asarray(pts, np.float64) - pts[0]) @ rot
    lo, hi = local.min(axis=0), local.max(axis=0)
    return pts[0] + ((lo + hi) / 2) @ rot.T, ROI_SCALE * float(max(hi - lo)), ang


def detect(full_bgr, face):
    """원본 사진과 YuNet 얼굴(원본 좌표)로 얼굴 점 478개 (478×2, 원본 좌표). 얼굴이 아니라고 보면 None.
    onnxruntime·모델이 없으면 MeshUnavailable."""
    x, y, w, h = (float(v) for v in face.box)
    box = np.array([[x, y], [x + w, y], [x, y + h], [x + w, y + h]])
    pts, _ = _run(full_bgr, *_roi(box, face.landmarks[0], face.landmarks[1]))
    pts, score = _run(full_bgr, *_roi(pts, pts[33], pts[263]))   # 첫 결과로 영역을 다시 잡아 한 번 더
    return pts if score > 0.0 else None


# ------------------------------------------------------------------------------------------- 다시 그리기
def _chaikin(q, iters=2):
    q = np.asarray(q, np.float64)
    for _ in range(iters):
        a, b = q[:-1], q[1:]
        mid = np.empty((2 * len(a), 2))
        mid[0::2], mid[1::2] = 0.75 * a + 0.25 * b, 0.25 * a + 0.75 * b
        q = np.vstack([q[:1], mid, q[-1:]])
    return q


def _part(q, t0, t1):
    """폴리라인의 길이 비율 t0~t1 구간."""
    q = np.asarray(q, np.float64)
    d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
    s = np.linspace(t0 * d[-1], t1 * d[-1], max(2, len(q)))
    return np.column_stack([np.interp(s, d, q[:, 0]), np.interp(s, d, q[:, 1])])


def frontal(pts):
    """두 눈 폭의 비 (작은/큰). 1에 가까울수록 정면."""
    widths = [np.linalg.norm(pts[e["upper"][0]] - pts[e["upper"][-1]]) for e in EYES]
    return min(widths) / max(max(widths), 1e-6)


def _eye(pts, eye, iris, pen_px):
    """한 눈의 (새 획들, 지울 영역 다각형)."""
    up, low = pts[list(eye["upper"])], pts[list(eye["lower"])]
    opening = float(np.max(np.linalg.norm(up[2:-2] - low[2:-2], axis=1)))
    poly = np.vstack([up, low[::-1][1:-1]])
    out = [_chaikin(up)]
    if opening < 1.5 * pen_px:                       # 감은 눈·실눈: 윗눈꺼풀 한 줄
        return out, poly
    out.append(_chaikin(_part(low, 0.0, 0.7)))       # 눈꼬리 ~ 가운데 너머: 눈앞머리 쪽은 비워 둠 (화가들의 생략)
    ir = pts[list(iris)]
    c, r = ir[0], float(np.mean(np.linalg.norm(ir[1:] - ir[0], axis=1)))
    if r >= 1.75 * pen_px:                           # 작은 얼굴은 홍채 원이 눈동자 점과 뭉쳐 얼룩이 됨: 점만
        t = np.linspace(0, 2 * np.pi, 48, endpoint=False)
        ring = c + r * np.column_stack([np.cos(t), np.sin(t)])
        inside = np.array([cv2.pointPolygonTest(poly.astype(np.float32), (float(p[0]), float(p[1])), False) > 0
                           for p in ring])
        if inside.any() and not inside.all():        # 눈꺼풀에 가린 부분을 빼고 이어진 호들로
            k = int(np.argmin(inside))
            ring, inside = np.roll(ring, -k, axis=0), np.roll(inside, -k)
        runs, cur = [], []
        for p, ok in zip(ring, inside):
            if ok:
                cur.append(p)
            elif cur:
                runs.append(cur)
                cur = []
        if cur:
            runs.append(cur + ([ring[0]] if inside.all() else []))
        out += [np.asarray(q) for q in runs if len(q) >= 3]
    dot = 0.35 * pen_px                               # 눈동자: 펜 굵기만 한 점
    t = np.linspace(0, 2 * np.pi, 9)
    out.append(c + dot * np.column_stack([np.cos(t), np.sin(t)]))
    return out, poly


def _closed_mouth(pts, pen_px):
    """다문 입이면 (새 획들, 지울 영역), 벌린 입이면 None."""
    iu, il = pts[list(LIP_INNER_UP)], pts[list(LIP_INNER_LOW)]
    width = float(np.linalg.norm(pts[61] - pts[291]))
    gap = float(np.max(np.linalg.norm(iu[2:-2] - il[2:-2], axis=1)))
    if gap > max(1.5 * pen_px, 0.08 * width):
        return None
    seam = 0.5 * (iu + il)
    seam[0], seam[-1] = pts[61], pts[291]            # 입꼬리까지
    # 위·아래 입술 바깥선은 가운데만: 입꼬리까지 이으면 입술 사이 선과 함께 닫힌 고리가 되어 벌린 입처럼 보임
    upper = _part(pts[list(LIP_OUTER_UP)], 0.25, 0.75)
    lower = _part(pts[list(LIP_OUTER_LOW)], 0.3, 0.7)
    poly = np.vstack([pts[list(LIP_OUTER_UP)], pts[list(LIP_OUTER_LOW)][::-1][1:-1]])
    return [_chaikin(seam), _chaikin(upper), _chaikin(lower)], poly


def _brow(pts, upper, lower, pen_px):
    """눈썹 (새 획들, 지울 영역). 위·아래 윤곽 사이 가운데 한 줄, 두꺼우면 1/3·2/3 두 줄(꼬리에서 만남)."""
    up, low = pts[list(upper)], pts[list(lower)]
    poly = np.vstack([up, low[::-1]])
    thick = float(np.mean(np.linalg.norm(up - low, axis=1)))
    if thick < 3 * pen_px:
        return [_chaikin(0.5 * (up + low))], poly
    a, b = up + (low - up) / 3, up + 2 * (low - up) / 3
    return [_chaikin(np.vstack([a[::-1], b[1:]]))], poly       # 머리 → 꼬리 → 머리: 펜을 한 번만 내림


def redraw(strokes, meshes, pen_px, shape):
    """얼굴마다 정면이면 눈·눈썹을, 다문 입이면 입을 얼굴 점으로 다시 그린다. 코는 선화 그대로 둔다
    (콧볼~코끝 밑을 얼굴 점 한 줄로 그려 보니 둘째 입처럼 보였고 콧구멍 모양이 사라졌음).
    meshes: 얼굴마다 478×2(작업 좌표) 또는 None. 반환: (새 획 목록, 지운 조각 목록, 입을 다시 그린 얼굴 번호 집합)."""
    gone_all, mouths = [], set()
    for i, pts in enumerate(meshes):
        if pts is None or frontal(pts) < FRONTAL_EYE_RATIO:
            continue
        new, polys = [], []
        for eye, iris in zip(EYES, IRISES):
            lines, poly = _eye(pts, eye, iris, pen_px)
            new += lines
            polys.append(poly)
        for upper, lower in BROWS:
            lines, poly = _brow(pts, upper, lower, pen_px)
            new += lines
            polys.append(poly)
        mouth = _closed_mouth(pts, pen_px)
        if mouth is not None:
            new += mouth[0]
            polys.append(mouth[1])
            mouths.add(i)
        mask = np.zeros(shape[:2], np.uint8)
        for poly in polys:
            cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 1)
        r = max(1, int(round(1.5 * pen_px)))
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))) > 0
        strokes, gone = sp.cut_by_mask(strokes, mask)
        gone_all += gone
        strokes = strokes + new
    return strokes, gone_all, mouths


def fill_jaw(strokes, meshes, contrast, pen_px):
    """정면 얼굴의 턱선(귀 아래 → 턱 끝 → 반대쪽)에서 선화가 놓친 구간만 얼굴 점 곡선으로 채운다.
    채우는 곳: 가까이(펜 2배)에 이미 그은 선이 없고, 사진에도 그 자리에 경계(색·밝기 대비)가 보이는 구간.
    contrast: declutter.contrast_map 결과. 반환: (새 획 목록, 더한 획 목록)."""
    h, w = contrast.shape[:2]
    added = []
    for pts in meshes:
        if pts is None or frontal(pts) < FRONTAL_EYE_RATIO:
            continue
        drawn = np.full((h, w), 255, np.uint8)
        for q in strokes + added:
            cv2.polylines(drawn, [np.round(np.asarray(q)).astype(np.int32).reshape(-1, 1, 2)], False, 0, 1)
        dist = cv2.distanceTransform(drawn, cv2.DIST_L2, 3)
        jaw = _chaikin(pts[list(JAW)])
        d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(jaw, axis=0), axis=1))])
        s = np.arange(0.0, d[-1], 1.0)
        line = np.column_stack([np.interp(s, d, jaw[:, 0]), np.interp(s, d, jaw[:, 1])])
        ix = np.clip(np.round(line[:, 0]).astype(int), 0, w - 1)
        iy = np.clip(np.round(line[:, 1]).astype(int), 0, h - 1)
        inside = (line[:, 0] >= 0) & (line[:, 0] < w) & (line[:, 1] >= 0) & (line[:, 1] < h)
        need = inside & (dist[iy, ix] > JAW_COVER_PEN * pen_px) & (contrast[iy, ix] >= JAW_MIN_CONTRAST)
        edges = np.flatnonzero(np.diff(need.astype(np.int8))) + 1
        for seg, flag in zip(np.split(line, edges), np.split(need, edges)):
            if flag[0] and len(seg) >= JAW_MIN_RUN_PEN * pen_px:
                added.append(seg)
    return strokes + added, added
