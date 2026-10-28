"""머리카락·모자처럼 결이 있는 어두운 덩어리를 긴 흐름선으로 다시 그립니다.

선화 모델은 머리카락을 짧은 조각으로 그립니다. 조각을 이어도 한 올의 흐름이 되지 않아,
여기서는 사진의 결 방향(구조 텐서)을 따라 고르게 떨어진 흐름선을 새로 만듭니다
(Jobard & Lefer 1997, Evenly-spaced streamlines; Kang et al. 2007, Coherent line drawing의 흐름장).
덩어리의 윤곽과 모자 접힘선 같은 긴 구조선은 선화에서 그대로 쓰고, 흐름선은 그 선을 넘지 않습니다.
"""

import cv2
import numpy as np

DARK_VALUE = 105         # HSV 명도. 조명 받은 모자 쪽도 포함
TEXTURE_MIN = 0.8       # 사진의 잔결 세기. 니트·머리결은 1 이상, 흐린 배경은 0.4 이하
SPACING_PX = 9.0         # 흐름선 사이 간격 (800px 기준, 종이에서 약 1mm)
MIN_LEN_PX = 22.0
MAX_LEN_PX = 170.0
FLOW_SIGMA = 5.0
STRUCTURE_PX = 70.0      # 이보다 긴 선화 획은 구조선(윤곽·접힘선)으로 남김


def head_region(shape, faces):
    """머리카락·모자가 있을 수 있는 영역. 얼굴 앞쪽은 눈썹 위만 (마이크·손 제외), 눈·코·입 주변은 뺌."""
    h, w = shape[:2]
    region = np.zeros((h, w), np.uint8)
    for face in faces:
        x, y, fw, fh = [float(v) for v in face.box]
        lm = np.asarray(face.landmarks, np.float64)
        back_right = float(lm[2][0]) < x + fw / 2      # 코가 왼쪽이면 뒤통수는 오른쪽
        cx = x + fw / 2
        top, bottom = y - 1.5 * fh, y + 2.1 * fh
        brow = min(lm[0][1], lm[1][1]) - 0.12 * fh
        if back_right:
            back = (cx - 0.1 * fw, top, x + fw + 1.4 * fw, bottom)
            front = (x - 0.35 * fw, top, cx, brow)
        else:
            back = (x - 1.4 * fw, top, cx + 0.1 * fw, bottom)
            front = (cx, top, x + fw + 0.35 * fw, brow)
        for x0, y0, x1, y1 in (back, front):
            cv2.rectangle(region, (int(max(0, x0)), int(max(0, y0))),
                          (int(min(w - 1, x1)), int(min(h - 1, y1))), 255, -1)
        hull = cv2.convexHull(lm.astype(np.float32)).astype(np.int32)
        core = np.zeros_like(region)
        cv2.fillConvexPoly(core, hull, 255)
        region[cv2.dilate(core, np.ones((3, 3), np.uint8), iterations=max(1, int(0.2 * fw))) > 0] = 0
    return region


def hair_mask(color, faces):
    """머리 영역 안에서, 어둡고 잔결이 있는 덩어리 (머리카락·모자). 흐린 배경은 결이 없어 빠짐."""
    if not faces:
        return np.zeros(color.shape[:2], np.uint8)
    value = cv2.cvtColor(cv2.GaussianBlur(color, (5, 5), 0), cv2.COLOR_BGR2HSV)[:, :, 2]
    g = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY).astype(np.float32), (0, 0), 0.8)
    texture = cv2.GaussianBlur(np.abs(cv2.Laplacian(g, cv2.CV_32F)), (0, 0), 5.0)
    mask = ((value < DARK_VALUE) & (texture > TEXTURE_MIN) & (head_region(color.shape, faces) > 0))
    mask = mask.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    keep = np.zeros_like(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= 0.01 * mask.size:
            keep[labels == i] = 255
    # 결이 안 보이는 아주 어두운 머리 부분(구멍)도 채움 — 비우면 그림에 빈 얼룩이 생김
    holes = cv2.bitwise_not(keep)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(holes, 4)
    border = set(np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]])))
    head = head_region(color.shape, faces) > 0
    for i in range(1, n):
        if i not in border and stats[i, cv2.CC_STAT_AREA] < 0.03 * keep.size:
            hole = labels == i
            if (value[hole] < DARK_VALUE).mean() > 0.8 and head[hole].mean() > 0.8:
                keep[hole] = 255
    return keep


def flow_field(gray, sigma=FLOW_SIGMA):
    """결 방향 (단위 벡터 tx, ty): 밝기 변화가 가장 큰 방향에 수직."""
    g = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), 1.0)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    jxx = cv2.GaussianBlur(gx * gx, (0, 0), sigma)
    jyy = cv2.GaussianBlur(gy * gy, (0, 0), sigma)
    jxy = cv2.GaussianBlur(gx * gy, (0, 0), sigma)
    theta = 0.5 * np.arctan2(2 * jxy, jxx - jyy) + np.pi / 2
    return np.cos(theta), np.sin(theta)


def _length(s):
    s = np.asarray(s, np.float64)
    return float(np.hypot(*np.diff(s, axis=0).T).sum()) if len(s) > 1 else 0.0


def _trace(seed, tx, ty, inside, taken, d_test, sign):
    h, w = inside.shape
    pts, prev = [], None
    x, y = seed
    for _ in range(int(MAX_LEN_PX)):
        xi, yi = int(round(x)), int(round(y))
        if not (0 <= xi < w and 0 <= yi < h) or not inside[yi, xi]:
            break
        if pts and taken[yi, xi] < d_test:
            break
        vx, vy = float(tx[yi, xi]), float(ty[yi, xi])
        if prev is None:
            vx, vy = vx * sign, vy * sign
        elif vx * prev[0] + vy * prev[1] < 0:
            vx, vy = -vx, -vy
        if prev is not None and vx * prev[0] + vy * prev[1] < 0.75:   # 41도 넘게 꺾이면 멈춤
            break
        pts.append((x, y))
        prev = (vx, vy)
        x, y = x + vx, y + vy
    return pts


def hair_strokes(color, mask, obstacles=(), spacing=SPACING_PX):
    """머리 마스크 안의 흐름선 획 목록. obstacles: 넘지 않을 구조선. 좌표는 작업 이미지 px (x, y)."""
    if not mask.any():
        return []
    tx, ty = flow_field(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
    inside = cv2.erode(mask, np.ones((5, 5), np.uint8)) > 0
    h, w = mask.shape
    occupied = np.full((h, w), 255, np.uint8)          # 0 = 이미 그린 선 위
    for s in obstacles:
        cv2.polylines(occupied, [np.round(np.asarray(s)).astype(np.int32).reshape(-1, 1, 2)], False, 0, 1)
    taken = cv2.distanceTransform(occupied, cv2.DIST_L2, 3)
    strokes = []
    step = max(2, int(spacing // 2))
    ys, xs = np.nonzero(inside[::step, ::step])
    for sx, sy in zip(xs * step, ys * step):
        if taken[sy, sx] < spacing:
            continue
        fwd = _trace((float(sx), float(sy)), tx, ty, inside, taken, 0.55 * spacing, 1)
        back = _trace((float(sx), float(sy)), tx, ty, inside, taken, 0.55 * spacing, -1)
        line = back[::-1] + fwd[1:]
        if len(line) < 2 or _length(line) < MIN_LEN_PX:
            continue
        arr = np.asarray(line, np.float64)
        strokes.append(arr)
        cv2.polylines(occupied, [np.round(arr).astype(np.int32).reshape(-1, 1, 2)], False, 0, 1)
        taken = cv2.distanceTransform(occupied, cv2.DIST_L2, 3)
    return strokes


def split_fragments(strokes, mask):
    """(남길 획, 구조선). 머리 덩어리 안에 대부분 들어간 짧은 조각은 버리고, 긴 선은 구조선으로 남김."""
    if not mask.any():
        return list(strokes), []
    inner = cv2.erode(mask, np.ones((7, 7), np.uint8)) > 0
    h, w = mask.shape
    kept, structure = [], []
    for s in strokes:
        s = np.asarray(s, np.float64)
        xi = np.clip(np.round(s[:, 0]).astype(int), 0, w - 1)
        yi = np.clip(np.round(s[:, 1]).astype(int), 0, h - 1)
        inside_share = float(inner[yi, xi].mean())
        if inside_share > 0.6 and _length(s) < STRUCTURE_PX:
            continue
        kept.append(s)
        if inside_share > 0.2:
            structure.append(s)
    return kept, structure
