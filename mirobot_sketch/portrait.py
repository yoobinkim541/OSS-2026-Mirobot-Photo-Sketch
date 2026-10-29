"""실사 인물 선화를 부위별로 다시 그립니다. 화가가 인물을 그리는 방식을 따릅니다.

- 모자: 바깥 윤곽 + 니트 결(흐름선) + 결을 가로지르는 접힘선 + 밝은 자수 무늬의 윤곽
- 머리카락(앞머리 포함): 결 방향 흐름선 (짧은 조각은 버림)
- 눈: 윗눈꺼풀(진하게) + 아랫눈꺼풀 가운데 + 눈꺼풀 안에 보이는 눈동자 호
- 눈썹: 가운데 한 줄 (앞머리에 가려 두껍게 잡힌 것은 앞머리로 봄)
- 입술: 앞쪽 실루엣 + 윗입술 윗선 + 입술 사이 선
- 귀: 피부·배경과 맞닿은 테두리 (모자·머리카락에 가린 쪽은 긋지 않음), 귀걸이: 작은 고리
- 옷(레이스 등): 바깥 윤곽 + 긴 선만, 잔무늬는 버림
- 피부: 윤곽만, 볼·턱의 조명 경계선과 작은 그림자 고리는 버림
- 그 밖(마이크·손): 선화 그대로, 점선처럼 끊긴 곳은 방향이 맞으면 이어 붙임

부위는 parsing.parse (얼굴 분할 모델), 결 방향은 hair.flow_field, 선은 sketch_pipeline으로 만듭니다.
"""

import cv2
import numpy as np

from . import hair, parsing
from . import sketch_pipeline as sp

SHADING_AB = 8.0        # 선 양쪽 색(Lab a·b) 차이가 이보다 작고 양쪽 다 피부면 조명 경계
CLOTH_MIN_PX = 45.0     # 옷 무늬에서 남길 최소 길이
CROSS_GRAIN_PX = 30.0   # 결 덩어리 안에서 결을 가로지르는 선은 이 길이 이상이면 구조선(접힘)으로 남김
HAT_SPACING_PX = 11.0
HAIR_SPACING_PX = 9.0
GAP_PX = 10.0           # 끊긴 선 이어 붙이기: 끝점 거리
GAP_TURN = 0.85         # 끊긴 선 이어 붙이기: 방향 코사인 (약 30도 이내)


# ---------------------------------------------------------------- 공통 도우미
def _at(img, pts):
    h, w = img.shape[:2]
    xi = np.clip(np.round(pts[:, 0]).astype(int), 0, w - 1)
    yi = np.clip(np.round(pts[:, 1]).astype(int), 0, h - 1)
    return img[yi, xi]


def _share(stroke, mask):
    return float((_at(mask, np.asarray(stroke, np.float64)) > 0).mean())


def _runs(points, good, min_len):
    """good이 참인 연속 구간만 잘라 획으로."""
    out, run = [], []
    for p, ok in zip(points, good):
        if ok:
            run.append(p)
            continue
        if len(run) >= min_len:
            out.append(np.asarray(run))
        run = []
    if len(run) >= min_len:
        out.append(np.asarray(run))
    return out


def _contours(mask, min_area=12.0, sigma=2.5, border=2, skip=None):
    """마스크 바깥 윤곽을 매끄러운 획으로. 사진 가장자리(잘린 곳)와 skip 영역에 닿은 부분은 긋지 않음."""
    h, w = mask.shape[:2]
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in cs:
        if cv2.contourArea(c) < min_area or len(c) < 8:
            continue
        pts = c[:, 0, :].astype(np.float64)
        edge = ((pts[:, 0] <= border) | (pts[:, 1] <= border)
                | (pts[:, 0] >= w - 1 - border) | (pts[:, 1] >= h - 1 - border))
        if skip is not None:
            edge |= _at(skip, pts) > 0
        if not edge.any():
            out.append(np.vstack([pts, pts[:1]]))
            continue
        start = int(np.argmax(edge))                   # 가장자리 점에서 시작하게 돌려 안쪽 구간만 나눔
        out += _runs(np.roll(pts, -start, 0), ~np.roll(edge, -start), 8)
    return sp.smooth_strokes(out, sigma)


def _edge_against(mask, labels, neighbor_ids, min_len=6, sigma=2.0):
    """마스크 윤곽 중 바깥쪽이 neighbor_ids 부위와 맞닿은 구간만 (예: 귀 중 모자에 가린 쪽은 빼고 피부·배경 쪽만)."""
    ring = cv2.dilate(mask, np.ones((5, 5), np.uint8)) & ~mask
    near = (np.isin(labels, list(neighbor_ids)) & (ring > 0)).astype(np.uint8) * 255
    near = cv2.dilate(near, np.ones((5, 5), np.uint8))
    out = []
    for c in _contours(mask, min_area=10, sigma=0):
        out += _runs(c, _at(near, c) > 0, min_len)
    return sp.smooth_strokes(out, sigma)


def _profile(mask, top=True, keep=1.0):
    """마스크의 위(또는 아래) 가장자리를 왼쪽에서 오른쪽으로 잇는 열린 곡선. keep<1이면 가운데 부분만."""
    cols = np.nonzero(mask.any(axis=0))[0]
    if len(cols) < 3:
        return None
    y = np.array([(np.nonzero(mask[:, x])[0].min() if top else np.nonzero(mask[:, x])[0].max()) for x in cols],
                 np.float64)
    pts = np.stack([cols.astype(np.float64), y], 1)
    if keep < 1.0:
        cut = int(round(len(pts) * (1 - keep) / 2))
        pts = pts[cut:len(pts) - cut]
    return pts if len(pts) >= 3 else None


def _centerline(mask, min_len=6, sigma=1.5):
    """가는 마스크(눈썹·입술 사이)의 가운데 한 줄."""
    if not mask.any():
        return []
    return sp.smooth_strokes(sp.merge_strokes(sp.trace_strokes(mask, min_len, 3), 3.0), sigma)


def _circle(center, radius, n=24):
    t = np.linspace(0, 2 * np.pi, n + 1)
    return np.stack([center[0] + radius * np.cos(t), center[1] + radius * np.sin(t)], 1)


def _tangent(s, at_start, k=5):
    s = np.asarray(s, np.float64)
    m = min(k, len(s) - 1)
    v = (s[m] - s[0]) if at_start else (s[-1] - s[-1 - m])
    n = np.hypot(*v)
    return v / n if n > 0 else np.zeros(2)


# ---------------------------------------------------------------- 선 정리
def join_gaps(strokes, gap=GAP_PX, turn=GAP_TURN):
    """점선처럼 끊긴 선을 잇기: 끝점이 gap 이내이고 진행 방향이 거의 같으면 한 획으로."""
    from scipy.spatial import cKDTree

    strokes = [np.asarray(s, np.float64) for s in strokes if len(s) >= 2]
    alive = [True] * len(strokes)
    while True:
        ends, owners = [], []
        for i, s in enumerate(strokes):
            if alive[i] and not sp.is_closed(s):
                ends += [s[0], s[-1]]
                owners += [(i, True), (i, False)]
        if len(ends) < 2:
            break
        best = None
        for a, b in cKDTree(np.asarray(ends)).query_pairs(gap):
            (i, ia), (j, jb) = owners[a], owners[b]
            if i == j:
                continue
            out_i = -_tangent(strokes[i], True) if ia else _tangent(strokes[i], False)   # i에서 나가는 방향
            into_j = _tangent(strokes[j], True) if jb else -_tangent(strokes[j], False)  # j로 들어가는 방향
            bridge = np.asarray(ends[b]) - np.asarray(ends[a])
            d = float(np.hypot(*bridge))
            ok = out_i @ into_j > turn and (d < 1.5 or (bridge / d) @ out_i > turn)
            if ok and (best is None or d < best[0]):
                best = (d, i, ia, j, jb)
        if best is None:
            break
        _, i, ia, j, jb = best
        si = strokes[i][::-1] if ia else strokes[i]
        sj = strokes[j] if jb else strokes[j][::-1]
        strokes[i] = np.vstack([si, sj])
        alive[j] = False
    return [s for s, a in zip(strokes, alive) if a]


def is_skin_shading(s, lab, ycc):
    """선 양쪽이 모두 피부색이고 색은 같고 밝기만 다르면 (턱·볼의 조명 경계) True. 화가는 이런 선을 긋지 않음."""
    if len(s) < 3:
        return False
    d = np.gradient(s, axis=0)
    n = np.stack([-d[:, 1], d[:, 0]], 1)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
    a, b = _at(lab, s + 4 * n), _at(lab, s - 4 * n)
    ya, yb = _at(ycc, s + 4 * n), _at(ycc, s - 4 * n)
    skin = lambda y: ((y[:, 1] > 135) & (y[:, 1] < 185) & (y[:, 2] > 100) & (y[:, 2] < 135)).mean() > 0.8
    return skin(ya) and skin(yb) and float(np.linalg.norm(a[:, 1:] - b[:, 1:], axis=1).mean()) < SHADING_AB


def _in_cheek_jaw(s, face):
    """획 대부분이 얼굴 테두리 타원 안이면서 눈보다 아래(볼·턱)인지."""
    from . import faces as fc
    (cx, cy), (ax, ay) = fc.ellipse_of(face)
    inside = ((s[:, 0] - cx) / ax) ** 2 + ((s[:, 1] - cy) / ay) ** 2 <= 1.0
    eye_y = float(np.asarray(face.landmarks)[:2, 1].max())
    return inside.mean() > 0.8 and float(np.median(s[:, 1])) > eye_y


def tidy_face(strokes, faces, color):
    """코 안의 잔선, 입가의 짧은 잔선, 피부 위 조명 경계선을 지움. 윤곽과 머리카락은 남김."""
    if not faces:
        return list(strokes)
    blurred = cv2.GaussianBlur(color, (0, 0), 1.2)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB).astype(np.float32)
    ycc = cv2.cvtColor(blurred, cv2.COLOR_BGR2YCrCb).astype(np.float32)
    kept = []
    for raw in strokes:
        s = np.asarray(raw, np.float64)
        if len(s) < 2:
            continue
        length = sp.polyline_length(s)
        drop = False
        for face in faces:
            width = float(face.box[2])
            nose, mouth_r, mouth_l = face.landmarks[2], face.landmarks[3], face.landmarks[4]
            mouth = (mouth_r + mouth_l) / 2
            nose_d = np.linalg.norm(s - nose, axis=1)
            if np.median(nose_d) < 0.14 * width and np.max(nose_d) < 0.32 * width:
                drop = True
            mouth_d = np.linalg.norm(s - mouth, axis=1)
            if np.median(mouth_d) < 0.12 * width and length < 0.45 * width:
                reaches_lip = (np.min(np.linalg.norm(s - mouth_r, axis=1)) < 0.1 * width
                               and np.min(np.linalg.norm(s - mouth_l, axis=1)) < 0.1 * width)
                drop = drop or not reaches_lip
            if _in_cheek_jaw(s, face) and is_skin_shading(s, lab, ycc):
                drop = True                                 # 볼·턱 조명 경계 (얼굴 앞의 손가락 선은 남김)
        if not drop:
            kept.append(s)
    return kept


def _along_grain(s, tx, ty):
    """획이 결 방향을 따라가는 정도 (0~1, 1 = 결과 나란함)."""
    s = np.asarray(s, np.float64)
    if len(s) < 3:
        return 1.0
    d = np.gradient(s, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-9
    return float(np.abs(d[:, 0] * _at(tx, s) + d[:, 1] * _at(ty, s)).mean())


# ---------------------------------------------------------------- 부위별 그리기
def _embroidery(color, hat):
    """(모자 안의 밝고 색이 옅은 무늬(자수·로고) 영역, 무늬 윤곽 획). 배경 제거로 생긴 흰 바탕과 모자 가장자리는 뺌."""
    if not hat.any():
        return np.zeros_like(hat), []
    w = hat.shape[1]
    inner = cv2.erode(hat, np.ones((5, 5), np.uint8))
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    paper = (hsv[:, :, 2] >= 248) & (hsv[:, :, 1] <= 8)        # 배경 제거 뒤 흰 바탕
    bright = ((hsv[:, :, 2] > 140) & (hsv[:, :, 1] < 110) & ~paper & (inner > 0)).astype(np.uint8) * 255
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(bright, 8)
    keep = np.zeros_like(bright)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= 25 and stats[i, cv2.CC_STAT_WIDTH] < 0.3 * w:
            keep[labels == i] = 255
    return cv2.dilate(keep, np.ones((7, 7), np.uint8)), _contours(keep, min_area=10, sigma=1.0)


def _eyes(eye_mask, gray):
    """눈마다: 윗눈꺼풀(두 번 그어 진하게, 눈꼬리로 조금 길게) + 아랫눈꺼풀 가운데 + 눈꺼풀 안에 보이는 눈동자 호."""
    out = []
    n, labels = cv2.connectedComponents(eye_mask, connectivity=8)
    for i in range(1, n):
        m = (labels == i).astype(np.uint8) * 255
        if cv2.countNonZero(m) < 8:
            continue
        upper, lower = _profile(m, top=True), _profile(m, top=False, keep=0.6)
        if upper is not None:
            reach = 0.15 * (upper[-1, 0] - upper[0, 0])
            out.append(np.vstack([upper, upper[-1] + np.array([reach, -0.3 * reach])]))
            out.append(upper + np.array([0.0, 1.2]))
        if lower is not None:
            out.append(lower)
        vals = gray[m > 0]
        pts = cv2.findNonZero(((gray <= np.percentile(vals, 35)) & (m > 0)).astype(np.uint8))
        if pts is None or len(pts) < 4:
            continue
        (cx, cy), r = cv2.minEnclosingCircle(pts)
        ys, _ = np.nonzero(m)
        ring = _circle((cx, cy), max(1.5, min(r, 0.6 * (ys.max() - ys.min() + 1))), 32)
        out += _runs(ring, _at(cv2.dilate(m, np.ones((3, 3), np.uint8)), ring) > 0, 3)   # 눈꺼풀에 가린 부분은 빼고
    return sp.smooth_strokes(out, 1.0)


def _brows(brow_mask, face_w):
    """(눈썹 획, 앞머리로 볼 영역). 두꺼운 '눈썹'은 앞머리를 눈썹으로 잘못 본 것."""
    bangs = np.zeros_like(brow_mask)
    thin = np.zeros_like(brow_mask)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(brow_mask, 8)
    for i in range(1, n):
        length = max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
        target = bangs if stats[i, cv2.CC_STAT_AREA] / max(length, 1) > 0.07 * face_w else thin
        target[labels == i] = 255
    return _centerline(cv2.dilate(thin, np.ones((2, 2), np.uint8)), min_len=6), bangs


LIP_REDNESS = 8.0      # 입술은 피부보다 붉음(YCrCb Cr). 손가락 등 다른 것을 입술로 잘못 본 영역은 이보다 덜 붉음


def _lips(labels, color):
    """윗입술과 아랫입술을 각각 도톰한 모양으로 (입 안·이는 긋지 않음). 하나로 이으면 부리처럼 보임.
    피부보다 붉지 않은 '입술'(입을 가린 손가락 등)은 긋지 않음."""
    cr = cv2.cvtColor(color, cv2.COLOR_BGR2YCrCb)[:, :, 1].astype(np.float32)
    skin = parsing.mask(labels, "skin") > 0
    skin_cr = float(np.median(cr[skin])) if skin.any() else 0.0
    out, kept = [], np.zeros(labels.shape, np.uint8)
    for part in ("u_lip", "l_lip"):
        m = cv2.morphologyEx((labels == parsing.PARTS.index(part)).astype(np.uint8) * 255, cv2.MORPH_CLOSE,
                             np.ones((3, 3), np.uint8))
        n, comp = cv2.connectedComponents(m, connectivity=8)
        for i in range(1, n):
            piece = comp == i
            if piece.sum() >= 15 and float(cr[piece].mean()) >= skin_cr + LIP_REDNESS:
                out += _contours(piece.astype(np.uint8) * 255, min_area=15, sigma=1.5)
                kept[piece] = 255
    return out, kept


def _earring(mask):
    pts = cv2.findNonZero(mask)
    if pts is None or len(pts) < 3:
        return []
    (cx, cy), r = cv2.minEnclosingCircle(pts)
    return [_circle((cx, cy), max(2.0, r))]


def compose(strokes, color, faces):
    """선화 획 + 원본 사진 + 얼굴 -> 부위별로 다시 그린 획. 얼굴이나 분할 모델이 없으면 None (기존 방식 사용)."""
    if not faces or not parsing.available():
        return None
    labels = parsing.parse(color, faces)
    m = {g: parsing.mask(labels, g) for g in parsing.GROUPS}
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    tx, ty = hair.flow_field(gray)
    k3 = np.ones((3, 3), np.uint8)
    face_w = float(max(f.box[2] for f in faces))
    brows, bangs = _brows(m["brow"], face_w)
    m["hair"] = m["hair"] | bangs

    embroidery, motif = _embroidery(color, m["hat"])
    lips, lip_area = _lips(labels, color)
    features = cv2.dilate(m["eye"] | (m["brow"] & ~bangs) | lip_area | m["earring"], np.ones((7, 7), np.uint8))
    hat_body = cv2.erode(m["hat"], k3) & ~embroidery
    hair_body = cv2.erode(m["hair"], k3) & ~features
    textured = hat_body | hair_body
    cloth = cv2.erode(m["cloth"], k3)
    skin = m["skin"]

    kept, structure = [], []
    for s in tidy_face(strokes, faces, color):
        if _share(s, features) > 0.4 or _share(s, embroidery) > 0.5:
            continue                                        # 눈·입·자수는 아래에서 새로 그림
        length = sp.polyline_length(s)
        loop = sp.is_closed(s) or float(np.hypot(*(s[0] - s[-1]))) < 0.25 * length
        if loop and length < 0.4 * face_w and _share(s, skin) > 0.6:
            continue                                        # 피부 위 작은 그림자 고리 (거의 닫힌 것 포함)
        if _share(s, textured) > 0.5:
            if length >= CROSS_GRAIN_PX and _along_grain(s, tx, ty) < 0.55:
                kept.append(s)                              # 결을 가로지르는 접힘·경계선
                structure.append(s)
            continue                                        # 결 조각은 흐름선이 대신함
        if _share(s, cloth) > 0.5 and length < CLOTH_MIN_PX:
            continue                                        # 레이스 잔무늬
        kept.append(s)

    unseen = cv2.dilate(255 - parsing.coverage(color.shape, faces), np.ones((7, 7), np.uint8))
    hat_outline = _contours(m["hat"], min_area=200, sigma=3.0, skip=unseen)
    walls = structure + hat_outline + motif
    drawn = kept + hat_outline + motif + _contours(m["cloth"], min_area=300, sigma=3.0, skip=unseen)
    drawn += sp.smooth_strokes(hair.hair_strokes(color, hat_body, walls, HAT_SPACING_PX), 2.0)
    drawn += sp.smooth_strokes(hair.hair_strokes(color, hair_body, walls, HAIR_SPACING_PX), 2.0)
    drawn += _eyes(m["eye"], gray) + brows + lips
    ear_edge_ids = [0] + [parsing.PARTS.index(n) for n in ("hair", "cloth", "neck")]
    drawn += _edge_against(m["ear"], labels, ear_edge_ids, min_len=8, sigma=2.5)
    drawn += _earring(m["earring"])
    return join_gaps(drawn)
