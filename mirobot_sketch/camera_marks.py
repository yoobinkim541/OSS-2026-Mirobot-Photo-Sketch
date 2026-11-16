"""종이 모서리 표시(사람이 그린 X·십자)를 카메라 사진에서 찾습니다.

observe_setup(): [준비 확인]용. X 4개와 펜 끝이 보이는지만 보고 mm로 바꾸지 않습니다. 접촉 깊이도 판정하지 않습니다.
find_corner_marks(): 종이 위치 맞추기용. X·십자의 두 획을 직선으로 맞춘 교점(중심)을 사진 px로 돌려주고,
paper_corners_from_marks()가 그 네 점을 종이 mm 위치(가장자리에서 정해진 거리 안쪽)에 맞춰, 종이 위치 맞추기에서
찍는 네 귀퉁이와 같은 점을 계산합니다. 표시가 없으면 아무것도 바꾸지 않습니다(저장된 종이 위치를 그대로 씀).
"""

import itertools

import cv2
import numpy as np


def _cross_like(ink, box):
    """어두운 덩어리가 X자인지. 대각선에는 잉크가 있고 네 변의 가운데는 비어 있어야 합니다."""
    x, y, w, h = box
    if w < 12 or h < 12 or not (0.6 <= w / h <= 1.7):
        return False
    roi = ink[y:y + h, x:x + w]
    if roi.size == 0:
        return False
    fill = float(roi.mean())
    if not 0.06 <= fill <= 0.42:
        return False
    yy, xx = np.mgrid[0:h, 0:w]
    band = max(2, int(0.12 * min(w, h)))
    diag = (np.abs(xx / max(w - 1, 1) - yy / max(h - 1, 1)) * max(w, h) <= band) | (
        np.abs(xx / max(w - 1, 1) + yy / max(h - 1, 1) - 1) * max(w, h) <= band)
    # X의 끝은 귀퉁이에 닿으므로, 비어 있어야 하는 곳은 네 변의 가운데다.
    m = 0.35
    edges = ((yy < h * 0.18) & (xx > w * m) & (xx < w * (1 - m))) | (
        (yy > h * 0.82) & (xx > w * m) & (xx < w * (1 - m))) | (
        (xx < w * 0.18) & (yy > h * m) & (yy < h * (1 - m))) | (
        (xx > w * 0.82) & (yy > h * m) & (yy < h * (1 - m)))
    if edges.sum() == 0 or float(roi[diag].mean()) < 0.35 or float(roi[edges].mean()) > 0.15:
        return False
    return True


def _pen_like(ink, box):
    x, y, w, h = box
    if w < 4 or h < 4 or w > 28 or h > 28 or not (0.5 <= w / h <= 2.0):
        return False
    roi = ink[y:y + h, x:x + w]
    return float(roi.mean()) >= 0.45


def _stable_quad(boxes):
    if len(boxes) != 4:
        return False
    centers = np.array([[x + w / 2, y + h / 2] for x, y, w, h in boxes], np.float32)
    area = abs(cv2.contourArea(cv2.convexHull(centers)))
    if area < 80 * 80:
        return False
    sizes = np.array([w * h for *_, w, h in boxes], np.float32)
    return float(sizes.max() / max(sizes.min(), 1)) <= 2.5


def _quad_center(boxes):
    """사각형의 대각선 교점: 원근 투영된 종이의 중심."""
    centers = np.array([[x + (w - 1) / 2, y + (h - 1) / 2] for x, y, w, h in boxes], np.float32)
    hull = cv2.convexHull(centers).reshape(-1, 2)
    if len(hull) != 4:
        return None
    a, b, c, d = [np.array([*p, 1.0]) for p in hull]
    p = np.cross(np.cross(a, c), np.cross(b, d))
    if abs(p[2]) < 1e-8:
        return None
    return (float(p[0] / p[2]), float(p[1] / p[2]))


MARK_INSET_MM = 0.0              # 모서리 표시 중심이 종이 가장자리에서 안쪽으로 떨어진 거리 (가로·세로 같음). 귀퉁이에 붙여 그림
                                 # (실측 2026-11-12: 표시를 귀퉁이로 보고 잰 123.2 mm가 자로 잰 123 mm와 맞음 → 약 0~1.4 mm)
MARK_MIN_DARK = 14               # 표시 획에 이만큼 이상 어두운 점이 있어야 함 (로봇 펜 선은 5~12)
MARK_WEAK_DARK = 7               # 그 진한 점과 이어진 옅은 부분(연필 획 끝)까지는 이만큼만 어두워도 표시로 붙임
MARK_SIZE_FRAC = (0.006, 0.10)   # 표시 한 변의 크기: 사진 가로의 0.6%~10% (4K에서 약 23~380px, 멀리 있는 작은 십자 포함)
MARK_MIN_ANGLE = 50.0            # 두 획이 이 각도 이상 엇갈려야 X·십자


def _dark_map(frame_bgr):
    """종이보다 어두운 정도 (회색 단계). 조명이 고르지 않아도 되도록 넓은 중앙값을 종이 밝기로 봄."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    k = max(31, (min(gray.shape) // 20) | 1)
    bg = cv2.medianBlur(gray, min(k, 255))
    return np.clip(bg.astype(np.int16) - gray.astype(np.int16), 0, 255).astype(np.uint8)


def _fit_line(pts):
    """점들에 맞춘 직선 (점, 단위 방향)."""
    vx, vy, x0, y0 = cv2.fitLine(pts.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    return np.array([x0, y0]), np.array([vx, vy])


def _mark_center(mask):
    """덩어리 마스크(0/1)가 X·십자이면 두 획 직선의 교점 (x, y), 아니면 None.
    잉크 점의 방향을 두 무리로 나눠(가장 많이 겹치지 않는 두 각도) 각 무리에 직선을 맞춘다."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 30:
        return None
    h, w = mask.shape
    lines = cv2.HoughLinesP(mask.astype(np.uint8) * 255, 1, np.pi / 180, threshold=max(10, int(0.25 * max(h, w))),
                            minLineLength=int(0.45 * max(h, w)), maxLineGap=max(3, int(0.08 * max(h, w))))
    if lines is None or len(lines) < 2:
        return None
    segs = np.asarray(lines).reshape(-1, 4).astype(np.float64)       # OpenCV 4는 (N,1,4), 5는 (N,4)
    ang = np.degrees(np.arctan2(segs[:, 3] - segs[:, 1], segs[:, 2] - segs[:, 0])) % 180
    # 두 방향 고르기: 첫 방향 = 가장 긴 선분, 둘째 = 그와 가장 많이 엇갈린 선분
    lengths = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
    a0 = ang[int(np.argmax(lengths))]
    diff = np.abs((ang - a0 + 90) % 180 - 90)
    if diff.max() < MARK_MIN_ANGLE:
        return None
    a1 = ang[int(np.argmax(diff))]
    pts = np.c_[xs, ys].astype(np.float64)
    fits = []
    for a in (a0, a1):
        d = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        n = np.array([-d[1], d[0]])
        group = [s for s, sa in zip(segs, ang) if abs((sa - a + 90) % 180 - 90) < 20]
        # 그 방향 선분들 근처(굵기 안)의 잉크 점에 직선을 맞춤
        near = np.zeros(len(pts), bool)
        for x1, y1, x2, y2 in group:
            near |= np.abs((pts - [x1, y1]) @ n) <= max(3.0, 0.06 * max(h, w))
        if near.sum() < 10:
            return None
        fits.append(_fit_line(pts[near]))
    (p0, d0), (p1, d1) = fits
    m = np.array([d0, -d1]).T
    if abs(np.linalg.det(m)) < 0.3:
        return None
    t = np.linalg.solve(m, p1 - p0)
    c = p0 + t[0] * d0
    # 교점이 표시 안쪽(가운데 근처)이어야 함: ㄱ·L자 같은 꺾인 선은 교점이 끝에 생김
    if not (0.2 * w <= c[0] <= 0.8 * w and 0.2 * h <= c[1] <= 0.8 * h):
        return None
    # 두 획 모두 교점 양쪽으로 비슷하게 뻗고(T·ㅏ자 제외), 획 굵기가 길이에 비해 얇아야 함(펜·홀더 같은 굵은 물건 제외)
    band = max(3.0, 0.08 * max(h, w))
    lengths_ = []
    for d in (d0, d1):
        n = np.array([-d[1], d[0]])
        along = (pts - c) @ d
        on = np.abs((pts - c) @ n) <= band
        if on.sum() < 10:
            return None
        plus, minus = float(along[on].max()), float(-along[on].min())
        if min(plus, minus) < 0.4 * max(plus, minus):
            return None
        lengths_.append(plus + minus)
    if len(pts) / sum(lengths_) > 0.2 * max(lengths_):
        return None
    return float(c[0]), float(c[1])


def _order_corners(pts):
    """네 점을 왼쪽 위, 오른쪽 위, 오른쪽 아래, 왼쪽 아래 순서로.
    가로가 긴 종이의 위쪽 긴 변(평균 y가 더 작은 쪽)의 왼쪽 끝에서 시작한다.
    저장된 자리가 있으면 find_corner_marks가 그 순서를 그대로 쓴다."""
    pts = np.asarray(pts, np.float64)
    c = pts.mean(axis=0)
    order = np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
    ring = pts[order]
    signed = sum(float(ring[i, 0] * ring[(i + 1) % 4, 1] - ring[(i + 1) % 4, 0] * ring[i, 1]) for i in range(4))
    if signed < 0:
        ring = ring[::-1]
    lengths, means = [], []
    for i in range(4):
        a, b = ring[i], ring[(i + 1) % 4]
        lengths.append(float(np.linalg.norm(b - a)))
        means.append(float((a[1] + b[1]) / 2))
    pair = (0, 2) if lengths[0] + lengths[2] >= lengths[1] + lengths[3] else (1, 3)
    top = min(pair, key=lambda i: means[i])
    return np.roll(ring, -top, axis=0)


def _on_paper(gray, sat, labels, i, box):
    """표시 둘레가 종이인지: 표시보다 뚜렷이 밝고 색이 옅음. 보드 테두리·로봇·책상 물건을 뺌."""
    x, y, w, h = box
    pad = int(0.3 * max(w, h))
    H, W = gray.shape
    x0, y0, x1, y1 = max(0, x - pad), max(0, y - pad), min(W, x + w + pad), min(H, y + h + pad)
    local = labels[y0:y1, x0:x1]
    ring, mark = local != i, local == i
    if not ring.any() or not mark.any():
        return False
    gray_box, sat_box = gray[y0:y1, x0:x1], sat[y0:y1, x0:x1]
    return (float(np.median(gray_box[ring])) >= float(np.median(gray_box[mark])) + 24
            and float(np.median(sat_box[ring])) <= 60)


def _norm_cond(q, paper_mm):
    """점·종이 좌표를 같은 크기로 맞춘 호모그래피의 조건수. 찌그러진 사각형은 커진다."""
    from .vision import paper_corners_mm

    def nrm(p):
        p = np.asarray(p, np.float64)
        c = p.mean(axis=0)
        s = max(float(np.linalg.norm(p - c, axis=1).mean()), 1e-6)
        return (p - c) / s

    H = cv2.getPerspectiveTransform(nrm(q).astype(np.float32), nrm(paper_corners_mm(paper_mm)).astype(np.float32))
    if not np.isfinite(H).all():
        return float("inf")
    return float(np.linalg.cond(H))


def _shape_cost(q, paper_mm):
    """각이 90°에 가깝고 변 비율이 종이에 가까울수록 작다."""
    sides = [float(np.linalg.norm(q[(i + 1) % 4] - q[i])) for i in range(4)]
    ang_err = 0.0
    for i in range(4):
        a, b, c = q[i - 1], q[i], q[(i + 1) % 4]
        u, v = a - b, c - b
        ang = np.degrees(np.arccos(np.clip(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-9), -1, 1)))
        ang_err += abs(float(ang) - 90.0)
    aspect = float(paper_mm[0]) / max(float(paper_mm[1]), 1e-9)
    ratio = (sides[0] + sides[2]) / max(sides[1] + sides[3], 1e-9)
    return ang_err + 25.0 * abs(float(np.log((ratio + 1e-9) / aspect)))


def find_corner_marks(frame_bgr, expected_px=None, radius_px=None, paper_mm=None):
    """사진에서 모서리 표시 4개의 중심을 찾음. 반환: (4×2 배열 왼쪽 위부터 시계 방향, 찾은 후보 수) 또는 (None, 후보 수).
    expected_px: 저장된 종이 위치로 계산한 표시 자리(왼쪽 위부터 시계 방향 4점). 주면 각 자리에서 radius_px 안의
    가장 가까운 후보만 쓰고, 없으면 각·변 비율이 종이에 가장 가까운 볼록 사각형을 고름. 4개가 안 되면 None."""
    if paper_mm is None:
        from .paper_mapping import A4_LANDSCAPE_MM
        paper_mm = A4_LANDSCAPE_MM
    dark = _dark_map(frame_bgr)
    # 진한 점을 씨앗으로, 이어진 옅은 획까지 한 덩어리로 (연필처럼 옅게 그린 획이 끊겨 십자가 토막 나지 않게)
    weak = cv2.morphologyEx((dark >= MARK_WEAK_DARK).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _n, weak_lab = cv2.connectedComponents(weak, connectivity=8)
    seeds = np.unique(weak_lab[(dark >= MARK_MIN_DARK) & (weak > 0)])
    ink = np.isin(weak_lab, seeds[seeds > 0]).astype(np.uint8)
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    sat = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)[..., 1]
    H, W = ink.shape
    lo, hi = MARK_SIZE_FRAC[0] * W, MARK_SIZE_FRAC[1] * W
    n, labels, stats, _ = cv2.connectedComponentsWithStats(ink, 8)
    centers = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if not (lo <= max(w, h) <= hi and 0.4 <= w / max(h, 1) <= 2.5):
            continue
        if area / float(w * h) > 0.45:            # 꽉 찬 덩어리(펜 몸통·테두리 천)는 X가 아님
            continue
        if not _on_paper(gray, sat, labels, i, (x, y, w, h)):
            continue
        c = _mark_center(labels[y:y + h, x:x + w] == i)
        if c is not None:
            centers.append((x + c[0], y + c[1]))
    if expected_px is not None:
        if not centers:
            return None, 0
        pts = np.asarray(centers, np.float64)
        chosen = []
        for e in np.asarray(expected_px, np.float64):
            d = np.linalg.norm(pts - e, axis=1)
            k = int(np.argmin(d))
            if d[k] > radius_px:
                return None, len(centers)
            chosen.append(pts[k])
        q = np.asarray(chosen)
        if len({tuple(p) for p in q}) < 4 or not cv2.isContourConvex(q.astype(np.float32).reshape(-1, 1, 2)):
            return None, len(centers)
        return q, len(centers)
    if len(centers) < 4:
        return None, len(centers)
    best, best_cost = None, float("inf")
    min_side = 0.04 * min(W, H)
    for quad in itertools.combinations(centers, 4):
        q = _order_corners(quad).astype(np.float32)
        if not cv2.isContourConvex(q.reshape(-1, 1, 2)) or not _paper_like(q):
            continue
        q = q.astype(np.float64)
        sides = [float(np.linalg.norm(q[(i + 1) % 4] - q[i])) for i in range(4)]
        if min(sides) < min_side or _norm_cond(q, paper_mm) > 8.0:
            continue
        cost = _shape_cost(q, paper_mm)
        if cost < best_cost:
            best, best_cost = q, cost
    if best is None or abs(cv2.contourArea(best.astype(np.float32))) < 0.02 * W * H:
        return None, len(centers)
    return best, len(centers)


def _paper_like(q):
    """사전 정보 없이 고른 네 점이 종이 모서리처럼 놓였는지: 네 각이 50~130°, 가로변이 세로변의 0.7~3배.
    가운데 펜·홀더에서 잡힌 후보가 끼면 찌그러진 사각형이 됨."""
    for i in range(4):
        a, b, c = q[i - 1], q[i], q[(i + 1) % 4]
        u, v = a - b, c - b
        ang = np.degrees(np.arccos(np.clip(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-9), -1, 1)))
        if not 50 <= ang <= 130:
            return False
    side = [np.linalg.norm(q[(i + 1) % 4] - q[i]) for i in range(4)]
    ratio = (side[0] + side[2]) / max(side[1] + side[3], 1e-9)
    return 0.7 <= ratio <= 3.0


def mark_positions_mm(paper_mm, inset_mm=MARK_INSET_MM):
    """모서리 표시 중심의 종이 좌표 (왼쪽 위부터 시계 방향, 종이 중심 기준 mm, y 위)."""
    hw, hh = paper_mm[0] / 2 - inset_mm, paper_mm[1] / 2 - inset_mm
    return np.array([(-hw, hh), (hw, hh), (hw, -hh), (-hw, -hh)], np.float64)


def paper_corners_from_marks(marks_px, paper_mm, inset_mm=MARK_INSET_MM):
    """표시 네 중심(사진 px)으로 종이 네 귀퉁이가 사진의 어디에 오는지 (종이 위치 맞추기에 찍는 점과 같은 뜻)."""
    from .vision import paper_corners_mm, project
    H = cv2.getPerspectiveTransform(mark_positions_mm(paper_mm, inset_mm).astype(np.float32),
                                    np.asarray(marks_px, np.float32))
    return project(H, paper_corners_mm(paper_mm))


def registration_from_marks(camera, marks_px, frame_size, paper_mm, inset_mm=MARK_INSET_MM):
    """표시 네 중심으로 만든 종이 위치 등록 (손으로 귀퉁이를 찍은 것과 같은 Registration)."""
    from .vision import Registration
    return Registration.from_pixels(camera, paper_corners_from_marks(marks_px, paper_mm, inset_mm), frame_size, paper_mm)


def paper_frame_mm(registration, marks_px, frame_size, paper_mm, inset_mm=MARK_INSET_MM):
    """로봇 기준 등록(카메라 px ↔ 로봇이 그리는 종이 mm)으로 본 실제 종이의 자리와 크기.
    표시 네 중심(왼쪽 위부터 시계 방향)을 등록의 mm로 옮겨, 종이 중심(네 점 평균), 가로 방향(위·아래 변 평균)의 각도,
    가로·세로 길이를 잰다. 종이 크기가 달라도 표시는 가장자리에서 inset_mm 안쪽에 그린다고 보고 실제 종이 크기를 계산.
    반환: {"cx", "cy"(로봇 종이 중심 기준 mm), "deg"(시계 반대 +), "width_mm", "height_mm",
    "paper"(맞는 종이 (이름, (가로, 세로)) 또는 None), "scale"(그림 배율: 맞는 종이가 있으면 그 종이와 설정한 종이의
    가로·세로 비율 중 작은 쪽, 없으면 1 — 잰 크기를 그대로 배율로 쓰면 등록 배율이 틀렸을 때 로봇이 엉뚱한 크기로 그림)}."""
    from .vision import project
    m = project(np.linalg.inv(registration.matrix(frame_size)), np.asarray(marks_px, np.float64))
    tl, tr, br, bl = m
    top, bottom = tr - tl, br - bl
    across = (top + bottom) / 2
    width = (np.linalg.norm(top) + np.linalg.norm(bottom)) / 2 + 2 * inset_mm
    height = (np.linalg.norm(tl - bl) + np.linalg.norm(tr - br)) / 2 + 2 * inset_mm
    c = m.mean(axis=0)
    paper = known_paper(width, height, paper_mm)
    scale = 1.0 if paper is None else min(paper[1][0] / paper_mm[0], paper[1][1] / paper_mm[1])
    return {"cx": float(c[0]), "cy": float(c[1]), "deg": float(np.degrees(np.arctan2(across[1], across[0]))),
            "width_mm": float(width), "height_mm": float(height), "paper": paper, "scale": float(scale)}


KNOWN_PAPERS = {"A3": (420.0, 297.0), "A4": (297.0, 210.0), "A5": (210.0, 148.0), "B5": (257.0, 182.0),
                "Letter": (279.4, 215.9)}
PAPER_SIZE_TOL = 0.03            # 잰 종이 크기가 아는 크기와 가로·세로 모두 3% 안이면 그 종이로 봄


def known_paper(width_mm, height_mm, paper_mm):
    """잰 종이(가로로 놓인 것)가 설정한 종이나 아는 종이 크기와 맞으면 (이름, (가로, 세로)), 아니면 None.
    맞지 않는 크기는 종이가 특이한 게 아니라 저장된 종이 위치(등록) 배율이 틀렸을 가능성이 커서 배율에 쓰지 않는다."""
    candidates = [("설정한 종이", tuple(float(v) for v in paper_mm))] + list(KNOWN_PAPERS.items())
    for name, (w, h) in candidates:
        if abs(width_mm / w - 1) <= PAPER_SIZE_TOL and abs(height_mm / h - 1) <= PAPER_SIZE_TOL:
            return name, (w, h)
    return None


def observe_setup(frame_bgr):
    """프레임에서 X 4개와 펜 끝 후보를 봅니다. 접촉·이동량은 반환하지 않습니다."""
    dark = _dark_map(frame_bgr)
    ink = dark >= 30                 # 종이보다 이만큼 어두운 점. 절대 회색 90은 어두운 종이와 연필을 놓침
    n, labels, stats, _ = cv2.connectedComponentsWithStats(ink.astype(np.uint8), 8)
    crosses, pens = [], []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < 20:
            continue
        box = (int(x), int(y), int(w), int(h))
        if _cross_like(ink, box):
            crosses.append(box)
        elif _pen_like(ink, box):
            pens.append(box)
    stable = _stable_quad(crosses)
    return {
        "crosses": len(crosses),
        "crosses_stable": stable,
        "pen_tip_seen": len(pens) == 1,
        "paper_center_px": _quad_center(crosses) if stable else None,
        "pen_tip_px": (pens[0][0] + (pens[0][2] - 1) / 2, pens[0][1] + (pens[0][3] - 1) / 2) if len(pens) == 1 else None,
        "contact_checked": False,
        "center_moved": False,
    }


def attach_camera_check(guide, port_choice, observation):
    """robot_guide 결과에 USB·카메라 관찰만 얹습니다. guide의 ready를 카메라로 바꾸지 않습니다."""
    out = dict(guide)
    out["usb"] = {"port": port_choice.port, "source": port_choice.source}
    out["camera"] = dict(observation)
    out["contact_checked"] = False
    out["center_moved"] = False
    notes = list(out.get("notes") or [])
    if port_choice.port is None:
        from . import robot_status
        notes.append(robot_status.port_choice_problem(port_choice) + ". 포트를 열지 않았습니다.")
    if not observation["crosses_stable"]:
        notes.append("종이 모서리 X 4개가 안정적으로 보이지 않습니다. 종이 위치는 옮기지 않습니다.")
    elif not observation["pen_tip_seen"]:
        notes.append("펜 끝을 하나로 구분하지 못했습니다. 접촉은 확인하지 않습니다.")
    else:
        notes.append("X 4개와 펜 끝이 화면에 보입니다. 접촉 깊이와 종이 기울기는 확인하지 않았습니다.")
    out["notes"] = notes
    return out


def ready_button_visible(camera_index, frame):
    """웹캠이 프레임을 줄 때만 준비 확인 버튼을 보입니다."""
    return camera_index is not None and frame is not None


def ready_text(report):
    """준비 확인 버튼이 보여줄 짧은 문구."""
    cam = report.get("camera") or {}
    usb = report.get("usb") or {}
    lines = []
    if report.get("ready"):
        lines.append("그림·시뮬레이션: 확인됨")
    else:
        issues = report.get("issues") or []
        lines.append(issues[0] if issues else report.get("next", "그림 준비가 되지 않았습니다."))
    port = usb.get("port")
    lines.append(f"USB: {port}" if port else "USB: 로봇 포트 없음")
    if cam.get("crosses_stable") and cam.get("pen_tip_seen"):
        lines.append("카메라: X 4개·펜 끝 보임")
    else:
        lines.append(f"카메라: X {cam.get('crosses', 0)}개, 펜 끝 {'보임' if cam.get('pen_tip_seen') else '구분 못 함'}")
    lines.append("접촉은 확인하지 않음")
    return "\n".join(lines)
