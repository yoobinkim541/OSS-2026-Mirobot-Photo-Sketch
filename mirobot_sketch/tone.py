"""
명암(톤) 빗금과 사진-그림 비교
==============================
윤곽선만으로는 어두운 머리카락·옷·그림자가 비어 보여 사진과 인상이 달라집니다. 펜 한 자루로 명암을 내는
가장 확실한 방법은 빗금(해칭)입니다: 어두운 면을 평행선으로 채우고, 더 어두우면 방향을 바꿔 한 번 더 긋습니다.

- hatch_strokes: 사진의 어두운 정도로 빗금 획을 만듭니다. 이어지는 빗금 끝은 펜을 떼지 않고 지그재그로 이어
  획 수(= 펜 올림 횟수, 시간)를 줄입니다.
- compare_report: 사진과 그려질 그림이 얼마나 닮았는지 (명암 분포·윤곽 재현·잡음)를 숫자와 나란히 보기 그림으로.

모든 좌표는 다른 획과 같은 작업 이미지 픽셀(x, y)입니다.
"""

import hashlib
import math

import cv2
import numpy as np

# 단계 수별 어두운 정도 기준 (0~1). 앞 단계가 가장 밝은 기준이라 넓은 면을 먼저 채움
LEVEL_THRESHOLDS = {1: (0.55,), 2: (0.45, 0.72), 3: (0.38, 0.58, 0.78)}
# 단계마다 같은 방향 평행선을 사이사이에 더 넣어 진하게 함 (간격 대비 위치). 교차 빗금은 0.5mm 펜으로 그리면 머리카락이
# 바구니 같은 격자로 보여(사용자 사진) 쓰지 않는다: 2단계는 그 사이 가운데, 3단계는 다시 그 사이.
LEVEL_PHASES = ((0.0,), (0.5,), (0.25, 0.75))
MAX_COVERAGE = 0.55                    # 첫 단계가 화면의 이만큼 넘게 덮이면 빗금을 만들지 않음 (배경이 어두운 사진)
BLANK_NOTE = "배경이 어두워 빗금이 화면 대부분을 덮습니다. 배경 제거를 켜 보세요."


def darkness_map(gray, sigma_px=0.0):
    """0(밝음)~1(어두움). 흰 배경(배경 제거 결과)은 0으로 두고, 나머지 밝기 분포를 2~98 백분위로 늘림."""
    g = gray.astype(np.float32)
    content = g < 250
    if int(content.sum()) < 100:
        return np.zeros(g.shape, np.float32)
    lo, hi = np.percentile(g[content], (2, 98))
    span = float(hi - lo)
    # 평평한 면은 상대 대비가 0이다. 범위가 좁고 절대적으로 어두우면 그 면을 어두운 것으로 본다.
    if span < 20.0 and hi <= 80.0:
        d = np.zeros(g.shape, np.float32)
        d[content] = 1.0
    else:
        d = np.clip((hi - g) / max(span, 20.0), 0.0, 1.0)
        d[~content] = 0.0
    if sigma_px > 0.5:
        d = cv2.GaussianBlur(d, (0, 0), sigma_px)
    return d


def _clean_mask(mask, spacing_px):
    """작은 얼룩을 없애고 틈을 메운 빗금 영역."""
    k = max(3, int(round(spacing_px * 0.8)) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    m = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    keep = np.zeros(m.shape, np.uint8)
    min_area = (3.0 * spacing_px) ** 2
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            keep[labels == i] = 1
    return keep


def _runs_on_lines(mask, angle_deg, spacing_px, min_len_px, phase=0.0):
    """각도 angle_deg의 평행선을 spacing_px 간격으로 (phase × 간격만큼 비켜) 긋고, 마스크 안의 구간을
    [(줄 번호, 시작점, 끝점)]으로."""
    h, w = mask.shape
    th = math.radians(angle_deg)
    u = np.array([math.cos(th), math.sin(th)])
    nrm = np.array([-math.sin(th), math.cos(th)])
    center = np.array([w / 2.0, h / 2.0])
    radius = math.hypot(w, h) / 2.0
    s = np.arange(-radius, radius + 1.0, 1.0)
    runs = []
    n_lines = int(math.floor(radius / spacing_px))
    for li, k in enumerate(range(-n_lines, n_lines + 1)):
        base = center + (k + phase) * spacing_px * nrm
        xs = base[0] + s * u[0]
        ys = base[1] + s * u[1]
        inside = (xs >= 0) & (xs <= w - 1) & (ys >= 0) & (ys <= h - 1)
        if not inside.any():
            continue
        on = np.zeros(len(s), bool)
        on[inside] = mask[np.round(ys[inside]).astype(int), np.round(xs[inside]).astype(int)] > 0
        edge = np.diff(np.concatenate([[0], on.astype(np.int8), [0]]))
        starts, ends = np.nonzero(edge == 1)[0], np.nonzero(edge == -1)[0] - 1
        for a, b in zip(starts, ends):
            if s[b] - s[a] >= min_len_px:
                runs.append((li, base + s[a] * u, base + s[b] * u))
    return runs


def _chain_runs(runs, mask, join_px):
    """이웃한 줄의 구간 끝을 이어 지그재그 획으로 만듦 (펜을 떼는 횟수를 줄임). 이음 구간의 가운데가 영역 안일 때만."""
    h, w = mask.shape

    def inside(p):
        x, y = int(round(p[0])), int(round(p[1]))
        return 0 <= x < w and 0 <= y < h and mask[y, x] > 0

    by_line = {}
    for li, p0, p1 in runs:
        by_line.setdefault(li, []).append((p0, p1))
    chains, open_prev, prev_li = [], [], None
    for li in sorted(by_line):
        if prev_li is None or li != prev_li + 1:
            open_prev = []                         # 줄이 비어 있으면 이어지지 않음
        open_now, taken = [], set()
        for p0, p1 in by_line[li]:
            best = None
            for ci, tail in open_prev:
                if ci in taken:
                    continue
                d0, d1 = float(np.linalg.norm(tail - p0)), float(np.linalg.norm(tail - p1))
                d = min(d0, d1)
                near, far = (p0, p1) if d0 <= d1 else (p1, p0)
                if d <= join_px and inside((tail + near) / 2.0) and (best is None or d < best[0]):
                    best = (d, ci, near, far)
            if best is not None:
                _, ci, near, far = best
                chains[ci].extend([near, far])
                taken.add(ci)
                open_now.append((ci, far))
            else:
                chains.append([p0, p1])
                open_now.append((len(chains) - 1, p1))
        open_prev, prev_li = open_now, li
    return [np.array(c, dtype=np.float64) for c in chains]


def hatch_strokes(gray, spacing_px, levels, angle_deg=45.0, min_len_px=8.0, bias=0.0, keep_out=None):
    """어두운 면의 빗금 획들. 반환: (획 리스트, 안내 문구). levels <= 0이면 ([], "").
    spacing_px: 빗금 간격, min_len_px: 이보다 짧은 빗금은 버림, bias: +면 기준이 올라가 빗금이 줄고 -면 늘어남.
    keep_out: True인 곳(얼굴 피부 등)에는 빗금을 긋지 않는 마스크."""
    levels = int(min(max(levels, 0), 3))
    if levels == 0:
        return [], ""
    spacing_px = max(float(spacing_px), 2.0)
    d = darkness_map(gray, sigma_px=spacing_px)
    strokes = []
    for level, thr in enumerate(LEVEL_THRESHOLDS[levels]):
        mask = _clean_mask((d > min(0.95, max(0.05, thr + bias))).astype(np.uint8), spacing_px)
        if level == 0 and mask.mean() > MAX_COVERAGE:
            return [], BLANK_NOTE
        if keep_out is not None:
            mask[keep_out] = 0
        if not mask.any():
            continue
        for phase in LEVEL_PHASES[level]:
            runs = _runs_on_lines(mask, angle_deg, spacing_px, min_len_px, phase)
            strokes += _chain_runs(runs, mask, join_px=spacing_px * 2.5)
    return strokes, ""


def _dense(q, step=1.0):
    """폴리라인을 step px 간격 점으로."""
    q = np.asarray(q, np.float64).reshape(-1, 2)
    if len(q) < 2:
        return q
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.arange(0.0, t[-1] + 1e-9, step)
    return np.column_stack([np.interp(s, t, q[:, 0]), np.interp(s, t, q[:, 1])])


def _tensor_flow(g, sig):
    """구조 텐서(밝기 기울기의 외적을 sig로 흐린 것)의 주된 기울기에 수직인 단위 방향 (fx, fy)."""
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    j11 = cv2.GaussianBlur(gx * gx, (0, 0), sig)
    j22 = cv2.GaussianBlur(gy * gy, (0, 0), sig)
    j12 = cv2.GaussianBlur(gx * gy, (0, 0), sig)
    theta = 0.5 * np.arctan2(2 * j12, j11 - j22) + np.pi / 2
    return np.cos(theta).astype(np.float32), np.sin(theta).astype(np.float32)


def edge_tangent_flow(g, mask, radius, iters=3):
    """경계 접선 흐름(ETF, Kang·Lee·Chui 2007): 기울기에 수직인 접선을 반경 radius 안의 이웃 접선으로 되풀이해
    다듬는다. 이웃 가중 = 기울기 크기가 더 큰 쪽(1+tanh(ĝ(y)−ĝ(x)))/2 × 방향이 비슷한 정도 |t(x)·t(y)|, 부호는 방향을
    맞춘다. 약한 곳이 주변 뚜렷한 결의 방향을 따르면서도 가는 결이 남는다(구조 텐서는 넓게 흐려 결이 뭉개짐).
    계산은 마스크 둘레 상자만 한다. 반환: 단위 방향 (fx, fy), 상자 밖과 접선이 없는 곳은 구조 텐서 방향."""
    fx, fy = _tensor_flow(g, max(2.0, radius))
    ys, xs = np.nonzero(mask)
    r = int(max(2, round(radius)))
    h, w = g.shape
    y0, y1 = max(0, ys.min() - 2 * r), min(h, ys.max() + 2 * r + 1)
    x0, x1 = max(0, xs.min() - 2 * r), min(w, xs.max() + 2 * r + 1)
    sub = g[y0:y1, x0:x1]
    gx = cv2.Sobel(sub, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(sub, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)
    mag /= max(float(mag.max()), 1e-6)
    tx, ty = -gy, gx
    n = np.hypot(tx, ty)
    ok = n > 1e-6
    tx, ty = np.where(ok, tx / np.maximum(n, 1e-6), 0), np.where(ok, ty / np.maximum(n, 1e-6), 0)
    offs = [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1) if 0 < dy * dy + dx * dx <= r * r]
    sh, sw = sub.shape
    mp = np.pad(mag, r, mode="edge")
    for _ in range(iters):
        tp, qp = np.pad(tx, r, mode="edge"), np.pad(ty, r, mode="edge")
        ax, ay = np.zeros_like(tx), np.zeros_like(ty)
        for dy, dx in offs:
            sx = tp[r + dy:r + dy + sh, r + dx:r + dx + sw]
            sy = qp[r + dy:r + dy + sh, r + dx:r + dx + sw]
            dot = tx * sx + ty * sy
            wgt = 0.5 * (1 + np.tanh(mp[r + dy:r + dy + sh, r + dx:r + dx + sw] - mag)) * dot   # |dot|·부호 = dot
            ax += wgt * sx
            ay += wgt * sy
        n = np.hypot(ax, ay)
        ok = n > 1e-6
        tx, ty = np.where(ok, ax / np.maximum(n, 1e-6), tx), np.where(ok, ay / np.maximum(n, 1e-6), ty)
    has = np.hypot(tx, ty) > 0.5
    fx[y0:y1, x0:x1] = np.where(has, tx, fx[y0:y1, x0:x1])
    fy[y0:y1, x0:x1] = np.where(has, ty, fy[y0:y1, x0:x1])
    return fx, fy


_FLOW_CACHE = {}


def _cached_flow(g, mask, radius, iters):
    """edge_tangent_flow를 같은 사진·머리 영역·반경이면 다시 계산하지 않는다. 잔선 강도만 바꾸면 결 획은 다시 긋지만
    결 방향은 그대로라, 긴 머리 사진은 후보마다 ETF가 40초씩 들어 자동 세팅이 385초 걸렸다. 마지막 하나만 기억한다."""
    key = (g.shape, float(radius), iters, hashlib.blake2b(g.tobytes(), digest_size=16).digest(),
           hashlib.blake2b(np.packbits(np.asarray(mask, bool)).tobytes(), digest_size=16).digest())
    hit = _FLOW_CACHE.get("last")
    if hit is not None and hit[0] == key:
        return hit[1]
    flow = edge_tangent_flow(g, mask, radius, iters)
    _FLOW_CACHE["last"] = (key, flow)
    return flow


def hair_flow_strokes(gray, mask, spacing_px, min_len_px, max_len_px, obstacles=()):
    """머리카락 결: 마스크(머리 영역) 안에서 사진의 결 방향을 따라 고르게 띄운 흐름선을 긋는다.
    결 방향은 edge_tangent_flow(반경 = 간격). 구조 텐서만 쓰면 짧은 머리에서 결이 머리 윤곽을 따라 동심원으로 돌았고,
    ETF는 정수리에서 앞머리로 흘러내리는 방향이 살아났다(사용자 사진 1). 씨앗 점을 간격마다 놓고
    양쪽으로 1px씩 따라가다 마스크를 벗어나거나, 이미 그은 선에 간격의 절반보다 가까워지거나, max_len_px에 이르면 멈춘다
    (Jobard–Lefer 방식). obstacles(선화가 이미 그린 머리 결 획)도 이미 그은 선으로 쳐서, 결 선은 그 사이 빈 곳만 채운다.
    채색 전 스케치는 면을 빗금으로 메우지 않고 결을 선으로 남기는 편이 자연스럽다.
    반환: 단순화한 획 목록 (작업 이미지 좌표)."""
    if mask is None or not mask.any():
        return []
    spacing_px = max(float(spacing_px), 3.0)
    g = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), 1.5)
    fx, fy = _cached_flow(g, mask, spacing_px, iters=4)
    h, w = mask.shape
    cell = spacing_px / 2.0
    grid = {}
    for q in obstacles:
        for p in _dense(q):
            grid.setdefault((int(p[0] // cell), int(p[1] // cell)), []).append(p)

    def near(p, r):
        cx, cy = int(p[0] // cell), int(p[1] // cell)
        for i in range(cx - 1, cx + 2):
            for j in range(cy - 1, cy + 2):
                for q in grid.get((i, j), ()):
                    if (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 < r * r:
                        return True
        return False

    def trace(seed, sign):
        pts, p, prev = [], np.array(seed, np.float64), None
        for _ in range(int(max_len_px)):
            ix, iy = int(round(p[0])), int(round(p[1]))
            if not (0 <= ix < w and 0 <= iy < h) or not mask[iy, ix] or near(p, 0.5 * spacing_px):
                break
            d = np.array([fx[iy, ix], fy[iy, ix]])
            if prev is None:
                d = d * sign
            elif d @ prev < 0:
                d = -d
            pts.append(p.copy())
            p, prev = p + d, d
        return pts

    out = []
    for y in np.arange(spacing_px / 2, h, spacing_px):
        for x in np.arange(spacing_px / 2, w, spacing_px):
            if not mask[int(y), int(x)] or near((x, y), 0.8 * spacing_px):
                continue
            fwd, back = trace((x, y), 1.0), trace((x, y), -1.0)
            line = back[::-1][:-1] + fwd if back and fwd else (back[::-1] or fwd)
            if len(line) < 2:
                continue
            line = np.asarray(line)
            if float(np.linalg.norm(np.diff(line, axis=0), axis=1).sum()) < min_len_px:
                continue
            for q in line:
                grid.setdefault((int(q[0] // cell), int(q[1] // cell)), []).append(q)
            out.append(cv2.approxPolyDP(line.astype(np.float32).reshape(-1, 1, 2), 0.8, False).reshape(-1, 2)
                       .astype(np.float64))
    return out


# ---------------------------------------------------------------------------------- 사진-그림 비교
def ink_image(strokes, shape, thickness):
    """획을 펜 굵기로 그린 흰 바탕 그레이 이미지 (검정 = 잉크)."""
    img = np.full(shape[:2], 255, np.uint8)
    for s in strokes:
        pts = np.round(np.asarray(s)).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, 0, max(1, int(thickness)), cv2.LINE_AA)
    return img


def _normalized(blurred):
    top = float(np.percentile(blurred, 98))
    return np.clip(blurred / top, 0.0, 1.0) if top > 1e-6 else blurred


def photo_edges(gray, sigma=1.5):
    """사진 자체의 뚜렷한 윤곽 (비교 기준). 선화 경로의 선 지도는 획을 만든 바로 그 지도라 비교 기준으로 쓰면
    정확도가 늘 높게(0.95 안팎) 나오므로, 사진을 살짝 흐린 뒤 밝기 중앙값에 맞춘 Canny로 따로 찾는다.
    흰 배경(배경 제거)은 중앙값 계산에서 뺀다."""
    g = cv2.GaussianBlur(np.asarray(gray), (0, 0), sigma)
    subject = g[g < 250]
    med = float(np.median(subject)) if subject.size else 128.0
    return cv2.Canny(g, int(max(10, 0.5 * med)), int(max(20, med)))


def compare_report(gray, color, strokes, pen_px, edges=None, outline=None, lineart=False):
    """사진과 그려질 그림을 비교. 반환: (지표 dict, 나란히 보기 BGR 이미지).

    tone_match  : 명암 분포의 닮은 정도 (0~1, 클수록 닮음). 크게 흐려서 큰 덩어리의 밝고 어두움을 비교 (1 - 평균 차이)
    edge_recall : 사진의 뚜렷한 윤곽 중 그림에 그려진 비율 (0~1)
    edge_precision : 윤곽 획 중 사진의 윤곽 근처인 비율 (0~1, 낮으면 잡음). 빗금은 윤곽이 아니라서 outline을 주면 뺌
    under_shaded_pct / over_shaded_pct : 사진보다 너무 밝게 / 너무 어둡게 그려진 면적 비율 (%)
    lineart: 실사 선화 경로면 윤곽 안내를 그 경로의 조절(얼굴 세밀·잔선 정리)로 함 (Canny 기준은 쓰이지 않음)
    """
    h, w = gray.shape[:2]
    photo_d = darkness_map(gray, 0.0)
    ink = ink_image(strokes, (h, w), pen_px)
    ink_d = (1.0 - ink.astype(np.float32) / 255.0)
    sigma = max(2.0, 0.025 * max(h, w))
    pn = _normalized(cv2.GaussianBlur(photo_d, (0, 0), sigma))
    inn = _normalized(cv2.GaussianBlur(ink_d, (0, 0), sigma))
    diff = pn - inn
    tone = max(0.0, 1.0 - float(np.abs(diff).mean()))     # 1이면 큰 덩어리의 밝고 어두움이 같음
    metrics = {"tone_match": round(tone, 3),
               "under_shaded_pct": round(100.0 * float((diff > 0.3).mean()), 1),
               "over_shaded_pct": round(100.0 * float((diff < -0.3).mean()), 1),
               "ink_coverage_pct": round(100.0 * float((ink < 128).mean()), 1)}
    if edges is not None:
        e = (np.asarray(edges) > 0).astype(np.uint8)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        drawn = (ink < 128).astype(np.uint8)
        lines = drawn if outline is None else (ink_image(outline, (h, w), pen_px) < 128).astype(np.uint8)
        metrics["edge_recall"] = round(float(cv2.dilate(drawn, k)[e > 0].mean()) if e.any() else 1.0, 3)
        metrics["edge_precision"] = round(float(cv2.dilate(e, k)[lines > 0].mean()) if lines.any() else 0.0, 3)
    hint = []
    if tone < 0.5:
        hint.append("명암 분포가 사진과 많이 다름: 어두운 면이 비어 있으면 tone_levels를 올리고, 배경이 어두우면 배경 제거(rembg)를 켜세요")
    if metrics["under_shaded_pct"] > 15:
        hint.append("사진보다 너무 밝게 그려진 면이 넓음: tone_levels를 올리거나 tone_bias를 내리세요")
    if metrics["over_shaded_pct"] > 15:
        hint.append("사진보다 너무 어둡게(빽빽하게) 그려진 면이 넓음: tone_levels를 내리거나 tone_spacing_mm를 늘리거나 tone_bias를 올리세요")
    if edges is not None and metrics["edge_recall"] < 0.6:
        hint.append("사진의 윤곽 중 그려지지 않은 게 많음: " + (
            "얼굴 세밀 처리를 켜 보세요. 주름·결까지 원하면 잔선 정리(declutter)를 끄세요" if lineart
            else "canny_low를 낮추거나 min_length_px를 줄이세요"))
    if edges is not None and metrics["edge_precision"] < 0.5 and metrics["ink_coverage_pct"] > 0:
        hint.append("사진에 없는 선이 많음(잡음·빗금 위주): " + (
            "잔선 정리(declutter)를 켜거나 잡음 획을 지우세요" if lineart else "상세도를 낮추거나 잡음 획을 지우세요"))
    metrics["hints"] = hint or ["큰 문제 없음: 눈으로 원본과 나란히 보고 마무리하세요"]

    side = 360
    scale = side / h
    tile = lambda img: cv2.resize(img, (max(1, int(w * scale)), side), interpolation=cv2.INTER_AREA)
    heat = cv2.applyColorMap(np.clip((diff + 1.0) * 127.5, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
    canvas = np.hstack([tile(color), np.full((side, 8, 3), 255, np.uint8),
                        tile(cv2.cvtColor(ink, cv2.COLOR_GRAY2BGR)), np.full((side, 8, 3), 255, np.uint8),
                        tile(heat)])
    return metrics, canvas
