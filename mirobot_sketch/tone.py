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

import math

import cv2
import numpy as np

# 단계 수별 어두운 정도 기준 (0~1). 앞 단계가 가장 밝은 기준이라 넓은 면을 먼저 채움
LEVEL_THRESHOLDS = {1: (0.55,), 2: (0.45, 0.72), 3: (0.38, 0.58, 0.78)}
LEVEL_ANGLE_OFFSETS = (0, 90, 45)      # 1단계 방향, 2단계 직각(교차 빗금), 3단계 대각
MAX_COVERAGE = 0.55                    # 첫 단계가 화면의 이만큼 넘게 덮이면 빗금을 만들지 않음 (배경이 어두운 사진)
BLANK_NOTE = "배경이 어두워 빗금이 화면 대부분을 덮습니다. 배경 제거를 켜 보세요."


def darkness_map(gray, sigma_px=0.0):
    """0(밝음)~1(어두움). 흰 배경(배경 제거 결과)은 0으로 두고, 나머지 밝기 분포를 2~98 백분위로 늘림."""
    g = gray.astype(np.float32)
    content = g < 250
    if int(content.sum()) < 100:
        return np.zeros(g.shape, np.float32)
    lo, hi = np.percentile(g[content], (2, 98))
    d = np.clip((hi - g) / max(float(hi - lo), 20.0), 0.0, 1.0)
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


def _runs_on_lines(mask, angle_deg, spacing_px, min_len_px):
    """각도 angle_deg의 평행선을 spacing_px 간격으로 긋고, 마스크 안의 구간을 [(줄 번호, 시작점, 끝점)]으로."""
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
        base = center + k * spacing_px * nrm
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


def hatch_strokes(gray, spacing_px, levels, angle_deg=45.0, min_len_px=8.0, bias=0.0):
    """어두운 면의 빗금 획들. 반환: (획 리스트, 안내 문구). levels <= 0이면 ([], "").
    spacing_px: 빗금 간격, min_len_px: 이보다 짧은 빗금은 버림, bias: +면 기준이 올라가 빗금이 줄고 -면 늘어남."""
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
        if not mask.any():
            continue
        angle = angle_deg + LEVEL_ANGLE_OFFSETS[level]
        runs = _runs_on_lines(mask, angle, spacing_px, min_len_px)
        strokes += _chain_runs(runs, mask, join_px=spacing_px * 2.5)
    return strokes, ""


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


def compare_report(gray, color, strokes, pen_px, edges=None, outline=None):
    """사진과 그려질 그림을 비교. 반환: (지표 dict, 나란히 보기 BGR 이미지).

    tone_match  : 명암 분포의 닮은 정도 (0~1, 클수록 닮음). 크게 흐려서 큰 덩어리의 밝고 어두움을 비교 (1 - 평균 차이)
    edge_recall : 사진의 뚜렷한 윤곽 중 그림에 그려진 비율 (0~1)
    edge_precision : 윤곽 획 중 사진의 윤곽 근처인 비율 (0~1, 낮으면 잡음). 빗금은 윤곽이 아니라서 outline을 주면 뺌
    under_shaded_pct / over_shaded_pct : 사진보다 너무 밝게 / 너무 어둡게 그려진 면적 비율 (%)
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
        hint.append("사진의 윤곽 중 그려지지 않은 게 많음: canny_low를 낮추거나 min_length_px를 줄이세요")
    if edges is not None and metrics["edge_precision"] < 0.5 and metrics["ink_coverage_pct"] > 0:
        hint.append("사진에 없는 선이 많음(잡음·빗금 위주): 상세도를 낮추거나 잡음 획을 지우세요")
    metrics["hints"] = hint or ["큰 문제 없음: 눈으로 원본과 나란히 보고 마무리하세요"]

    side = 360
    scale = side / h
    tile = lambda img: cv2.resize(img, (max(1, int(w * scale)), side), interpolation=cv2.INTER_AREA)
    heat = cv2.applyColorMap(np.clip((diff + 1.0) * 127.5, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
    canvas = np.hstack([tile(color), np.full((side, 8, 3), 255, np.uint8),
                        tile(cv2.cvtColor(ink, cv2.COLOR_GRAY2BGR)), np.full((side, 8, 3), 255, np.uint8),
                        tile(heat)])
    return metrics, canvas
