"""
픽셀 획 -> 종이 좌표(mm) 변환과 JSON 내보내기
============================================
좌표 약속 (이 파일이 만드는 JSON의 기준):
  - 원점: 종이 중심
  - x_mm: 종이를 정면에서 봤을 때 오른쪽이 +
  - y_mm: 종이를 정면에서 봤을 때 위쪽이 +
  이미지 좌표는 y가 아래로 커지므로 변환할 때 부호를 뒤집습니다.

로봇 축(Y/Z)으로의 변환은 여기서 하지 않고 robot/draw_executor.py가
robot/drawing_config.json의 부호·중심 설정으로 처리합니다. CV 결과는
로봇 설정이 바뀌어도 다시 만들 필요가 없습니다.

배치 규칙 (설계 문서 5.2):
  - 획 전체의 경계 상자 중심을 종이 중심에 맞춘다.
  - 가로세로 비율을 유지하며 그리기 상자(box_width_mm x box_height_mm) 안에 넣는다.
  - 그리기 상자는 종이 안전 여백 안에 있어야 한다.
  - region(허용 영역, limits.Region)을 주면: 종이 중심에 들어가지 않는 큰 그림은 가장 적게 아래로 옮기고,
    그래도 안 되면 들어갈 만큼 줄인다 (로봇 팔은 위쪽 여유가 작음).
"""

import json
from datetime import datetime

import numpy as np

A4_LANDSCAPE_MM = (297.0, 210.0)
DEFAULT_MARGIN_MM = 20.0
# 실물에서 끝까지 실행된 가장 큰 도형이 100mm 원이라 기본 그리기 크기를 100mm로 둡니다.
# 더 크게 그리려면 도달 범위(Soft limit)를 먼저 확인하세요.
DEFAULT_BOX_MM = (100.0, 100.0)

SCHEMA_VERSION = 1


def strokes_bbox(strokes):
    allp = np.vstack([np.asarray(s, dtype=np.float64) for s in strokes])
    return allp.min(axis=0), allp.max(axis=0)


def pixels_to_paper(strokes_px, box_mm=DEFAULT_BOX_MM, paper_mm=A4_LANDSCAPE_MM,
                    margin_mm=DEFAULT_MARGIN_MM, region=None, warp=None):
    """픽셀 획들을 종이 중심 기준 mm 획들로 변환합니다.

    region이 있으면 그 안에 들어가게 배치합니다. warp(x, y) -> (x, y)를 주면 로봇에 실제로 보낼(미리 비튼)
    좌표가 region 안에 들어갈 때까지 줄입니다 (실행기 범위 검사 check_limits와 같은 기준).
    반환: (strokes_mm, placement) — placement에는 배율, 실제 그림 크기 등이 들어갑니다.
    """
    strokes_px = [np.asarray(s, dtype=np.float64) for s in strokes_px if len(s) >= 2]
    if not strokes_px:
        raise ValueError("변환할 획이 없습니다.")

    box_w, box_h = box_mm
    safe_w = paper_mm[0] - 2 * margin_mm
    safe_h = paper_mm[1] - 2 * margin_mm
    if region is None and (box_w > safe_w or box_h > safe_h):
        raise ValueError(
            f"그리기 상자 {box_w}x{box_h}mm가 종이 안전 영역 {safe_w}x{safe_h}mm보다 큽니다."
        )

    lo, hi = strokes_bbox(strokes_px)
    size = np.maximum(hi - lo, 1e-9)
    scale = min(box_w / size[0], box_h / size[1])  # mm per px, 비율 유지
    center = (lo + hi) / 2.0
    ox = oy = 0.0
    if region is not None:
        pos = region.fit(size[0] * scale, size[1] * scale)
        if pos is None:                              # 영역에 안 들어감: 들어갈 만큼 줄임
            scale = region.max_scale(size[0], size[1])
            pos = region.fit(size[0] * scale, size[1] * scale) or (0.0, 0.0)
        if warp is not None:
            scale, pos = _shrink_for_warp(strokes_px, center, size, scale, pos, region, warp)
        ox, oy = pos

    strokes_mm = []
    for s in strokes_px:
        x = (s[:, 0] - center[0]) * scale + ox
        y = -(s[:, 1] - center[1]) * scale + oy  # 이미지 y(아래 +) -> 종이 y(위 +)
        strokes_mm.append(np.column_stack([x, y]))

    placement = {
        "paper_width_mm": paper_mm[0],
        "paper_height_mm": paper_mm[1],
        "margin_mm": margin_mm,
        "box_width_mm": box_w,
        "box_height_mm": box_h,
        "scale_mm_per_px": round(float(scale), 6),
        "drawing_width_mm": round(float(size[0] * scale), 3),
        "drawing_height_mm": round(float(size[1] * scale), 3),
        "center_px": [round(float(center[0]), 3), round(float(center[1]), 3)],   # mm -> px 역변환용
        # 반올림하지 않은 변환 (세션의 mm<->px 계산용: 반올림 값을 쓰면 가장자리 점이 허용 범위를 0.0001mm 넘음)
        "transform": {"scale": float(scale), "cx": float(center[0]), "cy": float(center[1]), "ox": ox, "oy": oy},
        "offset_mm": [ox, oy],   # 그림 중심의 종이 위치 (큰 그림은 아래로 옮김)
        "alignment": "bounding_box_center_to_paper_center" if (ox, oy) == (0.0, 0.0) else "moved_down_into_reach",
    }
    return strokes_mm, placement


def _shrink_for_warp(strokes_px, center, size, scale, pos, region, warp, min_ratio=0.3):
    """비튼 좌표가 region을 벗어나면 배율을 이분 탐색으로 줄인다 (자리는 줄인 크기로 다시 맞춤).
    검사는 볼록 껍질의 변을 종이 1 mm 간격으로 채운 점으로 한다: 비틀기는 2차식이라 꼭짓점 사이의 변이
    가운데에서 바깥으로 휠 수 있다 (50 mm 직선에 약 0.4 mm). 모든 점은 실행 전 check_limits가 다시 검사한다."""
    import cv2

    pts = np.vstack(strokes_px).astype(np.float32)
    corners = cv2.convexHull(pts).reshape(-1, 2).astype(np.float64)
    step_px = 1.0 / max(scale, 1e-9)                 # 지금 배율에서 1 mm (줄이면 더 촘촘해짐)
    edges = []
    for a, b in zip(corners, np.roll(corners, -1, axis=0)):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step_px)))
        edges.append(a + (b - a) * (np.arange(n)[:, None] / n))
    hull = np.vstack(edges)

    def placed(sc):
        p = region.fit(size[0] * sc, size[1] * sc)
        if p is None:
            return None
        xs = (hull[:, 0] - center[0]) * sc + p[0]
        ys = -(hull[:, 1] - center[1]) * sc + p[1]
        return p if all(region.contains(*warp(x, y)) for x, y in zip(xs, ys)) else None

    if placed(scale) is not None:
        return scale, pos
    lo, hi = scale * min_ratio, scale
    if placed(lo) is None:
        return scale, pos                            # 줄여도 안 들어감: 실행 전 검사가 알려 줌
    for _ in range(20):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if placed(mid) is not None else (lo, mid)
    return lo, placed(lo)


def check_within_paper(strokes_mm, paper_mm=A4_LANDSCAPE_MM, margin_mm=DEFAULT_MARGIN_MM):
    """모든 점이 종이 안전 영역 안에 있는지 확인합니다. 벗어난 점 수를 반환합니다."""
    half_w = paper_mm[0] / 2 - margin_mm
    half_h = paper_mm[1] / 2 - margin_mm
    bad = 0
    for s in strokes_mm:
        s = np.asarray(s)
        bad += int(np.sum((np.abs(s[:, 0]) > half_w + 1e-6) | (np.abs(s[:, 1]) > half_h + 1e-6)))
    return bad


def render_paper_preview(strokes_mm, line_width_mm=0.5, px_per_mm=4.0, paper_mm=A4_LANDSCAPE_MM,
                         limit_mm=None, pending_limit_mm=None):
    """A4 위에 실제 크기·펜 굵기로 그린 모습을 BGR 이미지로 만듭니다 (GUI 미리보기).

    - 종이: 흰 바탕, 회색 테두리
    - limit_mm: 실행기 허용 범위 파란 점선, pending_limit_mm: 실물 미확인 넓은 범위 주황 점선
      (±mm 숫자 또는 limits.Region — Region이면 지붕 모양 테두리)
    - 선 굵기 = line_width_mm x px_per_mm (촘촘한 선이 실제로 뭉개지는지 볼 수 있음)
    """
    import cv2

    w, h = int(round(paper_mm[0] * px_per_mm)), int(round(paper_mm[1] * px_per_mm))
    img = np.full((h, w, 3), 255, np.uint8)
    cx, cy = w / 2.0, h / 2.0

    def to_px(x, y):
        return int(round(cx + x * px_per_mm)), int(round(cy - y * px_per_mm))

    def dashed_rect(area, color):
        if hasattr(area, "outline"):
            pts = area.outline()
        else:
            half = area
            pts = [(-half, -half), (half, -half), (half, half), (-half, half), (-half, -half)]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            n = max(1, int(np.hypot(x1 - x0, y1 - y0) / 3))
            for k in range(0, n, 2):
                a = to_px(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
                b = to_px(x0 + (x1 - x0) * (k + 1) / n, y0 + (y1 - y0) * (k + 1) / n)
                cv2.line(img, a, b, color, 1, cv2.LINE_AA)

    if pending_limit_mm:
        dashed_rect(pending_limit_mm, (0, 140, 255))
    if limit_mm:
        dashed_rect(limit_mm, (200, 120, 30))
    cv2.drawMarker(img, to_px(0, 0), (160, 160, 160), cv2.MARKER_CROSS, 12, 1)
    thick = max(1, int(round(line_width_mm * px_per_mm)))
    for s in strokes_mm:
        pts = np.array([to_px(x, y) for x, y in np.asarray(s)], np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, (30, 30, 30), thick, cv2.LINE_AA)
    cv2.rectangle(img, (0, 0), (w - 1, h - 1), (150, 150, 150), 2)
    return img


def build_strokes_document(strokes_mm, placement, metrics_mm=None, source=None):
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": "sketch_strokes",
        "created_local": datetime.now().astimezone().isoformat(timespec="seconds"),
        "units": "mm",
        "paper": {"size": "A4", "orientation": "landscape"},
        "coordinate_frame": {
            "origin": "paper_center",
            "x_axis": "paper_right_positive (viewed from the front of the paper)",
            "y_axis": "paper_up_positive",
            "robot_mapping": "done by robot/draw_executor.py using robot/drawing_config.json",
        },
        "placement": placement,
        "source": source or {},
        "metrics": metrics_mm or {},
        "strokes": [
            {
                "closed": bool(len(s) > 2 and np.allclose(s[0], s[-1])),
                "points_xy_mm": [[round(float(x), 3), round(float(y), 3)] for x, y in s],
            }
            for s in strokes_mm
        ],
    }


def save_strokes_json(path, document):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(document, f, ensure_ascii=False, indent=1)


def load_strokes_json(path):
    with open(path, "r", encoding="utf-8") as f:
        doc = json.load(f)
    if doc.get("kind") != "sketch_strokes":
        raise ValueError(f"sketch_strokes 형식이 아닙니다: {path}")
    strokes = [np.asarray(s["points_xy_mm"], dtype=np.float64) for s in doc["strokes"]]
    return doc, strokes
