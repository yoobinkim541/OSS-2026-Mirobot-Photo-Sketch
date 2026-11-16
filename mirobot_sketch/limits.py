"""
드로잉 허용 영역 (종이 중심 기준 mm, x 오른쪽 +, y 위 +)
========================================================
- 실물 확인 범위(limits): 대칭 사각형 {max_abs_paper_x_mm, max_abs_paper_y_mm}
- 넓은 범위(limits_pending_verification, 실물 미확인): 도달 지도에 맞춘 "지붕 모양"
  {x_max_mm, y_min_mm, roof_mm: [[|x|, 위쪽 y], ...]} — 로봇 팔은 좌우·아래로는 여유가 크고 위쪽은
  B(J5) 한계에 걸리며, 좌우 끝으로 갈수록 위쪽 한계가 낮아짐. 점 사이는 직선.
예전 형식(대칭 사각형)도 그대로 읽습니다.
"""

import copy
import math
import json

import numpy as np

from . import paths

LEGACY_PENDING = (60.0, 60.0)     # 예전 기본 넓은 범위 (±60mm) — 사용자가 손대지 않았으면 새 영역으로 바꿈


class Region:
    def __init__(self, x_max, y_min, roof):
        self.x_max, self.y_min = float(x_max), float(y_min)
        deduped = []
        for x, y in sorted((float(x), float(y)) for x, y in roof):
            if deduped and deduped[-1][0] == x:
                if y < deduped[-1][1]:
                    deduped[-1] = (x, y)
            else:
                deduped.append((x, y))
        if not deduped:
            raise ValueError("지붕(roof_mm)이 비어 있습니다")
        self._rx = np.array([p[0] for p in deduped])
        self._ry = np.array([p[1] for p in deduped])

    @classmethod
    def from_cfg(cls, lim):
        if "roof_mm" in lim:
            if not lim["roof_mm"]:
                raise ValueError("지붕(roof_mm)이 비어 있습니다")
            return cls(lim["x_max_mm"], lim["y_min_mm"], lim["roof_mm"])
        x, y = float(lim["max_abs_paper_x_mm"]), float(lim["max_abs_paper_y_mm"])
        return cls(x, -y, [(0, y), (x, y)])

    def top(self, ax):
        """|x| = ax에서 위쪽 한계."""
        return float(np.interp(min(abs(ax), self.x_max), self._rx, self._ry))

    def _ceiling(self, half_w):
        """|x| ≤ half_w 전체에서 가장 낮은 위쪽 한계 (그림 상자의 윗변이 넘지 않아야 하는 높이)."""
        inner = self._ry[self._rx <= half_w]
        return float(min([self.top(half_w), *inner]))

    def contains(self, x, y, eps=1e-6):
        return abs(x) <= self.x_max + eps and self.y_min - eps <= y <= self.top(x) + eps

    def outline(self):
        """테두리 다각형 (닫힘): 아래 변 → 오른쪽 → 지붕 → 왼쪽."""
        right = [(float(x), float(y)) for x, y in zip(self._rx, self._ry)]
        if right[-1][0] < self.x_max:
            right.append((self.x_max, self.top(self.x_max)))
        roof = list(reversed(right)) + [(-x, y) for x, y in right if x > 0]
        pts = [(-self.x_max, self.y_min), (self.x_max, self.y_min)] + roof
        return pts + [pts[0]]

    def fit(self, w, h, eps=1e-6):
        """w×h 그림 상자를 둘 중심 (0, cy). 종이 중심에 들어가면 (0, 0), 아니면 가장 적게 아래로. 안 되면 None."""
        if not _positive_finite(w, h):
            return None
        if w / 2 > self.x_max + eps:
            return None
        ceiling = self._ceiling(w / 2)
        if h > ceiling - self.y_min + eps:
            return None
        cy = max(min(0.0, ceiling - h / 2), self.y_min + h / 2)
        return 0.0, float(cy)

    def max_scale(self, w, h):
        """fit(s·w, s·h)가 되는 가장 큰 s. 크기가 유한한 양수가 아니면 0."""
        if not _positive_finite(w, h):
            return 0.0
        lo, hi = 0.0, 2 * self.x_max / w
        for _ in range(60):
            mid = (lo + hi) / 2
            if self.fit(mid * w, mid * h) is None:
                hi = mid
            else:
                lo = mid
        return lo


def _positive_finite(w, h):
    return math.isfinite(w) and math.isfinite(h) and w > 0 and h > 0


def executor_region(cfg):
    return Region.from_cfg(cfg["limits"])


def pending_region(cfg):
    return Region.from_cfg(cfg["limits_pending_verification"])


def migrate(cfg):
    """사용자 폴더 설정의 넓은 범위가 예전 기본값(±60, 손대지 않음)이면 패키지의 새 기본 영역으로 바꾼 사본."""
    lim = cfg.get("limits_pending_verification", {})
    if "roof_mm" in lim or (lim.get("max_abs_paper_x_mm"), lim.get("max_abs_paper_y_mm")) != LEGACY_PENDING:
        return cfg
    with open(paths.DATA_DIR / "drawing_config.json", encoding="utf-8") as f:
        default = json.load(f)
    out = copy.deepcopy(cfg)
    out["limits_pending_verification"] = default["limits_pending_verification"]
    return out


def effective_long_mm(region, box_mm, w, h):
    """긴 변 box_mm로 요청한 w×h 그림이 영역에 맞춰 실제로 그려질 긴 변 (mm). 큰 세로 그림은 위쪽 한계 때문에 줄어듦."""
    if not _positive_finite(w, h) or not (math.isfinite(box_mm) and box_mm > 0):
        return 0.0
    long_side = max(w, h)
    scale = min(box_mm / long_side, region.max_scale(w, h))
    return scale * long_side
