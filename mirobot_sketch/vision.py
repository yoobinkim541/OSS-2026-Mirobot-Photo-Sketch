"""
카메라 화면과 종이 좌표 맞추기 (V1: 손으로 귀퉁이 4개 찍기)
=========================================================
종이는 평면이라 호모그래피 하나로 "종이 mm ↔ 카메라 px"를 바꿀 수 있습니다. 사용자가 카메라 사진에서 종이 네 귀퉁이를
찍으면 그 위치와 종이 크기(기본 A4 가로, 종이 중심 = 호밍 직후 펜 끝)로 행렬을 만들고, 계획한 획·펜 범위를 화면에 겹칩니다.

찍은 점은 사진 크기에 대한 비율(0~1)로 저장해, 미리보기(축소)와 원본 사진 어디에나 같은 등록을 씁니다.
카메라마다 따로 저장합니다(장치 이름 기준). 카메라나 판을 옮기면 다시 찍어야 합니다.
로봇을 움직이지 않습니다: 보여 주기와 기록만 합니다.

설계: docs/specs/2026-11-09-vision-guided-drawing.md
"""

import json
from dataclasses import dataclass
from datetime import datetime

import cv2
import numpy as np

from . import paths
from .paper_mapping import A4_LANDSCAPE_MM

SETTINGS_FILE = "vision_registration.json"
CORNER_NAMES = ("왼쪽 위", "오른쪽 위", "오른쪽 아래", "왼쪽 아래")
MIN_AREA_RATIO = 0.02          # 찍은 사각형이 사진의 이만큼보다 작으면 잘못 찍은 것으로 봄
MIN_SIDE_FRAC = 0.08           # 짧은 변이 사진 짧은 변의 이보다 작으면 클릭 오차가 mm로 커짐
MAX_ASPECT_FACTOR = 2.5        # 가로/세로가 종이 비율에서 이 배 넘게 벗어나면 거절
MIN_CORNER_DEG = 50.0          # 이보다 뾰족한 귀퉁이는 잘못 찍은 것

# 겹쳐 그리는 색 (BGR)
PAPER_COLOR = (80, 200, 80)
PEN_RANGE_COLOR = (230, 140, 30)
PENDING_RANGE_COLOR = (0, 150, 255)
STROKE_COLOR = (200, 40, 220)
LINE_SEEN_COLOR = (40, 170, 0)       # 카메라에서 선이 보인 획
LINE_MISSING_COLOR = (40, 40, 230)   # 다 그렸는데 선이 안 보인 획


def paper_corners_mm(paper_mm=A4_LANDSCAPE_MM):
    """종이 네 귀퉁이 (왼쪽 위, 오른쪽 위, 오른쪽 아래, 왼쪽 아래), 종이 중심 기준 mm (x 오른쪽, y 위)."""
    hw, hh = paper_mm[0] / 2, paper_mm[1] / 2
    return np.array([(-hw, hh), (hw, hh), (hw, -hh), (-hw, -hh)], np.float64)


def _corner_deg(pts, i):
    a, b, c = pts[i - 1], pts[i], pts[(i + 1) % 4]
    u, v = a - b, c - b
    return float(np.degrees(np.arccos(np.clip(
        np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-9), -1, 1))))


def check_corners(points_px, frame_size, paper_mm=A4_LANDSCAPE_MM):
    """찍은 네 점이 쓸 만한 사각형인지. 문제가 있으면 이유, 없으면 None."""
    pts = np.asarray(points_px, np.float32)
    if pts.shape != (4, 2):
        return "귀퉁이 4개를 모두 찍어 주세요."
    w, h = frame_size
    if np.any((pts[:, 0] < 0) | (pts[:, 1] < 0) | (pts[:, 0] >= w) | (pts[:, 1] >= h)):
        return "점이 사진 밖에 있습니다. 종이 귀퉁이를 사진 안에서 찍어 주세요."
    if not cv2.isContourConvex(pts.reshape(-1, 1, 2)):
        return "네 점이 꼬였습니다. 왼쪽 위 → 오른쪽 위 → 오른쪽 아래 → 왼쪽 아래 순서로 찍어 주세요."
    if cv2.contourArea(pts) < MIN_AREA_RATIO * w * h:
        return "사각형이 너무 작습니다. 종이 귀퉁이를 찍었는지 확인해 주세요."
    # 화면에서 시계 방향(이미지 y는 아래로)이어야 위·오른쪽 순서가 맞음
    signed = sum(float(pts[i, 0] * pts[(i + 1) % 4, 1] - pts[(i + 1) % 4, 0] * pts[i, 1]) for i in range(4))
    if signed <= 0:
        return "순서가 거꾸로입니다. 왼쪽 위 → 오른쪽 위 → 오른쪽 아래 → 왼쪽 아래 순서로 찍어 주세요."
    if int(np.argmin(pts.sum(axis=1))) != 0:
        return "첫 점은 왼쪽 위여야 합니다. 왼쪽 위 → 오른쪽 위 → 오른쪽 아래 → 왼쪽 아래 순서로 찍어 주세요."
    sides = [float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])) for i in range(4)]
    if min(sides) < MIN_SIDE_FRAC * min(w, h):
        return "변이 너무 짧습니다. 종이 귀퉁이를 더 크게 찍어 주세요."
    aspect = float(paper_mm[0]) / max(float(paper_mm[1]), 1e-9)
    ratio = (sides[0] + sides[2]) / max(sides[1] + sides[3], 1e-9)
    if not (aspect / MAX_ASPECT_FACTOR <= ratio <= aspect * MAX_ASPECT_FACTOR):
        return "가로·세로 비율이 종이와 너무 다릅니다."
    if any(_corner_deg(pts, i) < MIN_CORNER_DEG for i in range(4)):
        return "각이 너무 작습니다. 종이 귀퉁이를 다시 찍어 주세요."
    return None


@dataclass(frozen=True)
class Registration:
    camera: str                     # 장치 이름
    corners: tuple                  # 귀퉁이 4개, 사진 크기에 대한 비율 ((x, y), ...)
    paper_mm: tuple = A4_LANDSCAPE_MM
    saved_at: str = ""
    frame_size: tuple = ()          # 찍을 때 사진 크기 (가로, 세로). 예전 등록은 비어 있음

    @classmethod
    def from_pixels(cls, camera, points_px, frame_size, paper_mm=A4_LANDSCAPE_MM):
        w, h = frame_size
        corners = tuple((round(float(x) / w, 6), round(float(y) / h, 6)) for x, y in points_px)
        return cls(camera, corners, tuple(float(v) for v in paper_mm),
                   datetime.now().astimezone().isoformat(timespec="seconds"), (int(w), int(h)))

    def fits(self, frame_size):
        """이 크기의 사진에 쓸 수 있는지. 비율로 저장하므로 같은 화면 비율이면 크기만 달라도 되지만,
        화면 비율이 다르면 카메라가 보여 주는 범위 자체가 다르다(예: 4K 16:9와 640×480 4:3)."""
        if not self.frame_size:
            return True
        (w0, h0), (w, h) = self.frame_size, frame_size
        return abs(w0 / h0 - w / h) < 0.01

    def corners_px(self, frame_size):
        w, h = frame_size
        return np.array([(x * w, y * h) for x, y in self.corners], np.float64)

    def matrix(self, frame_size):
        """종이 mm → 사진 px 호모그래피 (3×3)."""
        return cv2.getPerspectiveTransform(paper_corners_mm(self.paper_mm).astype(np.float32),
                                           self.corners_px(frame_size).astype(np.float32))

    def to_dict(self):
        out = {"camera": self.camera, "corners": [list(c) for c in self.corners],
               "paper_mm": list(self.paper_mm), "saved_at": self.saved_at}
        if self.frame_size:
            out["frame_size"] = list(self.frame_size)
        return out

    @classmethod
    def from_dict(cls, data):
        return cls(str(data["camera"]), tuple(tuple(float(v) for v in c) for c in data["corners"]),
                   tuple(float(v) for v in data.get("paper_mm", A4_LANDSCAPE_MM)), str(data.get("saved_at", "")),
                   tuple(int(v) for v in data.get("frame_size", ())))


def project(matrix, points_mm):
    pts = np.asarray(points_mm, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, matrix).reshape(-1, 2)


def view_box(registration, frame_size, strokes_mm=None, margin_mm=15.0):
    """그림(없으면 종이) 둘레를 담는 사진 속 사각형 (x0, y0, x1, y1) px, 사진 안으로 자름. 크게 보기·공유용."""
    if strokes_mm:
        pts = np.vstack([np.asarray(s, np.float64).reshape(-1, 2) for s in strokes_mm])
        (x0, y0), (x1, y1) = pts.min(axis=0) - margin_mm, pts.max(axis=0) + margin_mm
        outline = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    else:
        outline = paper_corners_mm(registration.paper_mm) * 1.05
    px = project(registration.matrix(frame_size), outline)
    w, h = frame_size
    x0, y0 = np.clip(np.floor(px.min(axis=0)), 0, None).astype(int)
    x1, y1 = np.ceil(px.max(axis=0)).astype(int)
    x1, y1 = min(int(x1), w), min(int(y1), h)
    if x1 - x0 < 16 or y1 - y0 < 16:
        return 0, 0, w, h
    return int(x0), int(y0), x1, y1


def crop_registration(registration, frame_size, box):
    """사진의 box 부분만 잘라 낸 사진에 쓸 등록 (잘라 낸 사진을 어떤 크기로 줄여도 그대로 맞음)."""
    x0, y0, x1, y1 = box
    corners = tuple(((x - x0) / (x1 - x0), (y - y0) / (y1 - y0)) for x, y in registration.corners_px(frame_size))
    return Registration(registration.camera, corners, registration.paper_mm, registration.saved_at,
                        (x1 - x0, y1 - y0))


def _load_all():
    try:
        data = json.loads((paths.user_dir() / SETTINGS_FILE).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def load(camera):
    """이 카메라(장치 이름)에 저장한 등록, 없으면 None."""
    data = _load_all().get(camera)
    if not data:
        return None
    try:
        return Registration.from_dict(data)
    except (KeyError, TypeError, ValueError):
        return None


def save(registration):
    data = _load_all()
    data[registration.camera] = registration.to_dict()
    (paths.user_dir() / SETTINGS_FILE).write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")


def forget(camera):
    data = _load_all()
    if data.pop(camera, None) is not None:
        (paths.user_dir() / SETTINGS_FILE).write_text(json.dumps(data, ensure_ascii=False, indent=1),
                                                      encoding="utf-8")


class Overlay:
    """계획한 획·펜 범위를 카메라 프레임에 겹쳐 그림. 투영한 좌표는 (사진 크기, 등록, 획)이 같으면 다시 계산하지 않음."""

    def __init__(self):
        self._key = None
        self._shapes = None

    def _project_all(self, registration, frame_size, strokes_mm, regions):
        key = (registration, frame_size, id(strokes_mm), len(strokes_mm or ()), tuple(id(r) for r, _ in regions))
        if key == self._key:
            return self._shapes
        m = registration.matrix(frame_size)
        outline = paper_corners_mm(registration.paper_mm)
        shapes = {
            "paper": project(m, np.vstack([outline, outline[:1]])),
            "regions": [(project(m, region.outline()), color) for region, color in regions],
            "center": project(m, [(0.0, 0.0)])[0],
            "strokes": [],
        }
        if strokes_mm:
            lengths = [len(s) for s in strokes_mm]
            flat = project(m, np.vstack([np.asarray(s, np.float64) for s in strokes_mm]))
            shapes["strokes"] = np.split(flat, np.cumsum(lengths)[:-1])
        self._key, self._shapes = key, shapes
        return shapes

    def draw(self, frame, registration, strokes_mm=None, regions=(), stroke_colors=None, marks=()):
        """frame(BGR)에 겹친 복사본. regions: [(limits.Region, BGR 색), ...]. stroke_colors: 획마다 BGR (없으면 모두 같은 색).
        marks: [((x, y) 종이 mm, BGR 색), ...] 원으로 표시할 점 (세로 줄이 생긴 끝점 등)."""
        out = frame.copy()
        h, w = frame.shape[:2]
        shapes = self._project_all(registration, (w, h), strokes_mm, tuple(regions))
        thin = max(1, round(w / 640))

        def poly(points, color, width):
            cv2.polylines(out, [np.round(points).astype(np.int32).reshape(-1, 1, 2)], False, color, width,
                          cv2.LINE_AA)

        poly(shapes["paper"], PAPER_COLOR, thin + 1)
        for points, color in shapes["regions"]:
            poly(points, color, thin)
        for i, stroke in enumerate(shapes["strokes"]):
            poly(stroke, stroke_colors[i] if stroke_colors else STROKE_COLOR, thin)
        cx, cy = np.round(shapes["center"]).astype(int)
        cv2.drawMarker(out, (int(cx), int(cy)), PAPER_COLOR, cv2.MARKER_CROSS, 10 * thin, thin, cv2.LINE_AA)
        if marks:
            m = registration.matrix((w, h))
            for (px, py), color in zip(project(m, [p for p, _ in marks]), (c for _, c in marks)):
                cv2.circle(out, (int(round(px)), int(round(py))), 4 * thin, color, thin, cv2.LINE_AA)
        return out


# 획마다 "종이에 선이 생겼나" 확인 (카메라 사진만 봄, 로봇은 움직이지 않음)
LINE_PPM = 6.0             # 펴진 종이 해상도 (px/mm). 4K로 A4를 찍으면 약 7.5 px/mm
LINE_MARGIN_MM = 12.0      # 그림 둘레로 더 펼 폭 (빈 종이에서 잡음 크기를 잼)
LINE_TOL_MM = 2.0          # 계획 경로에서 이만큼 안에 선이 있으면 그 점은 그려진 것으로 봄 (펜이 끌리며 생기는 어긋남 포함)
LINE_SEEN = 0.5            # 경로의 이 비율 이상에서 선이 보이면 "보임"
LINE_MIN_MM = 3.0          # 이보다 짧은 어두운 조각은 종이 얼룩으로 봄
# 획 끝점의 세로 줄: 펜을 세게 누르거나 뗄 때 펜 끝이 종이 위아래로 미끄러져 끝점에서 위·아래로 짧은 줄이 생김
TICK_BAND_MM = 1.2         # 끝점 좌우 이 폭 안의 세로 띠에서 찾음 (종이 위치 오차 포함)
TICK_CLEAR_MM = 1.2        # 계획한 경로(어느 획이든)에서 이만큼 안의 선은 획 자체로 보고 세지 않음
TICK_REACH_MM = 6.0        # 끝점에서 위아래로 이만큼까지 봄
TICK_GAP_MM = 0.5          # 줄이 이만큼 넘게 끊기면 거기서 끝
TICK_MIN_MM = 1.8          # 끝점에서 세로로 이만큼 넘게 뻗은 선이 있으면 "세로 줄"
TICK_COLOR = (0, 140, 255)          # 세로 줄이 생긴 끝점 표시 (주황)


class LineCheck:
    """등록된 카메라 사진에서 계획한 획마다 선이 보이는지 잼.

    add(frame)로 원본 사진 몇 장을 넣으면 그림 둘레만 펴서 평균하고(작은 배열만 보관), measure(i)가
    획 i의 경로 중 선이 LINE_TOL_MM 안에 있는 비율을 돌려줌. 팔·홀더처럼 종이와 밝기·색이 크게 다른 곳에
    경로가 절반 넘게 가려지면 None.
    """

    def __init__(self, registration, strokes_mm, frame_size):
        strokes = [np.asarray(s, np.float64).reshape(-1, 2) for s in strokes_mm]
        pts = np.vstack(strokes)
        x0, y0 = pts.min(axis=0) - LINE_MARGIN_MM
        x1, y1 = pts.max(axis=0) + LINE_MARGIN_MM
        self.size = (int(np.ceil((x1 - x0) * LINE_PPM)), int(np.ceil((y1 - y0) * LINE_PPM)))
        mm_to_flat = np.array([[LINE_PPM, 0, -x0 * LINE_PPM], [0, -LINE_PPM, y1 * LINE_PPM], [0, 0, 1]])
        self.frame_size = tuple(frame_size)
        self.warp = mm_to_flat @ np.linalg.inv(registration.matrix(self.frame_size))
        self.paths = [self._densify(project(mm_to_flat, s)) for s in strokes]
        self._sum = None
        self.frames = 0
        self._maps = None
        self._near = None

    @staticmethod
    def _densify(p, step=0.5 * LINE_PPM):
        out = [p[:1]]
        for a, b in zip(p[:-1], p[1:]):
            n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
            out.append(a + (b - a) * (np.arange(1, n + 1)[:, None] / n))
        return np.round(np.vstack(out)).astype(int)

    def add(self, frame):
        """원본 크기 BGR 사진 한 장 (등록한 사진 크기와 같아야 함)."""
        h, w = frame.shape[:2]
        if (w, h) != self.frame_size:
            raise ValueError(f"사진 크기 {w}x{h}가 확인을 만든 크기 {self.frame_size}와 다릅니다")
        flat = cv2.warpPerspective(frame, self.warp, self.size, flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_REPLICATE).astype(np.float32)
        self._sum = flat if self._sum is None else self._sum + flat
        self.frames += 1
        self._maps = None

    def _analyse(self):
        if self._maps is not None:
            return self._maps
        if self._sum is None:
            raise ValueError("사진을 먼저 add() 하세요")
        color = self._sum / self.frames
        g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        # 선의 어둡기: 주변(가장 밝은 종이)에 대한 비율, 굵은 그늘·얼룩은 빼고 가는 선만
        bg = cv2.GaussianBlur(cv2.dilate(g, np.ones((21, 21), np.uint8)), (0, 0), 8)
        ink = np.clip(1 - g / np.maximum(bg, 1), -1, 1).astype(np.float32)
        fine = cv2.blur(ink, (3, 3)) - cv2.medianBlur((np.clip(ink, 0, 1) * 255).astype(np.uint8),
                                                      15).astype(np.float32) / 255
        # 가림: 굵은 영역만. 가는 어두운 선은 잉크로 남기고, 채도는 종이 중앙값보다 진할 때만
        h, w = g.shape
        small = cv2.resize(np.clip(g, 0, 255).astype(np.uint8), (max(1, w // 4), max(1, h // 4)),
                           interpolation=cv2.INTER_AREA)
        # 팔이 종이 밝기로 잡히면 가장자리만 남아 열림에 지워진다. 펜 선·팔보다 넓은 중앙값을 쓴다.
        blur_k = min(81, small.shape[0], small.shape[1])
        blur_k = blur_k if blur_k % 2 else blur_k - 1
        bg = cv2.medianBlur(small, blur_k) if blur_k >= 3 else small
        paper = cv2.resize(bg, (w, h)).astype(np.float32)
        sat = cv2.cvtColor(np.clip(color, 0, 255).astype(np.uint8), cv2.COLOR_BGR2HSV)[..., 1]
        paper_sat = float(np.median(sat))
        raw = ((np.abs(g - paper) > 22) | (sat > paper_sat + 40)).astype(np.uint8)
        open_k = int(round(1.5 * LINE_PPM)) | 1          # 펴진 사진의 펜 선(약 1mm)보다 큼
        thick = cv2.morphologyEx(raw, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))
        occ = cv2.dilate(thick, np.ones((18, 18), np.uint8)).astype(bool)
        clear = fine[~occ]
        if clear.size:
            noise = 1.4826 * float(np.median(np.abs(clear - np.median(clear))))   # 선이 많아도 흔들리지 않게
        else:
            noise = 1.0
        lines = ((fine > 4 * max(noise, 1e-4)) & ~occ).astype(np.uint8)
        n, lab, st, _ = cv2.connectedComponentsWithStats(lines, connectivity=8)
        keep = np.zeros(n, bool)
        keep[1:] = np.maximum(st[1:, 2], st[1:, 3]) >= LINE_MIN_MM * LINE_PPM
        lines = keep[lab].astype(np.uint8)
        dist = cv2.distanceTransform(1 - lines, cv2.DIST_L2, 5) / LINE_PPM
        self._maps = (dist, occ, lines.astype(bool))
        return self._maps

    def measure(self, index):
        """획 index에서 선이 보이는 경로 비율(0~1), 절반 넘게 가려졌으면 None."""
        dist, occ, _ = self._analyse()
        q = self.paths[index]
        x = np.clip(q[:, 0], 0, dist.shape[1] - 1)
        y = np.clip(q[:, 1], 0, dist.shape[0] - 1)
        visible = ~occ[y, x]
        if visible.mean() < 0.5:
            return None
        return float((dist[y, x][visible] <= LINE_TOL_MM).mean())

    def _near_paths(self):
        """계획한 경로(모든 획)에서 TICK_CLEAR_MM 안인 곳."""
        if self._near is None:
            drawn = np.full((self.size[1], self.size[0]), 255, np.uint8)
            for p in self.paths:
                cv2.polylines(drawn, [p.reshape(-1, 1, 2).astype(np.int32)], False, 0, 1)
            self._near = cv2.distanceTransform(drawn, cv2.DIST_L2, 5) <= TICK_CLEAR_MM * LINE_PPM
        return self._near

    def _tick_at(self, point, tangent):
        _, occ, lines = self._analyse()
        near = self._near_paths()
        h, w = lines.shape
        x, y = int(point[0]), int(point[1])
        if not (0 <= x < w and 0 <= y < h) or occ[y, x]:
            return None
        t = np.asarray(tangent, np.float64)
        nrm = float(np.linalg.norm(t))
        if nrm < 1e-6:
            return 0.0
        t = t / nrm
        n = np.array([-t[1], t[0]])          # 끝 접선에 수직인 방향만 세로 줄
        b = max(1, round(TICK_BAND_MM * LINE_PPM))
        reach, gap, clear = (round(v * LINE_PPM) for v in (TICK_REACH_MM, TICK_GAP_MM, TICK_CLEAR_MM))
        best = 0
        for d in (-1, 1):
            last = 0
            for k in range(1, reach + 1):
                center = np.array([x, y], np.float64) + d * k * n
                cx, cy = int(round(center[0])), int(round(center[1]))
                if not (0 <= cx < w and 0 <= cy < h) or occ[cy, cx]:
                    break
                hit = False
                for s in range(-b, b + 1):
                    p = center + s * t
                    xi, yi = int(round(p[0])), int(round(p[1]))
                    if 0 <= xi < w and 0 <= yi < h and lines[yi, xi] and not near[yi, xi]:
                        hit = True
                        break
                if hit:
                    last = k
                elif k > max(last, clear) + gap:
                    break
            best = max(best, last)
        return best / LINE_PPM

    def end_ticks(self, index):
        """획 index의 (시작점, 끝점)에서 접선에 수직으로 뻗은 줄 길이(mm, 없으면 0). 끝점이 가려졌으면 그 자리는 None."""
        q = self.paths[index]
        t0 = q[1] - q[0] if len(q) > 1 else np.array([1.0, 0.0])
        t1 = q[-1] - q[-2] if len(q) > 1 else np.array([1.0, 0.0])
        return self._tick_at(q[0], t0), self._tick_at(q[-1], t1)


def line_summary(coverage):
    """{획: 비율 또는 None} → (보임, 안 보임, 가려짐) 개수."""
    values = list(coverage.values())
    seen = sum(1 for v in values if v is not None and v >= LINE_SEEN)
    hidden = sum(1 for v in values if v is None)
    return seen, len(values) - seen - hidden, hidden


def tick_summary(ticks, coverage=None):
    """{획: (시작 mm|None, 끝 mm|None)} → {"start": (세로 줄, 잰 수), "end": (...)}.
    coverage를 주면 선이 보인 획의 끝점만 셈 (안 그려진 획에는 세로 줄도 없음)."""
    out = {}
    for k, name in enumerate(("start", "end")):
        vals = [t[k] for i, t in ticks.items() if t[k] is not None
                and (coverage is None or (coverage.get(i) is not None and coverage[i] >= LINE_SEEN))]
        out[name] = (sum(1 for v in vals if v >= TICK_MIN_MM), len(vals))
    return out


def save_snapshot(frame, registration, strokes_mm, regions, path):
    """끝난 뒤 기록: 왼쪽 카메라 사진, 오른쪽 같은 사진에 계획을 겹친 것."""
    overlaid = Overlay().draw(frame, registration, strokes_mm, regions)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".png", np.hstack([frame, overlaid]))
    if not ok:
        raise OSError("사진을 저장하지 못했습니다")
    path.write_bytes(buf.tobytes())        # 한글 경로도 그대로 (cv2.imwrite는 Windows 한글 경로에서 실패)
    return path
