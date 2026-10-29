"""
사진 -> 로봇용 획(stroke) 추출 파이프라인
==========================================
GUI(gui.py)와 CLI(make_strokes.py)가 같이 쓰는 공용 모듈입니다.

단계:
  1. load_gray / remove_background : 이미지 읽기(+선택적 배경 제거), 크기 정규화
  2. compute_edges                 : Gaussian blur + Canny
  3. trace_strokes                 : 엣지를 1px 중심선으로 만든 뒤 픽셀 그래프를
                                     따라가며 "한 번씩만" 지나가는 획으로 분리
  3b. dedupe_strokes               : 펜 굵기보다 가까운 이중선(굵은 선의 양쪽 경계) 하나로
  3c. merge_strokes                : 끝점이 만나는 획을 이어 붙여 펜 올림 횟수 줄이기
  4. simplify_strokes              : approxPolyDP로 점 개수 줄이기
  5. order_strokes                 : 가까운 끝점부터 이어 펜업 이동 거리 줄이기
  6. stroke_metrics                : 획 수, 점 수, 펜다운/펜업 거리

왜 findContours를 기본으로 쓰지 않는가:
  findContours는 "영역의 경계"를 돌려줍니다. 두께 1px인 Canny 선에 쓰면
  선을 따라 갔다가 반대쪽 경계를 따라 되돌아오는 폐곡선이 나오므로, 로봇이
  같은 선을 두 번 그리게 됩니다(280px 직선 -> 윤곽 길이 559px로 실측).
  비교용으로 extract_strokes_contour()를 남겨두었습니다.
"""

import io
import math

import cv2
import numpy as np

DEFAULT_MAX_SIDE = 800  # 파라미터(px 단위)가 사진 해상도에 따라 달라지지 않도록 정규화

# 8-이웃 오프셋 (dy, dx)
_NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


# ---------------------------------------------------------------------------
# 1. 입력
# ---------------------------------------------------------------------------

def resize_max_side(img, max_side=DEFAULT_MAX_SIDE, upscale=True):
    """긴 변을 max_side로 맞춥니다(비율 유지). None/0이면 그대로 둡니다.

    작은 이미지도 키웁니다(upscale=True). px 단위 파라미터(최소 길이, 단순화 오차)가
    원본 해상도와 상관없이 같은 의미를 갖게 하고, 저해상도 원본의 계단 모양 선을
    매끄럽게 하기 위해서입니다 (267px 일러스트: 눈·안경이 뭉개짐 -> 800px로 키우면 보임).
    """
    if not max_side:
        return img
    h, w = img.shape[:2]
    scale = max_side / max(h, w)
    if scale == 1.0 or (scale > 1.0 and not upscale):
        return img
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC
    return cv2.resize(img, (round(w * scale), round(h * scale)), interpolation=interp)


def load_gray(img_path, max_side=DEFAULT_MAX_SIDE):
    # cv2.imread는 Windows의 한글 경로를 못 읽는 경우가 있어 바이트로 읽어 디코딩
    data = np.fromfile(str(img_path), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"이미지를 읽을 수 없습니다: {img_path}")
    raw = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if raw is not None and raw.ndim == 3 and raw.shape[2] == 4:
        # 투명 PNG: 그냥 흑백으로 읽으면 투명 부분이 검게 돼 선이 묻힘 -> 흰 배경에 합성 (load_color와 같게)
        return cv2.cvtColor(load_color(img_path, max_side), cv2.COLOR_BGR2GRAY)
    return resize_max_side(img, max_side)


def load_color(img_path, max_side=DEFAULT_MAX_SIDE):
    """화면 표시용 컬러 원본 (BGR). load_gray와 같은 크기로 맞춰 좌표가 일치합니다.
    투명 PNG는 흰 배경 위에 합성합니다 (그냥 읽으면 투명 부분이 검게 보임)."""
    data = np.fromfile(str(img_path), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError(f"이미지를 읽을 수 없습니다: {img_path}")
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.shape[2] == 4:
        a = img[:, :, 3:4].astype(np.float32) / 255.0
        img = (img[:, :, :3] * a + 255 * (1 - a)).astype(np.uint8)
    if img.dtype != np.uint8:   # 16비트 PNG 등
        img = cv2.convertScaleAbs(img, alpha=255.0 / max(1, int(img.max())))
    return resize_max_side(img, max_side)


def remove_background_bgr(img_path, max_side=DEFAULT_MAX_SIDE, cache_dir=None):
    """rembg로 배경을 제거하고 흰 배경 위에 합성한 컬러(BGR) 이미지를 반환합니다.
    rembg는 선택 기능이라 필요할 때만 import 합니다 (첫 실행 시 모델 다운로드).

    cache_dir를 주면 결과를 원본 파일 내용의 해시 이름으로 저장해 두고 재사용합니다
    (rembg는 이미지 한 장에 약 1분 — GUI를 다시 열 때마다 기다리지 않도록).
    색 차이(Lab) 선 검출도 쓸 수 있게 컬러로 저장합니다 (예전 흑백 캐시 rembg_*.png는 쓰지 않음)."""
    import hashlib
    from pathlib import Path

    with open(img_path, "rb") as f:
        data = f.read()
    cache_file = None
    if cache_dir:
        cache_file = Path(cache_dir) / f"rembg_bgr_{hashlib.sha1(data).hexdigest()[:16]}.png"
        if cache_file.exists():
            cached = cv2.imdecode(np.fromfile(str(cache_file), np.uint8), cv2.IMREAD_COLOR)
            if cached is not None:
                return resize_max_side(cached, max_side)

    from PIL import Image
    try:
        from rembg import remove
    except ImportError as e:
        raise RuntimeError(
            "배경 제거(rembg)가 설치되어 있지 않습니다. "
            "pip install \"mirobot-sketch[rembg]\" 로 설치하거나 배경 제거를 끄세요."
        ) from e

    fg = Image.open(io.BytesIO(remove(data))).convert("RGBA")
    white_bg = Image.new("RGBA", fg.size, (255, 255, 255, 255))
    rgb = np.array(Image.alpha_composite(white_bg, fg).convert("RGB"))
    composited = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if cache_file is not None:
        cache_file.parent.mkdir(parents=True, exist_ok=True)
        cv2.imencode(".png", composited)[1].tofile(str(cache_file))
    return resize_max_side(composited, max_side)


def remove_background(img_path, max_side=DEFAULT_MAX_SIDE, cache_dir=None):
    """remove_background_bgr의 흑백판 (명령줄 make_strokes용)."""
    return cv2.cvtColor(remove_background_bgr(img_path, max_side, cache_dir), cv2.COLOR_BGR2GRAY)


# ---------------------------------------------------------------------------
# 2. 엣지
# ---------------------------------------------------------------------------

def compute_edges(gray_img, canny_low=50, canny_high=150, blur_ksize=5):
    k = int(blur_ksize)
    k = k if k % 2 == 1 else k + 1
    blurred = cv2.GaussianBlur(gray_img, (k, k), 0) if k > 1 else gray_img
    return cv2.Canny(blurred, float(canny_low), float(canny_high))


def compute_dark_mask(gray_img, blur_ksize=5, threshold=None):
    """(선화 모드) 어두운 선 영역을 흰색으로 표시한 이진 이미지.

    Canny는 두께가 있는 선의 "양쪽 경계"를 잡아 한 선을 두 줄로 만듭니다.
    선화·일러스트처럼 어두운 선이 곧 그릴 선인 입력은, 선 영역 자체를 골라낸 뒤
    골격화하면 선의 중심선 한 줄을 얻습니다.
    threshold가 None이면 Otsu로 자동 결정합니다.
    """
    k = int(blur_ksize)
    k = k if k % 2 == 1 else k + 1
    blurred = cv2.GaussianBlur(gray_img, (k, k), 0) if k > 1 else gray_img
    if threshold is None:
        _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, mask = cv2.threshold(blurred, float(threshold), 255, cv2.THRESH_BINARY_INV)
    # 작은 점 잡음 제거
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


LAB_AB_GAIN = 2.0  # a·b 채널은 값 범위가 좁아(128 중심) 대비를 늘려 같은 임계값을 씀


def compute_edges_lab(bgr_img, canny_low=50, canny_high=150, blur_ksize=5):
    """색 차이 선 검출: Lab의 L·a·b 채널마다 Canny를 적용해 합칩니다.

    흑백 변환은 밝기가 비슷한 두 색(예: 주황 배경과 머리카락)의 경계를 지워 버립니다.
    a(초록↔빨강)·b(파랑↔노랑) 채널은 밝기가 같아도 색이 다르면 값이 달라 경계가 남습니다.
    """
    k = int(blur_ksize)
    k = k if k % 2 == 1 else k + 1
    lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
    out = np.zeros(bgr_img.shape[:2], np.uint8)
    for c in range(3):
        ch = lab[:, :, c]
        if c > 0:
            ch = np.clip(128 + (ch.astype(np.float32) - 128) * LAB_AB_GAIN, 0, 255).astype(np.uint8)
        if k > 1:
            ch = cv2.GaussianBlur(ch, (k, k), 0)
        out |= cv2.Canny(ch, float(canny_low), float(canny_high))
    return out


# ---------------------------------------------------------------------------
# 3. 획 추적
# ---------------------------------------------------------------------------

def skeletonize_edges(edges):
    """엣지 이미지를 1px 두께의 중심선(bool 배열)으로 만듭니다."""
    from skimage.morphology import skeletonize

    return skeletonize(edges > 0)


def _neighbor_list(skel):
    """골격 픽셀마다 8-이웃 골격 픽셀 목록을 만듭니다. 키는 (y, x)."""
    h, w = skel.shape
    ys, xs = np.nonzero(skel)
    nbrs = {}
    for y, x in zip(ys.tolist(), xs.tolist()):
        lst = []
        for dy, dx in _NEIGHBORS:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and skel[ny, nx]:
                lst.append((ny, nx))
        nbrs[(y, x)] = lst
    return nbrs


def trace_skeleton(skel):
    """1px 골격을 획 리스트로 바꿉니다.

    - 끝점(이웃 1개)·분기점(이웃 3개 이상)을 노드로 보고, 노드에서 노드까지를
      하나의 획으로 따라갑니다. 지나간 간선은 기록해 두 번 지나가지 않습니다.
    - 노드가 없는 순수한 고리(원 등)는 마지막에 따로 찾아 닫힌 획으로 만듭니다.

    반환: 각 획은 (N, 2) int 배열, 점 순서는 (x, y). 닫힌 획은 첫 점 == 마지막 점.
    """
    nbrs = _neighbor_list(skel)
    is_node = {p: len(n) != 2 for p, n in nbrs.items()}
    visited_edges = set()
    visited_px = set()
    strokes = []

    def edge_key(a, b):
        return (a, b) if a < b else (b, a)

    def walk(start, nxt):
        path = [start, nxt]
        visited_edges.add(edge_key(start, nxt))
        prev, cur = start, nxt
        while not is_node[cur] and cur != start:
            step = None
            for cand in nbrs[cur]:
                if cand != prev and edge_key(cur, cand) not in visited_edges:
                    step = cand
                    break
            if step is None:
                break
            visited_edges.add(edge_key(cur, step))
            path.append(step)
            prev, cur = cur, step
        return path

    # 노드에서 출발하는 획
    for p, is_n in is_node.items():
        if not is_n:
            continue
        visited_px.add(p)
        if not nbrs[p]:
            continue  # 고립 픽셀은 그릴 가치가 없음
        for q in nbrs[p]:
            if edge_key(p, q) in visited_edges:
                continue
            path = walk(p, q)
            visited_px.update(path)
            strokes.append(path)

    # 남은 고리
    for p in nbrs:
        if p in visited_px:
            continue
        unvisited = [q for q in nbrs[p] if edge_key(p, q) not in visited_edges]
        if not unvisited:
            continue
        path = walk(p, unvisited[0])
        if path[-1] != path[0]:
            # 시작점의 다른 이웃과 이어지면 고리를 닫아줌
            if path[0] in nbrs[path[-1]]:
                visited_edges.add(edge_key(path[-1], path[0]))
                path.append(path[0])
        visited_px.update(path)
        strokes.append(path)

    return [np.array([(x, y) for y, x in path], dtype=np.int32) for path in strokes]


def polyline_length(pts):
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def is_closed(pts):
    return len(pts) > 2 and tuple(pts[0]) == tuple(pts[-1])


def neighbor_count(skel):
    """골격 각 픽셀의 8-이웃 개수 (1=끝점, 2=선 중간, 3 이상=분기점)."""
    k = np.ones((3, 3), np.float32)
    k[1, 1] = 0
    return cv2.filter2D(skel.astype(np.float32), -1, k, borderType=cv2.BORDER_CONSTANT).astype(np.int32)


def trace_strokes(edges, min_length_px=15, spur_px=6, discarded=None):
    """엣지 이미지 -> 한 번씩만 지나가는 획 리스트.

    잡음 제거는 두 단계로 합니다.
      1. min_length_px: "서로 이어진 선 덩어리" 전체 픽셀 수가 이보다 작으면 제거.
         (획 조각 하나하나에 적용하면 글자처럼 분기점이 많은 선이 잘게 끊겨 사라짐)
      2. spur_px: 한쪽 끝이 허공이고 다른 끝이 분기점인 짧은 잔가지(골격화 부작용) 제거.
    분기점 사이의 1~2px짜리 연결 조각도 버립니다 (종이 위에서 0.5mm 미만).
    discarded가 list면 버린 조각을 (점 배열, 이유)로 추가합니다: "small"(작은 덩어리), "spur"(잔가지).
    """
    skel = skeletonize_edges(edges)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(skel.astype(np.uint8), connectivity=8)
    keep = stats[:, cv2.CC_STAT_AREA] >= min_length_px
    keep[0] = False  # 배경
    kept = keep[labels]
    if discarded is not None:
        for s in trace_skeleton(skel & ~kept):
            if len(s) >= 2:
                discarded.append((np.asarray(s), "small"))
    skel = kept

    deg = neighbor_count(skel)
    out = []
    for s in trace_skeleton(skel):
        length = polyline_length(s)
        if length < 2:
            continue
        if not is_closed(s):
            end_a = deg[s[0][1], s[0][0]] == 1
            end_b = deg[s[-1][1], s[-1][0]] == 1
            if end_a != end_b and length < spur_px:
                if discarded is not None:
                    discarded.append((np.asarray(s), "spur"))
                continue
        out.append(s)
    return out


def _densify(s):
    """획을 1px 간격의 연속 픽셀 점으로 채웁니다."""
    s = np.asarray(s, dtype=np.float64)
    out = [s[0]]
    for a, b in zip(s[:-1], s[1:]):
        n = max(1, int(np.ceil(np.hypot(*(b - a)))))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.round(np.array(out)).astype(np.int32)


def dedupe_strokes(strokes, shape, dist_px=4, min_keep_px=8, overlap_px=2, discarded=None):
    """이미 그린 선과 dist_px 안에서 겹치는 부분을 지웁니다 (이중선 제거).

    Canny는 굵은 선의 양쪽 경계를 각각 잡아 한 선을 두 줄로 만듭니다. 펜 굵기
    (약 0.5mm = 800px 이미지를 100mm로 그릴 때 4px)보다 가까운 두 선은 종이에서
    구별되지 않으므로 하나만 그리면 됩니다.
    - 긴 획부터, 획을 따라가며 지나간 점 주위(dist_px)를 "그린 영역"으로 표시한다.
      표시는 조금 뒤(lag)에서 따라오므로, 한 획이 양 끝에서 이어진 굵은 선의 두 경계
      (닫힌 획 하나)처럼 스스로 되돌아와 겹치는 경우도 잡는다.
    - 그린 영역 밖의 구간만 남긴다 (min_keep_px보다 짧은 조각은 버림, 경계에서
      overlap_px만큼 겹치게 남겨 끊김을 막음).
    엣지를 부풀려 합치는 방식은 가까운 선들이 세포 모양 그물로 뭉쳐 눈·입이 망가져
    쓰지 않았습니다 (LOG 참고).
    discarded가 list면 겹쳐서 빠진 구간을 (점 배열, "overlap")으로 추가합니다.
    """
    order = sorted(range(len(strokes)), key=lambda i: -polyline_length(strokes[i]))
    occ = np.zeros(shape[:2], np.uint8)
    h, w = occ.shape
    lag = 3 * dist_px + 2  # 바로 앞 점들은 자기 자신이므로 이만큼 뒤의 점부터 표시
    out = []
    for i in order:
        pts = _densify(strokes[i])
        xs, ys = np.clip(pts[:, 0], 0, w - 1), np.clip(pts[:, 1], 0, h - 1)
        n = len(pts)
        free = np.zeros(n, dtype=bool)
        for k in range(n):
            free[k] = occ[ys[k], xs[k]] == 0
            m = k - lag
            if m >= 0 and free[m]:
                cv2.circle(occ, (int(xs[m]), int(ys[m])), dist_px, 255, -1)
        for m in range(max(0, n - lag), n):
            if free[m]:
                cv2.circle(occ, (int(xs[m]), int(ys[m])), dist_px, 255, -1)
        k = 0
        while k < n:
            if not free[k]:
                k += 1
                continue
            j = k
            while j < n and free[j]:
                j += 1
            a, b = max(0, k - overlap_px), min(n, j + overlap_px)
            if b - a >= min_keep_px:
                out.append(pts[a:b])
            k = j
        if discarded is not None:
            k = 0
            while k < n:
                if free[k]:
                    k += 1
                    continue
                j = k
                while j < n and not free[j]:
                    j += 1
                if j - k >= 2:
                    discarded.append((pts[k:j], "overlap"))
                k = j
    return out


def drop_ink_twins(strokes, gray, gap_px, ink_max=200, turn=0.85, min_keep_px=8, discarded=None,
                   protect_xy=None, protect_px=0):
    """사이가 먹인 나란한 두 경계만 지운다. 굵은 선은 양쪽 가장자리가 따로 잡혀 같은 선을 두 번 긋게 된다.
    사이가 밝은 가까운 선(눈, 머리카락)은 gap_px 안이어도 남긴다. 긴 획을 먼저 남긴다."""
    from scipy.spatial import cKDTree

    gray = np.asarray(gray)
    h, w = gray.shape[:2]
    strokes = [np.asarray(s, np.float64) for s in strokes if len(np.asarray(s)) >= 2]
    if gap_px < 1 or not strokes:
        return strokes
    dens = [_densify(s).astype(np.float64) for s in strokes]
    order = sorted(range(len(dens)), key=lambda i: -polyline_length(dens[i]))
    step = max(1, int(gap_px // 2))
    samples, owners, tangents = [], [], []
    for i, pts in enumerate(dens):
        for k in range(0, len(pts), step):
            samples.append(pts[k])
            owners.append(i)
            a, b = max(0, k - step), min(len(pts) - 1, k + step)
            v = pts[b] - pts[a]
            n = np.hypot(*v)
            tangents.append(v / n if n > 1e-6 else np.zeros(2))
    tree = cKDTree(np.asarray(samples))
    accepted = set()
    out = []
    for i in order:
        pts = dens[i]
        twin = np.zeros(len(pts), bool)
        if accepted:
            near_face = None
            if protect_xy is not None and protect_px > 0:
                lm = np.asarray(protect_xy, np.float64)
                near_face = np.min(np.linalg.norm(pts[:, None, :] - lm[None, :, :], axis=2), axis=1) <= protect_px
            for k in range(len(pts)):
                if near_face is not None and near_face[k]:
                    continue                                    # 눈·코·입 근처는 붙어 있어도 둘 다 남김
                for j in tree.query_ball_point(pts[k], gap_px):
                    if owners[j] not in accepted:
                        continue
                    a, b = max(0, k - step), min(len(pts) - 1, k + step)
                    v = pts[b] - pts[a]
                    nrm = np.hypot(*v)
                    if nrm < 1e-6 or abs(float((v / nrm) @ tangents[j])) < turn:
                        continue
                    a_pt, b_pt = pts[k], samples[j]
                    steps = max(3, int(np.hypot(*(b_pt - a_pt))))
                    t = np.linspace(0, 1, steps)[1:-1]
                    xs = np.clip(np.round(a_pt[0] + (b_pt[0] - a_pt[0]) * t).astype(int), 0, w - 1)
                    ys = np.clip(np.round(a_pt[1] + (b_pt[1] - a_pt[1]) * t).astype(int), 0, h - 1)
                    if len(xs) and int(gray[ys, xs].min()) <= ink_max:
                        twin[k] = True
                        break
        keep = ~twin
        k = 0
        n = len(pts)
        while k < n:
            if not keep[k]:
                k += 1
                continue
            j = k
            while j < n and keep[j]:
                j += 1
            if j - k >= min_keep_px:
                out.append(pts[k:j])
            k = j
        if discarded is not None and twin.any():
            discarded.append((pts[twin], "overlap"))
        if keep.mean() >= 0.5:
            accepted.add(i)
    return out


def merge_strokes(strokes, join_px=2.0, tangent_px=5):
    """끝점이 만나는 획들을 펜을 떼지 않는 긴 획으로 이어 붙입니다.

    분기점에서 나뉜 획들은 같은 점(또는 join_px 이내)에서 만나므로, 이어서 그려도
    종이 위 결과는 같고 펜 올림·내림 횟수만 줄어듭니다.
    - 획 끝점을 join_px 이내끼리 하나의 노드로 묶고, 획을 노드 사이의 간선으로 본다.
    - 홀수 차수 노드(선이 끝나는 곳)부터 출발해, 쓰지 않은 간선을 따라 걷는다.
      갈래가 여러 개면 진행 방향이 가장 덜 꺾이는 쪽을 고른다.
    반환: 이어 붙인 획 리스트 ((N,2) int 배열, 점 순서 (x, y)).
    """
    from scipy.spatial import cKDTree

    strokes = [np.asarray(s, dtype=np.int32) for s in strokes if len(s) >= 2]
    n = len(strokes)
    if n < 2 or join_px <= 0:
        return strokes

    # 끝점 2n개를 가까운 것끼리 묶기 (union-find)
    ends = np.array([p for s in strokes for p in (s[0], s[-1])], dtype=np.float64)
    parent = list(range(2 * n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b in cKDTree(ends).query_pairs(join_px):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    node_of_end = [find(k) for k in range(2 * n)]  # 끝점 k = 2*i(앞), 2*i+1(뒤)

    incident = {}
    for k, node in enumerate(node_of_end):
        incident.setdefault(node, []).append(k)
    used = [False] * n

    def oriented(k):
        """끝점 k에서 출발하도록 방향을 맞춘 획."""
        s = strokes[k // 2]
        return s if k % 2 == 0 else s[::-1]

    def direction(pts, at_start):
        m = min(tangent_px, len(pts) - 1)
        v = (pts[m] - pts[0]) if at_start else (pts[-1] - pts[-1 - m])
        norm = np.hypot(*v)
        return v / norm if norm > 0 else np.zeros(2)

    def walk(node, heading):
        path = []
        while True:
            options = [k for k in incident[node] if not used[k // 2]]
            if not options:
                return path
            if heading is None:
                k = options[0]
            else:
                k = max(options, key=lambda kk: float(np.dot(heading, direction(oriented(kk), True))))
            used[k // 2] = True
            seg = oriented(k)
            path.append(seg if not path else seg[1:] if tuple(seg[0]) == tuple(path[-1][-1]) else seg)
            heading = direction(seg, False)
            node = node_of_end[k ^ 1]  # 반대쪽 끝점의 노드

    # 홀수 차수 노드에서 먼저 출발해야 획 수가 최소에 가까워짐
    starts = sorted(incident, key=lambda nd: len(incident[nd]) % 2 == 0)
    merged = []
    for node in starts:
        while any(not used[k // 2] for k in incident[node]):
            path = walk(node, None)
            if path:
                merged.append(np.vstack(path).astype(np.int32))
    return merged


def extract_strokes_contour(edges, min_length_px=15):
    # 참고: 여기의 min_length_px는 기존 코드와 같이 "윤곽 하나의 길이" 기준
    """(비교용) 기존 방식: findContours 윤곽을 그대로 획으로 사용. 선이 두 번 그려짐."""
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    out = []
    for cnt in contours:
        pts = cnt.reshape(-1, 2)
        if cv2.arcLength(cnt, closed=False) >= min_length_px:
            out.append(pts.astype(np.int32))
    return out


# ---------------------------------------------------------------------------
# 4. 단순화
# ---------------------------------------------------------------------------

def clip_to_ellipse(strokes, center, axes, inside=True):
    """타원 안(inside=True) 또는 밖 부분만 남김. 경계를 지나는 획은 경계에서 나눔 (점 사이를 1px 이하로 채워 판정).
    타원과 전혀 겹치지 않아 통째로 남는 획은 원래 배열을 그대로 돌려줌. 2점 미만 조각은 버림."""
    (cx, cy), (ax, ay) = center, axes
    out = []
    for s in strokes:
        a = np.asarray(s, dtype=np.float64)
        if len(a) < 2:
            continue
        lo, hi = a.min(0), a.max(0)
        clear = hi[0] < cx - ax or lo[0] > cx + ax or hi[1] < cy - ay or lo[1] > cy + ay
        if clear:                          # 타원 상자 밖: 전부 바깥
            if not inside:
                out.append(s)
            continue
        pts = [a[0]]
        for p, q in zip(a[:-1], a[1:]):
            n = max(1, int(np.ceil(np.hypot(*(q - p)))))
            pts.extend(p + (q - p) * (k / n) for k in range(1, n + 1))
        pts = np.array(pts)
        ins = ((pts[:, 0] - cx) / ax) ** 2 + ((pts[:, 1] - cy) / ay) ** 2 <= 1.0
        want = ins if inside else ~ins
        if want.all():
            out.append(s)
            continue
        k, n = 0, len(pts)
        while k < n:
            if not want[k]:
                k += 1
                continue
            j = k
            while j < n and want[j]:
                j += 1
            if j - k >= 2:
                out.append(pts[k:j])
            k = j
    return out


def _end_tangent(s, at_start, k=5):
    m = min(k, len(s) - 1)
    v = (s[m] - s[0]) if at_start else (s[-1] - s[-1 - m])
    n = np.hypot(*v)
    return v / n if n > 0 else np.zeros(2)


def join_gaps(strokes, gap=10.0, turn=0.85):
    """점선처럼 끊긴 선을 잇기: 끝점이 gap 이내이고 진행 방향이 거의 같으면(코사인 turn 이상) 한 획으로.
    직각으로 꺾이는 두 선은 잇지 않음."""
    from scipy.spatial import cKDTree

    strokes = [np.asarray(s, np.float64) for s in strokes if len(s) >= 2]
    alive = [True] * len(strokes)
    while True:
        ends, owners = [], []
        for i, s in enumerate(strokes):
            if alive[i] and not is_closed(s):
                ends += [s[0], s[-1]]
                owners += [(i, True), (i, False)]
        if len(ends) < 2:
            break
        best = None
        for a, b in cKDTree(np.asarray(ends)).query_pairs(gap):
            (i, ia), (j, jb) = owners[a], owners[b]
            if i == j:
                continue
            out_i = -_end_tangent(strokes[i], True) if ia else _end_tangent(strokes[i], False)   # i에서 나가는 방향
            into_j = _end_tangent(strokes[j], True) if jb else -_end_tangent(strokes[j], False)  # j로 들어가는 방향
            bridge = np.asarray(ends[b]) - np.asarray(ends[a])
            d = float(np.hypot(*bridge))
            if out_i @ into_j > turn and (d < 1.5 or (bridge / d) @ out_i > turn) and (best is None or d < best[0]):
                best = (d, i, ia, j, jb)
        if best is None:
            break
        _, i, ia, j, jb = best
        si = strokes[i][::-1] if ia else strokes[i]
        sj = strokes[j] if jb else strokes[j][::-1]
        strokes[i] = np.vstack([si, sj])
        alive[j] = False
    return [s for s, a in zip(strokes, alive) if a]



def smooth_strokes(strokes, sigma_px=2.0):
    """획을 따라 가우시안으로 매끄럽게 (1px 계단·흔들림 제거). 열린 획은 끝점 고정, 닫힌 획은 고리째.
    뼈대에서 따라간 촘촘한 획(점 간격 1px)에 씁니다. sigma_px=0이면 그대로."""
    if sigma_px <= 0:
        return list(strokes)
    out = []
    for s in strokes:
        s = np.asarray(s)
        closed = is_closed(s)
        body = s[:-1] if closed else s
        n = len(body)
        r = min(int(np.ceil(3 * sigma_px)), n - 1)
        if n < 3 or r < 1:
            out.append(s)
            continue
        k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_px) ** 2)
        k /= k.sum()
        body = body.astype(np.float64)
        if closed:
            pad = np.take(body, np.arange(-r, n + r), axis=0, mode="wrap")
        else:   # 끝점을 중심으로 뒤집어 덧대면 끝 부분이 안쪽으로 끌려가지 않음
            pad = np.vstack([2 * body[0] - body[r:0:-1], body, 2 * body[-1] - body[-2:-r - 2:-1]])
        sm = np.stack([np.convolve(pad[:, i], k, "valid") for i in range(2)], 1)
        if closed:
            sm = np.vstack([sm, sm[:1]])
        else:
            sm[0], sm[-1] = body[0], body[-1]
        out.append(sm)
    return out


def simplify_strokes(strokes, epsilon_px=2.0):
    """approxPolyDP로 각 획의 점을 줄입니다. 원래 선에서 epsilon_px 이상 벗어나지 않음.
    정수 획은 정수로, 스무딩한 소수 획은 소수로 돌려줍니다."""
    out = []
    for s in strokes:
        s = np.asarray(s)
        exact = np.issubdtype(s.dtype, np.integer)
        s = s.astype(np.int32 if exact else np.float32)
        if len(s) < 3 or epsilon_px <= 0:
            out.append(s if exact else s.astype(np.float64))
            continue
        if is_closed(s):
            approx = cv2.approxPolyDP(s[:-1].reshape(-1, 1, 2), epsilon_px, closed=True).reshape(-1, 2)
            approx = np.vstack([approx, approx[:1]])
        else:
            approx = cv2.approxPolyDP(s.reshape(-1, 1, 2), epsilon_px, closed=False).reshape(-1, 2)
        out.append(approx.astype(np.int32 if exact else np.float64))
    return out


def _cut_corners(p, closed, max_dev_px):
    """꼭짓점마다 양옆으로 c만큼 떨어진 두 점으로 바꿈 (Chaikin처럼 짧은 쪽 선분의 1/4).
    c는 원래 꼭짓점에서 max_dev_px 이상 벗어나지 않게 줄임: 벗어남 = c·sin(꺾임/2).
    그래서 완만한 곡선은 둥글어지고, 머리카락 끝처럼 되돌아가는 뾰족한 끝은 거의 그대로 남음."""
    body = p[:-1] if closed else p
    n = len(body)
    idx = range(n) if closed else range(1, n - 1)
    out = [] if closed else [body[0]]
    for i in idx:
        v, a, b = body[i], body[i - 1], body[(i + 1) % n]
        da, db = a - v, b - v
        la, lb = np.hypot(*da), np.hypot(*db)
        if la < 1e-9 or lb < 1e-9:
            out.append(v)
            continue
        cos_in = np.clip(np.dot(da, db) / (la * lb), -1.0, 1.0)   # 안쪽 각의 cos (곧으면 -1)
        sin_half_turn = np.sqrt((1.0 + cos_in) / 2.0)             # sin(꺾임/2)
        if sin_half_turn < 1e-3:                                  # 거의 곧음: 점 그대로
            out.append(v)
            continue
        c = min(min(la, lb) / 4.0, max_dev_px / sin_half_turn)
        out.extend([v + da / la * c, v + db / lb * c])
    if closed:
        out.append(out[0])
    else:
        out.append(body[-1])
    return np.array(out)


def round_corners(strokes, iterations=1, max_dev_px=1.0):
    """모서리 둥글리기 (Chaikin 방식, 1회에 점 약 2배). 한 번에 꼭짓점에서 max_dev_px 이상 벗어나지 않음.
    열린 획은 끝점 고정, 닫힌 획은 고리째. iterations=0이면 그대로."""
    out = []
    for s in strokes:
        s = np.asarray(s)
        if iterations <= 0 or len(s) < 3:
            out.append(s)
            continue
        closed = is_closed(s)
        p = s.astype(np.float64)
        for _ in range(int(iterations)):
            p = _cut_corners(p, closed, max_dev_px)
        out.append(p)
    return out


# ---------------------------------------------------------------------------
# 5. 순서 정하기
# ---------------------------------------------------------------------------

def _rotate_closed(s, idx):
    """닫힌 획의 시작점을 idx 번째 점으로 바꿉니다."""
    body = s[:-1]
    body = np.roll(body, -idx, axis=0)
    return np.vstack([body, body[:1]])


def order_strokes(strokes, start=None):
    """가장 가까운 시작점부터 이어 그리는 탐욕 순서.

    - 열린 획은 앞/뒤 중 가까운 끝에서 시작하도록 뒤집을 수 있습니다.
    - 닫힌 획은 가장 가까운 꼭짓점에서 시작하도록 돌립니다.
    start: 펜의 시작 위치 (x, y). None이면 모든 획의 경계 상자 중심(=종이 중심에 대응).
    """
    remaining = [np.asarray(s, dtype=np.float64) for s in strokes if len(s) >= 2]
    if not remaining:
        return []
    if start is None:
        allp = np.vstack(remaining)
        start = (allp.min(axis=0) + allp.max(axis=0)) / 2.0
    pos = np.asarray(start, dtype=np.float64)

    # 모든 획의 "시작 후보점"을 한 배열에 모아 두고, 매 단계 numpy로 한 번에 거리를 잰다.
    # (획마다 Python 반복하던 방식은 획 2700개에 80초가 걸렸음)
    # 후보: 열린 획 = 앞 끝(mode 0) / 뒤 끝(mode 1), 닫힌 획 = 모든 꼭짓점(mode 2, j = 꼭짓점 번호)
    cand_pts, cand_owner, cand_mode, cand_j = [], [], [], []
    for i, s in enumerate(remaining):
        if is_closed(s):
            body = s[:-1]
            cand_pts.append(body)
            cand_owner.append(np.full(len(body), i))
            cand_mode.append(np.full(len(body), 2))
            cand_j.append(np.arange(len(body)))
        else:
            cand_pts.append(np.stack([s[0], s[-1]]))
            cand_owner.append(np.array([i, i]))
            cand_mode.append(np.array([0, 1]))
            cand_j.append(np.array([0, 0]))
    cand_pts = np.vstack(cand_pts)
    cand_owner = np.concatenate(cand_owner)
    cand_mode = np.concatenate(cand_mode)
    cand_j = np.concatenate(cand_j)
    alive = np.ones(len(cand_pts), dtype=bool)

    ordered = []
    for _ in range(len(remaining)):
        d = np.einsum("ij,ij->i", cand_pts - pos, cand_pts - pos)
        d[~alive] = np.inf
        k = int(np.argmin(d))
        i, mode, j = int(cand_owner[k]), int(cand_mode[k]), int(cand_j[k])
        s = remaining[i]
        if mode == 1:
            s = s[::-1]
        elif mode == 2:
            s = _rotate_closed(s, j)
        ordered.append(s)
        alive[cand_owner == i] = False
        pos = s[-1]
    return ordered


# ---------------------------------------------------------------------------
# 6. 지표
# ---------------------------------------------------------------------------

def stroke_metrics(strokes, start=None):
    """획 수, 점 수, 펜다운(그리는) 길이, 펜업(이동) 길이. 단위는 입력 좌표 단위."""
    strokes = [np.asarray(s, dtype=np.float64) for s in strokes if len(s) >= 1]
    pen_down = sum(polyline_length(s) for s in strokes)
    pen_up = 0.0
    if strokes:
        pos = strokes[0][0] if start is None else np.asarray(start, dtype=np.float64)
        for s in strokes:
            pen_up += float(np.linalg.norm(s[0] - pos))
            pos = s[-1]
        if start is not None:
            pen_up += float(np.linalg.norm(pos - np.asarray(start, dtype=np.float64)))
    return {
        "stroke_count": len(strokes),
        "point_count": int(sum(len(s) for s in strokes)),
        "pen_down_length": round(pen_down, 2),
        "pen_up_length": round(pen_up, 2),
    }


# ---------------------------------------------------------------------------
# 전체 실행 + 미리보기
# ---------------------------------------------------------------------------

def run_pipeline(gray_img, canny_low=50, canny_high=150, blur_ksize=5,
                 min_length_px=15, epsilon_px=2.0, method="skeleton", line_source="canny",
                 median_ksize=0, merge_join_px=4.0, dedupe_px=4):
    """회색조 이미지 -> (선 후보 이미지, 정렬된 획 리스트).

    line_source:  "canny"(사진 윤곽) | "dark"(선화: 어두운 선의 중심선)
    method:       "skeleton"(한 번씩만 지나감) | "contour"(비교용 기존 방식)
    median_ksize: 0이 아니면 먼저 미디언 블러. 만화 스크린톤(망점)처럼 작은 무늬를
                  지우고 선의 경계는 남긴다 (만화 1장: 2726획 -> 175획, 7px 기준).
    dedupe_px:     0이 아니면 이 거리 안에서 겹치는 이중선을 하나만 남긴다. 기본 4px은
                   Canny 블러(5px) 때문에 2px 선의 양쪽 경계가 4px 떨어져 잡히는 것에 맞춤.
    merge_join_px: 0이 아니면 끝점이 이 거리 안에서 만나는 획을 이어 붙여 펜 올림을
                   줄인다. 기본 4px은 100mm로 그릴 때 약 0.5mm(펜 굵기 수준).
    """
    if method == "contour":
        if median_ksize and median_ksize > 1:
            k = int(median_ksize)
            gray_img = cv2.medianBlur(gray_img, k if k % 2 == 1 else k + 1)
        edges = (compute_dark_mask(gray_img, blur_ksize) if line_source == "dark"
                 else compute_edges(gray_img, canny_low, canny_high, blur_ksize))
        raw = extract_strokes_contour(edges, min_length_px)
        return edges, order_strokes(simplify_strokes(raw, epsilon_px))

    from .stages import Pipeline, default_params   # stages가 이 모듈을 import하므로 함수 안에서

    params = {**default_params(), "median_ksize": median_ksize or 0, "blur_ksize": blur_ksize,
              "edge_mode": "dark" if line_source == "dark" else "luma",
              "canny_low": canny_low, "canny_high": canny_high, "min_length_px": min_length_px,
              "spur_px": 6, "dedupe_px": dedupe_px or 0, "merge_join_px": merge_join_px or 0,
              "epsilon_px": epsilon_px}
    color = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
    out = Pipeline().run({"gray": gray_img, "color": color}, None, params)["simplify"]
    return out["edges"], order_strokes(out["strokes"])


def draw_strokes_image(strokes, shape, thickness=1):
    canvas = np.zeros(shape[:2], dtype=np.uint8)
    for s in strokes:
        pts = np.round(np.asarray(s)).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [pts], isClosed=False, color=255, thickness=thickness)
    return canvas


def draw_order_preview(strokes, shape):
    """그리는 순서 확인용 컬러 미리보기: 획은 순서대로 파랑->빨강, 펜업 이동은 회색 점선."""
    h, w = shape[:2]
    canvas = np.full((h, w, 3), 255, dtype=np.uint8)
    n = max(len(strokes), 1)
    prev_end = None
    for i, s in enumerate(strokes):
        pts = np.round(np.asarray(s)).astype(np.int32)
        if prev_end is not None:
            _dashed_line(canvas, tuple(prev_end), tuple(pts[0]), (180, 180, 180))
        t = i / max(n - 1, 1)
        color = (int(255 * (1 - t)), 0, int(255 * t))  # BGR
        cv2.polylines(canvas, [pts.reshape(-1, 1, 2)], False, color, 1, cv2.LINE_AA)
        prev_end = pts[-1]
    return canvas


def _dashed_line(img, p0, p1, color, dash=6):
    length = math.dist(p0, p1)
    if length < 1:
        return
    steps = int(length // dash)
    for k in range(0, steps, 2):
        a = (int(p0[0] + (p1[0] - p0[0]) * k / steps), int(p0[1] + (p1[1] - p0[1]) * k / steps))
        b = (int(p0[0] + (p1[0] - p0[0]) * (k + 1) / steps), int(p0[1] + (p1[1] - p0[1]) * (k + 1) / steps))
        cv2.line(img, a, b, color, 1)
