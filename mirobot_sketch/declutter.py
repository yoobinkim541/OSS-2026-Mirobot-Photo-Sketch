"""실사 선화의 잔선 정리: 사람이 닮았다고 느끼는 선(눈·코·입, 얼굴 윤곽, 머리, 글자·문양)은 남기고
피부 위 주름과 짧고 흐린 잔선(자글자글한 결)은 지운다.

판단 근거
- 피부 위 주름: 얼굴 안쪽 타원에 있고, 선 양옆(띠 폭 band_px)이 거의 다 피부색이며, 눈·코·입 영역 밖이고,
  얼굴 폭의 45%보다 짧은 선. 피부색은 그 사진의 양 볼·코에서 잰다 (사진마다 다름).
  얼굴 윤곽·머리카락 경계는 한쪽이 피부가 아니므로 남는다.
- 짧고 흐린 잔선: 얼굴 밖에서 종이 long_mm보다 짧고, 사진 대비(Lab 기울기)가 그 사진 선들의
  contrast_pct 백분위보다 낮은 선. 글자·로고·문양은 짧아도 대비가 커서 남는다.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from . import sketch_pipeline as sp

WRINKLE = "피부 위 잔주름"
CLUTTER = "짧고 흐린 잔선"
BUDGET = "시간 예산으로 덜 중요한 선 생략"
MOUTH_TIDY = "입을 깔끔한 윤곽 하나로 다시 그림"
SPECK = "얼굴 속 점 같은 짧은 토막 (펜으로는 얼룩)"
FRONTAL_MAX = 0.15   # tidy_mouth: 코·입 중심이 두 눈 가운데에서 눈 사이 거리의 이 배수 안일 때만 (정면 얼굴)


@dataclass(frozen=True)
class Rules:
    long_mm: float = 3.0          # 얼굴 밖: 이보다 긴 선은 대비와 상관없이 남김 (머리 외곽·옷 윤곽)
    contrast_pct: float = 60.0    # 얼굴 밖 짧은 선: 이 백분위 이상 대비면 글자·문양으로 보고 남김
    contrast_floor: float = 60.0  # 위 문턱의 하한 (대비가 고르게 낮은 사진에서 잔선을 다 살리지 않게)
    contrast_cap: float = 150.0   # 위 문턱의 상한: 글자가 많은 사진에서 백분위가 올라가도 이만큼 뚜렷한 짧은 선은 남김
    min_px: float = 6.0           # 대비가 커도 이보다 짧으면 점이므로 버림
    face_ellipse: float = 0.45    # 주름을 볼 얼굴 안쪽 타원 (얼굴 상자 폭·높이의 배수, 반축)
    face_halo: float = 0.85       # 얼굴 둘레 타원: 그 안의 얼굴 밖 선(턱선·머리선·귀)은 강도를 올려도 기본 기준으로 봄
    face_long: float = 0.45       # 피부 위라도 얼굴 폭의 이 배수보다 긴 선은 남김
    near_landmark: float = 0.10   # 눈·코·입꼬리 랜드마크에서 얼굴 폭의 이 배수 안에 닿는 선은 남김 (아래 눈꺼풀·콧볼)
    skin_tol: float = 16.0        # 피부색과의 Lab 거리 (밝기 차는 반만 셈: 그늘진 피부도 피부)
    skin_frac: float = 0.8        # 선 양옆 띠에서 피부 비율이 이 이상이면 피부 위 선
    band_px: int = 3
    max_keep: int = 0             # 남길 획 수 상한 (0 = 제한 없음). 넘으면 얼굴 밖의 덜 중요한 획부터 뺌
    speck_mm: float = 1.25        # 얼굴 안: 종이 위 이보다 짧은 토막(펜 굵기 0.5mm의 2.5배)은 눈코입 근처라도 뺌
    hair_frac: float = 0.6        # 머리 영역에 이 비율 넘게 들어간 선화 획은 머리 결로 보고 대비와 상관없이 남김
    hair_min_mm: float = 2.0      # 머리 결 획의 최소 길이 = max(hair_min_mm, long_mm / hair_long_div). 펜 굵기의 4배보다 짧으면 결이 아니라 점·쉼표로 읽힘
    hair_long_div: float = 3.0    # 강도(long_mm)를 올리면 짧은 결부터 줄어듦


def contrast_map(color):
    """사진의 색·밝기 대비: Lab 세 채널 Sobel 크기의 최댓값 (선화 선이 경계에서 1~2px 비껴도 잡히게 5px 팽창)."""
    lab = cv2.cvtColor(cv2.GaussianBlur(color, (0, 0), 1.2), cv2.COLOR_BGR2LAB).astype(np.float32)
    mag = np.zeros(lab.shape[:2], np.float32)
    for c in range(3):
        mag = np.maximum(mag, cv2.magnitude(cv2.Sobel(lab[:, :, c], cv2.CV_32F, 1, 0),
                                            cv2.Sobel(lab[:, :, c], cv2.CV_32F, 0, 1)))
    return cv2.dilate(mag, np.ones((5, 5), np.uint8))


def _densify(q, step=1.0):
    q = np.asarray(q, np.float64)
    if len(q) < 2:
        return q
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    if t[-1] <= 0:
        return q[:1]
    s = np.arange(0.0, t[-1] + 1e-9, step)
    return np.column_stack([np.interp(s, t, q[:, 0]), np.interp(s, t, q[:, 1])])


def _in_ellipse(pts, center, axes):
    return ((pts[:, 0] - center[0]) / axes[0]) ** 2 + ((pts[:, 1] - center[1]) / axes[1]) ** 2 <= 1.0


def feature_zones(face):
    """눈(눈썹 포함)·코·입(아랫입술 포함) 영역 타원 [(중심, 반축)]. YuNet 랜드마크: 오른눈, 왼눈, 코, 오른 입꼬리, 왼 입꼬리."""
    w = float(face.box[2])
    lm = np.asarray(face.landmarks, np.float64)
    mouth = (lm[3] + lm[4]) / 2
    half = float(np.linalg.norm(lm[4] - lm[3])) / 2
    return [(lm[0] + (0, -0.06 * w), (0.17 * w, 0.15 * w)), (lm[1] + (0, -0.06 * w), (0.17 * w, 0.15 * w)),
            (lm[2] + (0, -0.05 * w), (0.11 * w, 0.13 * w)), (mouth + (0, 0.02 * w), (half + 0.08 * w, 0.14 * w))]


def _is_feature(pts, length, face, rules):
    """눈·코·입 영역에 절반 이상 들어가거나, 랜드마크에서 얼굴 폭의 near_landmark배 안에 닿거나(아래 눈꺼풀),
    얼굴 폭의 face_long배보다 긴 선."""
    fw = float(face.box[2])
    if length >= rules.face_long * fw:
        return True
    lm = np.asarray(face.landmarks, np.float64)
    if np.min(np.linalg.norm(pts[:, None, :] - lm[None, :, :], axis=2)) <= rules.near_landmark * fw:
        return True
    zone = np.zeros(len(pts), bool)
    for center, axes in feature_zones(face):
        zone |= _in_ellipse(pts, center, axes)
    return zone.mean() >= 0.5


def skin_mask(color, face, tol):
    """양 볼과 코 옆에서 잰 피부색에 가까운 픽셀."""
    lab = cv2.cvtColor(cv2.GaussianBlur(color, (0, 0), 1.5), cv2.COLOR_BGR2LAB).astype(np.float32)
    h, w = lab.shape[:2]
    lm = np.asarray(face.landmarks, np.float64)
    r = max(2, int(0.04 * float(face.box[2])))
    samples = []
    for x, y in ((lm[0] + lm[3]) / 2, (lm[1] + lm[4]) / 2, (lm[0] + lm[1] + 2 * lm[2]) / 4):
        x, y = int(np.clip(x, r, w - r - 1)), int(np.clip(y, r, h - r - 1))
        samples.append(lab[y - r:y + r + 1, x - r:x + r + 1].reshape(-1, 3))
    ref = np.median(np.vstack(samples), axis=0)
    d = np.sqrt(((lab[:, :, 0] - ref[0]) * 0.5) ** 2 + (lab[:, :, 1] - ref[1]) ** 2 + (lab[:, :, 2] - ref[2]) ** 2)
    return d <= tol


def classify(strokes, color, faces, px_per_mm, rules=Rules(), hair=None):
    """획마다 None(남김) 또는 버리는 이유. strokes는 작업 이미지 좌표 (x, y).
    hair: 머리카락 영역 마스크(없으면 None). 그 안의 선화 획은 사진 대비가 약해도 실제 머리 결이므로 짧은 토막만 빼고
    남기며, 선 개수 상한에서도 얼굴처럼 지킨다 (지우면 그 자리를 합성 결 선이 대신 채워 머리 모양이 달라짐)."""
    if not strokes:
        return []
    cmap = contrast_map(color)
    h, w = cmap.shape
    dense = [_densify(q) for q in strokes]
    contrast = np.array([float(np.median(cmap[np.clip(np.round(d[:, 1]).astype(int), 0, h - 1),
                                              np.clip(np.round(d[:, 0]).astype(int), 0, w - 1)]))
                         for d in dense])
    c_keep = min(rules.contrast_cap, max(rules.contrast_floor, float(np.percentile(contrast, rules.contrast_pct))))
    base = Rules()
    c_near = min(base.contrast_cap, max(base.contrast_floor, float(np.percentile(contrast, base.contrast_pct))))
    skins = [skin_mask(color, f, rules.skin_tol) for f in faces]
    kernel = np.ones((2 * rules.band_px + 1, 2 * rules.band_px + 1), np.uint8)
    rim = _silhouette_band(color)
    out, protected = [], []
    for q, pts, c in zip(strokes, dense, contrast):
        length = sp.polyline_length(np.asarray(q, np.float64))
        centers = [((float(f.box[0]) + float(f.box[2]) / 2, float(f.box[1]) + float(f.box[3]) / 2), f) for f in faces]
        in_face = [(f, skin) for (center, f), skin in zip(centers, skins)
                   if _in_ellipse(pts, center, (rules.face_ellipse * float(f.box[2]),
                                                rules.face_ellipse * 1.08 * float(f.box[3]))).mean() >= 0.5]
        near_face = any(_in_ellipse(pts, center, (rules.face_halo * float(f.box[2]),
                                                  rules.face_halo * 1.06 * float(f.box[3]))).mean() >= 0.5
                        for center, f in centers)
        if in_face and length < rules.speck_mm * px_per_mm:
            verdict = SPECK                          # 눈꺼풀 끝·콧구멍 둘레의 1mm 남짓한 토막: 펜으로는 점·얼룩
        elif in_face:                                # 겹친 얼굴은 하나라도 눈코입이면 남김
            if any(_is_feature(pts, length, f, rules) for f, _ in in_face):
                verdict = None
            else:
                band = np.zeros(cmap.shape, np.uint8)
                cv2.polylines(band, [np.round(np.asarray(q)).astype(np.int32).reshape(-1, 1, 2)], False, 255, 1)
                band = cv2.dilate(band, kernel) > 0
                on_skin = max(float(skin[band].mean()) for _, skin in in_face)
                verdict = WRINKLE if on_skin >= rules.skin_frac else None
        on_rim = rim is not None and _on_mask(rim, pts) >= 0.6
        on_hair = not in_face and hair is not None and _on_mask(hair, pts) >= rules.hair_frac
        if not in_face:   # 닫힌 고리도 같은 기준 (선화 hollow가 만든 작은 고리가 펜 경로로 남지 않게)
            # 얼굴 둘레(턱선·머리선·귀)와 인물 외곽선은 강도를 올려도 기본 기준으로: 인물 특징이 먼저 남게
            long_mm, c_min = (base.long_mm, c_near) if near_face or on_rim else (rules.long_mm, c_keep)
            keep = length >= long_mm * px_per_mm or (c >= c_min and length >= rules.min_px)
            if on_hair:   # 머리 결은 대비가 약해도 짧은 토막만 아니면 남김
                keep = keep or length >= max(rules.hair_min_mm, rules.long_mm / rules.hair_long_div) * px_per_mm
            verdict = None if keep else CLUTTER
        out.append(verdict)
        protected.append(2 if (in_face or near_face or on_rim) else 1 if on_hair else 0)
    if rules.max_keep:
        _apply_budget(out, protected, strokes, contrast, c_keep, rules.max_keep)
    return out


def tidy_mouth(strokes, faces, pen_px, bins=24):
    """정면 얼굴의 입 영역 선(입술 경계·이 사이 경계·입속 조각)을 입 모양 윤곽 하나로 바꾼다. 반환: (새 획 목록, 바꾼 획 목록).
    입꼬리 두 점을 잇는 축을 따라 칸마다 선들의 위·아래 끝을 모아 매끄럽게 이은 닫힌 곡선(웃으며 벌린 입)을 그린다.
    위아래 간격이 펜 굵기 1.5배보다 좁으면(다문 입) 가운데 한 줄만 그린다. 칸의 60% 이상에 선이 없으면 그대로 둔다.
    옆얼굴·고개를 돌린 얼굴(코나 입이 두 눈의 가운데에서 눈 사이 거리의 FRONTAL_MAX배 넘게 비낌)은 입꼬리 랜드마크가
    부정확하고 입술이 비대칭이라 곡선이 입술과 엇갈리므로 그대로 둔다 (마이크 옆얼굴 0.61, 기운 얼굴 0.26 / 정면 0.03~0.07)."""
    strokes = list(strokes)
    replaced = []
    for f in faces:
        lm = np.asarray(f.landmarks, np.float64)
        eye_d = float(np.linalg.norm(lm[1] - lm[0]))
        if eye_d <= 0:
            continue
        eye_axis, eye_mid = (lm[1] - lm[0]) / eye_d, (lm[0] + lm[1]) / 2
        if max(abs((lm[2] - eye_mid) @ eye_axis), abs(((lm[3] + lm[4]) / 2 - eye_mid) @ eye_axis)) > FRONTAL_MAX * eye_d:
            continue
        c0, c1 = lm[3], lm[4]
        width = float(np.linalg.norm(c1 - c0))
        if width < 4 * pen_px:
            continue
        u = (c1 - c0) / width
        n = np.array([-u[1], u[0]]) if u[0] >= 0 else np.array([u[1], -u[0]])
        if n[1] < 0:
            n = -n
        center, axes = feature_zones(f)[3]
        picked = [i for i, q in enumerate(strokes)
                  if sp.polyline_length(np.asarray(q, np.float64)) < 1.5 * width
                  and _in_ellipse(_densify(q), center, axes).mean() >= 0.6]
        if len(picked) < 2:
            continue
        pts = np.vstack([_densify(strokes[i]) for i in picked])
        t = (pts - c0) @ u / width
        v = (pts - c0) @ n
        edges = np.linspace(0.0, 1.0, bins + 1)
        top, bot, have = np.zeros(bins), np.zeros(bins), np.zeros(bins, bool)
        for b in range(bins):
            sel = (t >= edges[b]) & (t < edges[b + 1])
            if sel.any():
                top[b], bot[b], have[b] = v[sel].min(), v[sel].max(), True
        if have.mean() < 0.6:
            continue
        mid_t = ((edges[:-1] + edges[1:]) / 2)[have]
        # 입꼬리를 지나는 매끄러운 곡선 v = t(1-t)(a + b·t)로 위·아래 끝을 맞춤 (울퉁불퉁한 선화 조각을 따라가지 않게)
        basis = np.column_stack([mid_t * (1 - mid_t), mid_t * mid_t * (1 - mid_t)])
        coef_top = np.linalg.lstsq(basis, top[have], rcond=None)[0]
        coef_bot = np.linalg.lstsq(basis, bot[have], rcond=None)[0]
        ts = np.linspace(0.0, 1.0, 33)
        full = np.column_stack([ts * (1 - ts), ts * ts * (1 - ts)])
        top_c, bot_c = full @ coef_top, full @ coef_bot
        # 윗입술 위(인중)의 선이 섞이면 위 곡선이 입꼬리 선보다 크게 솟아 찡그린 입처럼 보임: 입 폭의 8%까지만 허용
        top_c = np.maximum(top_c, -0.08 * width * 4 * ts * (1 - ts))
        bot_c = np.maximum(bot_c, top_c)
        to_xy = lambda tt, vv: c0 + np.outer(tt * width, u) + np.outer(vv, n)
        added = []
        if float(np.max(bot_c - top_c)) < 1.5 * pen_px:
            added.append(to_xy(ts, (top_c + bot_c) / 2))
        else:
            added.append(np.vstack([to_xy(ts, top_c), to_xy(ts[::-1], bot_c[::-1])[1:]]))
            teeth = _teeth_line(t, v, ts, top_c, bot_c, pen_px, bins)
            if teeth is not None:
                added.append(to_xy(*teeth))
        replaced += [strokes[i] for i in picked]
        strokes = [q for i, q in enumerate(strokes) if i not in set(picked)] + added
    return strokes, replaced


def _teeth_line(t, v, ts, top_c, bot_c, pen_px, bins):
    """벌린 입 안에서 이의 아래 끝(윗니와 아랫입술·입속의 경계)을 한 줄로: 입 윤곽 사이에 있는 선화 점들을 칸마다
    가운데값으로 모아 2차 곡선을 맞추고, 입꼬리 쪽 18%는 비워 둔다. 이 하나하나의 경계는 그리지 않지만 이가 보이는
    웃음은 남는다. 안쪽 점이 가운데 칸의 40%를 못 채우면 None."""
    lo, hi = np.interp(t, ts, top_c) + 1.2 * pen_px, np.interp(t, ts, bot_c) - 1.2 * pen_px
    inner = (v > lo) & (v < hi) & (t > 0.15) & (t < 0.85)
    if not inner.any():
        return None
    edges = np.linspace(0.15, 0.85, max(4, int(bins * 0.7)) + 1)
    mids, vals = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        sel = inner & (t >= a) & (t < b)
        if sel.any():
            mids.append((a + b) / 2)
            vals.append(float(np.median(v[sel])))
    if len(mids) < 0.4 * (len(edges) - 1) or len(mids) < 3:
        return None
    coef = np.polyfit(np.asarray(mids) - 0.5, vals, 2)
    tt = np.linspace(0.18, 0.82, 17)
    vv = np.polyval(coef, tt - 0.5)
    span = np.interp(tt, ts, bot_c) - np.interp(tt, ts, top_c)
    if float(np.min(span)) < 3 * pen_px:
        return None
    vv = np.clip(vv, np.interp(tt, ts, top_c) + pen_px, np.interp(tt, ts, bot_c) - pen_px)
    return tt, vv


def _silhouette_band(color, width_px=4):
    """배경을 지운(흰 바탕) 사진이면 인물 외곽선 둘레 띠, 아니면 None. 머리 모양·어깨선은 예산에서 지킨다."""
    g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    subject = (g < 245).astype(np.uint8)
    if not 0.02 < subject.mean() < 0.95:
        return None
    subject = cv2.morphologyEx(subject, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    rim = cv2.morphologyEx(subject, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
    return cv2.dilate(rim, np.ones((2 * width_px + 1, 2 * width_px + 1), np.uint8)) > 0


def _on_mask(mask, pts):
    h, w = mask.shape
    return float(mask[np.clip(np.round(pts[:, 1]).astype(int), 0, h - 1),
                      np.clip(np.round(pts[:, 0]).astype(int), 0, w - 1)].mean())


def _apply_budget(out, protected, strokes, contrast, c_ref, max_keep):
    """남은 획이 max_keep개를 넘으면 덜 중요한 획부터 BUDGET으로 뺀다. protected: 0 = 보통, 1 = 머리 결(보통 획을 다
    뺀 뒤에도 넘을 때만, 짧은 것부터 뺌), 2 = 얼굴 안·얼굴 둘레·인물 외곽선(지킴).
    보통 획의 중요도 = 길이 × 대비 가중(0.5~1.5): 긴 윤곽과 뚜렷한 글자·문양이 먼저 남는다.
    머리 결까지 끝까지 지키면 머리가 긴 사진은 가장 강하게 정리해도 목표 시간을 넘었다(긴 생머리 사진 48.7분)."""
    kept = [i for i, why in enumerate(out) if why is None]
    extra = len(kept) - max_keep
    if extra <= 0:
        return
    length = {i: sp.polyline_length(np.asarray(strokes[i], np.float64)) for i in kept}
    loose = sorted((i for i in kept if protected[i] == 0),
                   key=lambda i: length[i] * (0.5 + min(1.0, contrast[i] / max(c_ref, 1.0))))
    hair = sorted((i for i in kept if protected[i] == 1), key=length.get)
    for i in (loose + hair)[:extra]:
        out[i] = BUDGET
