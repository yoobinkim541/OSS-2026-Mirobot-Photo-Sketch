"""
단계별 선 추출 파이프라인
=========================
원본 → 전처리 → 선 검출 → 뼈대·획 → 겹침 제거 → 이어 붙이기 → 얼굴 세밀 → 스무딩·단순화
각 단계는 조절 항목(ParamSpec), 계산(run), 미리보기(preview)를 가집니다. GUI의 조절 칸과
에이전트 도구 설명은 이 정의에서 만들어집니다. Pipeline은 단계별 결과를 캐시해 두고
설정이 바뀐 첫 단계부터만 다시 계산합니다. (편집·순서·종이 배치는 session.py가 이어서 처리)
"""

import dataclasses
import math
from dataclasses import dataclass
from typing import Callable

import cv2
import numpy as np

from . import declutter
from . import face_mesh
from . import faces as fc
from . import limits
from . import presets
from . import sketch_pipeline as sp
from . import tone


@dataclass(frozen=True)
class ParamSpec:
    key: str
    label: str
    kind: str                 # "int" | "float" | "bool" | "choice"
    default: object
    lo: float = 0
    hi: float = 0
    step: float = 1
    choices: tuple = ()       # choice: ((값, 화면 이름), ...)
    help: str = ""
    odd: bool = False         # 커널 크기처럼 홀수만

    def clamp(self, value):
        """범위 밖이면 잘라서, 종류에 맞게 바꿔 돌려줌. 선택지가 틀리면 ValueError."""
        if self.kind == "bool":
            return _clamp_bool(self.key, value)
        if self.kind == "choice":
            keys = [c[0] for c in self.choices]
            if value not in keys:
                raise ValueError(f"{self.key}는 {', '.join(keys)} 중 하나여야 합니다")
            return value
        try:
            v = float(value)
        except (TypeError, ValueError) as e:
            raise ValueError(str(e)) from None
        if not math.isfinite(v):
            _session_error(f"{self.key}는 유한한 숫자여야 합니다")
        v = min(max(v, self.lo), self.hi)
        if self.kind == "int":
            v = int(round(v))
            if self.odd and v % 2 == 0:
                v = v + 1 if v < self.hi else v - 1
            return v
        return round(round(v / self.step) * self.step, 4)


_BOOL_TRUE = {"true", "yes", "1"}
_BOOL_FALSE = {"false", "no", "0"}


def _session_error(msg):
    from .session import SessionError
    raise SessionError(msg)


def _clamp_bool(key, value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        word = value.strip().lower()
        if word in _BOOL_TRUE:
            return True
        if word in _BOOL_FALSE:
            return False
    _session_error(f"{key}는 true/false, yes/no, 1/0 중 하나여야 합니다")


@dataclass(frozen=True)
class Stage:
    id: str
    label: str
    params: tuple
    run: Callable = None      # run(prev: dict, p: dict) -> dict (prev를 이어받아 새 값을 더함)
    preview: Callable = None  # preview(out: dict) -> BGR 이미지 (입력과 같은 크기)
    desc: str = ""            # 이 단계가 하는 일 (한 줄, 화면 안내용)
    deps: tuple = ()          # 다른 단계의 설정 중 이 단계 계산에도 쓰는 것 (캐시 키에 포함)


class StaleRun(Exception):
    """더 새로운 설정이 들어와 이 계산이 필요 없어짐."""


# ---------------------------------------------------------------- 그리기 도우미
def _odd(k):
    k = int(k)
    return k if k % 2 == 1 else k + 1


def _bgr(gray):
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def draw_strokes_colored(strokes, shape, faded=()):
    """획마다 다른 색. faded(버린 조각)는 연회색으로 먼저 그림."""
    img = np.full((*shape[:2], 3), 255, np.uint8)
    for s, _ in faded:
        cv2.polylines(img, [np.round(s).astype(np.int32).reshape(-1, 1, 2)], False, (205, 205, 205), 1)
    for i, s in enumerate(strokes):
        hue = (i * 37) % 180
        c = cv2.cvtColor(np.uint8([[[hue, 200, 170]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
        cv2.polylines(img, [np.round(s).astype(np.int32).reshape(-1, 1, 2)], False, c, 1, cv2.LINE_AA)
    return img


# ---------------------------------------------------------------- 단계 계산
def _run_source(prev, p):
    return dict(prev)   # 배경 제거(rembg)는 세션이 입력 이미지를 고를 때 반영


def _run_prep(prev, p):
    gray, color = prev["gray"], prev["color"]
    if p["median_ksize"] > 1:
        k = _odd(p["median_ksize"])
        gray, color = cv2.medianBlur(gray, k), cv2.medianBlur(color, k)
    if p["blur_ksize"] > 1:
        k = _odd(p["blur_ksize"])
        gray, color = cv2.GaussianBlur(gray, (k, k), 0), cv2.GaussianBlur(color, (k, k), 0)
    return {**prev, "prep_gray": gray, "prep_color": color}


def _run_edges(prev, p):
    # 블러는 전처리에서 했으므로 여기서는 1(끔)
    color, gray = prev["prep_color"], prev["prep_gray"]
    missing = {}
    if p.get("lineart"):
        from . import lineart
        try:
            art, e = lineart.ink_mask(prev["color"])
        except lineart.LineartUnavailable as why:     # 오프라인 첫 실행 등: 기존 선 검출로 그리고 이유를 알림
            missing = {"lineart_missing": f"{why} 선화 대신 기본 선 검출로 그렸습니다."}
        else:
            # 선화는 따로 둔다: 명암 빗금처럼 사진의 밝기가 필요한 뒤 단계는 prep_gray(사진)를 그대로 써야 함
            return {**prev, "lineart_art": art, "edges": e, "lineart_used": True}
    if p["edge_mode"] == "dark":
        e = sp.compute_dark_mask(gray, 1)
    elif p["edge_mode"] == "lab":
        e = sp.compute_edges_lab(color, p["canny_low"], p["canny_high"], 1)
    else:
        e = sp.compute_edges(gray, p["canny_low"], p["canny_high"], 1)
    return {**prev, "edges": e, **missing}


def _run_trace(prev, p):
    disc = []
    st = sp.trace_strokes(prev["edges"], p["min_length_px"], p["spur_px"], discarded=disc)
    return {**prev, "strokes": st, "discarded_trace": disc}


def _run_dedupe(prev, p):
    if not p["dedupe_px"]:
        return {**prev, "discarded_dedupe": []}
    disc = []
    st = sp.dedupe_strokes(prev["strokes"], prev["edges"].shape, int(p["dedupe_px"]), discarded=disc)
    gray = prev.get("prep_gray", prev.get("gray"))
    # 굵은 선 양쪽 경계(Canny)만 지우는 단계. 선화는 이미 한 줄짜리 선이고, 두꺼운 먹은 hollow가 일부러 테두리 두 줄로
    # 만든 것이라 여기서 지우면 머리카락·눈꺼풀·입술처럼 가까운 선이 서로 쌍으로 사라진다 (2026-11-13 측정: 선화 재현율 0.95 → 0.41).
    if gray is not None and prev.get("region") is not None and not prev.get("lineart_used"):
        # 두 경계 사이 거리는 사진 속 선 굵기(px)라 종이 크기와 상관없다: 작업 이미지 긴 변의 비율로 둔다.
        # (종이 크기에 묶으면 크기만 바꿔도 이 단계부터 다시 계산돼 편집 번호·제안이 사라진다)
        gap = INK_TWIN_FRAC * max(gray.shape[:2])
        faces = prev.get("faces") or []
        protect = np.vstack([np.asarray(f.landmarks, np.float64) for f in faces]) if faces else None
        protect_px = 0.14 * max(float(f.box[2]) for f in faces) if faces else 0
        st = sp.drop_ink_twins(st, gray, gap, discarded=disc, protect_xy=protect, protect_px=protect_px)
    return {**prev, "strokes": st, "discarded_dedupe": disc}


LINEART_GAP_PX = 12.0      # 선화의 점선처럼 끊긴 선을 이을 끝점 거리 (작업 이미지 px)
LINEART_MIN_PX = 6.0       # 이었는데도 이보다 짧은 열린 토막은 버림 (펜 굵기 4px의 1.5배; 14px이면 선화 재현율이 크게 떨어짐)
INK_TWIN_FRAC = 0.02       # 굵은 선 양쪽 경계를 하나로 보는 거리: 작업 이미지 긴 변의 2% (800px면 16px, 100mm 그림의 2mm)


def _run_merge(prev, p):
    strokes = prev["strokes"]
    if p["merge_join_px"]:
        strokes = sp.merge_strokes(strokes, p["merge_join_px"])
    if prev.get("lineart_used"):                    # 선화를 그대로 따라 그림: 점선처럼 끊긴 선만 잇고 짧은 토막은 버림
        strokes = sp.join_gaps(strokes, LINEART_GAP_PX)
        near = _near_features(strokes, prev.get("faces") or [])
        strokes = [q for q, keep in zip(strokes, near)
                   if keep or sp.is_closed(q) or sp.polyline_length(q) >= LINEART_MIN_PX]
        # 짧은 획(눈동자 고리 등)은 세게 펴면 점으로 오그라듦: 길이에 맞춰 약하게
        strokes = [sp.smooth_strokes([q], min(3.0, sp.polyline_length(q) / 20.0))[0] for q in strokes]
    return {**prev, "strokes": strokes}


def _near_features(strokes, faces):
    """획마다 눈·코·입 랜드마크 가까이(얼굴 폭의 12% 이내)에 닿는지. 작은 눈·콧구멍은 짧아도 남기기 위함."""
    marks = [(np.asarray(f.landmarks, np.float64), 0.12 * float(f.box[2])) for f in faces]
    out = []
    for q in strokes:
        q = np.asarray(q, np.float64)
        out.append(any(np.min(np.linalg.norm(q[:, None, :] - lm[None, :, :], axis=2)) <= r for lm, r in marks))
    return out


FACE_REASON = "얼굴 세밀 처리로 교체"
TEXT_REASON = "사진 속 글자 (0.5mm 펜으로는 뭉개져 읽히지 않음)"
GREEN = (40, 160, 40)
def _px_per_mm(prev, p):
    """작업 이미지에서 종이 1 mm가 몇 px인지. 구도로 줄어든 실제 크기를 쓴다."""
    h, w = prev["gray"].shape[:2] if "gray" in prev else prev["prep_gray"].shape[:2]
    region = prev.get("region")
    long_mm = limits.effective_long_mm(region, p["box_mm"], w, h) if region is not None and "box_mm" in p else float(p.get("box_mm", 100))
    return max(h, w) / max(long_mm, 1.0)


def _face_strokes(crop, face, p, pen_px):
    """얼굴 조각(원본 해상도)에서 다시 찾은 획을 작업 좌표로. 펜 굵기보다 촘촘한 선은 합치고,
    펜 굵기 3배보다 짧은 선은 눈·코·입 근처가 아니면 버림 (계산은 촘촘한 조각 좌표에서)."""
    img, s, origin = crop["img"], crop["scale"], crop["origin"]
    q = {**p, "canny_low": p["canny_low"] * p["face_sensitivity"], "canny_high": p["canny_high"] * p["face_sensitivity"]}
    sub = {"gray": cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), "color": img}
    for run in (_run_prep, _run_edges, _run_trace, _run_dedupe, _run_merge):
        sub = run(sub, q)
    if not sub["strokes"]:
        return []
    pen = pen_px * s
    lm = (face.landmarks - origin) * s
    keep_r = 0.2 * face.box[2] * s
    out = []
    for t in sp.dedupe_strokes(sub["strokes"], img.shape, max(1, int(round(pen)))):
        t = np.asarray(t, np.float64)
        near = np.min(np.linalg.norm(t[:, None, :] - lm[None, :, :], axis=2)) <= keep_r
        if near or sp.polyline_length(t) >= 3 * pen:
            out.append(t / s + origin)
    return out


def _face_pen_px(prev, p):
    """펜 굵기(mm)를 작업 이미지 px로. 큰 세로 그림은 영역에 맞춰 요청(box_mm)보다 작게 그려지므로 실제 크기로 환산."""
    h, w = prev["gray"].shape[:2]
    region = prev.get("region")
    long_mm = limits.effective_long_mm(region, p["box_mm"], w, h) if region is not None else p["box_mm"]
    return p["pen_mm"] / (long_mm / max(h, w))


def _run_face(prev, p):
    found = prev.get("faces") or []
    if not found or not p["face_detail"]:
        return {**prev, "discarded_face": [], "faces_used": 0, "uses_deps": False}
    pen_px = _face_pen_px(prev, p)
    strokes, gone = list(prev["strokes"]), []
    for face, crop in zip(found, prev["face_crops"]):
        center, axes = fc.ellipse_of(face)
        fine = _face_strokes(crop, face, p, pen_px)
        gone += [(q, FACE_REASON) for q in sp.clip_to_ellipse(strokes, center, axes, inside=True)]
        strokes = (sp.clip_to_ellipse(strokes, center, axes, inside=False)
                   + sp.clip_to_ellipse(fine, center, axes, inside=True))
    return {**prev, "strokes": strokes, "discarded_face": gone, "faces_used": len(found)}


def _run_clutter(prev, p):
    """실사 선화의 잔선 정리 (declutter.py). 얼굴 세밀 처리와 나눠, 강도만 바꿀 때 얼굴 조각의 선화를
    다시 돌리지 않는다 (자동 세팅이 강도를 여러 번 바꿔 본다)."""
    if not (prev.get("lineart_used") and p["declutter"]):
        return {**prev, "discarded_clutter": [], "uses_deps": False}
    rules = declutter.Rules(long_mm=float(p["clutter_mm"]), contrast_pct=float(p["clutter_pct"]),
                            max_keep=int(p["clutter_max"]))
    strokes = list(prev["strokes"])
    faces = prev.get("faces") or []
    pen_px = _face_pen_px(prev, p)
    texts_gone = []
    if p["remove_text"] and prev.get("text_boxes"):
        strokes, texts_gone = sp.cut_by_mask(strokes, _text_mask(prev, pen_px))
    hair = _hair_mask(prev, _face_skin_mask(prev) if p["tone_skip_face"] else None) if p["hair_flow"] else None
    reasons = declutter.classify(strokes, prev["prep_color"], faces, _px_per_mm(prev, p), rules, hair=hair)
    kept = [q for q, why in zip(strokes, reasons) if not why]
    gone = [(q, TEXT_REASON) for q in texts_gone] + [(q, why) for q, why in zip(strokes, reasons) if why]
    meshes = [c.get("mesh") for c in prev.get("face_crops") or []] if p["face_mesh"] else []
    redrawn_mouths = set()
    if any(m is not None for m in meshes):
        kept, cut, redrawn_mouths = face_mesh.redraw(kept, meshes, pen_px, prev["gray"].shape)
        gone += [(q, face_mesh.REDRAWN) for q in cut]
        kept, _ = face_mesh.fill_jaw(kept, meshes, declutter.contrast_map(prev["prep_color"]), pen_px)
    # 다문 입은 얼굴 점으로 그렸으니 기존 입 정리는 벌린 입·얼굴 점이 없는 얼굴에만
    rest = [f for i, f in enumerate(faces) if i not in redrawn_mouths]
    kept, redrawn = declutter.tidy_mouth(kept, rest, pen_px)
    return {**prev, "strokes": kept, "discarded_clutter": gone + [(q, declutter.MOUTH_TIDY) for q in redrawn],
            "hair_mask": hair, "mesh_wanted": bool(p["face_mesh"] and faces), "text_wanted": bool(p["remove_text"])}


def _text_mask(prev, pen_px):
    """지울 글자 영역 (검출한 사각형을 펜 굵기만큼 넓힘). 얼굴 타원 안에 가운데가 있는 사각형은 오검출로 보고 뺀다."""
    h, w = prev["gray"].shape[:2]
    mask = np.zeros((h, w), np.uint8)
    for box in prev["text_boxes"]:
        c = box.mean(axis=0)
        if any(((c[0] - cx) / ax) ** 2 + ((c[1] - cy) / ay) ** 2 <= 1
               for (cx, cy), (ax, ay) in (fc.ellipse_of(f) for f in prev.get("faces") or [])):
            continue
        cv2.fillPoly(mask, [np.round(box).astype(np.int32)], 1)
    r = max(1, int(round(pen_px)))
    return cv2.dilate(mask, np.ones((2 * r + 1, 2 * r + 1), np.uint8)) > 0


def _preview_face(o):
    img = draw_strokes_colored(o["strokes"], o["gray"].shape, o.get("discarded_face", ()))
    found = o.get("faces") or []
    for f in found:
        (cx, cy), (ax, ay) = fc.ellipse_of(f)
        cv2.ellipse(img, (int(round(cx)), int(round(cy))), (int(round(ax)), int(round(ay))), 0, 0, 360, GREEN, 1,
                    cv2.LINE_AA)
        for x, y in f.landmarks:
            cv2.circle(img, (int(round(x)), int(round(y))), 2, GREEN, -1)
    if not found:
        cv2.putText(img, "no face", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1, cv2.LINE_AA)
    return img


def _run_simplify(prev, p):
    st = sp.smooth_strokes(prev["strokes"], p["smooth_sigma_px"])
    st = sp.simplify_strokes(st, p["epsilon_px"])
    return {**prev, "strokes": sp.round_corners(st, int(p["round_iters"]))}


def _mm_per_px(prev, p):
    """작업 이미지 1px이 종이에서 몇 mm인지 (큰 세로 그림은 영역에 맞춰 요청보다 작게 그려지므로 실제 크기로 환산)."""
    h, w = prev["gray"].shape[:2]
    region = prev.get("region")
    long_mm = limits.effective_long_mm(region, p["box_mm"], w, h) if region is not None else p["box_mm"]
    return long_mm / max(h, w)


def _run_tone(prev, p):
    """명암 빗금과 머리카락 결: 윤곽 획(strokes)은 그대로 두고 빗금·결 획을 hatch_strokes에 따로 만든다.
    (편집 번호표는 윤곽 획만 다루고, 빗금·결은 그리기 직전에 세션이 합친다)"""
    outline = prev["strokes"]
    levels = int(p["tone_levels"])
    flow_on = bool(p["hair_flow"]) and bool(prev.get("faces"))
    if levels <= 0 and not flow_on:
        return {**prev, "outline_strokes": outline, "hatch_strokes": [], "hair_strokes": 0, "tone_note": "",
                "uses_deps": False}
    mm_px = _mm_per_px(prev, p)
    skin = _face_skin_mask(prev) if p["tone_skip_face"] else None
    hair = (prev["hair_mask"] if "hair_mask" in prev else _hair_mask(prev, skin)) if flow_on else None
    # 윤곽 획(선화가 그린 머리 결·머리 외곽)을 이미 그은 선으로 쳐서, 결 선은 그 사이 빈 곳만 채운다
    flow = (tone.hair_flow_strokes(prev["prep_gray"], hair, p["hair_spacing_mm"] / mm_px, 3.0 / mm_px, 25.0 / mm_px,
                                   obstacles=outline)
            if hair is not None else [])
    hatch, note = [], ""
    if levels > 0:
        keep_out = skin if hair is None else (hair if skin is None else (skin | hair))   # 결을 그린 머리는 빗금 안 함
        hatch, note = tone.hatch_strokes(prev["prep_gray"], p["tone_spacing_mm"] / mm_px, levels, p["tone_angle"],
                                         p["tone_min_mm"] / mm_px, p["tone_bias"], keep_out=keep_out)
    return {**prev, "outline_strokes": outline, "hatch_strokes": hatch + flow, "hair_strokes": len(flow),
            "tone_note": note, "uses_deps": True}


def _hair_mask(prev, skin=None):
    """머리카락 영역: 사진의 어두운 덩어리 중 얼굴 위·옆(이마 위 얼굴 높이 0.7배부터 눈높이까지, 얼굴 폭 양옆 0.4배)에
    걸친 것을, 머리 둘레(얼굴 상자 위로 1배·양옆 0.8배, 아래로는 턱 바로 밑까지)로 자른다. 머리와 이어진 어두운 옷은
    잘려 나가고(초상의 남색 셔츠), 턱 아래로 내려온 긴 머리는 결을 그리지 않고 윤곽만 남는다. 얼굴 피부는 뺀다."""
    faces = prev.get("faces") or []
    if not faces:
        return None
    gray = prev["prep_gray"]
    dark = (tone.darkness_map(gray, sigma_px=2.0) > 0.45).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, labels, _, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    h, w = gray.shape[:2]
    keep = np.zeros(n, bool)
    head = np.zeros((h, w), bool)
    for f in faces:
        x, y, fw, fh = (float(v) for v in f.box)
        eye_y = float(min(f.landmarks[0][1], f.landmarks[1][1]))
        x0, x1 = int(max(0, x - 0.4 * fw)), int(min(w, x + 1.4 * fw))
        y0, y1 = int(max(0, y - 0.7 * fh)), int(min(h, eye_y))
        if x1 > x0 and y1 > y0:
            keep[np.unique(labels[y0:y1, x0:x1])] = True
        hx0, hx1 = int(max(0, x - 0.8 * fw)), int(min(w, x + 1.8 * fw))
        hy0, hy1 = int(max(0, y - 1.0 * fh)), int(min(h, y + 1.05 * fh))
        head[hy0:hy1, hx0:hx1] = True
    keep[0] = False
    mask = keep[labels] & head
    if skin is not None:
        mask &= ~skin
    return mask if mask.any() else None


def _face_skin_mask(prev):
    """빗금을 긋지 않을 얼굴 영역: 눈썹 아래의 얼굴 타원(눈~턱)과, 얼굴 주변(이마·목·귀)에서 그 사진의 피부색인 곳.
    종이 위 얼굴은 수 cm라 피부 그림자에 빗금을 그으면 흉터·수염처럼 보여 표정을 가린다. 눈썹·눈·입은 윤곽선이 그림.
    눈썹 위는 피부색인 곳만 빼서 앞머리·머리카락의 빗금(머리 모양)은 그대로 남긴다."""
    faces = prev.get("faces") or []
    if not faces:
        return None
    color = prev["prep_color"]
    h, w = color.shape[:2]
    out = np.zeros((h, w), np.uint8)
    for f in faces:
        x, y, fw, fh = (float(v) for v in f.box)
        cx, cy = x + fw / 2, y + fh / 2
        oval = np.zeros((h, w), np.uint8)
        cv2.ellipse(oval, (int(round(cx)), int(round(cy))), (int(round(0.55 * fw)), int(round(0.68 * fh))),
                    0, 0, 360, 1, -1)
        brow = int(max(0.0, min(f.landmarks[0][1], f.landmarks[1][1]) - 0.12 * fh))
        oval[:brow] = 0
        out |= oval
        near = np.zeros((h, w), np.uint8)
        cv2.rectangle(near, (int(x - 0.6 * fw), int(y)), (int(x + 1.6 * fw), int(y + 2.0 * fh)), 1, -1)
        skin = declutter.skin_mask(color, f, declutter.Rules().skin_tol * 1.5)
        out |= (skin & (near > 0)).astype(np.uint8)
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return out > 0


def _preview_tone(o):
    img = np.full((*o["gray"].shape[:2], 3), 255, np.uint8)
    for s in o.get("hatch_strokes", ()):
        cv2.polylines(img, [np.round(s).astype(np.int32).reshape(-1, 1, 2)], False, (200, 120, 40), 1, cv2.LINE_AA)
    for s in o["outline_strokes"]:
        cv2.polylines(img, [np.round(s).astype(np.int32).reshape(-1, 1, 2)], False, (50, 50, 50), 1, cv2.LINE_AA)
    if o.get("tone_note"):
        cv2.putText(img, "dark background: turn on rembg", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 1,
                    cv2.LINE_AA)
    return img


FRAMES = (("auto", "자동"), ("full", "전체"), ("bust", "상반신"), ("face", "얼굴"))
EDGE_MODES = (("luma", "밝기"), ("lab", "색 차이"), ("dark", "어두운 선"))
_hi, _med = presets.DETAIL_PRESETS["high"], presets.IMAGE_TYPES["illustration"]["median"]

STAGES = (
    Stage("source", "원본", (
        ParamSpec("rembg", "배경 제거 (rembg)", "bool", False, help="첫 실행은 약 1분, 이후 캐시"),
        ParamSpec("frame", "구도", "choice", "auto", choices=FRAMES,
                  help="자동=사진 속 얼굴이 종이에서 25mm보다 작으면 상반신으로 자름 (원본 해상도에서 자름)")),
        _run_source, lambda o: o["color"].copy()),
    Stage("prep", "전처리", (
        ParamSpec("median_ksize", "미디언 (px, 망점 제거)", "int", _med, 0, 15,
                  help="만화 스크린톤을 지움. 0 = 끔, 만화는 11 정도"),
        ParamSpec("blur_ksize", "가우시안 블러 (px)", "int", 5, 1, 15, odd=True,
                  help="클수록 잔선이 줄고 윤곽이 부드러워짐")),
        _run_prep, lambda o: _bgr(o["prep_gray"])),
    Stage("edges", "선 검출", (
        ParamSpec("lineart", "실사 선화로 바꾼 뒤 따기", "bool", False,
                  help="실사만. 선화 모델(처음 한 번 내려받음)이 그린 선을 그대로 따라 그림. 모델을 못 쓰면 아래 방식으로"),
        ParamSpec("edge_mode", "방식", "choice", "luma", choices=EDGE_MODES,
                  help="선화를 끈 때: 밝기=흑백 Canny, 색 차이=Lab Canny, 어두운 선=선화 중심선"),
        ParamSpec("canny_low", "약한 선 민감도 (Canny 하한)", "int", _hi[0], 0, 255, help="낮을수록 약한 선도 잡음"),
        ParamSpec("canny_high", "강한 선 기준 (Canny 상한)", "int", _hi[1], 0, 400, help="선이 시작되는 강한 경계 기준")),
        _run_edges, lambda o: _bgr(255 - o["edges"])),
    Stage("trace", "뼈대·획", (
        ParamSpec("min_length_px", "작은 조각 버리기 (px)", "int", _hi[2], 1, 100, help="이보다 작은 선 덩어리는 버림"),
        ParamSpec("spur_px", "잔가지 버리기 (px)", "int", 6, 0, 20, help="분기점에 붙은 이보다 짧은 가지는 버림")),
        _run_trace, lambda o: draw_strokes_colored(o["strokes"], o["gray"].shape, o["discarded_trace"])),
    Stage("dedupe", "겹침 제거", (
        ParamSpec("dedupe_px", "겹친 선 합치기 (px, 0=끔)", "int", presets.DEFAULT_DEDUPE_PX, 0, 8,
                  help="이 거리 안에서 겹치는 선은 하나만 남김. 사이가 먹인 굵은 선 양쪽은 작업 이미지 긴 변의 2%(800px면 16px)까지 하나로 합치고, 사이가 밝은 선과 눈·입은 남김"),),
        _run_dedupe, lambda o: draw_strokes_colored(o["strokes"], o["gray"].shape, o["discarded_dedupe"])),
    Stage("merge", "이어 붙이기", (
        ParamSpec("merge_join_px", "끊긴 선 이어 붙이기 (px, 0=끔)", "float", presets.DEFAULT_MERGE_JOIN_PX, 0, 8, 0.5,
                  help="끝점이 이 거리 안이면 펜을 떼지 않고 이어 그림"),),
        _run_merge, lambda o: draw_strokes_colored(o["strokes"], o["gray"].shape)),
    Stage("face", "얼굴 세밀", (
        ParamSpec("face_detail", "얼굴 세밀 처리", "bool", True,
                  help="사진에서 찾은 얼굴을 원본 해상도로 다시 처리 (얼굴이 없으면 그대로)"),
        ParamSpec("face_sensitivity", "얼굴 선 민감도", "float", 0.6, 0.3, 1.0, 0.05,
                  help="얼굴 부분 Canny 기준에 곱함. 낮을수록 약한 선(눈매·입술)도 잡음"),
        ParamSpec("pen_mm", "펜 굵기 (mm)", "float", 0.5, 0.2, 1.5, 0.05,
                  help="이보다 촘촘한 얼굴 선은 합침 (종이 크기에 맞춰 자동 환산)")),
        _run_face, _preview_face,
        deps=("box_mm", "median_ksize", "blur_ksize", "edge_mode", "canny_low", "canny_high",
              "min_length_px", "spur_px", "dedupe_px", "merge_join_px")),
    Stage("clutter", "잔선 정리", (
        ParamSpec("declutter", "잔선 정리 (실사 선화)", "bool", True,
                  help="피부 위 주름과 짧고 흐린 잔선을 지우고, 눈·코·입·얼굴 윤곽·머리·글자·문양은 남김"),
        ParamSpec("clutter_mm", "얼굴 밖 짧은 선 기준 (mm)", "float", 3.0, 1.0, 12.0, 0.5,
                  help="얼굴 밖에서 이보다 짧고 흐린 선은 지움. 클수록 선이 줄고 빨리 그림 (뚜렷한 글자·문양은 남김)"),
        ParamSpec("clutter_pct", "흐린 선 기준 (백분위)", "int", 60, 30, 95, 5,
                  help="사진 대비가 이 백분위보다 낮은 짧은 선을 흐린 선으로 봄. 클수록 더 많이 지움"),
        ParamSpec("clutter_max", "선 개수 상한 (0=제한 없음)", "int", 0, 0, 3000, 10,
                  help="넘으면 얼굴 밖에서 덜 중요한(짧고 흐린) 선부터 뺌. 얼굴 안 선은 지킴. 자동 세팅이 시간에 맞춰 정함"),
        ParamSpec("face_mesh", "눈·눈썹·입 다시 그리기 (얼굴 점)", "bool", True,
                  help="얼굴 점 478개로 눈을 눈꺼풀·눈동자로, 눈썹을 가운데 선으로, 다문 입을 입술 사이 선으로 "
                       "다시 그리고 끊긴 턱선을 이음. 정면에 가까운 얼굴만"),
        ParamSpec("remove_text", "사진 속 글자 지우기", "bool", True,
                  help="화보 제목·이름 자막·옷의 로고 글자처럼 글자로 찾은 영역의 선을 지움. 얼굴 안은 그대로")),
        _run_clutter, lambda o: draw_strokes_colored(o["strokes"], o["gray"].shape, o.get("discarded_clutter", ())),
        deps=("box_mm", "pen_mm", "hair_flow", "tone_skip_face")),
    Stage("simplify", "스무딩·단순화", (
        ParamSpec("smooth_sigma_px", "선 매끄럽게 (px, 0=끔)", "float", 2.0, 0, 5.0, 0.5,
                  help="픽셀 계단·흔들림을 없앰. 클수록 매끄럽지만 작은 모양이 둥글어짐"),
        ParamSpec("epsilon_px", "선 단순화 (px, 클수록 단순)", "float", _hi[3], 0.5, 5.0, 0.1,
                  help="클수록 점·명령 수가 줄지만 곡선이 거칠어짐"),
        ParamSpec("round_iters", "모서리 둥글리기 (회, 0=끔)", "int", 1, 0, 3,
                  help="꺾인 곳을 둥글게. 1회마다 점·명령 수가 약 2배"),),
        _run_simplify, lambda o: draw_strokes_colored(o["strokes"], o["gray"].shape)),
    Stage("tone", "명암 빗금", (
        ParamSpec("tone_levels", "명암 단계 (0=끔)", "int", 0, 0, 3,
                  help="어두운 면을 평행 빗금으로 채움. 1=어두운 면, 2=더 어두운 곳은 사이에 한 줄씩 더, 3=더 진하게. 시간이 늘어남"),
        ParamSpec("tone_spacing_mm", "빗금 간격 (mm)", "float", 1.5, 0.6, 4.0, 0.1,
                  help="좁을수록 진하고 오래 걸림 (펜 굵기 0.5mm의 2~3배가 무난)"),
        ParamSpec("tone_angle", "빗금 방향 (도)", "int", 45, 0, 175, 5,
                  help="빗금의 각도. 단계가 올라도 같은 방향으로 촘촘해짐 (교차 빗금은 펜으로 그리면 격자로 보임)"),
        ParamSpec("tone_min_mm", "짧은 빗금 버리기 (mm)", "float", 4.0, 1.0, 15.0, 0.5,
                  help="이보다 짧은 빗금은 그리지 않음 (얼룩 방지)"),
        ParamSpec("tone_bias", "어두움 기준 조절", "float", 0.0, -0.3, 0.3, 0.05,
                  help="+면 더 어두운 곳만 빗금(줄어듦), -면 밝은 곳까지 빗금(늘어남)"),
        ParamSpec("tone_skip_face", "얼굴 피부에는 빗금 안 함", "bool", True,
                  help="얼굴과 목의 피부 그림자에 빗금을 그으면 흉터·수염처럼 보여 표정을 가림. 머리카락·옷은 그대로 빗금"),
        ParamSpec("hair_flow", "머리카락 결 그리기", "bool", False,
                  help="머리카락을 빗금으로 메우지 않고 사진의 결 방향을 따라 흐름선을 그음 (채색 전 스케치처럼). 얼굴을 찾은 사진만"),
        ParamSpec("hair_spacing_mm", "결 간격 (mm)", "float", 2.0, 1.0, 4.0, 0.25,
                  help="결 선 사이 간격. 좁을수록 촘촘하고 오래 걸림")),
        _run_tone, _preview_tone, deps=("box_mm",)),
)
ALL_STAGES = STAGES + (
    Stage("edit", "편집", ()),
    Stage("paper", "순서·종이", (
        ParamSpec("box_mm", "그리기 크기 (mm, 긴 변)", "int", 100, 30, 250,
                  help="100mm 넘게는 넓은 범위(실물 미확인): 큰 그림은 조금 아래로 옮기고 세로 그림은 줄여서 맞춤"),
        ParamSpec("pen_join_mm", "가까운 획 이어 그리기 (mm, 0=끔)", "float", 1.0, 0.0, 2.0, 0.25,
                  help="앞 획 끝에서 이 거리 안에 다음 획이 시작하면 펜을 들지 않고 짧은 직선으로 이어 그림. "
                       "펜 올림·내림이 획마다 약 5초라 시간이 많이 줄지만, 크면 이음선이 보임"))),
)
_DESCS = {
    "source": "입력 사진 (긴 변 800px로 맞춤). 배경 제거를 켜면 인물만 남김",
    "prep": "흑백으로 바꾸고 블러·미디언으로 잔무늬와 망점을 줄임",
    "edges": "밝기(또는 색)가 급히 바뀌는 곳을 경계로 찾음",
    "trace": "두께 있는 경계를 1px 중심선으로 만들고, 이어진 선마다 획 하나로 따라감",
    "dedupe": "굵은 선의 양쪽 경계가 두 줄로 잡힌 이중선을 하나로 줄임",
    "merge": "끝이 닿는 획을 이어 펜을 드는 횟수를 줄임",
    "face": "사진 속 얼굴을 원본 해상도로 다시 따라가, 펜 굵기에 맞는 밀도로 바꿔 넣음 (초록 = 얼굴 영역·눈코입)",
    "clutter": "실사 선화에서 피부 위 주름과 짧고 흐린 잔선을 지우고 눈·코·입·윤곽·머리·글자·문양은 남김 (연회색 = 지운 선)",
    "simplify": "선을 매끄럽게 다듬고(계단·지그재그 제거), 모양은 유지하며 점 수(= 로봇 명령 수)를 줄임",
    "tone": "어두운 면을 평행 빗금(더 어두우면 같은 방향으로 더 촘촘히)으로 채워 사진 같은 명암을 냄 (파랑 = 빗금, 회색 = 윤곽). 기본은 꺼짐",
    "edit": "번호 붙은 최종 획. 살리기·지우기·점 편집 제안을 확인하고 적용",
    "paper": "그리는 순서를 정해 A4 위에 배치하고 시간을 추정",
}
STAGES = tuple(dataclasses.replace(st, desc=_DESCS[st.id]) for st in STAGES)
ALL_STAGES = tuple(dataclasses.replace(st, desc=_DESCS[st.id]) for st in ALL_STAGES)
PIPELINE_IDS = tuple(s.id for s in STAGES)
STAGE_BY_ID = {s.id: s for s in ALL_STAGES}
PARAM_SPECS = {p.key: p for s in ALL_STAGES for p in s.params}


def stage_title(stage_id):
    """번호 붙은 단계 이름 (예: "④ 뼈대·획")."""
    k = next(i for i, st in enumerate(ALL_STAGES) if st.id == stage_id)
    return f"{'①②③④⑤⑥⑦⑧⑨⑩⑪⑫'[k]} {ALL_STAGES[k].label}"


def default_params():
    return {k: s.default for k, s in PARAM_SPECS.items()}


def candidates_of(outputs):
    """버린 조각 전부: [(점 배열, 이유)]."""
    last = outputs["simplify"]
    return (list(last.get("discarded_trace", [])) + list(last.get("discarded_dedupe", []))
            + list(last.get("discarded_face", [])) + list(last.get("discarded_clutter", [])))


class Pipeline:
    """단계별 결과 캐시. 단계 k의 키 = (단계 k-1의 키, 단계 id, 단계 설정)."""

    def __init__(self, stages=STAGES):
        self.stages = stages
        self._cache = {}                              # stage id -> (key, output)
        self.run_counts = {s.id: 0 for s in stages}

    @staticmethod
    def _stage_params(stage, params):
        p = {spec.key: params[spec.key] for spec in stage.params}
        p.update({k: params[k] for k in stage.deps})
        return p

    @staticmethod
    def _key_pair(stage, prev_key, params):
        """(deps 없는 키, deps 포함 키). 결과가 uses_deps=False라고 밝히면 deps 없는 키로 저장해,
        그 설정(예: 종이 크기)만 바뀌었을 때 이 단계와 뒤 단계를 다시 계산하지 않음."""
        own = {spec.key: params[spec.key] for spec in stage.params}
        base = (prev_key, stage.id, tuple(sorted(own.items())))
        if not stage.deps:
            return base, base
        full = {**own, **{k: params[k] for k in stage.deps}}
        return base, (prev_key, stage.id, tuple(sorted(full.items())))

    def _hit(self, stage, prev_key, params):
        """캐시에 맞는 결과가 있으면 (키, 결과), 없으면 (None, None)."""
        base, full = self._key_pair(stage, prev_key, params)
        hit = self._cache.get(stage.id)
        if hit is not None and hit[0] in (base, full):
            return hit
        return None, None

    def run(self, inputs, input_key, params, is_current=lambda: True):
        prev, outputs, key = inputs, {}, ("input", input_key)
        for st in self.stages:
            hkey, out = self._hit(st, key, params)
            if hkey is not None:
                key = hkey
            else:
                if not is_current():
                    raise StaleRun()
                out = st.run(prev, self._stage_params(st, params))
                self.run_counts[st.id] += 1
                base, full = self._key_pair(st, key, params)
                key = base if out.get("uses_deps") is False else full
                self._cache[st.id] = (key, out)
            outputs[st.id] = out
            prev = out
        return outputs

    def dirty_from(self, input_key, params):
        """다시 계산해야 하는 첫 단계 id. 전부 캐시에 있으면 None."""
        key = ("input", input_key)
        for st in self.stages:
            key, _ = self._hit(st, key, params)
            if key is None:
                return st.id
        return None
