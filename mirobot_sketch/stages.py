"""
단계별 선 추출 파이프라인
=========================
원본 → 전처리 → 선 검출 → 뼈대·획 → 겹침 제거 → 이어 붙이기 → 얼굴 세밀 → 스무딩·단순화
각 단계는 조절 항목(ParamSpec), 계산(run), 미리보기(preview)를 가집니다. GUI의 조절 칸과
에이전트 도구 설명은 이 정의에서 만들어집니다. Pipeline은 단계별 결과를 캐시해 두고
설정이 바뀐 첫 단계부터만 다시 계산합니다. (편집·순서·종이 배치는 session.py가 이어서 처리)
"""

import dataclasses
from dataclasses import dataclass
from typing import Callable

import cv2
import numpy as np

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
            return bool(value)
        if self.kind == "choice":
            keys = [c[0] for c in self.choices]
            if value not in keys:
                raise ValueError(f"{self.key}는 {', '.join(keys)} 중 하나여야 합니다")
            return value
        v = min(max(float(value), self.lo), self.hi)
        if self.kind == "int":
            v = int(round(v))
            if self.odd and v % 2 == 0:
                v = v + 1 if v < self.hi else v - 1
            return v
        return round(round(v / self.step) * self.step, 4)


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
    if p["edge_mode"] == "dark":
        e = sp.compute_dark_mask(prev["prep_gray"], 1)
    elif p["edge_mode"] == "lab":
        e = sp.compute_edges_lab(prev["prep_color"], p["canny_low"], p["canny_high"], 1)
    else:
        e = sp.compute_edges(prev["prep_gray"], p["canny_low"], p["canny_high"], 1)
    return {**prev, "edges": e}


def _run_trace(prev, p):
    disc = []
    st = sp.trace_strokes(prev["edges"], p["min_length_px"], p["spur_px"], discarded=disc)
    return {**prev, "strokes": st, "discarded_trace": disc}


def _run_dedupe(prev, p):
    if not p["dedupe_px"]:
        return {**prev, "discarded_dedupe": []}
    disc = []
    st = sp.dedupe_strokes(prev["strokes"], prev["edges"].shape, int(p["dedupe_px"]), discarded=disc)
    return {**prev, "strokes": st, "discarded_dedupe": disc}


def _run_merge(prev, p):
    if not p["merge_join_px"]:
        return dict(prev)
    return {**prev, "strokes": sp.merge_strokes(prev["strokes"], p["merge_join_px"])}


FACE_REASON = "얼굴 세밀 처리로 교체"
GREEN = (40, 160, 40)


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
    """명암 빗금: 윤곽 획(strokes)은 그대로 두고 어두운 면의 빗금 획을 hatch_strokes에 따로 만든다.
    (편집 번호표는 윤곽 획만 다루고, 빗금은 그리기 직전에 세션이 합친다)"""
    outline = prev["strokes"]
    levels = int(p["tone_levels"])
    if levels <= 0:
        return {**prev, "outline_strokes": outline, "hatch_strokes": [], "tone_note": "", "uses_deps": False}
    mm_px = _mm_per_px(prev, p)
    hatch, note = tone.hatch_strokes(prev["prep_gray"], p["tone_spacing_mm"] / mm_px, levels, p["tone_angle"],
                                     p["tone_min_mm"] / mm_px, p["tone_bias"])
    return {**prev, "outline_strokes": outline, "hatch_strokes": hatch, "tone_note": note, "uses_deps": True}


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
        ParamSpec("edge_mode", "방식", "choice", "luma", choices=EDGE_MODES,
                  help="밝기=흑백 Canny, 색 차이=Lab Canny(밝기가 같은 색 경계도 찾음), 어두운 선=선화 중심선"),
        ParamSpec("canny_low", "약한 선 민감도 (Canny 하한)", "int", _hi[0], 0, 255, help="낮을수록 약한 선도 잡음"),
        ParamSpec("canny_high", "강한 선 기준 (Canny 상한)", "int", _hi[1], 0, 400, help="선이 시작되는 강한 경계 기준")),
        _run_edges, lambda o: _bgr(255 - o["edges"])),
    Stage("trace", "뼈대·획", (
        ParamSpec("min_length_px", "작은 조각 버리기 (px)", "int", _hi[2], 1, 100, help="이보다 작은 선 덩어리는 버림"),
        ParamSpec("spur_px", "잔가지 버리기 (px)", "int", 6, 0, 20, help="분기점에 붙은 이보다 짧은 가지는 버림")),
        _run_trace, lambda o: draw_strokes_colored(o["strokes"], o["gray"].shape, o["discarded_trace"])),
    Stage("dedupe", "겹침 제거", (
        ParamSpec("dedupe_px", "겹친 선 합치기 (px, 0=끔)", "int", presets.DEFAULT_DEDUPE_PX, 0, 8,
                  help="이 거리 안에서 겹치는 선은 하나만 남김"),),
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
                  help="어두운 면을 평행 빗금으로 채움. 1=어두운 면, 2=교차 빗금, 3=더 진하게. 시간이 늘어남"),
        ParamSpec("tone_spacing_mm", "빗금 간격 (mm)", "float", 1.5, 0.6, 4.0, 0.1,
                  help="좁을수록 진하고 오래 걸림 (펜 굵기 0.5mm의 2~3배가 무난)"),
        ParamSpec("tone_angle", "빗금 방향 (도)", "int", 45, 0, 175, 5,
                  help="첫 단계 빗금의 각도. 2단계는 직각, 3단계는 그 사이로 교차"),
        ParamSpec("tone_min_mm", "짧은 빗금 버리기 (mm)", "float", 4.0, 1.0, 15.0, 0.5,
                  help="이보다 짧은 빗금은 그리지 않음 (얼룩 방지)"),
        ParamSpec("tone_bias", "어두움 기준 조절", "float", 0.0, -0.3, 0.3, 0.05,
                  help="+면 더 어두운 곳만 빗금(줄어듦), -면 밝은 곳까지 빗금(늘어남)")),
        _run_tone, _preview_tone, deps=("box_mm",)),
)
ALL_STAGES = STAGES + (
    Stage("edit", "편집", ()),
    Stage("paper", "순서·종이", (
        ParamSpec("box_mm", "그리기 크기 (mm, 긴 변)", "int", 100, 30, 250,
                  help="100mm 넘게는 넓은 범위(실물 미확인): 큰 그림은 조금 아래로 옮기고 세로 그림은 줄여서 맞춤"),)),
)
_DESCS = {
    "source": "입력 사진 (긴 변 800px로 맞춤). 배경 제거를 켜면 인물만 남김",
    "prep": "흑백으로 바꾸고 블러·미디언으로 잔무늬와 망점을 줄임",
    "edges": "밝기(또는 색)가 급히 바뀌는 곳을 경계로 찾음",
    "trace": "두께 있는 경계를 1px 중심선으로 만들고, 이어진 선마다 획 하나로 따라감",
    "dedupe": "굵은 선의 양쪽 경계가 두 줄로 잡힌 이중선을 하나로 줄임",
    "merge": "끝이 닿는 획을 이어 펜을 드는 횟수를 줄임",
    "face": "사진 속 얼굴을 원본 해상도로 다시 따라가, 펜 굵기에 맞는 밀도로 바꿔 넣음 (초록 = 얼굴 영역·눈코입)",
    "simplify": "선을 매끄럽게 다듬고(계단·지그재그 제거), 모양은 유지하며 점 수(= 로봇 명령 수)를 줄임",
    "tone": "어두운 면을 평행 빗금(더 어두우면 교차)으로 채워 사진 같은 명암을 냄 (파랑 = 빗금, 회색 = 윤곽). 기본은 꺼짐",
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
            + list(last.get("discarded_face", [])))


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
