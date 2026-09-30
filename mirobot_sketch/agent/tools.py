"""
에이전트 도구 — OpenRouter(앱 안에서 직접)와 MCP(Claude Code / Codex)가 같은 정의를 씀
======================================================================================
도구는 SketchSession의 설정·획을 바꾸고 로봇 준비 상태를 읽기만 합니다. 로봇을 움직이거나 파일을 쓰는
도구는 없습니다 (설계 문서: 모델은 관절각이나 G-code를 직접 실행하지 않음). 실제 드로잉은
사람이 앱의 '로봇으로 그리기' 창에서 확인해야만 시작됩니다.

도구 결과는 부분(part)의 목록입니다:
  {"type": "text", "text": ...}
  {"type": "image", "data": <base64 PNG>, "mime": "image/png"}
"""

import base64
import json

import cv2

from .. import stages
from ..session import SessionError

SYSTEM_PROMPT = """당신은 사진을 로봇 팔(WLKATA Mirobot)이 펜으로 그릴 선 경로로 바꾸는 앱의 편집 도우미입니다.
사용자의 요청에 맞게 도구로 처리 설정을 바꾸고, 결과 그림을 직접 보고 확인한 뒤 필요한 획을 정리합니다.

작업 방식
- 먼저 get_state로 현재 상태를 보고, view로 원본과 결과를 눈으로 확인하세요.
- 설정을 바꾼 뒤에는 view로 결과를 다시 확인하고, 획 수·예상 시간의 변화를 사용자에게 알려 주세요.
- 선을 다듬을 때:
  1) view(kind="edit", overlay_original=0.4, show_candidates=true)로 원본 위의 획과 버린 선(후보)을 봅니다.
  2) 문제 부위는 region_mm로 확대하고, list_strokes로 번호·이유를 확인합니다.
  3) propose_edits로 제안하고 돌려받은 그림(빨강=사라짐, 초록=생김)으로 스스로 검토합니다.
  4) 사용자에게 "초록 813·820은 머리카락 윤곽을 살리고, 빨강 5·8은 배경 잡음을 지웁니다"처럼 번호로 설명하고
     확인을 기다립니다. 사용자가 "5번 빼고 적용"이라 하면 apply_proposals(exclude=[5]).
  사용자가 바로 하라고 했을 때만 apply_now=true를 쓰세요.
- 점 편집(move_point 등)은 get_stroke로 점 번호와 좌표를 확인한 뒤에 하세요.
- 설정을 바꿔 다시 계산하면 번호가 새로 매겨지고 적용 전 제안은 취소됩니다(적용한 편집은 유지).
  설정을 먼저 정하고 편집은 마지막에 하세요.
- 좌표는 종이 중심이 원점인 mm이며 x는 오른쪽, y는 위쪽이 +입니다. 허용 범위는 두 가지입니다.
  펜으로 그릴 수 있는 범위(state의 executor_limit_outline_mm)와 공중 확인용 넓은 범위(pending_limit_outline_mm)이며,
  넓은 범위 밖 좌표는 거부됩니다. 큰 그림은 위쪽 도달 한계 때문에 종이 중심보다 아래로 배치됩니다
  (120mm 정사각형은 y −65 ~ +55).

처리 단계 (view의 kind로 각 단계 결과를 볼 수 있음)
- source 원본: rembg(배경 제거), frame(구도: auto 자동 / full 전체 / bust 상반신 / face 얼굴.
  auto는 사진 속 얼굴이 하나이고 종이에서 25mm보다 작으면 상반신으로 자름. 결과 요약의 frame·faces로 확인)
- prep 전처리: median_ksize(만화 망점 제거, 11 정도), blur_ksize(가우시안 블러)
- edges 선 검출: edge_mode = luma(밝기) / lab(색 차이 — 밝기가 비슷한 색 경계도 찾음, 컬러 일러스트·사진에 유리) /
  dark(어두운 선 중심선, 선화), canny_low / canny_high
- trace 뼈대·획: min_length_px(작은 덩어리 제거), spur_px(잔가지 제거)
- dedupe 겹침 제거: dedupe_px / merge 이어 붙이기: merge_join_px
- face 얼굴 세밀: face_detail(켬/끔), face_sensitivity(낮을수록 약한 선도), pen_mm(이보다 촘촘한 얼굴 선은 합침).
  실사 사진에서만 얼굴을 찾음 (애니·만화 그림체는 보통 못 찾음). view(face)의 초록 타원이 처리 영역
- simplify 스무딩·단순화: smooth_sigma_px, epsilon_px, round_iters
- tone 명암 빗금: tone_levels(0=끔, 1~3), tone_spacing_mm, tone_angle, tone_min_mm, tone_bias. 어두운 면을 평행 빗금
  (더 어두우면 교차)으로 채워 사진 같은 명암을 냅니다. 켜면 획 수와 시간이 늘어나니 결과 요약의 예상 시간을 알려 주세요.
  배경이 어두운 사진은 먼저 rembg를 켜야 하고, 안 켜면 빗금이 화면을 덮어 만들지 않고 경고합니다(view tone에 표시).
- paper 종이: box_mm(그림 긴 변 크기. 실행기 허용 범위를 넘으면 실제 드로잉 전 별도 확인 필요)
- image_type(photo/illustration/manga)과 detail(low/medium/high)은 여러 값을 한꺼번에 채우는 프리셋
- 선이 빠졌으면 어느 단계에서 빠졌는지 view로 단계를 차례로 보고, 그 단계의 값을 바꾸세요.

사진과 비슷하고 예쁘게 만들기 (사용자가 "사진처럼", "예쁘게", "더 닮게"라고 하면)
- compare로 원본 | 그림 | 차이를 봅니다. 차이 그림에서 빨강 = 사진보다 밝게(덜) 그려짐, 파랑 = 사진보다 어둡게(빽빽하게) 그려짐,
  초록 = 잘 맞음. 지표: tone_match(명암 닮음, 높을수록 좋음), edge_recall(윤곽 재현), edge_precision(잡음이 적을수록 높음),
  under/over_shaded_pct. 돌려주는 hints를 따라 설정을 바꾸고 다시 compare해서 좋아졌는지 확인하세요(2~3번). 나빠지면 되돌리고 말하세요.
- 눈으로도 확인하세요: 얼굴(눈·코·입)이 살아 있는지, 배경 잡음이 튀지 않는지, 어두운 면과 밝은 면의 대비가 있는지,
  예상 시간이 사용자가 원하는 범위인지. 숫자만 믿지 말고 view로 최종 그림을 보세요.
- 순서: 배경 제거(rembg) → 이미지 종류·상세도 프리셋 → 잔선·획 정리 → tone_levels 1~2로 명암 → 시간이 너무 늘면 줄임.
  빗금은 시간이 늘어나는 대가가 있으니 사용자가 "닮게/예쁘게"를 원할 때, 혹은 시간 여유가 있을 때 제안하세요.

로봇으로 그리기까지 (사용자가 "로봇으로 그려줘", "이제 어떻게 해?"라고 하면)
- 편집을 마치면 simulate로 관절 한계를 확인하고, robot_guide로 이 그림이 펜으로 그릴 수 있는 범위인지·시뮬레이션이
  통과했는지·종이 보정이 있는지 확인해 결과를 쉬운 말로 알려 주세요. issues가 있으면 원인과 해결(그림 크기 줄이기 등)을,
  notes가 있으면 주의점을 전하세요. steps_for_the_person의 순서로 사람이 할 일을 안내하세요.
- 로봇 연결·호밍·시작·멈춤은 할 수 없습니다. 사람이 앱의 '로봇으로 그리기' 창에서 직접 합니다. 안내할 때 이렇게 말하세요.
- 펜으로 그릴 수 있는 범위 밖이면 그 그림은 펜 대신 공중 모드(펜을 대지 않고 경로만)로만 확인할 수 있습니다.
  펜으로 그리려면 크기를 줄이거나 '종이·펜 위치 변경: 다시 보정'으로 접촉 영역을 측정해야 합니다.
- 사람이 종이 가운데에 펜 끝을 닿게 세팅하고 시작하면 앱이 그 자세를 종이 중심으로 씁니다. 세팅을 잊으면 그림이 어긋납니다.
- 그리는 동안에는 설정·편집을 바꿀 수 없고 읽기만 됩니다. 진행 상황과 로봇 상태는 앱 창에서 사람이 봅니다.
- '내보내기'(JSON)는 명령줄 실행기(mirobot-draw)에 넘길 때만 쓰는 별도 경로입니다.
답변은 한국어로 짧고 분명하게 하세요."""

VIEW_KINDS = ["original", *stages.PIPELINE_IDS, "edit", "lines", "strokes", "paper"]


def _stage_of(key):
    return next(st.id for st in stages.ALL_STAGES if any(p.key == key for p in st.params))


def param_schema():
    """단계 정의(ParamSpec)에서 set_params 입력 스키마를 만듦 (GUI 조절 칸과 같은 출처)."""
    props = {
        "image_type": {"type": "string", "enum": ["photo", "illustration", "manga"],
                       "description": "이미지 종류 프리셋 (먼저 적용된 뒤 나머지 값이 덮어씀)"},
        "detail": {"type": "string", "enum": ["low", "medium", "high"], "description": "상세도 프리셋"},
    }
    for spec in stages.PARAM_SPECS.values():
        if spec.kind == "bool":
            sch = {"type": "boolean"}
        elif spec.kind == "choice":
            sch = {"type": "string", "enum": [c[0] for c in spec.choices]}
        else:
            sch = {"type": "integer" if spec.kind == "int" else "number", "minimum": spec.lo, "maximum": spec.hi}
        sch["description"] = f"[{stages.STAGE_BY_ID[_stage_of(spec.key)].label}] {spec.label}. {spec.help}".strip()
        props[spec.key] = sch
    return props


TOOLS = [
    {
        "name": "get_state",
        "description": "현재 이미지, 처리 설정, 결과(획 수, 예상 시간, 크기, 편집 기록), 시뮬레이션 결과를 JSON으로 돌려줍니다.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "view",
        "description": (
            "그림을 봅니다. kind: original(컬러 원본), 단계별 결과(source 원본 / prep 전처리 / edges 선 검출 / "
            "trace 뼈대·획, 연회색=버린 조각 / dedupe 겹침 제거, 연회색=빠진 조각 / merge 이어 붙이기 / "
            "simplify 단순화), edit(최종 획), paper(A4 종이 미리보기, 펜 굵기 반영), "
            "strokes(획을 종이 mm 좌표로 확대, 10mm 격자, numbered=true면 번호, region_mm=[x0,y0,x1,y1]로 확대). "
            "edit는 현재 획(회색)과 번호, 제안(빨강=사라짐, 초록=생김)을 보여 줍니다."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "kind": {"type": "string", "enum": VIEW_KINDS},
                "numbered": {"type": "boolean"},
                "region_mm": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4},
                "show_candidates": {"type": "boolean", "description": "edit에서 버린 선(살릴 후보)을 회색 점선과 번호로"},
                "overlay_original": {"type": "number", "minimum": 0, "maximum": 1,
                                     "description": "edit에서 컬러 원본을 비치게 (0~1, 빠진 선 찾기에 좋음)"},
            },
            "required": ["kind"],
            "additionalProperties": False,
        },
    },
    {
        "name": "set_params",
        "description": (
            "처리 설정을 바꾸고, 바뀐 단계부터 다시 계산합니다. 넣은 항목만 바뀌고 범위 밖 값은 잘립니다. "
            "결과로 실제 적용값과 획 수·예상 시간을 돌려줍니다."
        ),
        "parameters": {"type": "object", "properties": param_schema(), "additionalProperties": False},
    },
    {
        "name": "list_strokes",
        "description": "획(과 후보)의 표: 번호, 종류(stroke/candidate), 이유(small 작은 덩어리, spur 잔가지, "
                       "overlap 겹침, deleted 지운 획, added 추가), 길이 mm, 테두리 상자 mm, 점 수. "
                       "region_mm로 좁히면 편합니다. 최대 300줄.",
        "parameters": {"type": "object", "properties": {
            "region_mm": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4},
            "include_candidates": {"type": "boolean"}}, "additionalProperties": False},
    },
    {
        "name": "get_stroke",
        "description": "획 하나의 점 좌표 [점 번호, x_mm, y_mm] (최대 400점). 점 편집 전에 확인하세요.",
        "parameters": {"type": "object", "properties": {"id": {"type": "integer"}}, "required": ["id"],
                       "additionalProperties": False},
    },
    {
        "name": "propose_edits",
        "description": (
            "편집을 제안합니다(바로 적용되지 않음). 화면에 빨강(사라짐)·초록(생김)과 번호로 표시되고, "
            "바뀌는 부위를 확대한 그림을 돌려줍니다. 하나라도 틀리면 아무것도 바뀌지 않습니다. 최대 200개.\n"
            "ops 항목: {op:'delete', ids:[...]} | {op:'restore', ids:[후보 번호]} | "
            "{op:'delete_region', region_mm:[x0,y0,x1,y1], mode:'inside'|'crossing'|'outside'} | "
            "{op:'move_point', id, index, to_mm:[x,y]} | {op:'delete_points', id, indices:[...]} | "
            "{op:'insert_point', id, after_index, at_mm:[x,y]} | {op:'smooth', id, strength:1~5} | "
            "{op:'split', id, index} | {op:'join', a, b} | {op:'add_stroke', points_mm:[[x,y],...]}\n"
            "apply_now=true는 사용자가 '바로 해'라고 했을 때만 쓰세요."
        ),
        "parameters": {"type": "object", "properties": {
            "ops": {"type": "array", "minItems": 1, "maxItems": 200,
                    "items": {"type": "object", "properties": {"op": {"type": "string", "enum": [
                        "delete", "restore", "delete_region", "move_point", "delete_points", "insert_point",
                        "smooth", "split", "join", "add_stroke"]}}, "required": ["op"]}},
            "apply_now": {"type": "boolean"}}, "required": ["ops"], "additionalProperties": False},
    },
    {
        "name": "apply_proposals",
        "description": "제안을 적용합니다. 모두 적용하거나, exclude=[번호]로 빼거나, only=[번호]만. "
                       "잇기·자르기로 묶인 번호는 함께 골라야 적용됩니다. 고르지 않은(뺀) 제안은 확인 전 제안으로 남습니다(취소는 discard_proposals).",
        "parameters": {"type": "object", "properties": {
            "exclude": {"type": "array", "items": {"type": "integer"}},
            "only": {"type": "array", "items": {"type": "integer"}}}, "additionalProperties": False},
    },
    {
        "name": "discard_proposals",
        "description": "제안을 취소합니다. ids를 빼면 전부.",
        "parameters": {"type": "object", "properties": {"ids": {"type": "array", "items": {"type": "integer"}}},
                       "additionalProperties": False},
    },
    {
        "name": "undo",
        "description": "마지막으로 적용한 편집을 되돌립니다.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "simulate",
        "description": "로봇 기구학 시뮬레이션으로 관절 한계(Soft limit)를 검사합니다. 수십 초 걸릴 수 있습니다.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "compare",
        "description": (
            "사진과 지금 그려질 그림이 얼마나 닮았는지 읽기 전용으로 비교합니다. 원본 | 그림 | 차이(빨강=사진보다 너무 밝게, "
            "파랑=너무 어둡게, 초록=일치) 그림과 지표(tone_match 명암 닮음, edge_recall 윤곽 재현, edge_precision, "
            "under/over_shaded_pct, ink_coverage_pct)와 개선 hints를 돌려줍니다."),
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "robot_guide",
        "description": (
            "이 그림을 로봇으로 그릴 준비가 됐는지 읽기 전용으로 알려 줍니다: 펜으로 그릴 수 있는 범위와 공중 확인용 "
            "범위, 그림이 그 범위 안인지, 시뮬레이션·종이 보정 상태, 문제(issues)와 주의(notes), 사람이 '로봇으로 그리기' "
            "창에서 할 순서. 로봇을 움직이지는 않습니다."),
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
]
TOOL_NAMES = {t["name"] for t in TOOLS}
READ_ONLY_TOOLS = {"get_state", "view", "list_strokes", "get_stroke", "simulate", "robot_guide", "compare"}   # 그리는 중에도 쓸 수 있는 도구


def image_part(img_bgr, max_side=1024):
    h, w = img_bgr.shape[:2]
    s = max_side / max(h, w)
    if s < 1:
        img_bgr = cv2.resize(img_bgr, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".png", img_bgr)
    return {"type": "image", "data": base64.b64encode(buf.tobytes()).decode("ascii"), "mime": "image/png"}


def text_part(obj):
    return {"type": "text", "text": obj if isinstance(obj, str) else json.dumps(obj, ensure_ascii=False)}


class AgentToolbox:
    """도구 실행기. on_change()는 세션이 바뀌었을 때 화면을 갱신하도록 GUI가 넘겨 줌."""

    def __init__(self, session, on_change=None):
        self.session = session
        self.on_change = on_change or (lambda what: None)

    def call(self, name, args):
        """(parts, is_error). 예외를 밖으로 던지지 않고 오류 문구로 돌려줌."""
        try:
            if name not in TOOL_NAMES:
                raise SessionError(f"알 수 없는 도구: {name}")
            if getattr(self.session, "drawing_lock", False) and name not in READ_ONLY_TOOLS:
                raise SessionError("로봇이 그리는 중이라 설정·편집을 바꿀 수 없습니다. 끝난 뒤 다시 요청하세요.")
            return getattr(self, "_" + name)(**(args or {})), False
        except SessionError as e:
            return [text_part(f"오류: {e}")], True
        except TypeError as e:  # 잘못된 인자
            return [text_part(f"오류: 인자가 올바르지 않습니다 ({e})")], True
        except Exception as e:  # 예상 못 한 오류도 대화를 끊지 않게
            return [text_part(f"오류: {type(e).__name__}: {e}")], True

    # --- 도구 구현
    def _get_state(self):
        return [text_part(self.session.state())]

    def _view(self, kind, numbered=None, region_mm=None, show_candidates=False, overlay_original=0.0):
        numbered = (kind == "edit") if numbered is None else numbered
        img = self.session.render(kind, region_mm=region_mm, numbered=numbered,
                                  show_candidates=show_candidates, overlay=overlay_original)
        label = f"{kind}" + (" (번호)" if numbered else "") + (f" 영역 {region_mm}" if region_mm else "")
        return [text_part(f"그림: {label}"), image_part(img)]

    def _set_params(self, image_type=None, detail=None, **changes):
        s = self.session
        if image_type or detail:
            s.apply_preset(image_type, detail)
        applied = s.update_params(changes) if changes else {}
        recomputed = s.dirty_stages()
        s.run_current()
        self.on_change("result")
        st = s.state()
        return [text_part({"applied": applied, "recomputed": recomputed,
                           "image_type": st["image_type"], "detail": st["detail"],
                           "result": st.get("result"), "edit": st.get("edit")})]

    def _list_strokes(self, region_mm=None, include_candidates=False):
        return [text_part(self.session.list_strokes(region_mm, include_candidates))]

    def _get_stroke(self, id):
        return [text_part(self.session.get_stroke(id))]

    def _propose_edits(self, ops, apply_now=False):
        s = self.session
        out = s.propose_edits(ops, apply_now=apply_now)
        self.on_change("result" if apply_now else "proposals")
        if apply_now:
            return [text_part({**out, "result": s.state().get("result")})]
        region = s.proposal_region_mm(out["proposed"])
        parts = [text_part({**out, "legend": "빨강=사라짐, 초록=생김, 숫자=번호"})]
        if region:
            parts.append(image_part(s.render("edit", region_mm=region, numbered=True, overlay=0.3)))
        return parts

    def _apply_proposals(self, exclude=None, only=None):
        out = self.session.apply_proposals(exclude=exclude, only=only)
        self.on_change("result")
        return [text_part({**out, "result": self.session.state().get("result")})]

    def _discard_proposals(self, ids=None):
        n = self.session.discard_proposals(ids)
        self.on_change("proposals")
        return [text_part({"discarded": n})]

    def _undo(self):
        desc = self.session.undo()
        self.on_change("result")
        return [text_part({"undone": desc, "result": self.session.state().get("result")})]

    def _simulate(self):
        summary = self.session.simulate()
        self.on_change("sim")
        return [text_part(summary)]

    def _compare(self):
        metrics, img = self.session.compare()
        return [text_part(metrics), image_part(img, max_side=1400)]

    def _robot_guide(self):
        return [text_part(self.session.robot_guide())]
