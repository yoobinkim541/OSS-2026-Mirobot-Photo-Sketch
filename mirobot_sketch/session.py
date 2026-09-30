"""
SketchSession — 이미지 한 장의 작업 상태 (GUI와 에이전트가 함께 사용)
======================================================================
이미지, 처리 설정, 처리 결과(획), 편집 기록(되돌리기), 시뮬레이션 결과를 한곳에
둡니다. GUI 버튼과 에이전트 도구가 모두 이 객체를 통해 같은 상태를 바꾸므로,
화면과 대화가 어긋나지 않습니다. Tk에 의존하지 않아 단위 테스트가 가능합니다.

스레드: 무거운 계산은 호출한 스레드에서 돌고, 상태 변경은 lock으로 보호합니다.
"""

import dataclasses
import threading

import cv2
import numpy as np

from . import draw_executor as de
from . import edits
from . import paper_mapping as pm
from . import faces, limits, paths, presets, stages
from . import sketch_pipeline as sp

FACE_CROP_SIDE = 500     # 얼굴 세밀 처리용 조각의 긴 변 (px)


class SessionError(Exception):
    """사용자·에이전트에게 그대로 보여 줄 수 있는 오류."""


def _as_int(v, what="번호"):
    """정수만 받음 (에이전트가 "12"·3.7·true를 보내면 거부: 문자열은 글자마다 번호로 읽히는 등 엉뚱한 획을 고침)."""
    if isinstance(v, bool) or not isinstance(v, (int, np.integer)):
        raise SessionError(f"{what}는 정수여야 합니다: {v!r}")
    return int(v)


def _as_ints(v, what="번호"):
    if not isinstance(v, (list, tuple)):
        raise SessionError(f"{what} 목록이어야 합니다: {v!r}")
    return [_as_int(x, what) for x in v]


def _as_xy(v):
    if (not isinstance(v, (list, tuple)) or len(v) != 2
            or not all(isinstance(t, (int, float, np.number)) and not isinstance(t, bool) for t in v)):
        raise SessionError(f"좌표는 [x, y] 숫자 두 개여야 합니다: {v!r}")
    x, y = float(v[0]), float(v[1])
    if not (np.isfinite(x) and np.isfinite(y)):
        raise SessionError(f"좌표가 유한한 숫자가 아닙니다: {v!r}")
    return x, y


class SketchSession:
    def __init__(self, cfg=None):
        self.cfg = cfg or de.load_config()
        self.lock = threading.RLock()
        self._run_lock = threading.Lock()   # 파이프라인 계산은 한 번에 하나 (GUI 자동 재계산 + 에이전트)
        self.image_path = None
        self.color = None
        self.image_type = "illustration"
        self.detail = "high"
        self.generation = 0                 # 설정이 바뀔 때마다 +1 (오래된 계산 결과를 버리는 기준)
        self.params = stages.default_params()
        self.apply_preset("illustration")
        self.pipeline = stages.Pipeline()
        self._inputs_cache = {}
        self._faces_cache, self._frame_cache = {}, {}   # 원본 얼굴 검출 / 구도별 입력
        self._frame_box = None                          # 지금 결과의 자르기 틀 (원본 좌표, None=전체)
        self.result = None
        self.history = []      # 되돌리기용 스냅숏 (번호표, 편집 기록, 다음 번호, 설명)
        self.edit_log = []     # 적용된 편집 설명 (재현·기록용)
        self.sim = None
        self.drawing_lock = False   # 로봇이 그리는 중이면 True (에이전트 도구가 설정·편집을 바꾸지 못하게)
        self.table = {}        # 번호 -> {"kind": stroke|candidate|gone, "poly": px 배열, "reason": 이유}
        self.next_id = 1
        self.book = {"removed": [], "added": [], "trash": []}   # 모양 기반 편집 기록 (다시 계산해도 유지)
        self.pending = None    # 제안이 반영된 번호표 사본 (제안이 없으면 None)
        self.proposals = {}    # 번호 -> {"id", "group"}
        self._groups, self._group_seq = {}, 0
        self.unapplied = 0
        self.notice = ""
        self._last_simplify = None
        self.proposal_epoch = 0  # 번호를 새로 매길 때마다 +1 (다른 번호 체계의 제안 표시를 섞지 않게)
        self._fit_outline = []   # 종이 배치를 정하는 윤곽 획 (파이프라인 결과 기준으로 고정)
        self._fit_strokes = []   # 종이 배치(배율·중심)를 정하는 획 = 윤곽 + 명암 빗금: 편집해도 mm 좌표가 안 움직이게

    # ------------------------------------------------------------ 설정
    def param_specs(self):
        """단계 정의의 조절 항목. box_mm 상한은 설정 파일의 실물 미확인 범위(±60mm -> 120)."""
        plim = limits.pending_region(self.cfg).x_max * 2
        specs = dict(stages.PARAM_SPECS)
        specs["box_mm"] = dataclasses.replace(specs["box_mm"], hi=plim)
        return specs

    def apply_preset(self, image_type=None, detail=None):
        """이미지 종류 프리셋(상세도·rembg·미디언)과 상세도 프리셋(Canny·길이·단순화)을 적용."""
        with self.lock:
            if image_type:
                if image_type not in presets.IMAGE_TYPES:
                    raise SessionError(f"알 수 없는 이미지 종류: {image_type} (가능: {', '.join(presets.IMAGE_TYPES)})")
                t = presets.IMAGE_TYPES[image_type]
                self.image_type = image_type
                self.detail = t["detail"]
                self.params.update(rembg=t["rembg"], median_ksize=t["median"])
            if detail:
                if detail not in presets.DETAIL_PRESETS:
                    raise SessionError(f"알 수 없는 상세도: {detail} (가능: low, medium, high)")
                self.detail = detail
            lo, hi, ml, eps = presets.DETAIL_PRESETS[self.detail]
            self.params.update(canny_low=lo, canny_high=hi, min_length_px=ml, epsilon_px=eps)
            self.generation += 1

    def update_params(self, changes):
        """개별 설정 변경. 범위를 벗어나면 잘라서 적용하고 실제 적용값을 돌려줌."""
        specs = self.param_specs()
        applied = {}
        with self.lock:
            for k, v in changes.items():
                if k == "line_source":          # 예전 이름: canny -> 밝기(luma), dark -> 어두운 선
                    if v not in ("canny", "dark"):
                        raise SessionError("line_source는 canny 또는 dark")
                    k, v = "edge_mode", ("dark" if v == "dark" else "luma")
                spec = specs.get(k)
                if spec is None:
                    raise SessionError(f"알 수 없는 설정: {k}")
                try:
                    v = spec.clamp(v)
                except (TypeError, ValueError) as e:
                    raise SessionError(str(e)) from None
                self.params[k] = v
                applied[k] = v
            if applied:
                self.generation += 1
        return applied

    # ------------------------------------------------------------ 실패하면 되돌리기
    _STATE = ("result", "table", "next_id", "book", "pending", "proposals", "_groups", "_group_seq", "history",
              "edit_log", "notice", "_last_simplify", "unapplied", "sim", "_fit_strokes", "_fit_outline",
              "proposal_epoch", "_frame_box")

    def _snapshot(self):
        snap = {k: getattr(self, k) for k in self._STATE}
        snap["history"], snap["edit_log"] = list(self.history), list(self.edit_log)
        snap["proposals"], snap["_groups"] = dict(self.proposals), dict(self._groups)   # _add_group이 제자리에서 바꿈
        return snap

    def _restore(self, snap):
        for k, v in snap.items():
            setattr(self, k, v)

    def reset(self):
        """그림 한 장이 끝났을 때: 사진과 작업 내용(획·편집 기록·시뮬레이션·제안)을 모두 비우고 새 그림을 그릴 수 있게 한다.
        처리 설정(이미지 종류·상세도·크기 등)은 다음 그림에도 쓰도록 그대로 둔다."""
        with self.lock:
            self.image_path = None
            self.color = None
            self._inputs_cache, self._faces_cache, self._frame_cache = {}, {}, {}
            self._frame_box = None
            self.result = self.sim = None
            self.history, self.edit_log = [], []
            self.book = {"removed": [], "added": [], "trash": []}
            self.table, self.next_id = {}, 1
            self._clear_proposals()
            self.unapplied = 0
            self.notice = ""
            self._last_simplify = None
            self._fit_strokes, self._fit_outline = [], []
            self.pipeline = stages.Pipeline()          # 단계별 캐시(큰 이미지 포함)도 비움
            self.proposal_epoch += 1
            self.generation += 1

    # ------------------------------------------------------------ 처리
    def set_image(self, path):
        gray = sp.load_gray(path)
        color = sp.load_color(path)
        with self.lock:
            self.image_path = str(path)
            self.color = color            # 화면·에이전트에 보여 줄 컬러 원본
            self._inputs_cache = {(str(path), False): {"gray": gray, "color": color}}
            self._faces_cache, self._frame_cache = {}, {}
            self._frame_box = None
            self.result = self.sim = None
            self.history, self.edit_log = [], []
            self.book = {"removed": [], "added": [], "trash": []}
            self._clear_proposals()
            self._last_simplify = None
            self.proposal_epoch += 1
            self.generation += 1

    def _raw(self, path, cache, rembg):
        """원본 크기 컬러와 긴 변 800px 사본 (배경 제거면 rembg 결과)."""
        key = (path, bool(rembg))
        if key not in cache:
            full = sp.remove_background_bgr(path, max_side=None, cache_dir=paths.cache_dir())
            color = sp.resize_max_side(full)
            cache[key] = {"gray": cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), "color": color, "full": full}
        entry = cache[key]
        if "full" not in entry:
            entry["full"] = sp.load_color(path, None)
        return entry

    def _faces(self, path, full, faces_cache):
        """원본에서 찾은 얼굴 (이미지마다 한 번, 배경 제거 전 원본 기준)."""
        if path not in faces_cache:
            faces_cache[path] = faces.detect_faces(full)
        return faces_cache[path]

    def _inputs(self, rembg, frame="full", box_mm=100):
        """파이프라인 입력과 캐시 키 (경로, 배경 제거, 원본 기준 자르기 틀 또는 None).
        구도는 원본 해상도에서 자른 뒤 긴 변 800px로 맞춤. 얼굴은 작업 이미지 좌표로 옮기고,
        얼굴 세밀 처리용으로 얼굴 영역을 원본에서 잘라 긴 변 FACE_CROP_SIDE로 맞춘 조각을 같이 넘김."""
        # 배경 제거는 약 1분: 그사이 다른 이미지를 열어도 결과가 새 이미지 캐시에 섞이지 않게 시작 시점의 경로·캐시를 씀
        with self.lock:
            path, cache = self.image_path, self._inputs_cache
            faces_cache, frame_cache = self._faces_cache, self._frame_cache
        plain = self._raw(path, cache, False)
        found = self._faces(path, plain["full"], faces_cache)
        entry = self._raw(path, cache, rembg) if rembg else plain
        full = entry["full"]
        fh, fw = full.shape[:2]
        eff_mm = limits.effective_long_mm(limits.pending_region(self.cfg), box_mm, fw, fh)   # 영역에 맞춘 실제 크기
        kind, box, note = faces.choose_frame(found, fw, fh, eff_mm, frame)
        key = (path, bool(rembg), box)
        if (key, kind, note) in frame_cache:
            return frame_cache[(key, kind, note)], key
        if box is None:
            x0 = y0 = 0
            gray, color = entry["gray"], entry["color"]
            k = color.shape[0] / fh
        else:
            x0, y0, x1, y1 = box
            color = sp.resize_max_side(full[y0:y1, x0:x1])
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            k = color.shape[0] / (y1 - y0)
        h, w = gray.shape
        work_faces = [f.scaled(k, -x0 * k, -y0 * k) for f in found]
        work_faces = [f for f in work_faces
                      if 0 <= f.box[0] + f.box[2] / 2 < w and 0 <= f.box[1] + f.box[3] / 2 < h]
        crops = []
        for f in work_faces:
            (cx, cy), (ax, ay) = faces.ellipse_of(f)
            ox0, oy0 = max(0, int((cx - ax) / k + x0)), max(0, int((cy - ay) / k + y0))
            ox1, oy1 = min(fw, int(np.ceil((cx + ax) / k + x0))), min(fh, int(np.ceil((cy + ay) / k + y0)))
            piece = full[oy0:oy1, ox0:ox1]
            sc = FACE_CROP_SIDE / max(piece.shape[:2])
            img = cv2.resize(piece, (round(piece.shape[1] * sc), round(piece.shape[0] * sc)),
                             interpolation=cv2.INTER_CUBIC if sc > 1 else cv2.INTER_AREA)
            crops.append({"img": img, "scale": sc / k, "origin": np.array([(ox0 - x0) * k, (oy0 - y0) * k])})
        inputs = {"gray": gray, "color": color, "faces": work_faces, "face_crops": crops,
                  "region": limits.pending_region(self.cfg),
                  "frame": {"kind": kind, "box_orig": box, "notice": note}}
        frame_cache[(key, kind, note)] = inputs
        return inputs, key

    def needs_rembg(self):
        """지금 설정으로 계산하면 배경 제거(오래 걸림)를 새로 해야 하는지 (GUI 안내용)."""
        with self.lock:
            return bool(self.params.get("rembg")) and (self.image_path, True) not in self._inputs_cache

    def dirty_stages(self):
        """지금 설정으로 다시 계산될 단계들 (GUI가 흐리게 표시)."""
        with self.lock:
            p = dict(self.params)
        with self.lock:
            entry = self._inputs_cache.get((self.image_path, bool(p["rembg"])), {})
            found = self._faces_cache.get(self.image_path)
        if "full" in entry and found is not None:     # 구도 틀은 검출해 둔 얼굴로 바로 계산 (종이 크기로도 바뀜)
            fh, fw = entry["full"].shape[:2]
            eff_mm = limits.effective_long_mm(limits.pending_region(self.cfg), p["box_mm"], fw, fh)
            box = faces.choose_frame(found, fw, fh, eff_mm, p["frame"])[1]
            first = self.pipeline.dirty_from((self.image_path, bool(p["rembg"]), box), p)
        else:
            first = "source"
        ids = [s.id for s in stages.ALL_STAGES]
        start = ids.index(first) if first else ids.index("edit")
        return ids[start:]

    def run(self):
        """현재 설정으로 처리. 더 새로운 설정이 들어와 계산을 버렸으면 None."""
        if not self.image_path:
            raise SessionError("먼저 이미지를 열어야 합니다.")
        with self._run_lock:
            with self.lock:
                p, gen = dict(self.params), self.generation
            inputs, ikey = self._inputs(bool(p["rembg"]), p["frame"], p["box_mm"])
            try:
                outs = self.pipeline.run(inputs, ikey, p, is_current=lambda: self.generation == gen)
            except stages.StaleRun:
                return None
            with self.lock:
                if self.generation != gen:
                    return None
                snap = self._snapshot()
                try:
                    reframed = self.result is not None and ikey[2] != self._frame_box
                    if reframed:     # 좌표계가 바뀜: 이미지를 새로 열 때처럼 편집 기록을 비움
                        self.history, self.edit_log = [], []
                        self.book = {"removed": [], "added": [], "trash": []}
                        self._clear_proposals()
                    self._frame_box = ikey[2]
                    same = outs["simplify"] is self._last_simplify and self.result is not None
                    fr = inputs["frame"]
                    self.result = {"base": inputs["gray"], "color": inputs["color"], "edges": outs["edges"]["edges"],
                                   "frame": {"kind": fr["kind"], "notice": fr["notice"]},
                                   "faces": len(inputs["faces"]),
                                   "stages": outs, "path": self.image_path, "params": p,
                                   "image_type": self.image_type, "detail": self.detail,
                                   "placement": self.result["placement"] if same else None}
                    if not same:   # 파이프라인 결과가 바뀜: 번호를 새로 매기고, 적용 전 제안은 취소
                        n = len(self.proposals)
                        self._build_table(outs, inputs["gray"].shape)
                        self._clear_proposals()
                        self.proposal_epoch += 1
                        self.notice = ("설정이 바뀌어 번호를 새로 매겼습니다"
                                       + (f" (제안 {n}건을 취소했습니다)" if n else ""))
                        if reframed:
                            self.notice = "구도가 바뀌어 편집 기록을 초기화했습니다"
                        if fr["notice"]:
                            self.notice += f" · {fr['notice']}"
                        self._last_simplify = outs["simplify"]
                        self._fit_outline = [e["poly"] for e in self.table.values() if e["kind"] == "stroke"]
                    # 명암 빗금은 윤곽과 함께 배치를 정함 (빗금 설정만 바뀌어도 다시 맞춤)
                    self._fit_strokes = self._fit_outline + [np.asarray(h, np.float64)
                                                              for h in outs["tone"].get("hatch_strokes", [])]
                    self._refresh_drawing(refit=True)
                except Exception:
                    self._restore(snap)   # 반쯤 바뀐 결과를 남기지 않음 (예: 획이 0개)
                    raise
                return self.result

    def run_current(self, max_tries=5):
        """최신 설정의 결과가 나올 때까지 다시 시도 (에이전트용)."""
        for _ in range(max_tries):
            r = self.run()
            if r is not None:
                return r
        raise SessionError("설정이 계속 바뀌고 있어 계산을 마치지 못했습니다. 잠시 후 다시 시도하세요.")

    def _set_strokes(self, strokes_px, strokes_mm):
        """획이 바뀔 때마다 시간·미리보기·범위 검사를 다시 계산."""
        r = self.result
        r["strokes_px"], r["strokes_mm"] = strokes_px, strokes_mm
        lists = [[tuple(pt) for pt in s] for s in strokes_mm]
        r["timing"] = de.estimate_time(lists, self.cfg)
        r["paper"] = pm.render_paper_preview(strokes_mm, self.cfg["pen"].get("line_width_mm", 0.5), 4.0,
                                             limit_mm=limits.executor_region(self.cfg),
                                             pending_limit_mm=limits.pending_region(self.cfg))
        r["out_of_limits"] = len(de.check_limits(lists, self.cfg))
        r["out_of_pending"] = len(de.check_limits(lists, self.cfg, pending=True))
        self.sim = None

    def _need_result(self):
        if not self.result:
            raise SessionError("먼저 처리를 실행해야 합니다.")

    # ------------------------------------------------------------ 번호표
    def _build_table(self, outs, shape):
        """파이프라인 결과 + 편집 기록 -> 번호표 (획 1..N, 추가한 획, 후보 순)."""
        base = outs["simplify"]["strokes"]
        keep, hit = edits.remove_matching(base, [r["poly"] for r in self.book["removed"]], shape)
        table, nid = {}, 1

        def put(kind, poly, reason):
            nonlocal nid
            table[nid] = {"kind": kind, "poly": poly, "reason": reason}
            nid += 1

        for i in keep:
            put("stroke", np.asarray(base[i], np.float64), "")
        for a in self.book["added"]:
            put("stroke", a, "added")
        cands = [(np.asarray(sp.simplify_strokes([poly], 1.0)[0], np.float64), reason)
                 for poly, reason in stages.candidates_of(outs)]
        # 되살려 획이 된(추가한) 모양과 같은 후보는 다시 후보로 보이지 않게
        keep_c, _ = edits.remove_matching([c for c, _ in cands], self.book["added"], shape)
        for k in keep_c:
            put("candidate", *cands[k])
        for r, h in zip(self.book["removed"], hit):
            if h and r["reason"] == "deleted":
                put("candidate", r["poly"], "deleted")
        for t in self.book["trash"]:
            put("candidate", t, "deleted")
        self.table, self.next_id = table, nid
        self.unapplied = hit.count(False)

    def _refresh_drawing(self, refit=False):
        """번호표의 획 -> 그리는 순서 -> 종이 mm -> 시간·미리보기.
        종이 배치(배율·중심)는 refit일 때만(다시 계산·크기 변경) 파이프라인 결과 기준으로 정함. 편집을 적용할 때
        다시 맞추면 모든 획의 mm 좌표가 움직여, 에이전트가 들고 있는 좌표가 틀어짐."""
        strokes = [e["poly"] for _, e in sorted(self.table.items()) if e["kind"] == "stroke"]
        if not strokes:
            raise SessionError("획이 없습니다. 상세도를 높이거나 '약한 선 민감도(Canny 하한)'를 낮춰 보세요.")
        # 명암 빗금은 편집 번호표에 넣지 않고(번호가 수백 개로 불어남) 그리기 순서를 정할 때 윤곽과 합친다
        hatch = [np.asarray(h, np.float64) for h in self.result["stages"]["tone"].get("hatch_strokes", [])]
        ordered = sp.order_strokes(strokes + hatch)
        if refit or self.result.get("placement") is None:
            box = self.params["box_mm"]
            _, self.result["placement"] = pm.pixels_to_paper(self._fit_strokes or strokes, box_mm=(box, box),
                                                             region=limits.pending_region(self.cfg))
        strokes_mm = [self.px_to_mm(s) for s in ordered]
        allp = np.vstack(strokes_mm)
        size = allp.max(axis=0) - allp.min(axis=0)
        self.result["placement"] = {**self.result["placement"], "drawing_width_mm": round(float(size[0]), 3),
                                    "drawing_height_mm": round(float(size[1]), 3)}
        self._set_strokes(list(ordered), strokes_mm)

    # ------------------------------------------------------------ 좌표
    def mm_to_px(self, xy):
        x, y = _as_xy(xy)
        if not limits.pending_region(self.cfg).contains(x, y):
            raise SessionError(f"좌표 ({x:.1f}, {y:.1f})mm가 허용 범위(넓은 범위) 밖입니다")
        sc, cx, cy, ox, oy = self._xf()
        return ((x - ox) / sc + cx, -(y - oy) / sc + cy)

    def _xf(self):
        """종이 배치의 (배율 mm/px, 중심 x, 중심 y, 종이 위치 x, y mm) — 반올림하지 않은 값."""
        pl = self.result["placement"]
        t = pl.get("transform")
        if t:
            return t["scale"], t["cx"], t["cy"], t.get("ox", 0.0), t.get("oy", 0.0)
        return pl["scale_mm_per_px"], pl["center_px"][0], pl["center_px"][1], 0.0, 0.0

    def px_to_mm(self, pts):
        sc, cx, cy, ox, oy = self._xf()
        p = np.asarray(pts, np.float64)
        return np.column_stack([(p[:, 0] - cx) * sc + ox, -(p[:, 1] - cy) * sc + oy])

    def _region_px(self, region_mm):
        x0, y0, x1, y1 = [float(v) for v in region_mm]
        sc, cx, cy, ox, oy = self._xf()
        xs = sorted(((x0 - ox) / sc + cx, (x1 - ox) / sc + cx))
        ys = sorted((-(y0 - oy) / sc + cy, -(y1 - oy) / sc + cy))
        return xs[0], ys[0], xs[1], ys[1]

    # ------------------------------------------------------------ 편집 실행 (번호표 사본에)
    def _exec(self, op, table, alloc):
        """편집 하나를 table(사본)에 반영. 반환: 묶음 목록 [[번호...], ...]."""
        if not isinstance(op, dict):
            raise SessionError("편집은 {\"op\": ...} 객체여야 합니다")
        name = op.get("op")

        def entry(i, kind="stroke"):
            i = _as_int(i)
            e = table.get(i)
            if e is None or e["kind"] != kind:
                what = "획이" if kind == "stroke" else "후보가"
                raise SessionError(f"{i}번은 {what} 아닙니다")
            return i, e

        if name in ("delete", "delete_region"):
            ids = (_as_ints(op["ids"]) if name == "delete"
                   else self._ids_in_region(table, op["region_mm"], op.get("mode", "inside"),
                                            float(op.get("min_fraction", 0.5))))
            groups = []
            for i in ids:
                i, e = entry(i)
                table[i] = {**e, "kind": "candidate", "reason": "deleted"}
                groups.append([i])
            return groups
        if name == "restore":
            groups = []
            for i in _as_ints(op["ids"]):
                i, e = entry(i, "candidate")
                table[i] = {**e, "kind": "stroke"}
                groups.append([i])
            return groups
        if name in ("move_point", "delete_points", "insert_point", "smooth"):
            i, e = entry(op["id"])
            p = e["poly"]
            if name == "move_point":
                q = edits.move_point(p, _as_int(op["index"], "점 번호"), self.mm_to_px(op["to_mm"]))
            elif name == "delete_points":
                q = edits.delete_points(p, _as_ints(op["indices"], "점 번호"))
            elif name == "insert_point":
                q = edits.insert_point(p, _as_int(op["after_index"], "점 번호"), self.mm_to_px(op["at_mm"]))
            else:
                q = edits.smooth(p, _as_int(op.get("strength", 2), "strength"))
            table[i] = {**e, "poly": q}
            return [[i]]
        if name == "split":
            i, e = entry(op["id"])
            a, b = edits.split(e["poly"], _as_int(op["index"], "점 번호"))
            nid = alloc()
            table[i] = {**e, "poly": a}
            table[nid] = {"kind": "stroke", "poly": b, "reason": "added"}
            return [[i, nid]]
        if name == "join":
            (a, ea), (b, eb) = entry(op["a"]), entry(op["b"])
            if a == b:
                raise SessionError("서로 다른 두 획을 골라야 합니다")
            table[a] = {**ea, "poly": edits.join(ea["poly"], eb["poly"])}
            table[b] = {**eb, "kind": "gone"}
            return [[a, b]]
        if name == "add_stroke":
            pts = op["points_mm"]
            if not isinstance(pts, (list, tuple)) or not 2 <= len(pts) <= 200:
                raise SessionError("add_stroke는 점 2~200개")
            nid = alloc()
            table[nid] = {"kind": "stroke", "poly": np.array([self.mm_to_px(xy) for xy in pts]), "reason": "added"}
            return [[nid]]
        raise SessionError(f"알 수 없는 편집: {name} (delete, restore, delete_region, move_point, delete_points, "
                           "insert_point, smooth, split, join, add_stroke)")

    def _ids_in_region(self, table, region_mm, mode, min_fraction):
        if mode not in ("inside", "outside", "crossing"):
            raise SessionError("mode는 inside(대부분 안) / crossing(조금이라도 걸침) / outside(대부분 밖)")
        x0, y0, x1, y1 = self._region_px(region_mm)
        ids = []
        for i, e in table.items():
            if e["kind"] != "stroke":
                continue
            d = edits.densify(e["poly"])
            inside = ((d[:, 0] >= x0) & (d[:, 0] <= x1) & (d[:, 1] >= y0) & (d[:, 1] <= y1)).mean()
            hit = (inside > 0 if mode == "crossing"
                   else (inside if mode == "inside" else 1 - inside) >= min_fraction)
            if hit:
                ids.append(i)
        return ids

    # ------------------------------------------------------------ 제안
    def _clear_proposals(self):
        self.proposals, self._groups, self.pending = {}, {}, None   # 제안 목록을 먼저 비움 (pending만 None인 순간 없게)

    def _add_group(self, ids):
        merged = set(ids)
        for g in {self.proposals[i]["group"] for i in ids if i in self.proposals}:
            merged |= self._groups.pop(g)
        self._group_seq += 1
        self._groups[self._group_seq] = merged
        for i in merged:
            self.proposals[i] = {"id": i, "group": self._group_seq}

    def propose_edits(self, ops, apply_now=False):
        """편집을 제안 목록에 추가. 하나라도 틀리면 아무것도 바뀌지 않음."""
        with self.lock:
            self._need_result()
            if not isinstance(ops, list) or not ops:
                raise SessionError("ops가 비어 있습니다")
            if len(ops) > edits.MAX_OPS:
                raise SessionError(f"한 번에 편집은 {edits.MAX_OPS}개까지입니다 ({len(ops)}개)")
            table = dict(self.pending if self.pending is not None else self.table)
            counter = [self.next_id]

            def alloc():
                counter[0] += 1
                return counter[0] - 1

            groups = []
            for n, op in enumerate(ops):
                try:
                    groups += self._exec(op, table, alloc)
                except (KeyError, TypeError, ValueError, SessionError) as e:
                    label = op.get("op") if isinstance(op, dict) else op
                    msg = f"필요한 값 {e} 없음" if isinstance(e, KeyError) else str(e)
                    raise SessionError(f"ops[{n}] ({label}): {msg}") from None
            touched = {i for g in groups for i in g}
            if apply_now and touched & set(self.proposals):
                raise SessionError(f"{sorted(touched & set(self.proposals))}번에 확인 전 제안이 있어 바로 적용할 수 "
                                   "없습니다. 먼저 그 제안을 적용하거나 취소하세요.")
            snap = self._snapshot()
            self.pending, self.next_id = table, counter[0]
            for g in groups:
                self._add_group(g)
            created = sorted({i for g in groups for i in g})
            if apply_now:
                try:
                    return self.apply_proposals(only=created)
                except SessionError as e:
                    self._restore(snap)
                    raise SessionError(f"{e} (바로 적용이 거부되어 제안도 추가하지 않았습니다)") from None
            return {"proposed": created, "total": len(self.proposals)}

    def proposal_views(self):
        """제안마다 사라질 모양(before, 빨강)과 생길 모양(after, 초록). 다른 스레드가 바꾸는 중이면 기다림."""
        with self.lock:
            return self._proposal_views()

    def _proposal_views(self):
        out = []
        for i in sorted(self.proposals):
            old, new = self.table.get(i), self.pending[i]
            before = old["poly"] if old is not None and old["kind"] == "stroke" else None
            after = new["poly"] if new["kind"] == "stroke" else None
            if before is not None and after is before:
                continue
            out.append({"id": i, "before": before, "after": after})
        return out

    def proposal_region_mm(self, ids=None, margin_mm=5.0):
        with self.lock:
            polys = [p for v in self._proposal_views() if ids is None or v["id"] in ids
                     for p in (v["before"], v["after"]) if p is not None]
            if not polys:
                return None
            mm = self.px_to_mm(np.vstack(polys))
        (x0, y0), (x1, y1) = mm.min(axis=0) - margin_mm, mm.max(axis=0) + margin_mm
        return [float(x0), float(y0), float(x1), float(y1)]

    @staticmethod
    def _find(lst, poly):
        return next((k for k, x in enumerate(lst) if x is poly), None)

    def _record(self, book, old, new):
        """번호 하나의 변화(old -> new)를 모양 기록에 반영."""
        was = old is not None and old["kind"] == "stroke"
        now = new["kind"] == "stroke"
        if was and now and new["poly"] is old["poly"]:
            return
        if (not was and now and old is not None and old["reason"] == "deleted"
                and new["poly"] is old["poly"]):
            k = next((k for k, r in enumerate(book["removed"]) if r["poly"] is old["poly"]), None)
            if k is not None:
                book["removed"].pop(k)      # 지웠던 파이프라인 획을 되살림
                return
            k = self._find(book["trash"], old["poly"])
            if k is not None:
                book["trash"].pop(k)        # 지웠던 추가 획(점 편집 결과 등)을 되살림
                book["added"].append(new["poly"])
                return
        if was:
            k = self._find(book["added"], old["poly"])
            if k is not None:
                book["added"].pop(k)
                if new["kind"] == "candidate":
                    book["trash"].append(old["poly"])   # 다시 계산해도 후보로 남아 되살릴 수 있게
            else:
                reason = "deleted" if new["kind"] == "candidate" else "replaced"
                book["removed"].append({"poly": old["poly"], "reason": reason})
        if now:
            book["added"].append(new["poly"])

    def apply_proposals(self, exclude=None, only=None):
        with self.lock:
            if not self.proposals:
                raise SessionError("적용할 제안이 없습니다")
            ids = set(self.proposals)
            given = set(_as_ints(only if only is not None else exclude or []))
            if given - ids:
                raise SessionError(f"제안에 없는 번호: {sorted(given - ids)[:10]}")
            sel = given if only is not None else ids - given
            groups = [g for g in self._groups.values() if g <= sel]
            apply_ids = set().union(*groups) if groups else set()
            partial = sorted(sel - apply_ids)
            if not apply_ids:
                raise SessionError("적용할 제안이 선택되지 않았습니다" + (
                    f" ({partial}번은 다른 번호와 묶인 편집(잇기·자르기)이라 함께 골라야 합니다)" if partial else ""))
            table = dict(self.table)
            book = {k: list(v) for k, v in self.book.items()}
            for i in sorted(apply_ids):
                self._record(book, self.table.get(i), self.pending[i])
                table[i] = self.pending[i]
            if not any(e["kind"] == "stroke" for e in table.values()):
                raise SessionError("모든 획을 지울 수는 없습니다 (드로잉에는 획이 1개 이상 필요)")
            desc = f"편집 {len(apply_ids)}건 적용: {sorted(apply_ids)[:20]}"
            snap = self._snapshot()
            try:
                self.history.append((self.table, self.book, self.next_id, desc, self._last_simplify))
                rest = ids - apply_ids           # 고르지 않은 제안은 남김 (새 번호표 위의 제안으로)
                pending = dict(table)
                for i in rest:
                    pending[i] = self.pending[i]
                self.table, self.book = table, book
                self.pending = pending if rest else None
                self.proposals = {i: self.proposals[i] for i in rest}
                self._groups = {k: g for k, g in self._groups.items() if g <= rest}
                self.result = dict(self.result)   # 원본 결과는 스냅숏에 그대로 두고 사본을 고침
                self._refresh_drawing()
                self.edit_log.append(desc)
            except Exception:
                self._restore(snap)
                raise
            note = (f"{partial}번은 다른 번호와 묶인 편집(잇기·자르기)이라 함께 고르지 않아 적용하지 않았습니다"
                    if partial else "")
            return {"applied": sorted(apply_ids), "not_applied": sorted(ids - apply_ids), "note": note}

    def discard_proposals(self, ids=None):
        with self.lock:
            if ids is None:
                n = len(self.proposals)
                self._clear_proposals()
                return n
            drop = set()
            for i in ids:
                i = _as_int(i)
                if i in self.proposals:
                    drop |= self._groups.pop(self.proposals[i]["group"], set())
            for i in drop:
                self.proposals.pop(i, None)
                self.pending[i] = self.table.get(i, {"kind": "gone", "poly": None, "reason": ""})
            if not self.proposals:
                self._clear_proposals()
            return len(drop)

    def undo(self):
        with self.lock:
            if not self.history:
                return None
            snap = self._snapshot()
            table, book, next_id, desc, simplify = self.history.pop()
            self._clear_proposals()
            try:
                self.result = dict(self.result)
                if simplify is self._last_simplify:      # 같은 번호 체계: 번호표를 그대로 되돌림
                    self.table, self.book, self.next_id = table, book, next_id
                else:                                    # 그 뒤 다시 계산됨: 모양 기록으로 번호표를 다시 만듦
                    self.book = book
                    self._build_table(self.result["stages"], self.result["base"].shape)
                    self.proposal_epoch += 1
                    self.notice = "되돌리면서 번호를 새로 매겼습니다"
                self._refresh_drawing()
            except Exception:
                self._restore(snap)
                raise
            if self.edit_log:
                self.edit_log.pop()
            return desc

    # ------------------------------------------------------------ 조회
    def list_strokes(self, region_mm=None, include_candidates=False, limit=300):
        with self.lock:
            self._need_result()
            box = self._region_px(region_mm) if region_mm else None
            rows = []
            for i, e in sorted(self.table.items()):
                if e["kind"] == "gone" or (e["kind"] == "candidate" and not include_candidates):
                    continue
                mm = self.px_to_mm(e["poly"])
                (x0, y0), (x1, y1) = mm.min(axis=0), mm.max(axis=0)
                if box:
                    px0, py0 = e["poly"].min(axis=0)
                    px1, py1 = e["poly"].max(axis=0)
                    if px1 < box[0] or px0 > box[2] or py1 < box[1] or py0 > box[3]:
                        continue
                rows.append({"id": i, "kind": e["kind"], "reason": e["reason"],
                             "length_mm": round(float(np.hypot(*np.diff(mm, axis=0).T).sum()), 1),
                             "bbox_mm": [round(float(v), 1) for v in (x0, y0, x1, y1)], "points": len(mm)})
            return {"rows": rows[:limit], "truncated": len(rows) > limit, "total": len(rows)}

    def get_stroke(self, stroke_id, limit=400):
        with self.lock:
            self._need_result()
            e = self.table.get(_as_int(stroke_id))
            if e is None or e["kind"] == "gone":
                raise SessionError(f"없는 번호: {stroke_id}")
            mm = self.px_to_mm(e["poly"])
            pts = [[k, round(float(x), 2), round(float(y), 2)] for k, (x, y) in enumerate(mm[:limit])]
            return {"id": int(stroke_id), "kind": e["kind"], "reason": e["reason"], "points_mm": pts,
                    "truncated": len(mm) > limit}

    # ------------------------------------------------------------ 시뮬레이션
    def simulate(self, progress=None):
        """progress(끝낸 명령 수, 전체 명령 수): 화면이 진행률을 보여 줄 때."""
        from . import mirobot_sim as ms

        with self.lock:
            self._need_result()
            strokes = [[tuple(pt) for pt in s] for s in self.result["strokes_mm"]]
        res = ms.simulate(ms.plan_targets(strokes, self.cfg), progress=progress)
        j = int(np.argmin(res["min_margin_deg"]))
        summary = {"verdict": ms.verdict(res), "min_margin_deg": round(float(res["min_margin_deg"][j]), 1),
                   "axis": ms.CONTROLLER_AXES[j], "samples": len(res["samples"])}
        with self.lock:
            self.sim = {"summary": summary, "raw": res}
        return summary

    # ------------------------------------------------------------ 사진과 비교
    def compare(self):
        """사진과 지금 그려질 그림이 얼마나 닮았는지 (읽기 전용). 반환: (지표 dict, 나란히 보기 BGR 이미지)."""
        from . import tone

        with self.lock:
            self._need_result()
            r = self.result
            gray, color, edges = r["base"], r["color"], r["edges"]
            strokes = [np.asarray(s, np.float64) for s in r["strokes_px"]]
            outline = [np.asarray(e["poly"], np.float64) for e in self.table.values() if e["kind"] == "stroke"]
            pen_px = self.params["pen_mm"] / self._xf()[0]           # 펜 굵기(mm)를 작업 이미지 px로
        return tone.compare_report(gray, color, strokes, pen_px, edges, outline=outline)

    # ------------------------------------------------------------ 로봇 그리기 안내
    ROBOT_STEPS = (
        "종이를 고정하고, 로봇 펜 끝이 종이 가운데에 살짝 닿게 사람이 세팅한다 (앱은 시작 자세를 종이 중심으로 씀).",
        "앱 왼쪽의 '로봇으로 그리기'를 열고 연결 방식을 '로봇'으로 고른 뒤 [연결 시작]. 컨트롤러가 Alarm이면 로봇 가운데 버튼을 2초 눌러 호밍 (이미 Idle이면 자동으로 넘어감).",
        "처음이거나 큰 그림이면 '공중 모드'를 켜고 경로만 따라가 확인한다. 펜으로 그리려면 공중 모드를 끈다.",
        "'종이·펜·주변 확인' 체크 후 [시작]. 멈추려면 [멈춤] 또는 로봇 비상 정지 (멈춘 뒤 자동 복구는 없음).",
    )

    def robot_guide(self):
        """지금 그림이 로봇으로 갈 준비가 됐는지와 사람이 할 순서 (읽기 전용, 로봇은 앱 창에서 사람이 시작)."""
        with self.lock:
            cfg = self.cfg
            pen, wide = limits.executor_region(cfg), limits.pending_region(cfg)
            plane = cfg.get("plane_compensation", {}).get("status")
            g = {"drawing_now": bool(self.drawing_lock),
                 "pen_down_area_mm": {"x": [-pen.x_max, pen.x_max], "y": [pen.y_min, pen.top(0)],
                                      "outline": [list(p) for p in pen.outline()]},
                 "air_area_mm": {"x": [-wide.x_max, wide.x_max], "y": [wide.y_min, wide.top(0)]},
                 "plane_status": plane,
                 "paper_calibrated": plane == "verified",
                 "steps_for_the_person": list(self.ROBOT_STEPS),
                 "agent_cannot": "로봇 연결·호밍·시작·멈춤은 에이전트가 할 수 없다. 사람이 '로봇으로 그리기' 창에서 한다."}
            r = self.result
            if r is None:
                g.update(ready=False, next="이미지를 열고 처리하세요.")
                return g
            t, pl = r["timing"], r["placement"]
            g["drawing"] = {"strokes": t["stroke_count"], "estimated_minutes": round(t["total_s"] / 60, 1),
                            "size_mm": [pl["drawing_width_mm"], pl["drawing_height_mm"]]}
            sim = self.sim["summary"] if self.sim else None
            g["simulation"] = sim
            issues, notes = [], []
            if sim is None:
                issues.append("로봇 시뮬레이션이 아직 없다: simulate 도구로 관절 한계를 먼저 확인하세요.")
            elif sim["verdict"].startswith("FAIL"):
                issues.append(f"시뮬레이션 FAIL({sim['verdict']}): 그림 크기를 줄이세요.")
            if r["out_of_pending"]:
                issues.append(f"공중 확인용 넓은 범위 밖의 점이 {r['out_of_pending']}개: 그림 크기를 줄이세요.")
            g["pen_down_possible"] = r["out_of_limits"] == 0 and not issues
            if r["out_of_limits"] and not r["out_of_pending"]:
                notes.append(f"펜으로 그릴 수 있는 범위 밖의 점이 {r['out_of_limits']}개: 공중 모드(+'넓은 범위')로 경로를 확인하거나, "
                             "그림을 줄이거나, '종이·펜 위치 변경: 다시 보정'으로 접촉 영역을 측정한 뒤 펜으로 그린다.")
            if plane != "verified":
                notes.append("종이 평면 보정이 없다: 저장된 보정값을 쓴다. 모서리에서 펜이 안 닿거나 세게 눌릴 수 있으니 "
                             "처음에는 공중 모드로 확인하세요.")
            g.update(ready=not issues, issues=issues, notes=notes)
            return g

    # ------------------------------------------------------------ 상태·그림
    def state(self):
        with self.lock:
            s = {"image": self.image_path, "image_type": self.image_type, "detail": self.detail,
                 "params": dict(self.params),
                 "executor_limit_mm": limits.executor_region(self.cfg).x_max * 2,
                 "executor_limit_outline_mm": [list(p) for p in limits.executor_region(self.cfg).outline()],
                 "pending_limit_mm": limits.pending_region(self.cfg).x_max * 2,
                 "pending_limit_outline_mm": [list(p) for p in limits.pending_region(self.cfg).outline()],
                 "plane_status": self.cfg.get("plane_compensation", {}).get("status"),
                 "drawing": bool(self.drawing_lock)}
            s["stages"] = [{"id": st.id, "label": st.label, "params": {p.key: self.params[p.key] for p in st.params}}
                           for st in stages.ALL_STAGES]
            if self.result:
                t, pl = self.result["timing"], self.result["placement"]
                s["result"] = {
                    "strokes": t["stroke_count"], "commands": t["command_count"],
                    "drawing_mm": [pl["drawing_width_mm"], pl["drawing_height_mm"]],
                    "pen_down_mm": t["pen_down_mm"], "pen_up_mm": t["pen_up_mm"],
                    "estimated_minutes": round(t["total_s"] / 60, 1),
                    "out_of_executor_limits": self.result["out_of_limits"] > 0,
                    "edits": list(self.edit_log),
                    "frame": self.result["frame"]["kind"], "faces": self.result["faces"],
                }
                s["edit"] = {
                    "proposals": [{"id": v["id"], "change": ("delete" if v["after"] is None else
                                                              "add" if v["before"] is None else "modify")}
                                  for v in self.proposal_views()] if self.proposals else [],
                    "strokes": sum(1 for e in self.table.values() if e["kind"] == "stroke"),
                    "candidates": sum(1 for e in self.table.values() if e["kind"] == "candidate"),
                    "unapplied_edits": self.unapplied,
                    "notice": self.notice,
                }
            s["simulation"] = self.sim["summary"] if self.sim else None
            return s

    def stage_summaries(self):
        """단계마다 만든 결과 한 줄 (단계 띠·큰 보기 안내용). 결과가 없으면 빈 글자."""
        with self.lock:
            ids = [st.id for st in stages.ALL_STAGES]
            if not self.result:
                return {i: "" for i in ids}
            o, p, r = self.result["stages"], self.params, self.result

            def pts(strokes):
                return sum(len(x) for x in strokes)

            h, w = r["base"].shape
            t, pl = r["timing"], r["placement"]
            n_stroke = sum(1 for e in self.table.values() if e["kind"] == "stroke")
            n_cand = sum(1 for e in self.table.values() if e["kind"] == "candidate")
            return {
                "source": f"{w}×{h}px" + (" · 배경 제거" if p["rembg"] else "")
                          + ("" if r["frame"]["kind"] == "full" else
                             f" · {faces.FRAME_NAMES[r['frame']['kind']]}" + ("(자동)" if p["frame"] == "auto" else "")),
                "prep": f"블러 {p['blur_ksize']}px" + (f" · 미디언 {p['median_ksize']}" if p["median_ksize"] > 1 else ""),
                "edges": f"경계 {int((o['edges']['edges'] > 0).sum()):,}px",
                "trace": f"획 {len(o['trace']['strokes'])} · 버림 {len(o['trace']['discarded_trace'])}",
                "dedupe": f"획 {len(o['dedupe']['strokes'])} · 조각 {len(o['dedupe']['discarded_dedupe'])} 제거",
                "merge": f"획 {len(o['merge']['strokes'])}",
                "face": (f"얼굴 {o['face']['faces_used']} · 획 {len(o['face']['strokes'])}" if o["face"]["faces_used"]
                         else ("얼굴 없음" if not r["faces"] else "꺼짐")),
                "simplify": f"획 {len(o['simplify']['strokes'])} · 점 {pts(o['simplify']['strokes']):,}",
                "tone": (o["tone"].get("tone_note") and "배경이 어두움: 배경 제거" or
                         (f"빗금 {len(o['tone']['hatch_strokes'])}획" if o["tone"]["hatch_strokes"] else "꺼짐")),
                "edit": f"획 {n_stroke} · 후보 {n_cand}" + (f" · 제안 {len(self.proposals)}" if self.proposals else ""),
                "paper": f"{t['total_s'] / 60:.1f}분 · {pl['drawing_width_mm']:.0f}×{pl['drawing_height_mm']:.0f}mm",
            }

    def stage_change(self, stage_id):
        """이전 단계 결과 → 이 단계 결과 (첫 단계는 빈 글자)."""
        ids = [st.id for st in stages.ALL_STAGES]
        k = ids.index(stage_id)
        if k == 0 or not self.result:
            return ""
        sm = self.stage_summaries()
        return f"{stages.stage_title(ids[k - 1])[0]} {sm[ids[k - 1]]} → {stages.stage_title(stage_id)[0]} {sm[stage_id]}"

    def render(self, kind, region_mm=None, numbered=False, max_px=1000, show_candidates=False, overlay=0.0):
        """에이전트·화면용 그림 (BGR). kind: original | 단계 id | edit | lines | strokes | paper"""
        with self.lock:
            if kind == "original":
                if not self.image_path:
                    raise SessionError("이미지가 없습니다.")
                return self.color.copy()
            self._need_result()
            r = self.result
            if kind in stages.PIPELINE_IDS:
                return stages.STAGE_BY_ID[kind].preview(r["stages"][kind])
            if kind == "lines":
                return stages.STAGE_BY_ID["edges"].preview(r["stages"]["edges"])
            if kind == "edit":
                region_px = self._region_px(region_mm) if region_mm else None
                views = self.proposal_views() if self.proposals else []
                return edits.render_edit_view(r["base"].shape, self.table, views, region_px, numbered,
                                              show_candidates, r["color"], float(overlay), max_px)
            if kind == "paper":
                return r["paper"].copy()
            if kind == "strokes":
                return render_strokes_view(r["strokes_mm"], region_mm, numbered, max_px)
        kinds = ", ".join(("original", *stages.PIPELINE_IDS, "edit", "lines", "strokes", "paper"))
        raise SessionError(f"알 수 없는 그림 종류: {kind} ({kinds})")


def render_strokes_view(strokes_mm, region_mm=None, numbered=True, max_px=1000):
    """획을 종이 좌표로 그린 확대 그림. numbered면 획마다 번호(에이전트가 삭제할 번호를 고를 때).
    region_mm가 없으면 그림 전체 범위. 격자선은 10mm 간격(좌표 읽기용)."""
    pts = np.vstack([np.asarray(s) for s in strokes_mm])
    if region_mm is None:
        (x0, y0), (x1, y1) = pts.min(axis=0) - 3, pts.max(axis=0) + 3
    else:
        x0, y0, x1, y1 = [float(v) for v in region_mm]
        x0, x1 = sorted((x0, x1))
        y0, y1 = sorted((y0, y1))
    w_mm, h_mm = max(x1 - x0, 1.0), max(y1 - y0, 1.0)
    scale = max_px / max(w_mm, h_mm)
    w, h = int(w_mm * scale), int(h_mm * scale)
    img = np.full((h, w, 3), 255, np.uint8)

    def px(x, y):
        return int(round((x - x0) * scale)), int(round((y1 - y) * scale))

    for g in range(int(np.ceil(x0 / 10)) * 10, int(x1) + 1, 10):
        cv2.line(img, px(g, y0), px(g, y1), (235, 235, 235), 1)
        cv2.putText(img, str(g), (px(g, y0)[0] + 2, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (170, 170, 170), 1)
    for g in range(int(np.ceil(y0 / 10)) * 10, int(y1) + 1, 10):
        cv2.line(img, px(x0, g), px(x1, g), (235, 235, 235), 1)
        cv2.putText(img, str(g), (2, px(x0, g)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (170, 170, 170), 1)
    for i, s in enumerate(strokes_mm):
        a = np.array([px(x, y) for x, y in np.asarray(s)], np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [a], False, (60, 60, 60), 1, cv2.LINE_AA)
    if numbered:
        for i, s in enumerate(strokes_mm):
            s = np.asarray(s)
            mx, my = s[len(s) // 2]
            if not (x0 <= mx <= x1 and y0 <= my <= y1):
                continue
            x, y = px(mx, my)
            cv2.putText(img, str(i), (x + 2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(img, str(i), (x + 2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (30, 30, 220), 1, cv2.LINE_AA)
    return img
