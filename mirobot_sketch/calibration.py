"""사람 확인을 포함한 Mirobot 종이 평면·안전 사각형 캘리브레이션.

실행 전 로봇을 종이 중심에 호밍하고 종이를 고정합니다. 프로그램은 펜을 든 채
안전 사각형 경계를 단계적으로 따라간 다음, 선택한 사각형의 네 모서리에서
사용자가 0.25 mm 단위로 접촉을 맞춥니다. 힘/접촉 센서가 없으므로 접촉은 자동
감지하지 않으며, 사용자가 펜 끝을 확인하고 기록을 승인해야 합니다.

    mirobot-calibrate                    # 계획만 출력
    mirobot-calibrate --execute           # 포트 열기/수동 호밍/측정 시작
    python -m mirobot_sketch.calibration --execute
"""

import argparse
import copy
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from . import paths


class CalibrationError(RuntimeError):
    """상태 불일치·조작 취소 등으로 캘리브레이션을 안전하게 끝낼 수 없음."""


def contact_points(half_size_mm, half_height_mm=None):
    """센터와 네 모서리 접촉 샘플을 (이름, 종이 x, 종이 y) 순서로 반환."""
    hx = _positive_finite(half_size_mm, "half_size_mm")
    hy = hx if half_height_mm is None else _positive_finite(half_height_mm, "half_height_mm")
    return [("center", 0.0, 0.0), ("bottom_left", -hx, -hy),
            ("bottom_right", hx, -hy), ("top_right", hx, hy), ("top_left", -hx, hy)]


def contact_points_rect(x0, y0, x1, y1):
    """센터(종이 중심)와 종이 좌표 직사각형 네 모서리 접촉 샘플을 (이름, 종이 x, 종이 y) 순서로 반환."""
    x0, y0, x1, y1 = (_finite(v, n) for v, n in zip((x0, y0, x1, y1), ("x0", "y0", "x1", "y1")))
    if not (x0 < x1 and y0 < y1):
        raise ValueError("직사각형은 x0 < x1, y0 < y1 이어야 합니다.")
    return [("center", 0.0, 0.0), ("bottom_left", x0, y0), ("bottom_right", x1, y0),
            ("top_right", x1, y1), ("top_left", x0, y1)]


def square_outline(half_size_mm, segment_mm=10.0):
    """펜업 경로 점들. 각 변을 segment_mm 이하 길이로 나누고 끝에서 닫음."""
    h = _positive_finite(half_size_mm, "half_size_mm")
    step = _positive_finite(segment_mm, "segment_mm")
    corners = [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]
    out = [corners[0]]
    for start, end in zip(corners, corners[1:]):
        length = math.dist(start, end)
        n = max(1, math.ceil(length / step))
        for i in range(1, n + 1):
            t = i / n
            out.append((start[0] + (end[0] - start[0]) * t,
                        start[1] + (end[1] - start[1]) * t))
    return out


def _max_square_half_size(cfg):
    """현재 실물 승인 영역 안에 들어가는 가장 큰 중심 정렬 사각형의 반폭."""
    from .limits import Region

    region = Region.from_cfg(cfg["limits"])
    lo, hi = 0.0, region.x_max
    for _ in range(60):
        h = (lo + hi) / 2.0
        corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
        if all(region.contains(x, y) for x, y in corners):
            lo = h
        else:
            hi = h
    return lo


def staged_half_sizes(cfg, start_mm=30, step_mm=5, max_mm=None):
    """60×60 mm부터 시작해 사각형 한 변을 10 mm씩 늘리는 탐색 크기."""
    start = _positive_finite(start_mm, "start_mm")
    step = _positive_finite(step_mm, "step_mm")
    limit = _max_square_half_size(cfg)
    maximum = limit if max_mm is None else _positive_finite(max_mm, "max_mm")
    if maximum > limit + 1e-6:
        raise ValueError(f"최대 반폭 {maximum:g} mm가 현재 실물 허용 영역({limit:g} mm)을 넘습니다.")
    if start > maximum + 1e-6:
        raise ValueError("시작 반폭이 최대 반폭보다 큽니다.")
    values = []
    current = start
    while current < maximum - 1e-6:
        values.append(round(current, 6))
        current += step
    if not values or abs(values[-1] - maximum) > 1e-6:
        values.append(round(maximum, 6))
    return values


def fit_contact_plane(center_tcp_mm, corner_samples):
    """X_contact = center.x + a*dY + b*dZ 평면을 최소제곱으로 맞춤.

    corner_samples 각 항목은 {name, paper_xy_mm, tcp_mm:{x,y,z}} 형태입니다.
    반환값 residuals_mm는 측정한 다섯 지점에서 평면으로부터 벗어난 X 거리입니다.
    """
    center = _tcp(center_tcp_mm, "center")
    if len(corner_samples) < 2:
        raise ValueError("평면의 두 축 기울기를 계산하려면 서로 다른 모서리 샘플이 필요합니다.")
    matrix, target, names = [], [], []
    for i, sample in enumerate(corner_samples):
        tcp = _tcp(sample.get("tcp_mm"), f"corner_samples[{i}]")
        dy, dz = tcp["y"] - center["y"], tcp["z"] - center["z"]
        matrix.append([dy, dz])
        target.append(tcp["x"] - center["x"])
        names.append(str(sample.get("name", f"point_{i}")))
    matrix = np.asarray(matrix, dtype=float)
    target = np.asarray(target, dtype=float)
    coeffs, _, rank, _ = np.linalg.lstsq(matrix, target, rcond=None)
    if rank < 2:
        raise ValueError("접촉 샘플들이 한 직선에 있어 두 축의 평면 기울기를 구할 수 없습니다.")
    residual_values = target - matrix @ coeffs
    residuals = {name: float(value) for name, value in zip(names, residual_values)}
    residuals["center"] = 0.0
    absolute = np.abs(residual_values)
    return {
        "a_per_mm_y": float(coeffs[0]),
        "b_per_mm_z": float(coeffs[1]),
        "max_abs_residual_mm": float(np.max(absolute)) if len(absolute) else 0.0,
        "rmse_mm": float(np.sqrt(np.mean(residual_values ** 2))) if len(absolute) else 0.0,
        "residuals_mm": residuals,
    }


def apply_calibration(cfg, center_tcp_mm, plane_fit, half_size_mm, max_residual_mm=1.0):
    """품질 기준을 통과한 보정만 설정에 반영한 새 사본으로 반환."""
    rectangular = isinstance(half_size_mm, (tuple, list))
    rect = None
    if rectangular and len(half_size_mm) == 4:
        # 종이 좌표 직사각형 (x0, y0, x1, y1): 중심이 종이 중심과 달라도 됨. 실행기 한계는 좌우 대칭 + 평평한 위쪽 영역.
        from . import limits
        points = contact_points_rect(*half_size_mm)[1:]
        xm = max(abs(points[0][1]), abs(points[1][1]))
        y_lo, y_hi = points[0][2], points[2][2]
        region = limits.pending_region(cfg)
        if not all(region.contains(x, y) for x in (-xm, xm) for y in (y_lo, y_hi)):
            raise ValueError("측정 직사각형이 확장 허용 영역을 넘습니다.")
        rect = (xm, y_lo, y_hi)
    elif rectangular:
        if len(half_size_mm) != 2:
            raise ValueError("접촉 영역은 가로·세로 반폭 두 값이 필요합니다.")
        hx = _positive_finite(half_size_mm[0], "half_width_mm")
        hy = _positive_finite(half_size_mm[1], "half_height_mm")
        from . import limits
        region = limits.pending_region(cfg)
        if not all(region.contains(x, y) for _, x, y in contact_points(hx, hy)[1:]):
            raise ValueError("측정 직사각형이 확장 허용 영역을 넘습니다.")
    else:
        hx = hy = _positive_finite(half_size_mm, "half_size_mm")
        if hx > _max_square_half_size(cfg) + 1e-6:
            raise ValueError("측정 사각형이 현재 허용 영역을 넘습니다.")
    tolerance = _positive_finite(max_residual_mm, "max_residual_mm", allow_zero=True)
    if plane_fit["max_abs_residual_mm"] > tolerance:
        raise ValueError(
            f"평면 잔차 {plane_fit['max_abs_residual_mm']:.2f} mm가 허용값 {tolerance:.2f} mm보다 큽니다. "
            "종이를 평평하게 다시 고정한 뒤 캘리브레이션을 반복하세요."
        )
    center = _tcp(center_tcp_mm, "center")
    updated = copy.deepcopy(cfg)
    updated["paper_center_tcp_mm"] = center
    updated["plane_compensation"] = {
        "a_per_mm_y": float(plane_fit["a_per_mm_y"]),
        "b_per_mm_z": float(plane_fit["b_per_mm_z"]),
        "status": "verified",
    }
    updated["_plane_note"] = (
        "중심과 네 모서리의 사람 확인 접촉 샘플로 계산. "
        f"최대 X 잔차 {plane_fit['max_abs_residual_mm']:.3f} mm."
    )
    if rect is not None:
        xm, y_lo, y_hi = rect
        updated["limits"] = {"x_max_mm": float(xm), "y_min_mm": float(y_lo),
                             "roof_mm": [[0.0, float(y_hi)], [float(xm), float(y_hi)]]}
        updated["_limits_note"] = (
            f"캘리브레이션 중 펜업 경로와 네 모서리 접촉을 확인한 영역: 좌우 ±{xm:g}, 세로 {y_lo:g} ~ {y_hi:g} mm "
            "(종이 중심 기준). 설치/종이/펜이 바뀌면 다시 캘리브레이션할 것."
        )
    else:
        updated["limits"] = {"max_abs_paper_x_mm": float(hx), "max_abs_paper_y_mm": float(hy)}
        updated["_limits_note"] = (
            f"캘리브레이션 중 펜업 경로와 네 모서리 접촉을 확인한 ±{hx:g} × ±{hy:g} mm 영역. "
            "설치/종이/펜이 바뀌면 다시 캘리브레이션할 것."
        )
    updated["calibration"] = {
        "status": "verified",
        **({"rect_mm": [float(v) for v in half_size_mm]} if rect is not None else
           {"half_extents_mm": [hx, hy]} if rectangular else {"half_size_mm": hx}),
        "max_residual_mm": float(plane_fit["max_abs_residual_mm"]),
        "updated_local": datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    return updated


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name}은 유한한 수여야 합니다.")
    return value


def _positive_finite(value, name, allow_zero=False):
    value = float(value)
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        op = "0 이상" if allow_zero else "0보다 커야"
        raise ValueError(f"{name}은 유한한 수이고 {op} 합니다.")
    return value


def _tcp(value, name="tcp"):
    if not isinstance(value, dict) or any(k not in value for k in ("x", "y", "z")):
        raise ValueError(f"{name}은 x, y, z 좌표를 가져야 합니다.")
    tcp = {k: float(value[k]) for k in ("x", "y", "z")}
    if any(not math.isfinite(v) for v in tcp.values()):
        raise ValueError(f"{name} 좌표에 유한하지 않은 값이 있습니다.")
    return tcp


def _move_tcp(link, cfg, x, y, z, feed):
    """한 TCP 이동을 보내고 완료/좌표를 확인합니다. 오류 시 자동 후퇴하지 않습니다."""
    from .draw_executor import ControllerError, Planner

    line = Planner(cfg).gcode((float(x), float(y), float(z)), feed)
    try:
        link.send_and_ack(line, cfg["ack_timeout_s"])
        tcp = link.wait_idle(cfg["idle_timeout_s"])
        state, measured, raw = link.query_status()
    except ControllerError as e:
        raise CalibrationError(f"로봇 이동 중 컨트롤러 오류: {e}") from e
    if state != "Idle":
        raise CalibrationError(f"이동 후 상태가 Idle이 아닙니다 ({state}): {raw}")
    if measured is None:
        raise CalibrationError("이동 후 TCP 좌표를 읽지 못했습니다. 설정을 바꾸지 않았습니다.")
    if tcp is not None and math.dist(tcp, measured) > 1.0:
        raise CalibrationError("Idle 대기와 상태 조회의 TCP 좌표가 달라 이동 결과를 확인할 수 없습니다.")
    if math.dist((x, y, z), measured) > 2.0:
        raise CalibrationError(
            f"명령한 TCP ({x:.1f}, {y:.1f}, {z:.1f})와 실제 TCP {measured} 차이가 큽니다. 중단합니다."
        )
    return {"x": measured[0], "y": measured[1], "z": measured[2]}


def _status_tcp(link):
    state, tcp, raw = link.query_status()
    if state != "Idle" or tcp is None:
        raise CalibrationError(f"측정 전 상태/TCP 확인 실패 ({state}): {raw}")
    return {"x": tcp[0], "y": tcp[1], "z": tcp[2]}


def _paper_to_robot(cfg, center, px, py):
    return (center["y"] + cfg["paper_x_to_robot_y_sign"] * px,
            center["z"] + cfg["paper_y_to_robot_z_sign"] * py)


def _fit_so_far(center, samples):
    if len(samples) < 2:
        return None
    matrix = np.asarray([[s["tcp_mm"]["y"] - center["y"],
                          s["tcp_mm"]["z"] - center["z"]] for s in samples], dtype=float)
    if np.linalg.matrix_rank(matrix) < 2:
        return None
    target = np.asarray([s["tcp_mm"]["x"] - center["x"] for s in samples], dtype=float)
    coeffs, *_ = np.linalg.lstsq(matrix, target, rcond=None)
    return coeffs


def _ask(input_fn, output_fn, message):
    try:
        return input_fn(message).strip().lower()
    except (EOFError, KeyboardInterrupt):
        output_fn("입력이 끝나 캘리브레이션을 취소했습니다. 설정은 갱신하지 않습니다.")
        return "q"


def run_calibration(link, cfg, start_mm=30, step_mm=5, max_mm=None,
                    max_residual_mm=1.0, input_fn=input, output_fn=print,
                    target_half_extents_mm=None, target_rect_mm=None):
    """연결된 로봇으로 측정을 수행하고, 품질 검사 전 결과 자료를 반환.

    입력 흐름은 (1) 종이 중심 고정, (2) 펜업 외곽 승인/선택,
    (3) 네 모서리에서 0.25 mm 수동 키 조그 후 접촉 승인입니다.
    함수는 config 파일을 쓰지 않습니다. 결과가 완전하고 평면 잔차 기준을
    통과한 뒤 호출자가 apply_calibration/save_config를 실행해야 합니다.

    측정 영역: 기본은 30 mm부터 늘려 가는 종이 중심 정사각형, target_half_extents_mm=(가로, 세로 반폭)은
    종이 중심 직사각형 하나, target_rect_mm=(x0, y0, x1, y1)은 종이 좌표 직사각형 하나입니다
    (그림이 위쪽 한계 때문에 아래로 내려가 배치되는 큰 그림처럼 중심이 종이 중심과 다른 영역).
    """
    rect_mode = target_rect_mm is not None
    rectangular = rect_mode or target_half_extents_mm is not None
    if rect_mode:
        if len(target_rect_mm) != 4:
            raise ValueError("목표 영역은 x0, y0, x1, y1 네 값이 필요합니다.")
        x0, y0, x1, y1 = (_finite(v, name) for v, name in zip(target_rect_mm, ("x0", "y0", "x1", "y1")))
        if not (x0 < x1 and y0 < y1):
            raise ValueError("목표 영역은 x0 < x1, y0 < y1 이어야 합니다.")
        from . import limits
        region = limits.pending_region(cfg)
        if not all(region.contains(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))):
            raise ValueError("목표 접촉 영역이 확장 허용 영역을 넘습니다.")
        sizes = [(x0, y0, x1, y1)]
    elif rectangular:
        if len(target_half_extents_mm) != 2:
            raise ValueError("목표 영역은 가로·세로 반폭 두 값이 필요합니다.")
        hx = _positive_finite(target_half_extents_mm[0], "target_half_width_mm")
        hy = _positive_finite(target_half_extents_mm[1], "target_half_height_mm")
        from . import limits
        if not all(limits.pending_region(cfg).contains(x, y)
                   for _, x, y in contact_points(hx, hy)[1:]):
            raise ValueError("목표 접촉 영역이 확장 허용 영역을 넘습니다.")
        sizes = [(-hx, -hy, hx, hy)]
    else:
        sizes = [(-h, -h, h, h) for h in staged_half_sizes(cfg, start_mm=start_mm,
                                                           step_mm=step_mm, max_mm=max_mm)]

    def report(rect):
        """보고서에 쓰는 크기 표현: 정사각형 반폭 / [가로, 세로 반폭] / [x0, y0, x1, y1]."""
        if rect_mode:
            return [float(v) for v in rect]
        if rectangular:
            return [float(rect[2]), float(rect[3])]
        return float(rect[2])

    try:
        state, home_tcp = link.wait_for_homing(cfg["idle_timeout_s"], progress=output_fn)
    except Exception as e:
        raise CalibrationError(f"호밍 상태를 읽지 못했습니다: {e}") from e
    if state != "Idle" or home_tcp is None:
        raise CalibrationError(f"호밍 완료 상태 Idle/TCP가 필요합니다 (상태: {state}).")
    center = {"x": home_tcp[0], "y": home_tcp[1], "z": home_tcp[2]}
    output_fn("\n1/3 종이 중심 정렬")
    output_fn(f"현재 홈 TCP: X={center['x']:.3f}, Y={center['y']:.3f}, Z={center['z']:.3f} mm")
    output_fn("종이 중심을 펜 끝에 맞추고, 종이와 벽 간격을 사람이 조정해 종이를 단단히 고정하세요.")
    output_fn("접촉 센서는 없습니다. 종이 중앙에서 펜 끝이 살짝 닿는 것을 눈으로 확인하세요.")
    if _ask(input_fn, output_fn, "준비됐으면 yes 입력 (취소 q): ") != "yes":
        return {"result": "cancelled", "samples": [], "tested_half_sizes_mm": []}
    center = _status_tcp(link)

    retract_sign = float(cfg["pen"]["retract_x_sign"])
    if abs(retract_sign) != 1:
        raise CalibrationError("pen.retract_x_sign은 +1 또는 -1이어야 합니다.")
    clear = max(2.5, float(cfg["pen"]["up_clearance_mm"]))
    exploration_clear = max(8.0 if rectangular else 5.0, 2.0 * clear)
    exploration_x = center["x"] + retract_sign * exploration_clear
    y0, z0 = center["y"], center["z"]
    first, last_size = sizes[0], sizes[-1]
    output_fn("\n2/3 펜업 접촉 영역 탐색")
    output_fn(f"접촉 영역 탐색: {[report(r) for r in sizes]} mm "
              f"(가로 {first[2] - first[0]:g}–{last_size[2] - last_size[0]:g}, "
              f"세로 {first[3] - first[1]:g}–{last_size[3] - last_size[1]:g} mm), "
              f"벽에서 약 {exploration_clear:g} mm 후퇴")
    _move_tcp(link, cfg, exploration_x, y0, z0, cfg["feeds_mm_per_min"]["approach"])
    tested, tested_rects, selected = [], [], None
    choose_previous = False
    for rect in sizes:
        rx0, ry0, rx1, ry1 = rect
        width, height = rx1 - rx0, ry1 - ry0
        # 한 변씩 이동하고 각 모서리에서 사람 확인을 받습니다. 짧은 세그먼트마다
        # 정지하면 감속/재가속이 잦아 펜 홀더가 흔들릴 수 있습니다.
        outline = [(rx0, ry0), (rx1, ry0), (rx1, ry1), (rx0, ry1), (rx0, ry0)]
        for i, (px, py) in enumerate(outline):
            ry, rz = _paper_to_robot(cfg, center, px, py)
            measured = _move_tcp(link, cfg, exploration_x, ry, rz,
                                 cfg["feeds_mm_per_min"]["travel"])
            output_fn(f"  pen-up {width:g}×{height:g} mm, 모서리 {i+1}/{len(outline)}: "
                      f"Y={measured['y']:.1f}, Z={measured['z']:.1f} mm")
            action = _ask(input_fn, output_fn,
                          "안전하면 Enter, 이전에 통과한 크기로 선택은 c, 완전 취소는 q: ")
            if action == "c":
                if not tested_rects:
                    output_fn("아직 확인된 작은 크기가 없습니다. 이 사각형을 끝까지 확인하거나 취소하세요.")
                    continue
                selected = tested_rects[-1]
                choose_previous = True
                break
            if action == "q":
                return {"result": "aborted", "reason": "pen-up path safety not confirmed",
                        "samples": [], "tested_half_sizes_mm": tested}
        if choose_previous:
            break
        tested.append(report(rect))
        tested_rects.append(rect)
        if rect == last_size:
            selected = rect
            break
        action = _ask(input_fn, output_fn,
                      f"현재 {width:g}×{height:g} mm 경로가 안전합니다. [Enter] 다음 단계, "
                      "[c] 이 크기로 접촉 측정, [q] 취소: ")
        if action == "c":
            selected = rect
            break
        if action == "q":
            return {"result": "aborted", "reason": "user stopped rectangle expansion",
                    "samples": [], "tested_half_sizes_mm": tested}
        if action not in ("", "y", "yes"):
            raise CalibrationError("다음 단계, c(현재 크기 선택), q 중 하나를 입력하세요.")
    if selected is None:
        selected = tested_rects[-1]

    selected_report = report(selected)

    # 중심에서 펜을 충분히 떼고 각 모서리로 이동합니다.
    _move_tcp(link, cfg, exploration_x, y0, z0, cfg["feeds_mm_per_min"]["travel"])
    samples = []
    corners = contact_points_rect(*selected)[1:]
    approach_step = 0.25
    max_extra_touch_mm = 0.5  # 예측 접촉보다 벽 쪽으로 더 누르지 않음
    travel_x = exploration_x  # 미측정 모서리로 갈 때는 확인된 접촉점보다 멀리 물린다.
    for index, (name, px, py) in enumerate(corners, start=1):
        ry, rz = _paper_to_robot(cfg, center, px, py)
        coeffs = _fit_so_far(center, samples)
        predicted_x = center["x"]
        if coeffs is not None:
            predicted_x += coeffs[0] * (ry - center["y"]) + coeffs[1] * (rz - center["z"])
        current = _move_tcp(link, cfg, travel_x, ry, rz, cfg["feeds_mm_per_min"]["travel"])
        wall_dir = -retract_sign
        wallward_cap = predicted_x + wall_dir * max_extra_touch_mm
        output_fn(f"\n3/3 접촉점 {index}/4: {name} ({px:g}, {py:g}) mm")
        output_fn(f"펜이 든 위치에서 시작했습니다. [+] 벽 쪽 / [-] 후퇴를 한 번에 {approach_step:g} mm씩 "
                  "조정합니다. 기호를 반복 입력해 여러 번 조정하고, [Enter] 접촉 기록, [q] 취소.")
        while True:
            action = _ask(input_fn, output_fn, "조작: ")
            if action in ("", "s", "save", "yes"):
                contact_tcp = _status_tcp(link)
                if abs(contact_tcp["y"] - ry) > 1.0 or abs(contact_tcp["z"] - rz) > 1.0:
                    raise CalibrationError("접촉 확인 중 TCP의 Y/Z가 목표 모서리에서 벗어났습니다.")
                samples.append({"name": name, "paper_xy_mm": [float(px), float(py)],
                                "tcp_mm": contact_tcp})
                break
            if action == "q":
                return {"result": "aborted", "reason": "contact capture cancelled",
                        "samples": samples, "tested_half_sizes_mm": tested,
                        "selected_half_size_mm": selected_report, "center_tcp_mm": center}
            if not action or any(ch not in "+-jk" for ch in action):
                output_fn("+, -, Enter 또는 q를 입력하세요.")
                continue
            for direction in action:
                toward_wall = direction in ("+", "j")
                next_x = current["x"] + (wall_dir if toward_wall else -wall_dir) * approach_step
                if toward_wall and wall_dir * (next_x - wallward_cap) > 1e-7:
                    output_fn("예측 접촉점보다 0.5 mm 넘게 밀지 않도록 멈췄습니다. 종이 간격을 조정하고 다시 하세요.")
                    break
                current = _move_tcp(link, cfg, next_x, ry, rz,
                                    cfg["feeds_mm_per_min"]["approach"])
            output_fn(f"  현재 X={current['x']:.3f} mm")
        # 다음 위치 이동 전에 확실히 펜을 올립니다.
        measured = samples[-1]["tcp_mm"]
        retracted_x = measured["x"] + retract_sign * clear
        if retract_sign * (retracted_x - travel_x) > 0:
            travel_x = retracted_x
        _move_tcp(link, cfg, travel_x, ry, rz,
                  cfg["feeds_mm_per_min"]["approach"])

    ready_tcp = _move_tcp(link, cfg, travel_x, y0, z0,
                          cfg["feeds_mm_per_min"]["approach"])
    fit = fit_contact_plane(center, samples)
    output_fn(f"\n평면 계산: a={fit['a_per_mm_y']:.6f}, b={fit['b_per_mm_z']:.6f}, "
              f"최대 잔차={fit['max_abs_residual_mm']:.3f} mm (허용 {max_residual_mm:g} mm)")
    result = "ready" if fit["max_abs_residual_mm"] <= max_residual_mm else "needs_paper_adjustment"
    return {"result": result, "center_tcp_mm": center, "samples": samples,
            "plane_fit": fit, "selected_half_size_mm": selected_report,
            "selected_rect_mm": [float(v) for v in selected],
            "tested_half_sizes_mm": tested,
            "ready_tcp_mm": ready_tcp,
            "max_residual_mm": float(max_residual_mm)}


def save_config(config_path, cfg):
    """새 설정을 임시 파일에 완전히 쓴 뒤 원자적으로 교체."""
    path = Path(config_path)
    temp = path.with_name(path.name + ".tmp")
    path.parent.mkdir(parents=True, exist_ok=True)
    with temp.open("w", encoding="utf-8", newline="\n") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=1)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)


def save_report(report, config_path=None):
    """LOG/calibrations (설치판은 사용자 폴더)에 전체 측정 결과를 저장."""
    base = paths.runs_dir().parent / "calibrations"
    base.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    record = paths.scrub_paths({"created_local": datetime.now().astimezone().isoformat(timespec="seconds"),
                                "config_path": str(config_path or paths.config_path()), **report})
    path = base / f"calibration-{stamp}.json"
    with path.open("w", encoding="utf-8", newline="\n") as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
        f.write("\n")
    return path


def main():
    paths.safe_console()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--execute", action="store_true", help="USB 시리얼을 열고 실제 캘리브레이션 실행")
    parser.add_argument("--start-mm", type=float, default=30.0, help="시작 사각형 반폭 mm (기본 30 → 60×60 mm)")
    parser.add_argument("--step-mm", type=float, default=5.0, help="반폭 증가량 mm (기본 5 → 변 길이 10 mm씩 증가)")
    parser.add_argument("--max-mm", type=float, default=None, help="탐색할 최대 사각형 반폭 (기본: 현재 허용 한계)")
    parser.add_argument("--max-residual-mm", type=float, default=1.0, help="평면 계산 최대 X 잔차 허용값")
    parser.add_argument("--verbose", action="store_true", help="컨트롤러 응답 출력")
    args = parser.parse_args()

    from .draw_executor import load_config
    cfg = load_config(args.config)
    try:
        sizes = staged_half_sizes(cfg, args.start_mm, args.step_mm, args.max_mm)
    except ValueError as e:
        parser.error(str(e))
    print("종이 중심을 수동 정렬한 뒤 펜업 사각형을 탐색하고, 네 모서리에서 직접 접촉을 승인합니다.")
    print(f"탐색 반폭 {sizes} mm; 전체 사각형 {2*sizes[0]:g}–{2*sizes[-1]:g} mm")
    print("기본 실행은 계획만 보여주며 로봇을 움직이지 않습니다. 실제 측정에는 --execute를 사용하세요.")
    if not args.execute:
        return 0

    from .draw_executor import DrawError, open_link
    config_path = args.config or paths.config_path()
    try:
        link = open_link(cfg, verbose=args.verbose)
    except DrawError as e:
        print(e.message)
        print(f"실패 기록: {save_report({'result': 'connection_failed', 'reason': e.message}, config_path)}")
        return 3
    try:
        report = run_calibration(link, cfg, args.start_mm, args.step_mm, args.max_mm,
                                 args.max_residual_mm)
    except (CalibrationError, ValueError, KeyboardInterrupt) as e:
        report = {"result": "aborted", "reason": str(e), "samples": []}
        print(f"중단: {e}. 자동 후퇴 없이 설정을 갱신하지 않았습니다.")
    finally:
        link.close()

    report_path = save_report(report, config_path)
    print(f"측정 기록: {report_path}")
    if report.get("result") != "ready":
        if report.get("result") == "needs_paper_adjustment":
            print("종이 면 편차가 큽니다. 종이/받침을 평평하게 맞춘 뒤 다시 측정하세요. 기존 설정은 유지했습니다.")
        return 4
    try:
        updated = apply_calibration(cfg, report["center_tcp_mm"], report["plane_fit"],
                                    report["selected_half_size_mm"], args.max_residual_mm)
        save_config(config_path, updated)
    except (OSError, ValueError) as e:
        print(f"설정 파일을 갱신하지 못했습니다: {e}")
        return 5
    print(f"설정 갱신 완료: {config_path}")
    print(f"그리기 허용 사각형: {2*report['selected_half_size_mm']:g} × "
          f"{2*report['selected_half_size_mm']:g} mm; 다음 그림은 이 범위 안에서 dry-run으로 확인하세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
