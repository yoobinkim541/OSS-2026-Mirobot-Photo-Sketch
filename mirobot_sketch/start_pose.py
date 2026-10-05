"""
시작 자세: 펜 끝을 위로 올린 대기 자세 (시작 자세 설계 T2)
==========================================================
호밍 자세(모든 관절 0)는 플랜지가 아래를 향해 펜이 수직입니다. 그리려면 손목(J5)을 돌려 펜이
벽(+X) 쪽을 보게 해야 하는데,

  - 펜이 수평 근처일 때는 손목 특이점이라(T1, LOG/2026-10-05-start-pose-t1.md) 펜 끝을 pen_up_deg만큼
    위로 올린 자세로 그리고,
  - 손목을 벽 앞에서 돌리면 펜 끝이 펜 길이만큼 벽 쪽으로 쓸려 가므로, 베이스(J1)를 옆으로 돌린 채
    손목을 돌린 뒤 다시 벽 쪽으로 돌아옵니다. J1만 돌 때 펜 끝은 베이스 축을 중심으로 원을 그리므로
    벽 쪽으로 더 다가가지 않습니다.

순서 (모두 관절 이동 M21 — 특이점을 지나도 컨트롤러가 손목을 휘두르지 않음):
  1. J1 -> side_j1_deg (펜은 아래를 향한 채 옆으로)
  2. 옆을 본 채 J2~J6 -> 대기 자세 값 (팔을 들고 펜 끝을 위로)
  3. J1 -> 대기 자세 값(≈0) (벽 쪽으로 돌아옴)
대기 자세: 플랜지 (종이면 X - 여유 - 펜 길이·cos(올림), 0, standby_z_mm), 펜 끝은 종이에서 clearance_mm 앞.

계산은 시뮬레이터(URDF 기구학) 기준입니다. 옆으로 도는 쪽에 물건이 없는지는 사람이 확인해야 합니다.
"""

import numpy as np

from . import mirobot_sim as ms
from . import pen_pose as pp

DEFAULTS = {
    "pen_up_deg": 35.0,          # 펜 끝을 수평에서 위로 올리는 각도 (T1: 30~40°면 특이점에서 20° 이상)
    "pen_len_max_mm": 60.0,      # 플랜지 면 -> 펜 끝 (펜 축 방향). 안전 계산은 이 최댓값 기준
    "paper_x_mm": 230.0,         # 로봇 중심(J1 축) -> 종이 면 (로봇 X)
    "clearance_mm": 15.0,        # 대기 자세와 펜 올리는 동안 펜 끝과 종이 면 사이 최소 거리
    "standby_z_mm": 200.0,       # 대기 자세 플랜지 높이
    "side_j1_deg": -90.0,        # 손목을 돌리는 동안 베이스 방향 (-: 로봇 오른쪽)
    "joint_feed": 600,           # 관절 이동 속도 F (실물 단위 미확인: 느린 값으로 시작)
    "min_tip_z_mm": 30.0,        # 펜 끝이 책상 위로 이만큼은 떠 있어야 함
    "min_joint_margin_deg": 5.0,
    "min_singularity_deg": 15.0,
}
SAMPLES = 60    # 관절 이동 한 번을 나눠 검사하는 수
GAP_TOL_MM = 0.1   # IK 위치 허용 오차(0.05 mm) 때문에 대기 자세 끝점이 여유보다 조금 모자라게 계산되는 것을 허용
_UP_SEED = np.radians([0, -10, 20, 0, -130, 0])


class StartPoseError(Exception):
    """대기 자세로 안전하게 갈 수 없음 (설정·배치를 바꿔야 함)."""


def settings(cfg):
    """설정의 start_pose 블록 + 기본값."""
    return {**DEFAULTS, **cfg.get("start_pose", {})}


def joint_line(q_deg, feed):
    j = [f"{v:.3f}" for v in q_deg]
    return f"M21 G90 G01 X{j[0]} Y{j[1]} Z{j[2]} A{j[3]} B{j[4]} C{j[5]} F{feed:.0f}"


def _tip(q, pen_len):
    flange, _ = ms.fk(q)
    return flange[:3, 3] + pen_len * flange[:3, 2]


def _check_segments(poses, s):
    """관절 이동을 잘게 나눠 관절 여유·펜 끝과 종이 거리·펜 끝 높이를 검사. 반환: (최소 여유°, 최소 거리 mm)."""
    lo, hi = ms.JOINT_LIMITS_RAD[:, 0], ms.JOINT_LIMITS_RAD[:, 1]
    worst_margin, worst_gap = np.inf, np.inf
    for k, (a, b) in enumerate(zip(poses, poses[1:]), start=1):
        for t in np.linspace(0.0, 1.0, SAMPLES + 1):
            q = a + (b - a) * t
            margin = float(np.degrees(np.minimum(q - lo, hi - q)).min())
            tip = _tip(q, s["pen_len_max_mm"])
            gap = s["paper_x_mm"] - float(tip[0])
            if margin < s["min_joint_margin_deg"]:
                raise StartPoseError(f"{k}번째 이동에서 관절 한계 여유가 {margin:.1f}°입니다 "
                                     f"(필요 {s['min_joint_margin_deg']:g}°). side_j1_deg·standby_z_mm를 바꾸세요.")
            if gap < s["clearance_mm"] - GAP_TOL_MM:
                raise StartPoseError(f"{k}번째 이동에서 펜 끝이 종이 면에서 {gap:.1f} mm까지 다가갑니다 "
                                     f"(필요 {s['clearance_mm']:g} mm). 로봇을 벽에서 더 떨어뜨리거나 펜을 짧게 물리고 "
                                     f"paper_x_mm·pen_len_max_mm를 고치세요.")
            if tip[2] < s["min_tip_z_mm"]:
                raise StartPoseError(f"{k}번째 이동에서 펜 끝 높이가 {tip[2]:.1f} mm입니다 "
                                     f"(필요 {s['min_tip_z_mm']:g} mm 이상). standby_z_mm를 높이세요.")
            worst_margin, worst_gap = min(worst_margin, margin), min(worst_gap, gap)
    return worst_margin, worst_gap


def plan_raise(cfg):
    """호밍 자세 -> 펜 끝을 올린 대기 자세. 반환 dict: commands [(gcode, 설명)], joints_deg(각 이동 끝),
    standby_tcp_mm, min_joint_margin_deg, min_tip_clearance_mm, standby_singularity_deg.
    안전 조건을 못 맞추면 StartPoseError (명령을 보내기 전에)."""
    s = settings(cfg)
    up = np.radians(s["pen_up_deg"])
    x = s["paper_x_mm"] - s["clearance_mm"] - s["pen_len_max_mm"] * np.cos(up)
    target = np.array([x, 0.0, s["standby_z_mm"]])
    q_up, err, ok = ms.ik(target, _UP_SEED, r_goal=pp.pen_rotation(-s["pen_up_deg"]), iters=400)
    if not ok:
        raise StartPoseError(f"대기 자세 (X {x:.1f}, Z {target[2]:.1f} mm)에 도달할 수 없습니다 (오차 {err:.2f} mm).")
    q_up = pp._wrap(q_up)
    sing = pp.singularity_deg(q_up)
    if sing < s["min_singularity_deg"]:
        raise StartPoseError(f"대기 자세가 손목 특이점에서 {sing:.1f}°밖에 떨어져 있지 않습니다 "
                             f"(필요 {s['min_singularity_deg']:g}°). pen_up_deg를 바꾸세요.")
    side = np.radians(s["side_j1_deg"])
    poses = [np.zeros(6), np.array([side, 0, 0, 0, 0, 0]), np.array([side, *q_up[1:]]), q_up]
    margin, gap = _check_segments(poses, s)
    labels = ["base to side (pen down)", "raise pen tip (facing side)", "turn back to the wall"]
    deg = [np.degrees(q) for q in poses[1:]]
    return {
        "commands": [(joint_line(q, s["joint_feed"]), label) for q, label in zip(deg, labels)],
        "joints_deg": [q.round(3).tolist() for q in deg],
        "standby_tcp_mm": ms.fk(q_up)[0][:3, 3].round(3).tolist(),
        "min_joint_margin_deg": margin,
        "min_tip_clearance_mm": gap,
        "standby_singularity_deg": sing,
    }


HOME_JOINT_TOL_DEG = 0.5   # 호밍 직후로 볼 관절 각도 허용 오차


def run_raise(link, cfg, confirm, progress=print):
    """호밍 자세인지 확인 -> 계획을 보여 주고 confirm(plan)이 True일 때만 전송 -> 컨트롤러가 보고한 자세를 반환.
    오류 응답이면 바로 멈추고 자동 복구를 하지 않습니다(사람이 확인). 반환 dict: result, plan, reported, (error)."""
    from .draw_executor import ControllerError, parse_status

    s = settings(cfg)
    before = parse_status(link.query_status()[2])
    j = before["joints_deg"]
    if j is None or max(abs(v) for v in j) > HOME_JOINT_TOL_DEG:
        raise StartPoseError(f"호밍 자세(모든 관절 0°)가 아닙니다 (관절 {j}). 가운데 버튼 2초로 호밍한 뒤 다시 하세요.")
    plan = plan_raise(cfg)
    if not confirm(plan):
        return {"result": "cancelled", "plan": plan, "reported": before}
    for i, (line, label) in enumerate(plan["commands"], start=1):
        progress(f"  [{i}/{len(plan['commands'])}] {label}")
        try:
            link.send_and_ack(line, cfg["ack_timeout_s"])
            link.wait_idle(cfg["idle_timeout_s"])
        except ControllerError as e:
            return {"result": "stopped_on_error", "error": str(e), "plan": plan,
                    "failed_command": label, "reported": parse_status(link.query_status()[2])}
    after = parse_status(link.query_status()[2])
    progress(f"  끝: 관절 {after['joints_deg']}, TCP {after['tcp']}, A·B·C {after['abc_deg']} "
             f"(예상 TCP {plan['standby_tcp_mm']}, 펜 끝 {s['pen_up_deg']:g}° 위)")
    return {"result": "completed", "plan": plan, "reported": after}


def main():
    """실물/가상으로 펜 올리기 순서만 실행: python -m mirobot_sketch.start_pose [--execute | --virtual]"""
    import argparse
    import json
    from datetime import datetime

    from . import draw_executor as de
    from . import paths

    paths.safe_console()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--execute", action="store_true", help="실제 로봇으로 전송 (없으면 계획과 검사 결과만)")
    ap.add_argument("--virtual", action="store_true", help="가상 로봇으로 실행")
    args = ap.parse_args()
    cfg = de.load_config(args.config)
    s = settings(cfg)
    try:
        plan = plan_raise(cfg)
    except StartPoseError as e:
        print(f"계획 실패: {e}")
        return 2
    print(f"설정: 펜 끝 {s['pen_up_deg']:g}° 위, 펜 길이 최대 {s['pen_len_max_mm']:g} mm, 종이 면 X {s['paper_x_mm']:g} mm, "
          f"옆으로 J1 {s['side_j1_deg']:g}°")
    for line, label in plan["commands"]:
        print(f"  {line}    ; {label}")
    print(f"대기 자세 TCP {plan['standby_tcp_mm']}, 관절 여유 최소 {plan['min_joint_margin_deg']:.1f}°, "
          f"펜 끝-종이 최소 {plan['min_tip_clearance_mm']:.1f} mm, 특이점까지 {plan['standby_singularity_deg']:.1f}°")
    if not (args.execute or args.virtual):
        print("dry-run: 로봇을 움직이지 않았습니다. --execute로 실물 실행.")
        return 0
    link = de.open_link(cfg, virtual=args.virtual, virtual_speed=20.0)
    try:
        de.connect_and_home(link, cfg)

        def confirm(_plan):
            print(f"로봇 오른쪽(J1 {s['side_j1_deg']:g}°로 도는 쪽)과 위쪽에 물건이 없고, 펜 길이가 "
                  f"{s['pen_len_max_mm']:g} mm 이하이며, 종이 면이 로봇 중심에서 {s['paper_x_mm']:g} mm 이상 떨어져 있습니까?")
            return args.virtual or input("yes 입력 시 시작: ").strip().lower() == "yes"

        result = run_raise(link, cfg, confirm)
    except (de.DrawError, StartPoseError) as e:
        print(f"멈춤: {getattr(e, 'message', e)}")
        return 1
    finally:
        link.close()
    out = paths.runs_dir().parent / "start_pose"
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = out / f"raise-{stamp}.json"
    record = paths.scrub_paths({"started_local": stamp, "virtual": args.virtual, "settings": s, **result})
    with open(path, "w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=1, default=float)
    print(f"결과 {result['result']}, 기록: {path}")
    return 0 if result["result"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
