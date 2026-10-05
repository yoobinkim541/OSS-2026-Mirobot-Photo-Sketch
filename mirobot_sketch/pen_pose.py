"""
펜을 벽 쪽으로 눕힌 자세 검사 (시작 자세 설계 T1)
==================================================
호밍 자세(A·B·C=0)는 플랜지가 아래를 향해 펜이 수직입니다. 펜을 벽 쪽(+X)으로 눕히려면
J5를 약 -90°로 돌려야 하는데, 그러면 플랜지 축이 팔뚝 축(J4)과 거의 겹쳐 손목 특이점에
가까워집니다. 이 모듈은 펜 기울기·플랜지 위치별로 그리기 범위 전체의 관절 여유와
특이점까지의 각도를 계산합니다.

    python -m mirobot_sketch.pen_pose                       # 기울기 0/10/15/20°, 플랜지 X 150~200 mm
    python -m mirobot_sketch.pen_pose --pen-len 80          # 펜 끝이 닿는 벽 X도 함께

한계: 시뮬레이터와 같습니다(충돌 미검사, 펌웨어 IK와 세부가 다를 수 있음).
"""

import argparse

import numpy as np

from . import draw_executor as de
from . import mirobot_sim as ms
from . import paths
from .limits import executor_region

LEVEL_SEED = np.radians([0, 0, 0, 0, -90, 0])      # 팔뚝 수평, 플랜지가 벽 쪽
_LEVEL_ROT = ms.fk(LEVEL_SEED)[0][:3, :3]


def pen_rotation(tilt_down_deg):
    """플랜지 자세: 펜(플랜지 축)이 벽 쪽(+X)을 향하고 펜 끝을 tilt_down_deg만큼 아래로."""
    return ms._rpy(0.0, np.radians(tilt_down_deg), 0.0) @ _LEVEL_ROT


def singularity_deg(q):
    """J4 축과 J6 축 사이 각도(0~90°). 0이면 손목 특이점(J4·J6가 같은 축으로 돎)."""
    axes = ms.joint_axes(q)
    c = abs(float(np.dot(axes[3], axes[5])))
    return float(np.degrees(np.arccos(min(1.0, c))))


def wall_x(center_x, pen_len_mm, tilt_down_deg):
    """펜 끝이 종이에 닿는 X: 플랜지 X + 펜 길이(플랜지 면 -> 펜 끝, 펜 축 방향) × cos(기울기)."""
    return center_x + pen_len_mm * float(np.cos(np.radians(tilt_down_deg)))


def _paper_points(cfg, step):
    """실행기 그리기 범위(limits)의 격자점과 테두리 꼭짓점 (종이 mm), 중심에서 가까운 순."""
    region = executor_region(cfg)
    xs = np.arange(-region.x_max, region.x_max + 1e-9, step)
    ys = np.arange(region.y_min, region.top(0) + 1e-9, step)
    pts = [(float(x), float(y)) for x in xs for y in ys if region.contains(x, y)]
    pts += region.outline()[:-1]
    return sorted(set(pts), key=lambda p: p[0] ** 2 + p[1] ** 2)


def _wrap(q):
    """J1~J5를 한 바퀴(360°) 차이 나는 같은 자세 중 관절 범위 가운데에 가장 가까운 값으로 (J5 범위 -200°~+30°처럼
    ±180°를 넘는 범위도 그대로 둠). J6은 한계가 ±360°라 그대로."""
    q = np.array(q, float)
    mid = ms.JOINT_LIMITS_RAD[:5].mean(axis=1)
    q[:5] = mid + (q[:5] - mid + np.pi) % (2 * np.pi) - np.pi
    return q


def study(cfg, center_x, center_z, tilt_down_deg, step=10.0, center_y=0.0, rot=None, seed=None):
    """플랜지가 종이 중심 (center_x, center_y, center_z)에 있고 펜 자세가 고정일 때, 그리기 범위 전체를 검사.
    종이 면은 평평하다고 봅니다(X 일정). rot·seed를 주면 tilt_down_deg 대신 그 플랜지 자세·IK 시작 관절각을 씀
    (예: 직각 홀더는 홈 자세 ms.HOME_ROT, 관절 0에서 시작).
    반환: 최소 관절 여유·가장 빠듯한 축·특이점까지 최소 각도·도달 불가 점 수."""
    rot = pen_rotation(tilt_down_deg) if rot is None else rot
    seed = LEVEL_SEED if seed is None else np.asarray(seed, float)
    sy, sz = cfg["paper_x_to_robot_y_sign"], cfg["paper_y_to_robot_z_sign"]
    lo, hi = ms.JOINT_LIMITS_RAD[:, 0], ms.JOINT_LIMITS_RAD[:, 1]
    center_q, _, _ = ms.ik(np.array([center_x, center_y, center_z]), seed, r_goal=rot, iters=300)
    worst, worst_axis, sing, unreachable = np.inf, None, np.inf, 0
    points = _paper_points(cfg, step)
    for px, py in points:
        target = np.array([center_x, center_y + sy * px, center_z + sz * py])
        q, _, ok = ms.ik(target, center_q, r_goal=rot, iters=300)
        if not ok:
            q, _, ok = ms.ik(target, seed, r_goal=rot, iters=300)
        if not ok:
            unreachable += 1
            continue
        q = _wrap(q)
        margin = np.degrees(np.minimum(q - lo, hi - q))
        j = int(np.argmin(margin))
        if margin[j] < worst:
            worst, worst_axis = float(margin[j]), ms.CONTROLLER_AXES[j]
        sing = min(sing, singularity_deg(q))
    return {"min_margin_deg": worst, "worst_axis": worst_axis, "min_singularity_deg": sing,
            "unreachable": unreachable, "points": len(points)}


def main():
    paths.safe_console()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--tilt", type=float, nargs="+", default=[0.0, 10.0, 15.0, 20.0], help="펜 끝을 아래로 기울이는 각도(°)")
    ap.add_argument("--x", type=float, nargs="+", default=[150.0, 160.0, 170.0, 180.0, 190.0, 200.0],
                    help="종이 중심에서 플랜지 X(mm)")
    ap.add_argument("--z", type=float, default=230.0, help="종이 중심에서 플랜지 Z(mm)")
    ap.add_argument("--pen-len", type=float, default=None, help="플랜지 면 -> 펜 끝 길이(mm)")
    ap.add_argument("--step", type=float, default=10.0)
    args = ap.parse_args()
    cfg = de.load_config(args.config)
    print(f"플랜지 Z {args.z:g} mm, 그리기 범위 = 설정의 limits, 격자 {args.step:g} mm")
    print("기울기  플랜지X  최소여유(축)          특이점까지  도달불가" + ("  펜끝 X(벽)" if args.pen_len else ""))
    for tilt in args.tilt:
        for x in args.x:
            r = study(cfg, x, args.z, tilt, step=args.step)
            line = (f"{tilt:5.0f}°  {x:6.0f}  {r['min_margin_deg']:6.1f}° ({r['worst_axis']})"
                    f"  {r['min_singularity_deg']:8.1f}°  {r['unreachable']:3d}/{r['points']}")
            if args.pen_len:
                line += f"  {wall_x(x, args.pen_len, tilt):8.1f}"
            print(line)


if __name__ == "__main__":
    main()
