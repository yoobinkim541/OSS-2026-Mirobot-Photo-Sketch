"""
Mirobot 드로잉 경로 3D 시뮬레이터 (로봇 없이 실행)
==================================================
robot/draw_executor.py가 보낼 G-code 경로를 그대로 만들어, Mirobot 기구학
모델로 관절각을 풀고 관절 한계(=컨트롤러 Soft limit)를 넘는지 검사합니다.

    mirobot-sim trajectories/orientation-test-F.json
    mirobot-sim out/photo.json --gif out/photo_sim.gif
    mirobot-sim --shape heart_70x60mm_centered     (도형 라이브러리, 저장소에서만)
    mirobot-sim --reach-map                         (종이 위 도달 가능 영역)
    (저장소에서는 python sim/mirobot_sim.py ... 도 같음)

모델:
  - 관절 구조와 한계는 WLKATA 공식 ROS2 저장소의 wlkata_mirobot_description.urdf
    (joint1~6 origin/axis/limit)를 옮긴 것. 한계는 사용 설명서의 관절 범위와 같음.
  - URDF의 link6 원점은 손목 중심. 컨트롤러 TCP(플랜지)는 홈 자세에서
    (198.668, 0, 230.477) mm로 기록돼 있어, 둘의 차이를 공구 오프셋으로 둔다.
  - Cartesian 명령의 자세 A,B,C=0은 홈 자세의 플랜지 방향과 같다고 본다.
  - G01은 직선 보간이므로 명령 사이를 1mm 간격으로 나눠 IK를 푼다.

한계: 충돌(벽·종이·펜홀더와 팔)은 검사하지 않음. 컨트롤러 펌웨어의 실제 IK/한계
처리와 세부가 다를 수 있으므로, 여유가 적은 경로(한계에서 5도 이내)는 실물에서
천천히 확인할 것.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from . import draw_executor as de
from . import paths

# (origin xyz [m], origin rpy [rad], axis) — wlkata_mirobot_description.urdf
URDF_JOINTS = [
    ((0.0, 0.0, 0.127), (0.0, 0.0, 0.0), (0, 0, 1)),
    ((0.029687, 0.0, 0.0), (-1.5708, -1.5708, 0.0), (0, 0, 1)),
    ((0.108, 0.0, 0.0), (0.0, 0.0, 0.0), (0, 0, 1)),
    ((0.02, 0.16875, 0.0), (-1.5708, 0.0, 0.0), (0, 0, 1)),
    ((0.0, 0.0, 0.0), (1.5708, -1.5708, 0.0), (0, 0, 1)),
    ((0.0, 0.0, 0.0), (-1.5708, 0.0, 0.0), (0, 0, -1)),
]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CONTROLLER_AXES = ["X(J1)", "Y(J2)", "Z(J3)", "A(J4)", "B(J5)", "C(J6)"]
JOINT_LIMITS_RAD = np.array([
    (-1.919, 2.792), (-0.61, 1.221), (-2.094, 1.047),
    (-3.141, 2.53), (-3.49, 0.523), (-6.283, 6.283),
])
HOME_TCP_MM = np.array([198.668, 0.0, 230.477])  # 컨트롤러가 보고한 홈 자세 TCP
WARN_MARGIN_DEG = 5.0


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_angle(axis, q):
    a = np.asarray(axis, float)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


_FIXED = []
for xyz, rpy, _ in URDF_JOINTS:
    t = np.eye(4)
    t[:3, :3] = _rpy(*rpy)
    t[:3, 3] = np.array(xyz) * 1000.0  # mm
    _FIXED.append(t)


def fk_chain(q, tool=None):
    """관절각(rad) -> (플랜지 4x4 변환, 관절 위치 리스트[mm])."""
    t = np.eye(4)
    pts = [t[:3, 3].copy()]
    for fixed, (_, _, axis), qi in zip(_FIXED, URDF_JOINTS, q):
        rot = np.eye(4)
        rot[:3, :3] = _axis_angle(axis, qi)
        t = t @ fixed @ rot
        pts.append(t[:3, 3].copy())
    if tool is not None:
        t = t.copy()
        t[:3, 3] = t[:3, 3] + t[:3, :3] @ tool
        pts.append(t[:3, 3].copy())
    return t, pts


# 공구 오프셋: 홈 자세에서 손목 중심 -> 컨트롤러 TCP
_T0, _ = fk_chain(np.zeros(6))
TOOL_OFFSET_MM = _T0[:3, :3].T @ (HOME_TCP_MM - _T0[:3, 3])
HOME_ROT = _T0[:3, :3]


def fk(q):
    return fk_chain(q, TOOL_OFFSET_MM)


def _rot_error(r_cur, r_goal):
    r = r_goal @ r_cur.T
    return 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


_FIXED_ARR = np.array(_FIXED)                                   # (6, 4, 4)
_AXIS_SIGN = np.array([axis[2] for _, _, axis in URDF_JOINTS], float)   # 모든 관절 축이 z축(±1)이라 회전을 직접 계산


def fk_batch(qs, tool=TOOL_OFFSET_MM):
    """여러 관절각 (n, 6) -> 플랜지 변환 (n, 4, 4). fk와 같은 결과를 배치로 계산 (IK가 이걸로 7개 자세를 한 번에 풂)."""
    qs = np.atleast_2d(np.asarray(qs, float))
    t = np.broadcast_to(np.eye(4), (len(qs), 4, 4)).copy()
    for i in range(6):
        t = t @ _FIXED_ARR[i]
        ang = qs[:, i] * _AXIS_SIGN[i]
        c, s = np.cos(ang)[:, None], np.sin(ang)[:, None]
        col0, col1 = t[:, :, 0].copy(), t[:, :, 1].copy()       # t @ Rz(ang)
        t[:, :, 0] = c * col0 + s * col1
        t[:, :, 1] = -s * col0 + c * col1
    if tool is not None:
        t[:, :3, 3] += np.einsum("nij,j->ni", t[:, :3, :3], tool)
    return t


def _error_batch(t, target_mm, r_goal, w_rot):
    """(n, 6) 오차: [목표 - 위치, w_rot * 자세 오차]. 스칼라 버전(_rot_error)과 같은 식."""
    r = np.einsum("ij,nkj->nik", r_goal, t[:, :3, :3])          # r_goal @ r_cur.T
    rot = 0.5 * np.stack([r[:, 2, 1] - r[:, 1, 2], r[:, 0, 2] - r[:, 2, 0], r[:, 1, 0] - r[:, 0, 1]], axis=1)
    return np.concatenate([target_mm - t[:, :3, 3], w_rot * rot], axis=1)


def ik(target_mm, q0, r_goal=HOME_ROT, iters=100, tol_mm=1e-3):
    """위치+자세 IK (감쇠 최소제곱). 반환: (q, 위치오차 mm, 수렴 여부). 관절 한계는 적용하지 않음.
    유한 차분 야코비안을 위한 7개 자세를 배치로 계산한다 (예전 구현과 같은 알고리즘, 파이썬 호출 수만 줄임)."""
    q = np.array(q0, float)
    w_rot = 100.0  # 자세 오차(rad)를 mm 스케일로
    h = 1e-6
    steps = np.eye(6) * h
    damping = 0.5 ** 2 * np.eye(6)
    for _ in range(iters):
        e_all = _error_batch(fk_batch(np.vstack([q, q + steps])), target_mm, r_goal, w_rot)
        e = e_all[0]
        if np.linalg.norm(e[:3]) < tol_mm and np.linalg.norm(e[3:]) < w_rot * 1e-5:
            break
        jac = ((e - e_all[1:]) / h).T       # jac[:, i] = (e - e_i) / h
        q = q + jac.T @ np.linalg.solve(jac @ jac.T + damping, e)
    t = fk_batch(q)
    pos_err = float(np.linalg.norm(target_mm - t[0, :3, 3]))
    rot_err = float(np.linalg.norm(_rot_error(t[0, :3, :3], r_goal)))
    return q, pos_err, pos_err < 0.05 and rot_err < 1e-3


# ---------------------------------------------------------------------------
# 경로 시뮬레이션
# ---------------------------------------------------------------------------

_GCODE_XYZ = __import__("re").compile(r"X([-\d.]+) Y([-\d.]+) Z([-\d.]+)")


def pen_tip_offset(cfg):
    """플랜지(TCP) -> 펜 끝 오프셋(로봇 좌표, mm). 자세가 고정이라 경로 내내 일정. 시각화 전용."""
    o = cfg.get("pen_tip_offset_mm", {"x": 0.0, "y": 0.0, "z": 0.0})
    return np.array([o["x"], o["y"], o["z"]], dtype=float)


_GCODE_F = __import__("re").compile(r"F([\d.]+)")


def plan_targets(strokes, cfg, air=False):
    """실행기와 같은 계획 -> [(xyz_mm, 설명, pen_down, feed)]. 시작점은 종이 중심 펜다운(=사용자가 둔 위치)."""
    planner = de.Planner(cfg, air=air)
    out = [(np.array(planner.pose(0, 0, True)), "start (pen at paper center)", False, 0.0)]
    for line, label in planner.plan(strokes):
        xyz = np.array([float(v) for v in _GCODE_XYZ.search(line).groups()])
        out.append((xyz, label, label.endswith(" draw"), float(_GCODE_F.search(line).group(1))))
    return out


class SimulationCancelled(Exception):
    """progress 콜백이 던지면 시뮬레이션을 중간에 멈춘다 (더 이상 필요 없는 백그라운드 계산을 끝냄)."""


def simulate(targets, step_mm=1.0, progress=None):
    """직선 보간하며 IK. 반환 dict: samples(q, tcp, 설명, pen_down), 위반/실패 목록, 관절별 최소 여유.
    progress(끝낸 명령 수, 전체 명령 수)는 명령마다 불려 화면이 진행률을 보여 줄 수 있고,
    SimulationCancelled를 던지면 그대로 밖으로 나와 계산을 멈춘다."""
    q = np.zeros(6)
    samples = []
    failures = []
    prev = targets[0][0]
    cmd_feed = []   # 명령별 속도 (시작 자세 제외) — 실시간 따라가기의 보간 속도
    for t, (xyz, label, pen_down, feed) in enumerate(targets):
        if progress is not None:
            progress(t, len(targets))
        if t > 0:
            cmd_feed.append(feed)
        n = max(1, int(np.ceil(np.linalg.norm(xyz - prev) / step_mm)))
        q_before = None
        for k in range(1, n + 1):
            p = prev + (xyz - prev) * (k / n)
            # 직선 구간에서는 앞선 두 점의 변화량으로 다음 자세를 예측해 시작하면 IK 반복이 줄어든다
            # (해는 같음: 관절각 차이 1e-5 rad 이하)
            q_start = q if q_before is None else q + (q - q_before)
            q_before = q
            q, err, ok = ik(p, q_start)
            if not ok:
                failures.append({"command": label, "target_mm": p.round(2).tolist(), "pos_error_mm": round(err, 3)})
            samples.append((q.copy(), p.copy(), label, pen_down, t - 1))   # 마지막 값: G-code 명령 번호
        prev = xyz

    qs = np.array([s[0] for s in samples])
    lo, hi = JOINT_LIMITS_RAD[:, 0], JOINT_LIMITS_RAD[:, 1]
    margin = np.minimum(qs - lo, hi - qs)  # 음수면 한계 초과
    violations = []
    for j in range(6):
        bad = np.nonzero(margin[:, j] < 0)[0]
        if len(bad):
            i = int(bad[0])
            violations.append({
                "axis": CONTROLLER_AXES[j], "first_command": samples[i][2],
                "angle_deg": round(float(np.degrees(qs[i, j])), 2),
                "limit_deg": [round(float(np.degrees(lo[j])), 1), round(float(np.degrees(hi[j])), 1)],
                "samples_over_limit": int(len(bad)),
            })
    return {
        "samples": samples,
        "cmd_feed": cmd_feed,
        "failures": failures,
        "violations": violations,
        "min_margin_deg": np.degrees(margin.min(axis=0)),
        "range_deg": np.degrees(np.stack([qs.min(axis=0), qs.max(axis=0)], axis=1)),
    }


def trajectory_doc(result, cfg, source):
    """시뮬레이션 결과를 RViz 재생용 관절 궤적 문서로 (sim/rviz_playback.py가 읽는 형식).
    점마다 cmd = G-code 명령 번호(0부터, 시작 자세는 -1) — 실시간 따라가기가 로봇의 명령 응답과 맞춤."""
    tip = pen_tip_offset(cfg)
    return {
        "joint_names": JOINT_NAMES, "units": "rad", "step_mm": 1.0, "source": str(source),
        "pen_tip_offset_mm": tip.tolist(),
        "command_count": len(result["cmd_feed"]),
        "cmd_feed_mm_min": [float(f) for f in result["cmd_feed"]],
        "points": [{"q": [round(float(v), 5) for v in s[0]], "tcp_mm": [round(float(v), 3) for v in s[1]],
                    "pen_tip_mm": [round(float(v), 3) for v in s[1] + tip],
                    "pen_down": bool(s[3]), "command": s[2], "cmd": int(s[4])} for s in result["samples"]],
    }


def verdict(result):
    if result["failures"]:
        return "FAIL: 도달 불가(IK 실패)"
    if result["violations"]:
        return "FAIL: 관절 한계 초과 (실물에서 Soft limit 예상)"
    if result["min_margin_deg"].min() < WARN_MARGIN_DEG:
        return f"WARN: 한계까지 여유 {WARN_MARGIN_DEG}도 미만인 관절 있음"
    return "PASS"


def print_report(result, title):
    print(f"== {title}")
    print(f"샘플 {len(result['samples'])}개 (1mm 간격)")
    print(f"{'축':<7}{'최소(deg)':>11}{'최대(deg)':>11}{'한계 여유(deg)':>16}")
    for j in range(6):
        lo, hi = result["range_deg"][j]
        print(f"{CONTROLLER_AXES[j]:<7}{lo:>11.2f}{hi:>11.2f}{result['min_margin_deg'][j]:>16.2f}")
    for v in result["violations"]:
        print(f"  한계 초과: {v}")
    for f in result["failures"][:5]:
        print(f"  IK 실패: {f}")
    print(f"판정: {verdict(result)}")


# ---------------------------------------------------------------------------
# 그림
# ---------------------------------------------------------------------------

def save_joint_plot(result, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    qs = np.degrees(np.array([s[0] for s in result["samples"]]))
    down = np.array([s[3] for s in result["samples"]])
    fig, axes = plt.subplots(6, 1, figsize=(9, 11), sharex=True)
    for j, ax in enumerate(axes):
        lo, hi = np.degrees(JOINT_LIMITS_RAD[j])
        ax.plot(qs[:, j], color="#1f5fbf", lw=1.2)
        ax.fill_between(range(len(qs)), qs[:, j].min(), qs[:, j].max(), where=down, color="#1f5fbf", alpha=0.08, lw=0)
        for lim in (lo, hi):
            if qs[:, j].min() - 15 < lim < qs[:, j].max() + 15:
                ax.axhline(lim, color="#c0392b", ls="--", lw=1)
        ax.set_ylabel(f"{CONTROLLER_AXES[j]}\n(deg)", fontsize=8)
        ax.grid(alpha=0.3)
    axes[0].set_title("Joint angles along path (shaded = pen down, red dashed = joint limit)", fontsize=10)
    axes[-1].set_xlabel("sample (1 mm steps)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def save_animation(result, path, cfg, frames=90):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    samples = result["samples"]
    idx = np.linspace(0, len(samples) - 1, min(frames, len(samples))).astype(int)
    c = cfg["paper_center_tcp_mm"]
    tip = pen_tip_offset(cfg)
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    # 종이 (A4 가로) — 펜다운 때 펜 끝이 닿는 평면
    px = c["x"] + tip[0]
    ys = np.array([-148.5, 148.5, 148.5, -148.5, -148.5]) + c["y"] + tip[1]
    zs = np.array([-105, -105, 105, 105, -105]) + c["z"] + tip[2]

    def draw(k):
        ax.cla()
        i = idx[k]
        _, pts = fk_chain(samples[i][0], TOOL_OFFSET_MM)
        pts = np.array(pts)
        ax.plot([px] * 5, ys, zs, color="#999", lw=1)
        trail = np.array([s[1] + tip for s in samples[: i + 1] if s[3]]) if i else np.empty((0, 3))
        if len(trail):
            # 펜다운 구간만 점으로 (획 사이 연결선이 생기지 않게)
            ax.scatter(trail[:, 0], trail[:, 1], trail[:, 2], s=2, color="#c0392b")
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="#1f5fbf", lw=3, ms=4)
        pen_end = pts[-1] + tip
        ax.plot([pts[-1][0], pen_end[0]], [pts[-1][1], pen_end[1]], [pts[-1][2], pen_end[2]],
                color="#e6a100", lw=4)  # 펜홀더 + 펜
        ax.set_xlim(-20, 340)
        ax.set_ylim(-140, 140)
        ax.set_zlim(0, 330)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.view_init(elev=15, azim=-150)
        ax.set_title(samples[i][2], fontsize=9)

    anim = FuncAnimation(fig, draw, frames=len(idx))
    anim.save(path, writer=PillowWriter(fps=12))
    plt.close(fig)


def reach_map(cfg, half_y=150, half_z=110, step=10, path=None):
    """종이 평면(X=접촉면)의 격자점마다 IK를 풀어 도달/한계 여유를 표시."""
    c = cfg["paper_center_tcp_mm"]
    planner = de.Planner(cfg)
    ys = np.arange(-half_y, half_y + 1, step)
    zs = np.arange(-half_z, half_z + 1, step)
    grid = np.full((len(zs), len(ys)), np.nan)
    # 중심에서 바깥으로 풀어 연속된 해를 따라가도록 행 단위로 이어서 탐색
    for zi, dz in enumerate(zs):
        q = np.zeros(6)
        q, _, _ = ik(np.array([planner.contact_x(c["y"], c["z"] + dz), c["y"], c["z"] + dz]), q)
        row_start = q
        for sign in (1, -1):
            q = row_start
            order = [i for i in range(len(ys)) if (ys[i] >= 0 if sign > 0 else ys[i] < 0)]
            order = sorted(order, key=lambda i: abs(ys[i]))
            for yi in order:
                y, z = c["y"] + ys[yi], c["z"] + dz
                q, _, ok = ik(np.array([planner.contact_x(y, z), y, z]), q)
                if not ok:
                    break
                lo, hi = JOINT_LIMITS_RAD[:, 0], JOINT_LIMITS_RAD[:, 1]
                grid[zi, yi] = float(np.degrees(np.minimum(q - lo, hi - q).min()))
    if path:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 5.5))
        im = ax.imshow(grid, origin="lower", cmap="RdYlGn", vmin=-20, vmax=40,
                       extent=[ys[0] - step / 2, ys[-1] + step / 2, zs[0] - step / 2, zs[-1] + step / 2])
        ax.contour(ys, zs, np.nan_to_num(grid, nan=-99), levels=[0, WARN_MARGIN_DEG], colors=["k", "k"],
                   linestyles=["-", "--"])
        from .limits import executor_region, pending_region
        sy = cfg["paper_x_to_robot_y_sign"]
        for region, color in ((executor_region(cfg), "#1f5fbf"), (pending_region(cfg), "#e67e00")):
            o = np.array(region.outline())                      # 종이 x -> 로봇 dY(부호), 종이 y -> dZ
            ax.plot(o[:, 0] * sy, o[:, 1], color=color, lw=2)
        ax.add_patch(plt.Rectangle((-148.5, -105), 297, 210, fill=False, ec="#666", ls=":"))
        ax.set_xlabel("robot dY from paper center (mm)" + ("  [paper right = -Y]" if sy < 0 else ""))
        ax.set_ylabel("robot dZ from paper center (mm)")
        ax.set_title("Min joint-limit margin on paper plane (deg). black: limit, dashed: 5 deg\n"
                     "blue: executor limits, orange: wide range (unverified), dotted: A4", fontsize=9)
        fig.colorbar(im, ax=ax, label="deg (blank = unreachable)")
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
    return ys, zs, grid


def strokes_from_shape(shape_id, cfg):
    tdir = paths.trajectories_dir()
    if tdir is None:
        raise SystemExit("--shape은 저장소(trajectories/shape-library.json)에서 실행할 때만 쓸 수 있습니다.")
    lib = json.loads((tdir / "shape-library.json").read_text(encoding="utf-8"))
    shape = next(s for s in lib["shapes"] if s["id"] == shape_id)
    sy = cfg["paper_x_to_robot_y_sign"]
    # 도형 라이브러리 좌표는 로봇 Y/Z 기준 -> 종이 좌표로 바꿔 실행기와 같은 경로를 만듦
    pts = [(p["y_mm"] / sy, p["z_mm"]) for p in shape["points_yz_mm"]]
    if shape.get("path_closed") and pts[0] != pts[-1]:
        pts.append(pts[0])
    return [pts]


def main():
    paths.safe_console()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("strokes_json", type=Path, nargs="?")
    ap.add_argument("--shape", help="trajectories/shape-library.json의 도형 id")
    ap.add_argument("--config", type=Path, default=de.DEFAULT_CONFIG)
    ap.add_argument("--gif", type=Path, help="3D 애니메이션 저장 경로")
    ap.add_argument("--plot", type=Path, help="관절각 그래프 저장 경로")
    ap.add_argument("--export", type=Path, help="관절 궤적 JSON 저장 (RViz 재생용)")
    ap.add_argument("--reach-map", type=Path, nargs="?", const=Path("reach_map.png"),
                    help="종이 평면 도달/여유 지도 저장")
    args = ap.parse_args()
    cfg = de.load_config(args.config)

    if args.reach_map:
        _, _, grid = reach_map(cfg, path=args.reach_map)
        print(f"도달 지도 저장: {args.reach_map}")
        if not (args.strokes_json or args.shape):
            return 0

    if args.shape:
        strokes, title = strokes_from_shape(args.shape, cfg), args.shape
    elif args.strokes_json:
        _, strokes = de.load_strokes(args.strokes_json)
        title = str(args.strokes_json)
    else:
        ap.error("strokes_json 또는 --shape 또는 --reach-map 중 하나가 필요합니다.")

    bad = de.check_limits(strokes, cfg)
    if bad:
        print(f"참고: 실행기 범위(limits)를 벗어난 점 {len(bad)}개 — 실행기는 이 경로를 거부합니다.")
    result = simulate(plan_targets(strokes, cfg))
    print_report(result, title)

    if args.plot:
        save_joint_plot(result, args.plot)
        print(f"관절 그래프: {args.plot}")
    if args.gif:
        save_animation(result, args.gif, cfg)
        print(f"3D 애니메이션: {args.gif}")
    if args.export:
        args.export.write_text(json.dumps(trajectory_doc(result, cfg, title), ensure_ascii=False), encoding="utf-8")
        print(f"관절 궤적: {args.export}")
    return 5 if verdict(result).startswith("FAIL") else 0


if __name__ == "__main__":
    sys.exit(main())
