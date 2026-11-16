"""scale_rows 결과(설정 하나로 그은 20개 세로선의 실제 위치)를 기구학 모델로 설명해 보는 오프라인 분석.
가설: 관절 하나의 실제 회전이 명령의 k배(호밍 0 기준)이거나, 영점이 d°만큼 어긋났다.
선마다 앱과 같은 계획 → 모델 IK 관절각 → 가설 관절각 → 순기구학 펜 끝(종이 면 y·z) → 사진 위치와 비교한다.
종이 위치·회전은 모를 수 있으니 강체 변환(회전+이동, 배율 1)으로 맞춘 뒤 잔차를 본다. 로봇은 움직이지 않는다."""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..")))
from mirobot_sketch import draw_executor as dex, mirobot_sim as ms, start_pose as spm  # noqa: E402

D = os.path.join(os.environ["TEMP"], "scalerows")
P = 8.0
ROWS = (40.0, 10.0, -20.0, -50.0)
XS = (-40.0, -20.0, 0.0, 20.0, 40.0)
ROW_SHEET_Y = (51.0, 11.0, -26.0, -56.0)   # 사진에서 정한 줄 가운데(종이 장 mm)


def observed():
    """선마다 잉크 무게중심(종이 장 mm). 순서는 ROWS × XS."""
    meta = json.load(open(os.path.join(D, "meta.json")))
    W, H = int(297 * P), int(210 * P)
    dst = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)
    warp = lambda n, m: cv2.warpPerspective(cv2.imread(os.path.join(D, n + ".jpg")),
                                            cv2.getPerspectiveTransform(np.float32(m), dst), (W, H))
    g = lambda im: cv2.GaussianBlur(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
    d = g(warp("before", meta["before"])) - g(warp("after", meta["after"]))
    d = np.clip(d - (cv2.medianBlur(np.clip(d + 128, 0, 255).astype(np.uint8), 41).astype(np.float32) - 128), 0, None)
    d[d < 6] = 0
    out = []
    for yc in ROW_SHEET_Y:
        y0, y1 = int((105 - yc - 10) * P), int((105 - yc + 10) * P)
        band = d[y0:y1]
        prof = np.convolve(band.mean(0), np.ones(9) / 9, "same")
        from scipy.signal import find_peaks
        pk, _ = find_peaks(prof[int((148.5 - 60) * P):int((148.5 + 80) * P)], distance=int(12 * P))
        pk = sorted(sorted(pk, key=lambda i: -prof[i + int((148.5 - 60) * P)])[:5])
        for i in pk:
            cx = i + int((148.5 - 60) * P)
            win = band[:, cx - int(4 * P):cx + int(4 * P)]
            ys, xs = np.nonzero(win)
            wts = win[ys, xs]
            out.append(((cx - int(4 * P) + (xs * wts).sum() / wts.sum()) / P - 148.5,
                        105 - (y0 + (ys * wts).sum() / wts.sum()) / P))
    return np.array(out)


def commanded():
    cfg = json.load(open(os.path.join(HERE, "..", "..", "..", "robot", "drawing_config.json"), encoding="utf-8"))
    dc = spm.drawing_config_from_recorded_contact(cfg)
    pl = dex.Planner(dc)
    r_goal, q0 = ms.simulation_pose(dc)
    tool = np.asarray(cfg["requested_start_pose"]["tool_offset_mm"], float)
    qs = []
    for y in ROWS:
        for x in XS:
            xyz = np.array(pl.pose(x, y + 4.0, True))       # 선 가운데(8 mm 선의 중간)
            q, err, ok = ms.ik(xyz, q0, r_goal)
            if not ok:
                raise SystemExit(f"IK 실패 {x},{y}")
            qs.append(q)
            q0 = q
    return np.array(qs), tool, dc


def tip_yz(qs, tool):
    t = ms.fk_batch(qs)
    tip = t[:, :3, 3] + np.einsum("nij,j->ni", t[:, :3, :3], tool)
    # 종이 면(접촉 X 근처)까지 펜 방향(플랜지 z)으로 옮겨 평면 위 점으로 비교
    x0 = tip[:, 0].mean()
    ax = t[:, :3, 2]
    s = (x0 - tip[:, 0]) / ax[:, 0]
    p = tip + s[:, None] * ax
    return p[:, 1], p[:, 2]


def rigid_fit(a, b):
    """b ≈ R a + t (2D, 배율 1). 잔차 RMS."""
    ca, cb = a.mean(0), b.mean(0)
    h = (a - ca).T @ (b - cb)
    u, _, vt = np.linalg.svd(h)
    r = (u @ vt).T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = (u @ vt).T
    res = b - ((a - ca) @ r.T + cb)
    return float(np.sqrt((res ** 2).sum(1).mean())), res


def main():
    obs = observed()
    qs, tool, dc = commanded()
    sy, sz = dc["paper_x_to_robot_y_sign"], dc["paper_y_to_robot_z_sign"]

    def paper(q):
        y, z = tip_yz(q, tool)
        return np.stack([sy * y, sz * z], 1)      # 로봇 Y·Z → 종이 x·y

    base, _ = rigid_fit(paper(qs), obs)
    print(f"관측 선 {len(obs)}개. 모델 그대로(배율 1) 잔차 RMS {base:.2f} mm")
    plan = np.array([(x, y + 4.0) for y in ROWS for x in XS])
    print(f"계획 좌표 그대로 잔차 RMS {rigid_fit(plan, obs)[0]:.2f} mm")
    rows = []
    for j in range(6):
        best = None
        for k in np.linspace(0.5, 1.5, 201):
            q = qs.copy()
            q[:, j] *= k
            rms, _ = rigid_fit(paper(q), obs)
            if best is None or rms < best[0]:
                best = (rms, k, "gain")
        for dd in np.linspace(-20, 20, 161):
            q = qs.copy()
            q[:, j] += np.radians(dd)
            rms, _ = rigid_fit(paper(q), obs)
            if rms < best[0]:
                best = (rms, dd, "zero°")
        q = qs.copy()
        q[:, j] = 2 * qs[0, j] * 0 + (2 * qs[:, j].mean() - qs[:, j])     # 접촉 근처에서 방향 반전
        rms_flip, _ = rigid_fit(paper(q), obs)
        rows.append((j + 1, best, rms_flip))
        print(f"J{j + 1}: 가장 좋은 {best[2]} = {best[1]:+.3f} → 잔차 {best[0]:.2f} mm, 방향 반전 시 {rms_flip:.2f} mm")
    json.dump({"base_rms": base, "obs": obs.tolist(), "rows": [(j, list(b), f) for j, b, f in rows]},
              open(os.path.join(D, "joint_fit.json"), "w"), indent=1, ensure_ascii=False, default=float)


if __name__ == "__main__":
    main()
