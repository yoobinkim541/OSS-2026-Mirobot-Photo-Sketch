"""scale_rows 결과 한 장의 모양 판정: 줄마다 양 끝 선 사이(계획 80 mm), 이웃 간격(계획 20 mm),
세로선 기울기(위로 1 mm당 옆으로 mm), 줄 사이(계획 30 mm), 선 길이(계획 8 mm). 위치는 선 아래 끝(펜을 내린 점) 기준.

  python check_rows.py [결과 폴더 이름(기본 scalerows)]"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fit_warp as fw  # noqa: E402


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "scalerows"
    e = fw.line_ends(os.path.join(os.environ["TEMP"], name))
    if len(e) != 20:
        raise SystemExit(f"선 {len(e)}개를 찾았습니다(20개여야 함)")
    rows_y = []
    for r, plan_y in enumerate(fw.ROWS):
        s = e[r * 5:(r + 1) * 5]
        xb, yb, xt, yt = s.T
        slope = (xt - xb) / (yt - yb)
        rows_y.append(yb.mean())
        print(f"줄 y {plan_y:+.0f}: 양 끝 {xb[-1] - xb[0]:.1f} mm(계획 80), 이웃 {np.round(np.diff(xb), 1).tolist()}, "
              f"기울기 {np.round(slope, 3).tolist()}, 길이 {np.round(np.hypot(xt - xb, yt - yb), 1).tolist()}")
    print("줄 사이(계획 30):", np.round(-np.diff(rows_y), 1).tolist())


if __name__ == "__main__":
    main()
