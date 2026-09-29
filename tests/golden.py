"""리팩터링 전후 run_pipeline() 결과 비교용 기준값 (tests/data/pipeline_golden.json)

    python tests/golden.py --write     # 리팩터링 전에 한 번 실행해 기준값 저장
    python tests/golden.py --perf      # 샘플 이미지 단계별 시간 측정
합성 이미지 기준값은 CI에서도 비교하고, 샘플(input/, 저장소 밖)은 있을 때만 비교한다.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mirobot_sketch import presets  # noqa: E402
from mirobot_sketch import sketch_pipeline as sp  # noqa: E402

GOLDEN = ROOT / "tests" / "data" / "pipeline_golden.json"


def default_cfg():
    """패키지 기본 설정의 사본. 실제 로봇 보정이 robot/drawing_config.json을 바꿔도 테스트 기준이 흔들리지 않게 한다."""
    import copy

    from mirobot_sketch import draw_executor as de, paths

    return copy.deepcopy(de.load_config(paths.asset("drawing_config.json")))
SAMPLE_TYPES = {"illust1_color.jpg": "illustration", "illust2_color.jpg": "illustration",
                "illust3_manga.jpg": "manga", "photo1_mic.webp": "photo",
                "photo2_stage.jpg": "photo", "photo3_chair.jpg": "photo"}


def synthetic_images():
    """선화(사각형·원·잡음 점)와 면으로 된 도형(명암·글씨)."""
    line = np.full((400, 400), 255, np.uint8)
    cv2.rectangle(line, (80, 80), (320, 320), 0, 3)
    cv2.circle(line, (200, 200), 60, 0, 3)
    for x in range(20, 60, 12):
        cv2.line(line, (x, 20), (x + 6, 30), 0, 2)
    shade = np.full((300, 420), 230, np.uint8)
    cv2.circle(shade, (140, 150), 90, 90, -1)
    cv2.rectangle(shade, (230, 60), (380, 240), 160, -1)
    cv2.putText(shade, "Hi", (250, 170), cv2.FONT_HERSHEY_SIMPLEX, 2.5, 40, 6)
    return {"line": sp.resize_max_side(line), "shade": sp.resize_max_side(shade)}


def cases_for(image_type):
    """이미지 종류 프리셋 × 상세도(high, medium) × 선 후보(canny, dark). rembg는 끔."""
    t = presets.IMAGE_TYPES[image_type]
    out = []
    for detail in ("high", "medium"):
        lo, hi, ml, eps = presets.DETAIL_PRESETS[detail]
        for src in ("canny", "dark"):
            out.append({"canny_low": lo, "canny_high": hi, "blur_ksize": 5, "min_length_px": ml,
                        "epsilon_px": eps, "method": "skeleton", "line_source": src,
                        "median_ksize": t["median"], "merge_join_px": presets.DEFAULT_MERGE_JOIN_PX,
                        "dedupe_px": presets.DEFAULT_DEDUPE_PX})
    return out


def sample_cases(image_type):
    c = cases_for(image_type)
    return [c[0], c[3]]   # high+canny, medium+dark (샘플 1장에 2가지만: 시간 절약)


def signature(strokes):
    h = hashlib.sha1()
    total = 0.0
    for s in strokes:
        a = np.round(np.asarray(s, dtype=np.float64), 2)
        if len(a) > 1:
            total += float(np.hypot(*np.diff(a, axis=0).T).sum())
        h.update(a.tobytes())
        h.update(b"|")
    return {"count": len(strokes), "total_px": round(total, 1), "sha1": h.hexdigest()}


def compute_all(include_samples=True):
    out = {"synthetic": {}, "samples": {}}
    for name, img in synthetic_images().items():
        for i, c in enumerate(cases_for("illustration") + cases_for("manga")):
            out["synthetic"][f"{name}#{i}"] = signature(sp.run_pipeline(img, **c)[1])
    if include_samples:
        for fname, typ in SAMPLE_TYPES.items():
            p = ROOT / "input" / fname
            if not p.exists():
                continue
            gray = sp.load_gray(p)
            for i, c in enumerate(sample_cases(typ)):
                out["samples"][f"{fname}#{i}"] = signature(sp.run_pipeline(gray, **c)[1])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="기준값 저장")
    ap.add_argument("--perf", action="store_true", help="샘플 단계별 시간")
    args = ap.parse_args()
    if args.write:
        GOLDEN.parent.mkdir(parents=True, exist_ok=True)
        data = compute_all()
        GOLDEN.write_text(json.dumps(data, indent=1, ensure_ascii=False), encoding="utf-8")
        print(f"저장: {GOLDEN} (합성 {len(data['synthetic'])}, 샘플 {len(data['samples'])})")
    if args.perf:
        import time

        from mirobot_sketch import stages
        for fname in SAMPLE_TYPES:
            p = ROOT / "input" / fname
            if not p.exists():
                continue
            inp = {"gray": sp.load_gray(p), "color": sp.load_color(p)}
            pl, prm = stages.Pipeline(), stages.default_params()
            t0 = time.perf_counter()
            pl.run(inp, fname, prm)
            full = time.perf_counter() - t0
            t0 = time.perf_counter()
            pl.run(inp, fname, {**prm, "epsilon_px": 2.0})
            late = time.perf_counter() - t0
            lab = pl.run(inp, fname, {**prm, "edge_mode": "lab"})["simplify"]["strokes"]
            luma = stages.Pipeline().run(inp, fname, prm)["simplify"]["strokes"]
            print(f"{fname}: 전체 {full:.2f}s, 단순화만 {late:.3f}s, 획 수 밝기 {len(luma)} / 색 차이 {len(lab)}")


if __name__ == "__main__":
    main()
