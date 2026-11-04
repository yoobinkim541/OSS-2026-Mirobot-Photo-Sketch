"""
사진 한 장 -> 종이 좌표 획 JSON + 미리보기 이미지 (명령줄 버전)
==============================================================
GUI 없이 같은 파라미터로 결과를 재현할 때 사용합니다. 사용한 파라미터는
출력 JSON의 "source" 항목에 함께 저장됩니다.

사용법:
    mirobot-strokes photo.jpg --type photo
    mirobot-strokes manga.jpg --type manga --out out/manga
    mirobot-strokes illust.jpg --type illustration --detail medium --rembg
    (저장소에서는 python CV/make_strokes.py ... 도 같음)

출력:
    <out>.json          : robot/draw_executor.py 가 읽는 획 데이터 (mm)
    <out>_preview.png   : 입력 / 엣지 / 획 / 그리는 순서 비교 이미지
    <out>.svg           : (--svg) 펜 플로터·GRBL 장비용 SVG (실제 종이 크기, mm)
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from . import draw_executor as de
from . import paper_mapping as pm
from . import paths, presets
from . import sketch_pipeline as sp
from . import svg_export


def main():
    paths.safe_console()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", type=Path)
    ap.add_argument("--out", type=Path, help="출력 경로(확장자 제외). 기본: 입력 파일 이름_strokes")
    ap.add_argument("--type", choices=presets.IMAGE_TYPES,
                    help="이미지 종류 추천 설정 (photo / illustration / manga). 아래 개별 옵션이 우선")
    ap.add_argument("--detail", choices=presets.DETAIL_PRESETS)
    ap.add_argument("--canny", type=float, nargs=2, metavar=("LOW", "HIGH"))
    ap.add_argument("--min-length", type=float, help="이보다 작은 선 덩어리 제거 (px)")
    ap.add_argument("--epsilon", type=float, help="단순화 허용 오차 (px)")
    ap.add_argument("--blur", type=int, default=5)
    ap.add_argument("--max-side", type=int, default=sp.DEFAULT_MAX_SIDE)
    ap.add_argument("--method", choices=["skeleton", "contour"], default="skeleton",
                    help="skeleton: 선을 한 번씩만 지나감(기본) / contour: 비교용 기존 방식")
    ap.add_argument("--lines", choices=["canny", "dark"], default="canny",
                    help="canny: 사진 윤곽(기본) / dark: 선화의 어두운 선 중심선")
    ap.add_argument("--median", type=int, help="미디언 블러 크기(px). 만화 스크린톤 제거 (0 = 사용 안 함)")
    ap.add_argument("--rembg", action=argparse.BooleanOptionalAction, help="배경 제거 사용 (--no-rembg로 끔)")
    ap.add_argument("--dedupe", type=int, default=presets.DEFAULT_DEDUPE_PX,
                    help=f"이 거리(px) 안의 이중선은 하나만 (0 = 끔, 기본 {presets.DEFAULT_DEDUPE_PX})")
    ap.add_argument("--merge", type=float, default=presets.DEFAULT_MERGE_JOIN_PX,
                    help=f"끝점이 이 거리(px) 안이면 이어 그림 (0 = 끔, 기본 {presets.DEFAULT_MERGE_JOIN_PX:g})")
    ap.add_argument("--box", type=float, nargs=2, default=pm.DEFAULT_BOX_MM, metavar=("W", "H"),
                    help="그리기 상자 크기 mm (기본 100 100)")
    ap.add_argument("--config", type=Path, help="시간 추정에 쓰는 로봇 설정 (기본: 저장소 또는 사용자 폴더)")
    ap.add_argument("--svg", action="store_true", help="펜 플로터·GRBL 장비용 SVG(<out>.svg)도 저장")
    args = ap.parse_args()

    t = presets.IMAGE_TYPES.get(args.type, {})
    detail = args.detail or t.get("detail", "medium")
    rembg = args.rembg if args.rembg is not None else t.get("rembg", False)
    median = args.median if args.median is not None else t.get("median", 0)
    low, high, min_len, eps = presets.DETAIL_PRESETS[detail]
    if args.canny:
        low, high = args.canny
    if args.min_length is not None:
        min_len = args.min_length
    if args.epsilon is not None:
        eps = args.epsilon

    try:
        gray = (sp.remove_background(args.image, args.max_side, cache_dir=paths.cache_dir()) if rembg
                else sp.load_gray(args.image, args.max_side))
    except (RuntimeError, ValueError) as e:  # rembg 미설치, 이미지 읽기 실패 등
        print(f"오류: {e}")
        return 2
    edges, strokes_px = sp.run_pipeline(gray, low, high, args.blur, min_len, eps, args.method, args.lines,
                                        median, merge_join_px=args.merge, dedupe_px=args.dedupe)
    if not strokes_px:
        print("획이 하나도 없습니다. --detail high 또는 Canny 임계값을 낮춰보세요.")
        return 1

    strokes_mm, placement = pm.pixels_to_paper(strokes_px, box_mm=tuple(args.box))
    metrics = sp.stroke_metrics(strokes_mm, start=(0.0, 0.0))  # 펜은 종이 중심에서 출발/복귀
    cfg = de.load_config(args.config)
    timing = de.estimate_time([[tuple(p) for p in s] for s in strokes_mm], cfg)
    metrics["estimated_time"] = timing

    source = {
        "image": str(args.image),
        "image_size_px": [int(gray.shape[1]), int(gray.shape[0])],
        "params": {
            "type": args.type, "detail": detail, "canny_low": low, "canny_high": high, "blur_ksize": args.blur,
            "min_length_px": min_len, "epsilon_px": eps, "max_side": args.max_side,
            "method": args.method, "line_source": args.lines, "median_ksize": median, "rembg": rembg,
            "dedupe_px": args.dedupe, "merge_join_px": args.merge,
        },
    }
    doc = pm.build_strokes_document(strokes_mm, placement, metrics, source)

    out = args.out or args.image.with_name(args.image.stem + "_strokes")
    out.parent.mkdir(parents=True, exist_ok=True)
    pm.save_strokes_json(out.with_suffix(".json"), doc)
    if args.svg:
        svg_export.save_svg(out.with_suffix(".svg"), strokes_mm, svg_export.paper_size_mm(placement))

    preview = np.hstack([
        cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(255 - edges, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(255 - sp.draw_strokes_image(strokes_px, gray.shape), cv2.COLOR_GRAY2BGR),
        sp.draw_order_preview(strokes_px, gray.shape),
    ])
    preview_path = out.with_name(out.name + "_preview.png")
    cv2.imencode(".png", preview)[1].tofile(str(preview_path))

    print(f"획 {metrics['stroke_count']}개, 점 {metrics['point_count']}개")
    print(f"그림 크기 {placement['drawing_width_mm']} x {placement['drawing_height_mm']} mm")
    print(f"펜다운 {metrics['pen_down_length']} mm, 펜업 이동 {metrics['pen_up_length']} mm")
    print(f"예상 시간: {de.format_time(timing)}")
    print(f"저장: {out.with_suffix('.json')}")
    print(f"미리보기: {preview_path}")
    if args.svg:
        print(f"SVG: {out.with_suffix('.svg')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
