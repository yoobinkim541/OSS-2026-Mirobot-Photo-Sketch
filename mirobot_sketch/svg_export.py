"""
획 JSON -> SVG (펜 플로터·GRBL 장비용)
=====================================
sketch_strokes JSON(종이 중심 기준 mm)을 실제 종이 크기의 SVG로 바꿉니다.
AxiDraw(Inkscape 확장·axicli), vpype, SVG -> G-code 변환기처럼 SVG를 읽는 도구에 그대로 넘길 수 있습니다.

  - 문서 크기는 종이 크기(mm)이고, viewBox 한 단위가 1mm입니다.
  - SVG는 왼쪽 위가 원점이고 아래쪽이 +이므로 (x + W/2, H/2 - y)로 옮깁니다.
  - 획 하나가 <path> 하나이며, JSON의 그리는 순서를 그대로 둡니다.

사용법:
    mirobot-svg out/photo.json              -> out/photo.svg
    mirobot-svg out/photo.json -o photo.svg
    (사진에서 바로 만들 때: mirobot-strokes photo.jpg --svg)
"""

import argparse
import sys
from pathlib import Path

from . import paper_mapping as pm
from . import paths


def paper_size_mm(placement):
    """JSON placement의 종이 크기 (없으면 A4 가로)."""
    return (placement.get("paper_width_mm", pm.A4_LANDSCAPE_MM[0]),
            placement.get("paper_height_mm", pm.A4_LANDSCAPE_MM[1]))


def strokes_to_svg(strokes_mm, paper_mm=pm.A4_LANDSCAPE_MM, line_width_mm=0.5):
    w, h = paper_mm
    out = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:g}mm" height="{h:g}mm" viewBox="0 0 {w:g} {h:g}">',
        f'<g fill="none" stroke="black" stroke-width="{line_width_mm:g}"'
        ' stroke-linecap="round" stroke-linejoin="round">',
    ]
    for s in strokes_mm:
        if len(s) < 2:
            continue
        d = " L ".join(f"{x + w / 2:.3f} {h / 2 - y:.3f}" for x, y in s)
        out.append(f'<path d="M {d}"/>')
    out += ["</g>", "</svg>", ""]
    return "\n".join(out)


def save_svg(path, strokes_mm, paper_mm=pm.A4_LANDSCAPE_MM):
    Path(path).write_text(strokes_to_svg(strokes_mm, paper_mm), encoding="utf-8")


def main():
    paths.safe_console()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("strokes", type=Path, help="sketch_strokes JSON (mirobot-strokes 또는 GUI가 만든 파일)")
    ap.add_argument("-o", "--out", type=Path, help="SVG 경로 (기본: JSON과 같은 이름의 .svg)")
    args = ap.parse_args()

    try:
        doc, strokes = pm.load_strokes_json(args.strokes)
    except (OSError, ValueError, KeyError) as e:  # 파일 없음, JSON 오류, 다른 형식
        print(f"오류: {e}")
        return 2
    out = args.out or args.strokes.with_suffix(".svg")
    save_svg(out, strokes, paper_size_mm(doc.get("placement", {})))
    print(f"획 {len(strokes)}개 -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
