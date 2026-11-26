"""README의 이미지 종류별 처리 과정 그림을 만든다 (assets/pipeline/*.jpg).

예시 이미지 세 장을 각 종류의 추천 설정으로 돌려, 그 종류에서 결과가 크게 바뀌는 단계만 골라 한 줄로 잇는다.
그림 아래에는 앱의 단계 띠와 같은 요약(경계 화소 수, 획 수, 예상 시간)을 적는다.

실행 (저장소 루트): python assets/make_pipeline_figures.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mirobot_sketch import draw_executor as de, stages  # noqa: E402
from mirobot_sketch.session import SketchSession  # noqa: E402

OUT = ROOT / "assets/pipeline"
FONT, FONT_B = "C:/Windows/Fonts/malgun.ttf", "C:/Windows/Fonts/malgunbd.ttf"
TILE_H, GAP, LABEL_H = 340, 46, 74

CASES = {
    "photo": ("sample-portrait.jpg", ["source", "edges", "face", "clutter", "simplify", "paper"]),
    "illustration": ("sample-illustration.jpg", ["source", "prep", "edges", "trace", "simplify", "paper"]),
    "manga": ("sample-manga.jpg", ["source", "prep", "edges", "trace", "simplify", "paper"]),
}


def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:                       # Windows 밖에서는 기본 글꼴
        return ImageFont.load_default()


def tile(bgr, title, note):
    img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    img = img.resize((round(img.width * TILE_H / img.height), TILE_H), Image.LANCZOS)
    t = Image.new("RGB", (img.width, TILE_H + LABEL_H), "white")
    t.paste(img, (0, 0))
    d = ImageDraw.Draw(t)
    d.rectangle((0, 0, img.width - 1, TILE_H - 1), outline=(220, 222, 230))
    d.text((img.width / 2, TILE_H + 22), title, font=font(FONT_B, 24), fill=(32, 38, 59), anchor="mm")
    size = 19
    while size > 12 and d.textlength(note, font=font(FONT, size)) > img.width - 12:   # 긴 요약은 칸 너비에 맞춰 줄임
        size -= 1
    d.text((img.width / 2, TILE_H + 54), note, font=font(FONT, size), fill=(98, 109, 131), anchor="mm")
    return t


def strip(tiles):
    w = sum(t.width for t in tiles) + GAP * (len(tiles) - 1)
    out = Image.new("RGB", (w, TILE_H + LABEL_H), "white")
    d = ImageDraw.Draw(out)
    x = 0
    for k, t in enumerate(tiles):
        out.paste(t, (x, 0))
        x += t.width
        if k < len(tiles) - 1:
            d.text((x + GAP / 2, TILE_H / 2), "→", font=font(FONT_B, 32), fill=(98, 84, 223), anchor="mm")
            x += GAP
    return out


def main():
    OUT.mkdir(exist_ok=True)
    for kind, (sample, ids) in CASES.items():
        s = SketchSession(de.load_config())
        s.apply_preset(kind)
        s.set_image(str(ROOT / "assets/sample" / sample))
        r = s.run()
        sums = s.stage_summaries()
        tiles = []
        for sid in ids:
            bgr = s.render(sid, max_px=900)
            note = sums[sid]
            if sid == "paper":
                note = f"획 {r['timing']['stroke_count']} · " + note
            tiles.append(tile(bgr, stages.stage_title(sid), note))
        img = strip(tiles)
        img.thumbnail((2400, 2400), Image.LANCZOS)
        path = OUT / f"{kind}.jpg"
        img.save(path, quality=88)
        print(path.relative_to(ROOT), img.size, f"획 {r['timing']['stroke_count']}",
              f"{r['timing']['total_s'] / 60:.1f}분")


if __name__ == "__main__":
    main()
