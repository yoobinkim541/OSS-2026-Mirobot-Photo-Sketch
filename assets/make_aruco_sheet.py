"""
종이 위치 표식(ArUco) 인쇄용 A4 PDF 만들기
==========================================
카메라가 종이 네 귀퉁이를 번호로 알아보도록 OpenCV ArUco 표식 0~3번을 A4 한 장에 실제 크기로 그립니다.
오려서 0=왼쪽 위, 1=오른쪽 위, 2=오른쪽 아래, 3=왼쪽 아래 귀퉁이 바깥 여백(펜이 닿지 않는 곳)에 붙입니다.

    python assets/make_aruco_sheet.py

출력: assets/aruco_markers_a4.pdf
인쇄할 때 "실제 크기(100%)"로 인쇄하고, 아래 50 mm 자로 크기를 확인합니다.
"""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
OUT = HERE / "aruco_markers_a4.pdf"
DICTIONARY = cv2.aruco.DICT_4X4_50       # 4×4 칸: 칸이 커서 비스듬한 카메라에서도 잘 읽힘
MARKER_MM = 30.0                         # 검은 테두리 바깥 한 변
QUIET_MM = 6.0                           # 표식 둘레 흰 여백 (인식에 필요, 오릴 때 남김)
DPI = 300
A4_MM = (210.0, 297.0)
PLACES = {0: "왼쪽 위", 1: "오른쪽 위", 2: "오른쪽 아래", 3: "왼쪽 아래"}
FONT_CANDIDATES = ("malgun.ttf", "malgunbd.ttf", "AppleGothic.ttf", "NanumGothic.ttf",
                   "NotoSansCJK-Regular.ttc")


def px(mm):
    return round(mm / 25.4 * DPI)


def font(size_mm, bold=False):
    names = ("malgunbd.ttf",) + FONT_CANDIDATES if bold else FONT_CANDIDATES
    for name in names:
        try:
            return ImageFont.truetype(name, px(size_mm))
        except OSError:
            continue
    return ImageFont.load_default()


def marker_image(marker_id):
    side = px(MARKER_MM)
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    return Image.fromarray(cv2.aruco.generateImageMarker(dictionary, marker_id, side, borderBits=1))


def make():
    page = Image.new("L", (px(A4_MM[0]), px(A4_MM[1])), 255)
    draw = ImageDraw.Draw(page)
    title, body, small = font(7, bold=True), font(4.2), font(3.4)
    draw.text((px(15), px(12)), "Mirobot Sketch 종이 위치 표식", font=title, fill=0)
    lines = ["1. '실제 크기(100%)'로 인쇄하고 아래 50 mm 자로 크기를 확인합니다.",
             "2. 점선을 따라 오립니다 (표식 둘레 흰 여백은 남깁니다).",
             "3. 번호에 맞는 귀퉁이 바깥 여백에 붙입니다. 펜이 닿는 곳에는 붙이지 않습니다.",
             "4. 네 장이 모두 카메라 화면에 보이게 합니다. 구기거나 접지 않습니다."]
    for i, line in enumerate(lines):
        draw.text((px(15), px(26 + i * 7)), line, font=body, fill=0)

    cell = MARKER_MM + 2 * QUIET_MM
    gap = 20.0
    left0 = (A4_MM[0] - 2 * cell - gap) / 2
    top0 = 66.0
    label_h = 12.0
    for k, marker_id in enumerate(PLACES):
        col, row = k % 2, k // 2
        x, y = left0 + col * (cell + gap), top0 + row * (cell + label_h + gap)
        page.paste(marker_image(marker_id), (px(x + QUIET_MM), px(y + QUIET_MM)))
        # 오리는 선: 흰 여백 바깥, 점선
        box = (px(x), px(y), px(x + cell), px(y + cell + label_h))
        dash = px(2)
        for x0 in range(box[0], box[2], dash * 2):
            draw.line((x0, box[1], min(x0 + dash, box[2]), box[1]), fill=140, width=2)
            draw.line((x0, box[3], min(x0 + dash, box[2]), box[3]), fill=140, width=2)
        for y0 in range(box[1], box[3], dash * 2):
            draw.line((box[0], y0, box[0], min(y0 + dash, box[3])), fill=140, width=2)
            draw.line((box[2], y0, box[2], min(y0 + dash, box[3])), fill=140, width=2)
        draw.text((px(x + QUIET_MM), px(y + cell + 1)), f"{marker_id}번 · {PLACES[marker_id]}",
                  font=body, fill=0)

    # 크기 확인용 50 mm 자
    ruler_y = top0 + 2 * (cell + label_h) + gap + 18
    x0, x1 = px(left0), px(left0 + 50)
    draw.line((x0, px(ruler_y), x1, px(ruler_y)), fill=0, width=4)
    for mm in range(0, 51, 10):
        tick = px(4 if mm % 50 == 0 else 2.5)
        draw.line((px(left0 + mm), px(ruler_y) - tick, px(left0 + mm), px(ruler_y)), fill=0, width=3)
    draw.text((x1 + px(4), px(ruler_y - 4)), "50 mm (인쇄 후 자로 재서 맞는지 확인)", font=body, fill=0)
    draw.text((px(15), px(A4_MM[1] - 14)),
              f"OpenCV ArUco DICT_4X4_50 · 표식 0~3 · 한 변 {MARKER_MM:g} mm · 흰 여백 {QUIET_MM:g} mm",
              font=small, fill=90)
    page.save(OUT, "PDF", resolution=DPI)
    print("saved:", OUT)
    return page


if __name__ == "__main__":
    make()
