"""
상세도·이미지 종류 프리셋 (CLI와 GUI가 함께 사용)
=================================================
값은 2026-09-24~25 입력 이미지 6장(컬러 일러스트 2, 흑백 만화 1, 실사 3) 비교에서
정했습니다. 근거: LOG/2026-09-24-image-set-simulation.md
"""

# 상세도: (canny_low, canny_high, min_length_px, epsilon_px) — 800px 정규화 기준
DETAIL_PRESETS = {
    "low": (80, 200, 40, 3.0),
    "medium": (50, 150, 15, 2.0),
    "high": (30, 100, 8, 1.0),
}

# 이미지 종류별 추천
IMAGE_TYPES = {
    "photo": {
        "label": "실사 사진",
        "detail": "high", "rembg": True, "median": 0, "lineart": True,
        "why": "실사를 선화로 바꾼 뒤 얼굴·모자·실루엣만 따고, 배경은 rembg로 지움",
    },
    "illustration": {
        "label": "컬러 일러스트",
        "detail": "high", "rembg": False, "median": 0,
        "why": "선이 뚜렷해 배경 제거 없이도 깔끔함 (글씨가 있으면 rembg를 켜 볼 것)",
    },
    "manga": {
        "label": "흑백 만화",
        "detail": "high", "rembg": False, "median": 11,
        "why": "스크린톤(망점)을 미디언 블러로 지움 (800px 기준 11px)",
    },
}

# 선 정리 기본값 (px, 800px 이미지를 100mm로 그릴 때 1px ≈ 0.125mm)
DEFAULT_DEDUPE_PX = 4        # 펜 굵기(약 0.5mm)보다 가까운 이중선은 하나만
DEFAULT_MERGE_JOIN_PX = 4.0  # 끝점이 이 거리 안에서 만나면 펜을 떼지 않고 이어 그림
