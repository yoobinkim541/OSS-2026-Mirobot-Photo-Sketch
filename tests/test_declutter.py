"""실사 선화 잔선 정리: 피부 위 주름·짧고 흐린 잔선은 지우고 눈코입·긴 윤곽·대비 큰 글자는 남긴다."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirobot_sketch import declutter, stages  # noqa: E402
from mirobot_sketch import faces as fc  # noqa: E402

SKIN = (150, 180, 220)   # BGR


def line(x0, y0, x1, y1, n=12):
    return np.column_stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)])


class DeclutterTest(unittest.TestCase):
    def setUp(self):
        self.img = np.full((200, 200, 3), 255, np.uint8)
        cv2.ellipse(self.img, (100, 100), (50, 55), 0, 0, 360, SKIN, -1)
        cv2.line(self.img, (20, 180), (28, 180), (0, 0, 0), 2)          # 글자 획처럼 대비 큰 짧은 표시
        self.face = fc.Face(np.array([50.0, 50.0, 100.0, 100.0]),
                            np.array([[80, 85], [120, 85], [100, 105], [85, 125], [115, 125]], float), 1.0)
        self.strokes = {
            "wrinkle": line(70, 112, 80, 114),       # 볼 위 짧은 선 (눈코입 밖, 피부 위)
            "eye": line(74, 85, 86, 85),             # 눈
            "text": line(20, 180, 28, 180),          # 흰 바탕 위 검은 표시
            "faint": line(150, 180, 158, 180),       # 흰 바탕 위, 사진에 아무것도 없음
            "long": line(150, 20, 190, 20, 40),      # 종이 5 mm (1 mm = 8 px)
        }

    def verdicts(self):
        names = list(self.strokes)
        out = declutter.classify([self.strokes[k] for k in names], self.img, [self.face], px_per_mm=8.0)
        return dict(zip(names, out))

    def test_wrinkle_and_faint_short_lines_go_but_features_text_and_long_lines_stay(self):
        v = self.verdicts()
        self.assertEqual(v["wrinkle"], declutter.WRINKLE)
        self.assertEqual(v["faint"], declutter.CLUTTER)
        self.assertIsNone(v["eye"])
        self.assertIsNone(v["text"])
        self.assertIsNone(v["long"])

    def test_line_on_non_skin_inside_the_face_is_kept(self):
        # 얼굴 안이라도 머리카락처럼 피부색이 아닌 곳의 선은 주름이 아님
        cv2.rectangle(self.img, (60, 105), (90, 120), (40, 40, 40), -1)
        self.assertIsNone(self.verdicts()["wrinkle"])

    def test_lower_eyelid_and_lower_lip_just_outside_the_zones_are_kept(self):
        self.strokes = {"under_eye": line(74, 94, 86, 94),     # 눈 랜드마크(80, 85)에서 9px = 얼굴 폭의 9%
                        "lower_lip": line(92, 136, 108, 136)}  # 입 중심(100, 125)에서 11px 아래
        v = self.verdicts()
        self.assertIsNone(v["under_eye"])
        self.assertIsNone(v["lower_lip"])

    def test_tiny_speck_next_to_an_eye_goes_even_inside_the_feature_zone(self):
        # 1 mm = 8 px: 눈 옆 6px 토막(0.75mm)은 펜으로 점이 되므로 빼고, 12px 눈꺼풀(1.5mm)은 남김
        out = declutter.classify([line(77, 84, 83, 84, 6), line(74, 86, 86, 86, 12)], self.img, [self.face], px_per_mm=8.0)
        self.assertEqual(out, [declutter.SPECK, None])

    def test_small_closed_loop_outside_the_face_is_judged_like_any_short_line(self):
        t = np.radians(np.arange(0, 361, 30))
        loop = np.column_stack([170 + 3 * np.cos(t), 120 + 3 * np.sin(t)])   # 흰 바탕의 지름 6px 고리
        out = declutter.classify([loop], self.img, [self.face], px_per_mm=8.0)
        self.assertEqual(out, [declutter.CLUTTER])

    def test_short_sharp_marks_stay_even_when_most_strokes_are_sharp(self):
        # 글자가 많은 사진: 백분위 문턱이 올라가도 뚜렷한(대비 큰) 짧은 획은 남는다
        img = np.full((200, 200, 3), 255, np.uint8)
        marks = []
        for i in range(12):
            x = 10 + 15 * i
            cv2.line(img, (x, 100), (x + 8, 100), (0, 0, 0), 2)
            marks.append(line(x, 100, x + 8, 100))
        out = declutter.classify(marks, img, [], px_per_mm=8.0)
        self.assertEqual(out, [None] * 12)

    def test_without_faces_only_the_short_faint_rule_applies(self):
        names = list(self.strokes)
        out = dict(zip(names, declutter.classify([self.strokes[k] for k in names], self.img, [], px_per_mm=8.0)))
        self.assertNotIn(declutter.WRINKLE, out.values())     # 얼굴을 못 찾으면 피부 판단은 하지 않음
        self.assertEqual(out["faint"], declutter.CLUTTER)
        self.assertIsNone(out["text"])

    def test_clutter_stage_drops_clutter_only_on_the_lineart_path(self):
        st = [self.strokes["faint"], self.strokes["long"]]
        prev = {"strokes": st, "faces": [], "prep_color": self.img, "gray": self.img[:, :, 0], "region": None}
        params = {**stages.default_params(), "box_mm": 25}          # 200 px = 25 mm → 1 mm = 8 px
        canny = stages._run_clutter(prev, params)
        art = stages._run_clutter({**prev, "lineart_used": True}, params)
        off = stages._run_clutter({**prev, "lineart_used": True}, {**params, "declutter": False})
        self.assertEqual(len(canny["strokes"]), 2)
        self.assertEqual(len(off["strokes"]), 2)
        self.assertEqual(len(art["strokes"]), 1)
        self.assertEqual([why for _, why in art["discarded_clutter"]], [declutter.CLUTTER])   # "버린 선"에 이유와 함께
        stricter = stages._run_clutter({**prev, "lineart_used": True}, {**params, "clutter_mm": 6.0})
        self.assertEqual(len(stricter["strokes"]), 0)                # 기준을 6 mm로 올리면 5 mm 선도 지움

    def test_budget_drops_least_important_lines_outside_the_face_first(self):
        img = np.full((400, 400, 3), 255, np.uint8)
        strokes = [line(220, 20 + 10 * i, 240 + 10 * i, 20 + 10 * i, 30) for i in range(6)]   # 얼굴 둘레 밖, 길이 20~70 px
        strokes.append(self.strokes["eye"])                                                    # 얼굴 안 (지킴)
        out = declutter.classify(strokes, img, [self.face], px_per_mm=1.0, rules=declutter.Rules(max_keep=4))
        self.assertIsNone(out[-1])                                        # 눈은 남음
        self.assertEqual(out[:3], [declutter.BUDGET] * 3)                 # 가장 짧은 세 획이 빠짐
        self.assertEqual(out[3:6], [None] * 3)

    def test_faint_lineart_hair_strands_stay_and_go_last_under_the_budget(self):
        img = np.full((200, 200, 3), 255, np.uint8)             # 사진 대비 없음: 머리 결이 흐린 경우
        hair = np.zeros((200, 200), bool)
        hair[170:196, 140:172] = True
        strand, speck = line(150, 180, 170, 180, 20), line(150, 190, 162, 190)   # 2.5 mm 결 / 1.5 mm 토막
        longs = [line(10, 10 + 5 * i, 60, 10 + 5 * i, 50) for i in range(2)]     # 얼굴 밖 긴 선 (예산 후보)
        no_hair = declutter.classify([strand, speck], img, [self.face], px_per_mm=8.0)
        self.assertEqual(no_hair, [declutter.CLUTTER, declutter.CLUTTER])
        out = declutter.classify([strand, speck] + longs, img, [self.face], px_per_mm=8.0,
                                 rules=declutter.Rules(max_keep=1), hair=hair)
        self.assertIsNone(out[0])                                  # 머리 결은 남고 상한에서도 지킴
        self.assertEqual(out[1], declutter.CLUTTER)                # 점 같은 토막은 머리 안이라도 뺌
        self.assertEqual(out[2:], [declutter.BUDGET] * 2)
        long_strand = line(150, 186, 171, 186, 22)                                # 머리 결끼리는 짧은 것부터
        out = declutter.classify([strand, long_strand], img, [self.face], px_per_mm=8.0,
                                 rules=declutter.Rules(max_keep=1), hair=hair)
        self.assertEqual(out, [declutter.BUDGET, None])

    def test_open_smile_becomes_outline_plus_one_teeth_line(self):
        # 입꼬리 (85,125)-(115,125): 웃는 입의 위·아래 경계 + 윗니 아래 끝(가로선) + 이 사이 경계(짧은 세로선) 3개
        t = np.linspace(0, 1, 20)
        upper = np.column_stack([85 + 30 * t, 125 + 2 * np.sin(np.pi * t)])
        lower = np.column_stack([85 + 30 * t, 125 + 10 * np.sin(np.pi * t)])
        edge = line(89, 130, 111, 130, 23)
        gaps = [line(x, 127, x, 130, 4) for x in (94, 100, 106)]
        jaw = line(40, 160, 160, 160, 60)
        out, replaced = declutter.tidy_mouth([upper, lower, edge, *gaps, jaw], [self.face], pen_px=1.0)
        self.assertEqual(len(replaced), 6)                         # 입 안팎 조각은 모두 바뀜, 턱선은 그대로
        self.assertTrue(any(q is jaw for q in out))
        self.assertEqual(len(out), 3)                              # 턱선 + 입 윤곽 + 이 줄 하나 (이 사이 경계는 없음)
        mouth, teeth = out[-2], out[-1]
        self.assertAlmostEqual(float(mouth[:, 0].min()), 85.0, delta=1.0)    # 입꼬리에서 입꼬리까지
        self.assertAlmostEqual(float(mouth[:, 0].max()), 115.0, delta=1.0)
        self.assertGreater(float(np.ptp(mouth[:, 1])), 6.0)        # 벌린 입: 위아래가 떨어진 닫힌 곡선
        self.assertGreater(float(teeth[:, 0].min()), 89.0)         # 이 줄은 입꼬리 쪽을 비움
        self.assertLess(float(teeth[:, 0].max()), 111.0)
        self.assertLess(float(np.abs(teeth[:, 1] - 130).max()), 1.5)

    def test_closed_mouth_stays_a_single_line_and_sparse_mouth_is_left_alone(self):
        lips = [line(85, 125, 100, 126, 8), line(100, 126, 115, 125, 8)]
        out, replaced = declutter.tidy_mouth(lips, [self.face], pen_px=2.0)
        self.assertEqual(len(replaced), 2)
        self.assertLess(float(np.ptp(out[-1][:, 1])), 3.0)                    # 한 줄 (닫힌 곡선 아님)
        dot = [line(99, 125, 101, 125, 3), line(99, 127, 101, 127, 3)]       # 칸의 60%를 못 채움
        self.assertEqual(declutter.tidy_mouth(dot, [self.face], pen_px=1.0), (dot, []))

    def test_turned_face_mouth_is_left_alone(self):
        # 코가 두 눈 가운데에서 눈 사이(40px)의 0.375배 비낌: 옆으로 돌린 얼굴은 입꼬리 랜드마크가 부정확해 손대지 않음
        turned = fc.Face(self.face.box.copy(), self.face.landmarks + np.array([[0, 0], [0, 0], [15, 0], [0, 0], [0, 0]]), 1.0)
        t = np.linspace(0, 1, 20)
        lips = [np.column_stack([85 + 30 * t, 125 + 2 * np.sin(np.pi * t)]),
                np.column_stack([85 + 30 * t, 125 + 8 * np.sin(np.pi * t)])]
        self.assertEqual(declutter.tidy_mouth(lips, [turned], pen_px=1.0), (lips, []))


if __name__ == "__main__":
    unittest.main()
