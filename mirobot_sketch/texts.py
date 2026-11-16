"""사진 속 글자 영역 찾기 (화보 제목·이름 자막·로고 글자를 펜 선에서 빼려고).

PP-OCRv3 글자 검출 모델(DB, OpenCV Zoo, Apache-2.0, 2.4 MB)을 OpenCV `TextDetectionModel_DB`로 돌립니다.
처음 쓸 때 OpenCV Zoo의 Hugging Face 저장소에서 내려받아 SHA256을 확인합니다. 못 받으면 글자를 지우지 않습니다.
0.5mm 펜으로는 글자 획이 뭉개져 읽히지 않고(화보의 이름 자막), 한 글자마다 펜을 여러 번 올려 시간도 많이 듭니다.
"""

import cv2
import numpy as np

from . import models

MODEL_NAME = "text_detection_ppocrv3.onnx"
MODEL_URL = ("https://huggingface.co/opencv/text_detection_ppocr/resolve/main/"
             "text_detection_en_ppocrv3_2023may.onnx")
MODEL_SHA256 = "03f550c6b406fda8bf54bd8327815f6c7e2edd98cea02348c93d879254366587"
INPUT_SIDE = 736          # 모델 입력 (32의 배수, OpenCV Zoo 예제 값)
MIN_CONFIDENCE = 0.8

_model = None


class TextUnavailable(RuntimeError):
    """글자 검출 모델을 쓸 수 없음 (내려받기 실패)."""


def _load():
    global _model
    if _model is None:
        try:
            path = models.fetch(MODEL_NAME, MODEL_URL, MODEL_SHA256, "글자 검출 모델")
        except models.ModelUnavailable as e:
            raise TextUnavailable(f"{e} 글자를 지우지 않았습니다.") from None
        m = cv2.dnn.TextDetectionModel_DB(str(path))
        m.setBinaryThreshold(0.3).setPolygonThreshold(0.5).setMaxCandidates(200).setUnclipRatio(2.0)
        m.setInputParams(1.0 / 255.0, (INPUT_SIDE, INPUT_SIDE), (122.67891434, 116.66876762, 104.00698793))
        _model = m
    return _model


def detect(bgr):
    """글자 영역 사각형 목록 (각 4×2, bgr 좌표). 모델이 없으면 TextUnavailable."""
    boxes, conf = _load().detect(bgr)
    return [np.asarray(b, np.float64).reshape(4, 2) for b, c in zip(boxes, conf) if c >= MIN_CONFIDENCE]
