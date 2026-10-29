# 서드파티 고지

## YuNet 얼굴 검출 모델

- **파일:** `mirobot_sketch/data/face_detection_yunet_2023mar.onnx`
- **출처:** OpenCV Zoo, https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet
- **SHA256:** `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4`
- **라이선스:** MIT License

```
MIT License

Copyright (c) 2020 Shiqi Yu <shiqi.yu@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Informative Drawings 선화 모델

- **파일:** 저장소·설치 파일에 들어 있지 않습니다. 앱이 처음 실사 사진을 처리할 때 이 프로젝트의 GitHub Releases(`models-v1`의 `informative_drawings.onnx`)에서 내려받아 SHA256을 확인한 뒤 캐시 폴더(`out/cache/models/`, 설치판은 사용자 폴더의 `cache/models/`)에 둡니다.
- **출처:** Caroline Chan, Frédo Durand, Phillip Isola, "Learning to generate line drawings that convey geometry and semantics", CVPR 2022. 코드 https://github.com/carolineec/informative-drawings (MIT License). ONNX 변환본 https://huggingface.co/rocca/informative-drawings-line-art-onnx (`model.onnx`)
- **SHA256:** `1fef40b8f7126d827e30fbebccf95ae9b0b391795df926bf9366a821bad4f498`
- **용도:** 실사 사진을 선화로 바꾼 뒤 획을 땁니다 (`mirobot_sketch/lineart.py`).

## 예시 사진 (README·문서 스크린샷)

- **파일:** `assets/sample/sample-portrait.jpg` (원본을 긴 변 1400px로 줄임)
- **출처:** Wikimedia Commons, "Sally Ride, First U.S. Woman in Space - GPN-2004-00019.jpg" (NASA), https://commons.wikimedia.org/wiki/File:Sally_Ride,_First_U.S._Woman_in_Space_-_GPN-2004-00019.jpg
- **라이선스:** 퍼블릭 도메인 (NASA 제작물, Wikimedia Commons 표기 "Public domain")
- **용도:** `assets/screenshots/`의 앱 화면 캡처에 쓴 예시 입력입니다. 앱이나 패키지에는 포함되지 않습니다.
