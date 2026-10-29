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

- **파일:** `mirobot_sketch/data/informative_drawings.onnx`
- **출처:** Caroline Chan, Frédo Durand, Phillip Isola, "Learning to generate line drawings that convey geometry and semantics", CVPR 2022. 코드 https://github.com/carolineec/informative-drawings (MIT License). ONNX 변환본 https://huggingface.co/rocca/informative-drawings-line-art-onnx (`model.onnx`)
- **SHA256:** `1fef40b8f7126d827e30fbebccf95ae9b0b391795df926bf9366a821bad4f498`
- **용도:** 실사 사진을 선화로 바꾼 뒤 획을 땁니다 (`mirobot_sketch/lineart.py`).

## 얼굴 부위 분할 모델 (BiSeNet ResNet18)

- **파일:** `mirobot_sketch/data/face_parsing_resnet18.onnx`
- **출처:** https://github.com/yakhyo/face-parsing (MIT License), releases `weights/resnet18.onnx`
- **SHA256:** `0d9bd318e46987c3bdbfacae9e2c0f461cae1c6ac6ea6d43bbe541a91727e33f`
- **학습 데이터:** CelebAMask-HQ. 이 데이터셋은 비상업적 연구·교육 용도로만 쓸 수 있습니다(https://github.com/switchablenorms/CelebAMask-HQ). 가중치를 상업적으로 쓰려면 이 조건을 따로 확인해야 합니다.
- **용도:** 모자·머리카락·눈·눈썹·귀·입술·옷을 나눠 부위마다 다르게 그립니다 (`mirobot_sketch/parsing.py`, `mirobot_sketch/portrait.py`).

## 예시 사진 (README·문서 스크린샷)

- **파일:** `assets/sample/sample-portrait.jpg` (원본을 긴 변 1400px로 줄임)
- **출처:** Wikimedia Commons, "Sally Ride, First U.S. Woman in Space - GPN-2004-00019.jpg" (NASA), https://commons.wikimedia.org/wiki/File:Sally_Ride,_First_U.S._Woman_in_Space_-_GPN-2004-00019.jpg
- **라이선스:** 퍼블릭 도메인 (NASA 제작물, Wikimedia Commons 표기 "Public domain")
- **용도:** `assets/screenshots/`의 앱 화면 캡처에 쓴 예시 입력입니다. 앱이나 패키지에는 포함되지 않습니다.
