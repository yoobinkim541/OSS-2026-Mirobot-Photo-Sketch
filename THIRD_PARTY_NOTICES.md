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

## 얼굴 점 478개 모델 (MediaPipe Face Landmarker를 ONNX로 바꾼 것)

- **파일:** 저장소·설치 파일에 들어 있지 않습니다. 앱이 처음 얼굴을 처리할 때 이 프로젝트의 GitHub Releases(`models-v1`의 `face_landmarks_478.onnx`)에서 내려받아 SHA256을 확인한 뒤 캐시 폴더(`out/cache/models/`, 설치판은 사용자 폴더의 `cache/models/`)에 둡니다.
- **출처:** Google, MediaPipe Face Landmarker, https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker. 배포 파일 https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task (SHA256 `64184e229b263107bc2b804c6625db1341ff2bb731874b0bcc2fe6544e0bc9ff`) 안의 `face_landmarks_detector.tflite`를 tf2onnx 1.x(`python -m tf2onnx.convert --tflite face_landmarks_detector.tflite --opset 13`)로 ONNX로 바꿨습니다. 같은 입력에서 원본 TFLite와 출력 차이는 최대 0.0002(256px 좌표 기준)입니다. 가중치는 바꾸지 않았습니다.
- **SHA256:** `2d7005f71d33a9e754ef6e22a70de016d9130fbdab385ff41bee426d8bab05ec`
- **라이선스:** Apache License 2.0 (MediaPipe 저장소 https://github.com/google-ai-edge/mediapipe 의 LICENSE). 위 변환은 원저작물의 형식 변환입니다.
- **용도:** 정면 얼굴의 눈(눈꺼풀·눈동자)·눈썹·다문 입(입술 사이 선)을 다시 그리고 끊긴 턱선을 잇습니다 (`mirobot_sketch/face_mesh.py`).

## PP-OCRv3 글자 검출 모델 (OpenCV Zoo)

- **파일:** 저장소·설치 파일에 들어 있지 않습니다. 앱이 처음 사진을 처리할 때 OpenCV Zoo의 Hugging Face 저장소(https://huggingface.co/opencv/text_detection_ppocr, `text_detection_en_ppocrv3_2023may.onnx`)에서 내려받아 SHA256을 확인한 뒤 캐시 폴더에 `text_detection_ppocrv3.onnx`로 둡니다.
- **출처:** PaddlePaddle PaddleOCR PP-OCRv3 글자 검출(DB), OpenCV Zoo 변환본.
- **SHA256:** `03f550c6b406fda8bf54bd8327815f6c7e2edd98cea02348c93d879254366587`
- **라이선스:** Apache License 2.0 (저장소 LICENSE: "Copyright (c) 2016 PaddlePaddle Authors").
- **용도:** 사진 속 글자(화보 제목·이름 자막·옷의 로고 글자) 영역의 선을 지웁니다 (`mirobot_sketch/texts.py`).

## 예시 사진 (README·문서 스크린샷)

- **파일:** `assets/sample/sample-portrait.jpg` (원본을 긴 변 1400px로 줄임)
- **출처:** Wikimedia Commons, "Sally Ride, First U.S. Woman in Space - GPN-2004-00019.jpg" (NASA), https://commons.wikimedia.org/wiki/File:Sally_Ride,_First_U.S._Woman_in_Space_-_GPN-2004-00019.jpg
- **라이선스:** 퍼블릭 도메인 (NASA 제작물, Wikimedia Commons 표기 "Public domain")
- **용도:** `assets/screenshots/`의 앱 화면 캡처와 `assets/pipeline/photo.jpg`에 쓴 예시 입력입니다. 앱이나 패키지에는 포함되지 않습니다.

## 예시 일러스트 (README 이미지 종류별 과정)

- **파일:** `assets/sample/sample-illustration.jpg` (투명 배경을 흰색으로 채우고 긴 변 1400px로 줄임)
- **출처:** Wikimedia Commons, "Neko Wikipe-tan.png", 그린 사람 Kasuga (https://en.wikipedia.org/wiki/User:Kasuga~enwiki), https://commons.wikimedia.org/wiki/File:Neko_Wikipe-tan.png
- **라이선스:** CC BY-SA 3.0 (https://creativecommons.org/licenses/by-sa/3.0/). 이 그림을 처리한 `assets/pipeline/illustration.jpg`도 같은 CC BY-SA 3.0으로 배포합니다.
- **용도:** README의 컬러 일러스트 처리 과정 그림과 데모 영상의 예시 입력입니다. 앱이나 패키지에는 포함되지 않습니다.

## 예시 만화 (README 이미지 종류별 과정)

- **파일:** `assets/sample/sample-manga.jpg` (6칸 중 아래 줄 왼쪽 칸을 잘라 냄)
- **출처:** Wikimedia Commons, "Tagosaku to Mokube no Tokyo Kenbutsu.jpg", 기타자와 라쿠텐(北澤楽天, 1876–1955) 『田吾作と杢兵衛の東京見物』(1902), https://commons.wikimedia.org/wiki/File:Tagosaku_to_Mokube_no_Tokyo_Kenbutsu.jpg
- **라이선스:** 퍼블릭 도메인 (Wikimedia Commons 표기 "Public domain", 일본에서 2005년 저작권 만료)
- **용도:** README의 흑백 만화 처리 과정 그림과 데모 영상의 예시 입력입니다. 앱이나 패키지에는 포함되지 않습니다.

## 실물 그림 원본 썸네일 (README 실물 그림 비교)

- **파일:** `assets/real-drawings/sources/` (긴 변 200px로 줄인 썸네일 4장)
- **출처:** `stage-original.jpg`는 인터넷에 공개된 무대 사진(원 촬영자 미확인), `portrait-original.jpg`는 인터넷에 공개된 가수 아이유 사진(원 촬영자 미확인), `anime-original.jpg`는 『SPY×FAMILY』 애니메이션 장면(© 遠藤達哉/集英社・SPY×FAMILY製作委員会), `professor-original.jpg`는 로봇신문 「'젊은 로봇 공학자' (27) ETRI 최성록 박사」(2019-08-18, http://www.irobotnews.com/news/articleView.html?idxno=17997) 기사 사진입니다.
- **라이선스:** 저작권은 각 권리자에게 있습니다. 이 저장소의 MIT 라이선스는 이 파일들에 적용되지 않으며, 권리자가 요청하면 내립니다.
- **용도:** README에서 로봇이 그린 실물 그림과 원본을 나란히 비교하는 데만 씁니다. 앱이나 패키지에는 포함되지 않습니다.
