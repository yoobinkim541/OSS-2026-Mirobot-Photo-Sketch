# 2026-11-13 — 선행 연구 조사와 품질 개선 방향

사용자 요청: "더 품질을 올릴 방법이나 선행연구, 자료, 논문을 리서치해서 개선방향을 찾아줘."

## 지금 상태와 남은 약점

지금 파이프라인은 다음 순서로 그린다.

1. 사진을 Informative Drawings 선화로 바꾸고, 얼굴은 원본 해상도로 한 번 더 그린다.
2. 잔선 정리 단계에서 주름·잔선·얼굴 속 점을 지운다. 정면 얼굴의 입은 윤곽 하나와 이 줄로 다시 그린다.
3. 머리카락은 결 선(구조 텐서 + Jobard–Lefer 흐름선)으로 그린다. 빗금은 기본으로 끈다.
4. 자동 세팅이 45분 안에서 닮음 점수가 가장 높은 잔선 강도를 고른다.

사용자 사진 2장과 시험 사진 4장을 실제 펜 굵기로 보고 남은 약점을 추렸다.

| 약점 | 원인 (확인된 것 / 추정) |
|---|---|
| 작은 얼굴(사진 2, 원본 얼굴 약 57px)은 웃는 입이 일자로 나온다 | 원본 해상도 한계. 얼굴 세밀 처리가 조각을 키워도 정보가 없다 |
| 눈이 짧은 선 여러 개로 정밀하게 그려진다 | YuNet 랜드마크가 5점(눈 중심·코·입꼬리)뿐이라 눈 모양을 알 수 없다. 짧은 선을 더 지우면 눈이 통째로 사라진다(초상 22획 사라짐) |
| 입 정리는 정면 얼굴에만 쓴다 | 입꼬리 2점으로 곡선을 맞추므로 옆얼굴·기운 얼굴에서는 입술과 엇갈린다 |
| 머리 결이 정수리에서 흘러내리지 않고 머리 윤곽을 따라 돈다 | 결 방향을 밝기 기울기로만 추정한다(추정) |
| 비니·소매 같은 머리 근처 어두운 것에도 결 선이 생긴다 | 머리 영역을 "얼굴 위·옆의 어두운 덩어리"로 정한다 |
| 닮음 점수가 그림의 어색함을 잡지 못한다 | 3단계 빗금, 이 덩어리처럼 눈에 띄는 손해에도 점수가 오르거나 비슷했다. 손으로 만든 윤곽 재현율이라 사람의 판단과 다르다 |
| 실물에서 검증되지 않았다 | 모든 평가가 화면 미리보기 기준이다 |

## 선행 연구

### 인물 선화 생성 (학습 기반)

- **Informative Drawings** (Chan, Durand, Isola, CVPR 2022): 지금 쓰는 선화 모델이다. 짝 없는 데이터로 학습한다. 선화에서 깊이를 다시 추정하는 기하 손실, CLIP 의미 손실, 외관 손실로 형태와 의미를 전하는 선을 그린다. [논문](https://openaccess.thecvf.com/content/CVPR2022/html/Chan_Learning_To_Generate_Line_Drawings_That_Convey_Geometry_and_Semantics_CVPR_2022_paper.html)
- **APDrawingGAN** (Yi 외, CVPR 2019) / **APDrawingGAN++** (TPAMI 2020): 얼굴 부위마다 전용 지역 네트워크(눈·코·입 등 6개)를 두고 전역 네트워크와 합친다. 화가의 선이 사진 경계와 정확히 맞지 않는 것을 허용하는 거리 변환(DT) 손실을 쓴다. 짝 데이터가 필요하고, 코드가 Python 2.7·PyTorch 0.4 기반이다. [코드](https://github.com/yiranran/APDrawingGAN), [++](https://github.com/yiranran/APDrawingGAN2)
- **Unpaired Portrait Drawing** (Yi 외, CVPR 2020): 짝 없는 데이터로 학습한다. 엄격한 순환 일관성이 눈코입 같은 중요한 부분을 빼먹게 만든다는 점을 보이고, 비대칭 순환으로 고쳤다. 중요 얼굴 부위에 지역 판별기를 두고, 여러 화풍을 한 네트워크로 낸다. [코드](https://github.com/yiranran/Unpaired-Portrait-Drawing)
- **QMUPD** (Yi 외, TPAMI 2022/2023): 위 연구의 확장이다. 짝 없이 인물 선화의 품질을 판정하는 품질 지표 모델을 학습해 생성기 학습에 쓴다. 우리의 손으로 만든 닮음 점수를 대신할 후보다. [코드](https://github.com/yiranran/QMUPD), [arXiv](https://arxiv.org/abs/2202.03678)

### 로봇 인물 드로잉

- **AiSketcher** (Gao 외, IROS 2020, "Making Robots Draw A Vivid Portrait In Two Minutes"): 우리와 가장 비슷한 시스템이다. 3자유도 로봇팔로 평균 약 2분에 그린다.
  - 연속적이고 매끄러운 획을 내는 자기 일관성 손실을 쓴다.
  - 중요하지 않은 곳의 획 수를 줄이는 부위별 희소성 제약을 쓴다.
  - 얼굴 부위 분할 마스크로 부위를 나누고, 부위별 지역 스케치와 배경·세부 전후처리를 둔다.
  - [arXiv](https://arxiv.org/abs/2005.05526), [코드](https://github.com/fei-aiart/AiSketcher)
- **Paul the robot** (Tresset & Leymarie, Computers & Graphics 2013): 카메라를 "눈"으로 써서 그리는 중인 그림을 다시 찍고, 어디를 더 칠할지 정하는 물리적 시각 피드백을 다룬다. 작품 하나가 V&A 미술관에 소장됐다. [저장소](https://research.gold.ac.uk/id/eprint/9255/)

### 흐름 기반 선 그리기 (학습 없음)

- **Coherent Line Drawing** (Kang, Lee, Chui, NPAR 2007) / **Flow-based Image Abstraction** (TVCG 2009): 경계 접선 흐름(ETF)을 만든다. 기울기에 수직인 접선장을 비선형 필터(거리·기울기 크기·방향 가중)로 2~3번 다듬어, 약한 곳도 주변 강한 경계의 방향을 따르게 한다. 그 흐름을 따라 FDoG로 매끄러운 선을 찾는다. [PDF](https://cg.postech.ac.kr/papers/kang_npar07_hi.pdf), [ETF 구현](https://github.com/naotokimura/EdgeTangentFlow)
- **XDoG** (Winnemöller, Kyprianidis, Olsen, Computers & Graphics 2012): DoG에 날카롭게 하는 가중과 부드러운 문턱을 더해, 연필·잉크·명암 같은 여러 화풍을 한 연산자로 낸다. 흐름을 따르는 변형도 있다. [PDF](https://www.cs.princeton.edu/courses/archive/spring19/cos426/papers/Winnemoeller12.pdf)
- **Jobard & Lefer** (1997), **Turk & Banks** (SIGGRAPH 1996): 고르게 띄운 흐름선을 놓는다. 머리 결 선에 이미 쓰고 있다.

### 획 추상화·정리

- **CLIPasso** (Vinker 외, SIGGRAPH 2022): 베지어 곡선 N개를 미분 가능한 래스터라이저(diffvg)로 그려, CLIP 중간·최종 층 손실로 최적화한다. 획 수로 추상화 정도를 정한다. 학습이 필요 없다. 물체 위주이고, 인물의 개인 특징은 약할 수 있다(검증 안 함). [코드](https://github.com/yael-vinker/CLIPasso)
- **Learning to Simplify** (Simo-Serra 외, SIGGRAPH 2016) / Mastering Sketching (TOG 2018): 거친 스케치를 깨끗한 선으로 바꾸는 완전 합성곱 네트워크다. 사전학습 모델이 있지만 Torch7 형식이고, 라이선스는 학술·비영리 기관의 비상업 연구용만 허용한다(아래 표). [코드](https://github.com/bobbens/sketch_simplification)

### 얼굴 이해 도구

- **MediaPipe Face Landmarker**: 얼굴마다 478개 3D 점을 낸다. 눈·눈썹·입술 안팎·얼굴 윤곽·홍채(468~477)를 포함하고, 연결 목록(`FACE_LANDMARKS_LIPS`, `_LEFT_EYE`, `_LEFT_IRIS` 등)이 있다. 52개 표정 점수(눈 깜빡임 등)도 준다. Python 3.12용 1.2.3이 PyPI에 있다. [가이드](https://developers.google.com/edge/mediapipe/solutions/vision/face_landmarker/python)
- **BiSeNet 얼굴 분할** (CelebAMask-HQ 19부위: 피부·눈썹·눈·안경·코·입술·머리카락·모자·옷 등): ONNX로 바로 쓸 수 있는 ResNet18 가중치가 있다(코드 MIT, 가중치를 학습한 CelebAMask-HQ 데이터셋은 비상업 연구용). 이 프로젝트는 커밋 c8f79d2(10월 29일)에서 `face_parsing_resnet18.onnx`(53MB)와 `parsing.py`·`portrait.py`·`hair.py`로 부위별 재구성을 넣었다가, 같은 날 af1a838에서 모두 지웠다. af1a838 메시지에는 무엇을 지웠는지만 있고 지운 이유는 없다. 이번에는 머리·피부·모자 마스크 용도로만 쓰는 것을 검토한다. [yakhyo/face-parsing](https://github.com/yakhyo/face-parsing)
- **얼굴 복원** GFPGAN(CVPR 2021) / CodeFormer(NeurIPS 2022): 작고 흐린 얼굴을 고해상도로 복원한다. 사진 2처럼 얼굴이 작은 경우의 후보다. 복원은 없는 디테일을 만들어 내므로 닮음이 달라질 위험이 있다. 라이선스는 아래 표에 적었다. [GFPGAN](https://github.com/TencentARC/GFPGAN), [CodeFormer](https://github.com/sczhou/CodeFormer)

### 펜 플로터 경로

- **vpype** `linemerge`(끝점이 가까운 선 잇기), `linesort --two-opt`(펜 들고 이동하는 거리 최소화), `reloop`, `linesimplify`: 플로터 그림의 표준 최적화 도구다. 우리는 탐욕적 최근접 순서만 쓴다. [문서](https://vpype.readthedocs.io/en/latest/reference.html)

### 라이선스와 배포 가능성

이 앱은 `onnxruntime`만으로 모델을 돌린다. 모델은 GitHub Releases에서 받아 SHA256을 확인하고, 출처는 `THIRD_PARTY_NOTICES.md`에 적는다. 그래서 실제로 쓸 수 있는지는 논문 성능보다 아래 조건이 먼저 정한다. 라이선스는 2026-11-13에 GitHub API의 `license` 필드로 확인했고, API가 판정하지 못한(NOASSERTION) GFPGAN·CodeFormer·Learning to Simplify는 LICENSE 원문을 읽었다. "없음"은 저장소에 라이선스 파일이 없다는 뜻이고, 이 경우 기본적으로 재배포가 허용되지 않는다.

| 후보 | 코드 라이선스 | 가중치·데이터 | 런타임 적합성 |
|---|---|---|---|
| Informative Drawings (사용 중) | MIT | ONNX 변환본 사용 중 | 그대로 |
| MediaPipe Face Landmarker | Apache-2.0 | 모델 번들 포함 | pip 휠(3.12)이 있지만 의존성이 하나 늘고 설치판 빌드를 바꿔야 함 |
| BiSeNet 얼굴 분할 (yakhyo) | MIT | CelebAMask-HQ 비상업 연구용 | ONNX 제공, 그대로 |
| vpype (경로 정리) | MIT | 해당 없음 | 순수 Python, 그대로 |
| APDrawingGAN | Apache-2.0 | 확인 안 함 | Python 2.7·PyTorch 0.4, 이식과 ONNX 변환 필요 |
| APDrawingGAN++ / Unpaired Portrait / QMUPD | 없음 | 확인 안 함 | PyTorch, ONNX 변환 필요. 라이선스가 없어 재배포 불가 |
| AiSketcher | 없음 | 확인 안 함 | 재배포 불가 |
| GFPGAN | Apache-2.0. 단 StyleGAN2 유래 코드는 NVIDIA Source Code License(비상업: 연구·평가 용도만), DFDNet 유래 코드는 CC BY-NC-SA 4.0 (LICENSE 원문 기준) | LICENSE에 가중치 언급 없음, 확인 필요 | PyTorch, ONNX 변환 필요 |
| CodeFormer | S-Lab License 1.0 (비상업 목적의 재배포·사용만 허용, LICENSE 원문 기준) | 같은 조건으로 봄(확인 필요) | PyTorch, 재배포 제약 |
| Learning to Simplify | 자체 계약: "Academic or non-profit organization noncommercial research use only" (LICENSE 원문 기준) | README: 제공 모델은 비상업 Creative Commons (버전 명시 없음) | Torch7, 변환 필요. 배포 불가 |

결론: 바로 들일 수 있는 것은 MediaPipe·vpype·BiSeNet 코드다(BiSeNet 가중치는 데이터 라이선스 확인 필요). 인물 전용 선화 모델(APDrawingGAN++·QMUPD)은 라이선스가 없어 비교 실험까지만 하고 배포에는 넣지 않는다.

## 개선 방향 (우선순위)

평가는 계속 사진 6장을 실제 펜 굵기 미리보기로 보고, 사용자 판단을 받는다. 점수만으로 채택하지 않는다(이번 세션에서 점수와 그림이 여러 번 엇갈렸다).

### 1순위: 의존성 없이 바로 할 수 있는 것

1. **머리 결을 선화의 실제 머리선으로 살리기:** 선화 모델은 이미 머리 결을 따고 있었다. 잔선 정리와 선 개수 상한이 그 선을 지웠을 뿐이다. 머리 영역 안의 선화 획을 잔선 판정과 상한에서 지키고(얼굴 둘레·외곽선처럼), 결 선은 그 사이 빈 곳만 채운다. 합성 선보다 실제 머리 모양에 가깝다(추정).
2. **결 방향을 ETF로 다듬기 (Kang 2007):** 지금은 구조 텐서를 흐리기만 한다. ETF의 비선형 다듬기(기울기 크기·방향 가중, 2~3회)를 쓰면 약한 곳이 주변 강한 머리 경계의 방향을 따른다. 머리 윤곽을 도는 대신 머리카락 방향으로 흐르는 데 도움이 될 것으로 본다(추정). 코드 수십 줄 규모다.
3. **경로 최적화 (vpype 방식):** 2-opt 순서 개선과 끝점 잇기 허용치 조정으로 펜 이동·올림을 줄인다. 같은 45분에 더 많은 선을 남길 수 있다. 펜 5mm 짧은 올림(계산상 16.7분 절약)은 실물 확인 뒤에 넣는다.
4. **자동 세팅에서 사람 평가 받기:** 후보 상위 2~3개를 실제 펜 굵기로 나란히 보여 주고 사용자가 고르게 한다. 그 선택을 모으면 점수 가중치를 맞출 근거가 된다.

### 2순위: 작은 모델을 더하는 것

5. **MediaPipe 랜드마크로 눈·입 다시 그리기:** 눈꺼풀·홍채·입술 안팎 윤곽을 알면, 눈은 "윗눈꺼풀 곡선 + 눈동자", 입은 "입술 윤곽 + 이 줄"처럼 정해진 방식으로 깔끔하게 그릴 수 있다. 정면이 아닌 얼굴에도 적용된다. 입꼬리 랜드마크가 부정확한 사진 2도 나아질 것으로 본다(추정). Apache-2.0이라 배포에 문제가 없다. 라이브러리 수십 MB, 모델 약 4MB가 추가되고 설치판 빌드를 바꿔야 한다.
6. **얼굴 분할 마스크로 머리·피부·모자 구분:** 머리 영역을 어두운 덩어리 대신 "머리카락" 부위로 잡으면 비니·소매에 결 선이 생기지 않는다. 피부 마스크로 빗금 금지·주름 판정도 정확해진다. CelebAMask-HQ 비상업 라이선스를 확인해야 한다.
7. **작은 얼굴 복원:** 원본 얼굴이 약 100px보다 작을 때만 얼굴 조각을 복원한 뒤 선화를 돌린다. 두 후보 모두 비상업 조건이 걸린 코드가 들어 있어(위 표) 배포판에 넣기 어렵고, 닮음이 달라질 위험도 있다. 고해상도 원본을 받는 것이 먼저다.

### 3순위: 연구 규모

8. **인물 전용 선화 모델 비교:** 얼굴 조각에만 QMUPD나 APDrawingGAN++를 돌려 Informative Drawings와 비교한다. 얼굴 부위별 지역 네트워크라 눈코입이 더 정리된 선으로 나올 가능성이 있다. 두 저장소 모두 라이선스가 없어 로컬 비교 실험만 하고, 결과가 좋으면 저자에게 허락을 받거나 같은 구조를 직접 학습해야 한다.
9. **학습된 품질 지표로 점수 바꾸기:** QMUPD의 품질 지표 모델(라이선스 없음, 로컬 실험만), 또는 렌더링한 그림과 선화 사이의 지각 거리(LPIPS·CLIP)를 닮음 점수에 넣는다.
10. **획 수 최적화 (CLIPasso·AiSketcher 방식):** 시간 예산 안에서 어떤 획을 남길지를 지각 손실로 직접 고른다. AiSketcher의 "덜 중요한 부위의 획 수 줄이기"와 같은 목표다. diffvg 설치가 어렵다.
11. **카메라 폐루프 (Paul):** 이미 있는 BRIO 선 확인으로 실제 종이 결과를 찍어 빠진 선을 다시 그리거나 다음 그림의 설정을 고친다.

## 다음에 할 일 제안

1번(선화 머리선 보호)과 2번(ETF)을 먼저 시도해 사진 6장으로 비교하고, 그 결과를 보고 5번(MediaPipe)을 진행할지 정한다. 어느 것이든 실물로 한 장 그려 보는 것이 화면 평가보다 우선이다.

**진행(2026-11-14):** 1번과 2번을 넣었다. 결과와 시험한 값은 [2026-11-13-photo-lineart-detail.md](2026-11-13-photo-lineart-detail.md)의 "머리 결: 선화의 실제 머리선 지키기와 ETF 결 방향" 절에 있다. 사진 1의 결이 머리 윤곽을 도는 대신 정수리에서 흘러내리게 됐다. 초상의 곱슬머리는 선화 조각으로 남아 지저분해 보이는지 사용자 판단이 필요하다.

**진행(2026-11-15):** 5번(MediaPipe)을 넣었다. 정면 얼굴의 눈은 눈꺼풀·홍채·눈동자 점으로, 다문 입은 입술 사이 선으로 다시 그린다. 결과는 같은 로그의 "얼굴 점 478개(MediaPipe)로 눈·다문 입 다시 그리기" 절에 있다.
