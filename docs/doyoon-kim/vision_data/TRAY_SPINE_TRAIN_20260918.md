# 트레이 책 인식 학습 자료 전달 — 2026-09-18 (로봇팔 D 김도윤 → 비전 B 윤재민)

| 항목 | 값 |
| --- | --- |
| 문제 | 9/17 시연에서 트레이 책이 **하나의 큰 박스로 묶여** 인식됨 |
| 확인 | 책 종류를 6종으로 바꿔도 **그대로** (같은 책 6권 / 6종 섞기 결과 거의 동일) → 원인은 책 종류가 아님 |
| 원인 | **학습 데이터에 트레이 장면이 없다.** 손목 카메라가 보는 트레이 화면에는 트레이의 흰 **빗살 칸막이**가 책처럼 늘어서 있어, 모델이 트레이 전체를 한 덩어리로 본다 |
| 한 것 | Isaac Sim 에서 **트레이에 책을 꽂은 그대로** 여러 각도로 촬영 + 자동 라벨 → YOLO 재학습 |
| 결과 | 같은 시연 화면에서 **6권이 각각 잡힌다** (신뢰도 0.86~0.94, 임계값 0.75) |

## 1. 전후 비교 (같은 장면, 손목 RealSense 640×640)

| | 기존 모델 `book_best.pt` | **새 모델 `book_tray_best.pt`** |
| --- | --- | --- |
| 임계값 0.75 에서 검출 | **0개** | **6개 (책 6권 전부)** |
| 임계값 0.25 에서 검출 | 6개인데 **폭이 화면의 63~83 %** (묶임) | — |
| 박스 폭 | 화면의 63~83 % | **화면의 4~10 %** (책 한 권 크기) |
| 신뢰도 | 최고 0.72 | 0.86 ~ 0.94 |

그림: `samples/before_old_model_conf0.25.jpg`, `samples/after_new_model_conf0.75.jpg`

## 2. 새 모델

| 항목 | 값 |
| --- | --- |
| 파일 | `weights/book_tray_best.pt` (YOLO11n **detect**, 5.5 MB), `book_tray_last.pt` |
| 클래스 | `0: book` 한 개 |
| 입력 | 640×640 |
| 검증 성능 (val 166장, 상자 3,571개) | **mAP50 0.926, mAP50-95 0.787, P 0.951, R 0.868** |
| 학습 | 60 epoch, batch 16, RTX 5080, 11분. 좌우 반전 끔(`fliplr=0`), 색 증강 낮춤 |

기존 모델은 segment, 새 모델은 detect 입니다. `vision_manager` 는 박스만 쓰므로 그대로 바꿔 끼울 수 있습니다.
바꾸는 방법: `model_path` 파라미터를 이 파일로 지정.

## 3. 학습 데이터

| 항목 | 값 |
| --- | --- |
| 폴더 | `dataset/` (`train/` 942장, `val/` 166장, `data.yaml`) |
| 상자 | 25,644개 (이미지당 평균 23개) |
| 만든 방법 | Isaac Sim 에서 레벨 v5 + 트레이 + 책 6종을 그대로 두고, 카메라를 **책등이 보이는 각도**(고도 35~88°, 거리 0.14~0.75 m)로 옮겨 가며 촬영 |
| 라벨 | Isaac 의 `bounding_box_2d_tight` 주석기 — **가려진 부분은 자동으로 빠지고 책마다 상자 1개**. 사람 라벨 없음 |
| 변화 요소 | 트레이 책 3~6권, 어느 칸에 어떤 책이 오는지, 조명 방향·세기, 근접 촬영 40 % |

### 라벨 규칙 (중요)

- **라벨을 붙인 것**: 트레이 책 6종 + 레벨 서가의 책들
- **일부러 안 붙인 것**: **트레이의 흰 빗살 칸막이** — 지금 이것을 책으로 묶어 보는 것이 문제라서, "책이 아니다" 를 배워야 합니다

## 4. 데이터를 더 만들거나 다시 만들려면 (GPU PC)

```bash
# 촬영 (약 90초에 550장)
~/arm/isaac/run_capture.sh --views 70 --rounds 8 --seed 0 --out ~/spine_ds
# 학습
~/venv_yolo/bin/yolo detect train data=~/spine_yolo/data.yaml model=yolo11n.pt \
    epochs=60 imgsz=640 batch=16 device=0 fliplr=0.0 hsv_h=0.01
```

- 스크립트 원본: `scripts/capture_spines.py`, `scripts/run_capture.sh` (저장소 `shelving_manipulation/isaac_sim/isaac/`)
- 책 종류를 바꾸려면 `capture_spines.py` 의 `MIXED_BOOKS` 목록 수정 (레벨 `/World/books` prim 이름, 에셋 112종)
- GPU PC 학습 환경: `~/venv_yolo` (ultralytics 8.4.155, torch 2.14+cu130)

## 5. 아직 남은 것 · 확인 부탁

1. **실사 검증은 못 했습니다.** 전부 시뮬 영상입니다. 실물 카메라로 쓸 때 성능은 별도 확인이 필요합니다
2. **트레이 칸막이가 비어 있을 때**(책 0권) 오검출은 아직 안 봤습니다
3. `vision_manager` 에서 이 모델로 바꾼 뒤 **임계값 0.75 그대로** 쓰면 됩니다 (그 값에서 6/6 나옵니다)
4. 깊이 값으로 좌표를 뽑는 부분은 그대로입니다. 박스가 작아졌으니 **박스 중앙 깊이 중앙값**이 더 정확해질 것으로 봅니다

## 6. 폴더 구성

```
260918_train/
  README.md                     이 문서
  book_tray_train_260918.tar.gz 아래 내용 전체 압축
    weights/   book_tray_best.pt, book_tray_last.pt
    dataset/   train(942), val(166), data.yaml
    scripts/   capture_spines.py, run_capture.sh
    samples/   전후 비교 그림, 검증 이미지
    metrics/   results.csv, PR·F1 곡선, 혼동행렬, 촬영 통계
```
