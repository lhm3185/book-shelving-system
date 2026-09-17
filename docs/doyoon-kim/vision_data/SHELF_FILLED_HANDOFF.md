# 책이 꽂힌 서가(filled) 학습데이터 — 비전 담당자 인계

> 9/17 오전에 받은 **개방형·폐쇄형 서가 인계(`SHELF_VISION_HANDOFF.md`)에 세 번째 서가를 추가**하는 자료다.
> 촬영 파이프라인·라벨 규약·카메라 설정은 그 문서와 **완전히 같다.** 여기엔 달라진 점만 적는다.
> **학습은 하지 않았다** (담당자가 직접 학습). 학습에 바로 쓸 수 있게 준비만 했다.

| 항목 | 값 |
| --- | --- |
| 작성일 | 2026-09-17 |
| 추가 데이터 | `dataset_shelf/shelf_filled/` **600장** (컬러·깊이·마스크·6D 자세·bbox) |
| 만든 환경 | Blender 5.2.0 LTS, EEVEE, GPU PC(RTX 5080) |
| 검증 | bbox 600장 전부 마스크 기준, 3D 투영 오차 0.00px, 가시성 미달 0장, **무작위 16장 눈으로 확인** (`docs/sample_filled_mask.jpg`), 3클래스 내보내기 실제 실행 확인 |

---

## 1. 무엇이 추가됐나

| 에셋 | 폴더 | 크기(m) | 특징 |
| --- | --- | --- | --- |
| 개방형 (9/17 오전 인계) | `shelf_open/` | 1.41 × 0.31 × 1.80 | 갈색 목재, 비어 있음, 앞뒤 뚫림 |
| 폐쇄형 (9/17 오전 인계) | `shelf_closed/` | 1.35 × 0.37 × 1.80 | 검은 목재, 비어 있음, 뒷판 있음 |
| **책 꽂힌 서가 (신규)** | **`shelf_filled/`** | **1.41 × 0.31 × 1.80** | **개방형과 같은 틀 + 짙은 갈색 금속성 목재 + 칸 안이 책 텍스처로 채워짐** |

- **배경용 소품이다.** 로봇이 책을 꺼내거나 꽂지 않는다. 칸 안은 책 이미지를 붙인 상자다
- 원점은 바닥 중심, 높이 1.8 m — 앞의 두 서가와 같다
- 앞면·뒷면 모두 책 텍스처가 보인다 (글자 방향 정상, Isaac 에서도 확인)

### 개방형 서가와 헷갈릴 수 있는 점

틀 모양과 크기가 **개방형과 똑같다.** 둘을 가르는 건 **칸 안이 책으로 차 있는지뿐**이다.
옆면·뒷면에서 거의 옆으로 본 장면은 두 클래스가 매우 비슷하다. 혼동 행렬에서 이 둘을 먼저 볼 것.

---

## 2. 학습 준비 — 3클래스로 합치기

오전 인계의 `shelf_open`, `shelf_closed` 옆에 `shelf_filled` 를 넣으면 된다.

```
dataset_shelf/
├── shelf_open/      ← 오전 인계
├── shelf_closed/    ← 오전 인계
└── shelf_filled/    ← 이번 인계
```

```bash
# 1) YOLO seg 형식으로 내보내기 (3클래스)
python3 scripts/export_yolo.py dataset_shelf --out yolo_shelf3 --task segment \
    --class-by folder --split-by image

# 2) 학습
python3 scripts/train_yolo.py --data yolo_shelf3 --task segment --epochs 100 --device 0 \
    --batch 16 --name shelf3
```

GPU PC 에서 1)을 직접 돌려 확인한 결과:

| 분할 | 이미지 | shelf_closed | shelf_filled | shelf_open |
| --- | --- | --- | --- | --- |
| train | 1,440 | 482 | 481 | 477 |
| valid | 180 | 60 | 58 | 62 |
| test | 180 | 58 | 61 | 61 |

`data.yaml` 의 `names: ["shelf_closed", "shelf_filled", "shelf_open"]` (알파벳순, 클래스 id 0·1·2).

### 이번에 `export_yolo.py` 에 추가한 옵션 두 개 — **스크립트를 이 인계본으로 바꿀 것**

| 옵션 | 왜 필요한가 |
| --- | --- |
| `--class-by folder` | 폴더 이름(`shelf_open` 등)을 클래스로 쓴다. 기존 `book` 모드는 `dataset_shelf__shelf_open` 처럼 이름이 길어진다 |
| `--split-by image` | **기존 스크립트로는 1,800장이 전부 train 으로 가고 valid·test 가 0장이다.** (실제로 돌려서 확인) |

두 번째가 중요하다. 기존 분할은 **같은 책이 train 과 valid 에 섞이지 않도록 물체 단위로** 나눈다(책 데이터에서는 옳다).
서가는 클래스마다 물체가 하나뿐이라 물체 단위로 나누면 한 클래스가 통째로 한쪽에만 들어간다.

> **주의: 이미지 단위 분할이라 검증 점수가 낙관적이다.** valid 이미지도 train 과 같은 서가를 다른 각도에서 찍은 것이다.
> 점수가 높게 나와도 "새 서가에 일반화된다"는 뜻이 아니다. 실제 성능은 **Isaac Sim 레벨에서 찍은 이미지**로 확인할 것.

---

## 3. 에셋

| 파일 | 용도 |
| --- | --- |
| `assets/shelf/shelf_filled_render.blend` | **촬영에 쓴 파일.** 서가 틀과 책 상자를 한 오브젝트(`book_shelf_filled`)로 합치고 텍스처를 파일 안에 pack 했다 (다른 PC 에서도 흰색으로 나오지 않음). 목재 색을 Isaac 에 맞춰 어둡게 조정 (아래) |
| `assets/shelf/usd/shelf_darkbrown_filled.usda` | Isaac Sim 에 넣을 파일 (틀 + 책 상자 묶음) |
| `assets/shelf/usd/*.usdc`, `textures/` | 위 파일이 참조한다. **폴더째로 옮길 것** |

### 촬영 색을 Isaac 에 맞췄다 — 원본 blend 와 다른 점

원본 서가 목재에는 **금속성(Metallic) 0.8** 이 걸려 있고, 이 값이 Isaac 에서 서가를 **짙은 갈색**으로 보이게 한다.
그런데 원본 재질 그대로 Blender 로 찍으면 흰 벽·바닥이 금속면에 반사돼 **밝은 황갈색**이 나왔다.
로봇 카메라가 실제로 보는 건 Isaac 화면이므로, **촬영용 파일만** 목재 색을 어둡게 조정했다.

| 목재 픽셀 평균 RGB | 값 |
| --- | --- |
| Isaac 렌더 (목표) | (62, 53, 34) |
| 원본 재질로 촬영 | (149, 119, 90) — 2.4배 밝음 → **폐기** |
| 색 ×0.4 + 금속성 0 | (142, 117, 95) — 금속 반사가 원인이 아니었음 |
| **색 ×0.1 + 금속성 0.8 (채택)** | **(80, 64, 52)** — 무작위 조명 폭 안에서 Isaac 과 같은 톤 |

- 조정한 값: 목재 곱하기 색 (0.30, 0.16, 0.08) → **(0.03, 0.016, 0.008)**. `shelf_filled_render.blend` 에만 적용
- Isaac 용 USD 는 원본 의도(금속성 0.8)대로 두었다 (`docs/isaac_render_filled.jpg`)
- 조명이 무작위라 촬영 이미지 밝기 폭은 넓다 (목재 R 값 10~90% 구간 52~112)

---

## 4. 재생성 방법

```bash
blender -b assets/shelf/shelf_filled_render.blend -P scripts/render_book_dataset.py -- \
    --out dataset_shelf/shelf_filled --views 600 --book book_shelf_filled \
    --res 640 480 --hfov 90.5 --no-material-random --no-book-random \
    --elev -10 40 --fill 0.30 0.85 --roll 8 --backdrop room \
    --light-energy 40 150 --key-follows-camera --lights 2 3 --seed 11

python3 scripts/add_tight_bbox.py dataset_shelf/shelf_filled
```

개방형과 옵션이 같고 `--book`, `--seed`(7 → 11)만 다르다.

---

## 5. 알려진 한계

| 한계 | 설명 |
| --- | --- |
| 칸 안이 평면 이미지다 | 책 텍스처를 상자 면에 붙였다. 가까이서 비스듬히 보면 입체감이 없다 |
| 반쯤 찬 서가는 여전히 없다 | 비어 있음(open/closed)과 꽉 참(filled)만 있다. 오전 인계 6절의 "반쯤 찬 서가" 한계는 그대로다 |
| 검증 점수 낙관적 | 2절 주의 참고 |
| 도메인 갭 일부만 맞춤 | 목재 색 평균만 Isaac 에 맞췄다. 조명·반사·텍스처 선명도 차이는 측정하지 않았다 |

---

## 6. 이 묶음에 들어 있는 것

```
shelf_filled_handoff_20260917/
├── README.md                          ← 이 문서
├── dataset_shelf/shelf_filled/        600장 (images, depth, mask, labels, dataset_meta.json)
├── scripts/                           export_yolo.py(갱신), train_yolo.py, predict_check.py,
│                                      add_tight_bbox.py, check_labels.py, render_book_dataset.py, pose_math.py
├── assets/shelf/                      shelf_filled_render.blend, usd/
└── docs/
    ├── SHELF_FILLED_HANDOFF.md        (README 와 같은 내용)
    ├── sample_filled_mask.jpg         무작위 16장 + 마스크 윤곽(초록) + bbox(빨강)
    └── isaac_render_filled.jpg        Isaac Sim 렌더 (왼쪽 신규 서가, 오른쪽 기존 갈색 서가)
```
