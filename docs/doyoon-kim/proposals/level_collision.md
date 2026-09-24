# 레벨 콜리전 수정 — 무엇을 어떤 타입으로

작성 2026-09-24 · 김도윤(로봇팔) · **레벨 편집자용**

---

## 한 장 요약

| 대상 | 지금 | 바꿀 것 | 왜 |
|---|---|---|---|
| **트레이 책 5권** | `convexHull` | **`Bounding Cube`** | 꽂힌 책이 매번 다르게 2~3° 눕는다 |
| **서가 1개** | `convexDecomposition` | **`None`** (삼각망) | 책이 선반 판 위 18 mm 에 뜬다 |

**그 밖에는 아무것도 바꾸지 않습니다.** 특히 **트레이 자체(`tray_04`)는 건드리지 마세요** — 아래 §4.

---

## 1. 트레이 책 → `Bounding Cube`

### 대상

`/World/tray_books/` 아래 **`CollisionAPI` 가 붙은 메시 5개**입니다. 제 사본(9/23 22:47) 기준 경로:

```
/World/tray_books/decorative_book_set_01_2k__catalogue_hardcover_01_cover60_01/Plane_121_001/Plane_121_001
/World/tray_books/decorative_book_set_01_2k__book_softcover_01_cover58_01/Plane_119_001/Plane_119_001
/World/tray_books/decorative_book_set_01_2k__book_softcover_01_cover64_01/book_softcover_01_cover64_usd/Plane_125_001/Plane_125_001
/World/tray_books/decorative_book_set_01_2k__book_softcover_01_cover31_01/book_softcover_01_cover31_usd/Plane_095_001/Plane_095_001
/World/tray_books/decorative_book_set_01_2k__book_hardcover_01_cover41_01/book_hardcover_01_cover41_usd/Plane_105_001/Plane_105_001
```

**그 뒤 레벨을 덮어쓰셨으니 경로가 다를 수 있습니다.** 현재 레벨의 정확한 목록은 이 명령이 뽑아 줍니다 (Isaac 안 띄워도 됩니다):

```bash
python3 simulation/isaac/tools/measure_collider.py <레벨.usdc> --root /World/tray_books
```

`근사` 칸이 `convexHull` 인 줄이 대상입니다. `tray_04` 는 제외(§4).

### 바꾸는 법 (Isaac Sim GUI)

메시 프림 선택 → **Property** 패널 → **Physics / Collider** → **Approximation** 을 `Bounding Cube` 로.

### 왜

둥근 표지의 볼록 껍질은 **밑면이 평평하지 않습니다.** 그러면 어디서 어떻게 놓든 같은 각도로 눕습니다 — 끌개입니다.

그리고 저희 코드에 이미 그 주석이 있습니다 (책을 직접 스폰할 때):

> 표지가 둥근 책은 convexHull 이면 흔들려 넘어진다 (종류를 섞으면 특히).
> 책은 상자에 가까우므로 **boundingCube** 로 두면 트레이에 그대로 서 있는다.

레벨 책에는 그 처방이 안 돼 있었습니다.

### 효과 (실측 일곱 판)

```
꽂힌 책 기울기      2.08 ~ 3.51°  →  0.34 ~ 0.46°
칸 방향 폭          40.6 ~ 44.0   →  36.4 ~ 36.7 mm   (규격 두께 35.2)
판 간 흩어짐        1.4 mm        →  0.3 mm
겹침 임계까지 예비   최악 −0.1 mm  →  +3.7 mm
```

**`Bounding Cube` 는 동적 강체에 합법입니다.** 상자라서요.

---

## 2. 서가 → `None`

### 대상

`/World/bookshelves/` 아래 **`CollisionAPI` 가 붙은 메시 1개**입니다. 제 확인 기준:

```
/World/bookshelves/.../book_shelf_001_001      (x 1.866 ~ 3.278)
```

**서가 16개 중 콜리전이 있는 건 이것 하나뿐입니다.** 나머지 15개는 콜리전이 아예 없습니다.

```bash
python3 simulation/isaac/tools/measure_collider.py <레벨.usdc> --root /World/bookshelves
```

### 바꾸는 법

**Approximation** 을 `None` (또는 `Triangle Mesh`) 으로.

### 왜

```
시각 판 윗면   0.4976   (메시 점 8개에서 직접 읽음)
책이 앉는 곳   0.5155   ← 18 mm 위
```

2.5 m 짜리 통짜 메시를 `convexDecomposition` 으로 쿠킹하면 VHACD 가 복셀로 근사하는데, **얇은 선반 판에는 복셀 한 겹이 그대로 두께로 붙습니다.** 그 한 겹이 18 mm 입니다.

`None` 은 삼각망을 그대로 쓰므로 **콜라이더가 시각 형상과 같아집니다.**

### 안전한가

**네. 서가는 정적(static)이라 삼각망 충돌이 합법입니다.**

> **동적 강체에는 `None` 을 주면 안 됩니다.** PhysX 가 조용히 볼록 껍질로 갈아치웁니다 — 파일에는 `None` 이라 적혀 있는데 화면은 다르게 동작해서 저희가 며칠 헤맸습니다(트레이 `tray_v3` 건). 서가만 바꾸세요.

성능도 걱정 없습니다 — 그 메시가 **92점**입니다.

### 효과 (실측 다섯 판)

```
놓을 때 떠오름   +16 mm  →  +0.0 mm     (책이 판에 붙어 앉습니다)
```

그리고 **보기에도 고쳐집니다** — 지금은 꽂은 책 한 권만 이웃보다 1.8 cm 높게 떠 있습니다.

---

## 3. 지금은 런타임으로 덮어쓰고 있습니다

레벨을 안 고친 상태에서도 같은 효과가 나게 해 뒀습니다:

```bash
SIM_BOOK_COLL=boundingCube  SIM_SHELF_COLL=none  SIM_SHELF_ROW_MEASURE=1
```

`cli_exchange/scripts/run_vision.sh` 기본 조합에 들어가 있어 **그냥 돌리면 켜집니다.**

**레벨을 고치셔도 충돌하지 않습니다** — 이미 `boundingCube` 인 것에 다시 `boundingCube` 를 쓰는 것뿐이라 무해합니다. 레벨이 고쳐지면 나중에 이 스위치를 뺄 수 있습니다.

레벨에서 고치면 좋은 점: **비전팀도 같이 혜택을 봅니다.** 지금은 저희 실행 경로에서만 켜집니다.

---

## 4. 건드리지 말 것

| | 왜 |
|---|---|
| **`tray_04`** (트레이 본체) | `convexDecomposition` 이 맞습니다. 칸이 파인 형상이라 볼록 근사로 바꾸면 **칸이 메워져 책이 안 들어갑니다.** 9/22 에 그 함정으로 며칠 썼습니다 |
| `/World/books` (서가 낱권 책) | 콜리전이 없고, **없는 게 맞습니다.** 있으면 꽂을 때 밀어냅니다. 겹침은 저희가 기하로 검사합니다 |
| 서가 나머지 15개 | 콜리전 없음. 우리가 쓰는 칸이 `book_shelf_001_001` 안에 있어 문제없습니다 |

---

## 5. 고치신 뒤

레벨이 바뀌면 **저희가 다시 재야 합니다.** 바꾸신 뒤 알려 주시면 한 판 돌려 확인하겠습니다:

```
[놓음] 밑면−칸바닥 이 +0.0 mm 인가
[꽂은 자세] 가로 폭이 36 mm 대인가
min_margin 이 0.15 위인가
```

그리고 **레벨 md5 를 같이 알려 주세요** — 어느 판이 어느 레벨이었는지 기록에 남겨야 합니다.
