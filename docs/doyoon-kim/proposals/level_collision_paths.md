# 레벨 콜리전 — 현재 상태와 고칠 경로

잰 파일: `~/levels/Final_Level_Library/Final_Level_Library.usdc`
md5 `4221e8ddbd9fd6106a59a448c59f1d7d` · 저장 2026-09-24 00:04:36 (데스크탑 실측)

> 이 목록은 **시뮬이 실제로 쓰는 파일**에서 뽑았다. 사본으로 재면 틀린다 —
> 9/23 사본 기준으로 "트레이 책 콜리전 0" 이라고 본 적이 있는데 실제로는 6개였다.
>
> 재는 법: `<venv>/python simulation/isaac/tools/measure_collider.py <usd> --root <경로>`
> (`pxr` 이 시스템에도 Isaac python 에도 없다. `python3 -m venv usdenv && usdenv/bin/pip install usd-core numpy`)

## 1. 트레이 책 — 근사를 `boundingCube` 로 (가장 중요)

지금 다섯 권 전부 `convexHull` 이다. 둥근 표지의 볼록 껍질은 밑면이 평평하지 않아
트레이에서 권마다 다른 각도로 굴러 앉고, 그 기운 책을 그대로 집어 그대로 꽂는다.
**꽂은 책의 칸 방향 폭이 35.2 mm 대신 43~44 mm 가 되어 옆 책과의 예비가 0.4 mm 까지 깎인다**
(최악 판은 이미 −0.1 mm 였다). `boundingCube` 로 바꾸면 36.4 mm · 예비 3.7 mm 가 된다.

| 근사 | 경로 |
|---|---|
| `convexHull` → **`boundingCube`** | `/World/tray_books/decorative_book_set_01_2k__book_softcover_01_cover58_01/Plane_119_001/Plane_119_001` |
| `convexHull` → **`boundingCube`** | `/World/tray_books/decorative_book_set_01_2k__book_hardcover_01_cover41_01/book_hardcover_01_cover41_usd/Plane_105_001/Plane_105_001` |
| `convexHull` → **`boundingCube`** | `/World/tray_books/decorative_book_set_01_2k__book_softcover_01_cover64_01/book_softcover_01_cover64_usd/Plane_125_001/Plane_125_001` |
| `convexHull` → **`boundingCube`** | `/World/tray_books/decorative_book_set_01_2k__catalogue_hardcover_01_cover60_01/Plane_121_001/Plane_121_001` |
| `convexHull` → **`boundingCube`** | `/World/tray_books/decorative_book_set_01_2k__book_softcover_01_cover31_01/book_softcover_01_cover31_usd/Plane_095_001/Plane_095_001` |
| `convexDecomposition` (그대로 두기) | `/World/tray_books/tray_04/Cube_003_001/Cube_003_001` — 트레이 본체 |

**레벨을 안 고쳐도 된다.** `SIM_BOOK_COLL=boundingCube` 가 런타임에 같은 일을 하고,
지금 기본 조합에 들어가 있다 (일곱 판 확인). 레벨에서 고치면 그 스위치가 필요 없어진다.

## 2. 서가 — 콜라이더가 **하나뿐**이다

| 콜리전 | x 범위 | y 범위 | 경로 |
|---|---|---|---|
| **있음** `convexDecomposition` | [1.866, 3.278] | [−2.575, −2.270] | `/World/bookshelves/shelf_brown__book_shelf_01/book_shelf_usd/book_shelf_001_001/book_shelf_001_001` |
| 없음 ×30 | — | — | 나머지 `book_shelf_001_00X` / `Cube_001_0XX` 전부 |

다행히 그 하나가 우리가 꽂는 자리(월드 x 2.496)를 덮는다. **시연에는 문제가 없다.**
다른 칸을 쓰려면 그 칸에도 붙여야 한다.

그리고 그 하나도 **면이 어긋나 있다**:

```
시각 판 윗면 (메시 점 8개)   0.4976
책이 실제로 앉는 높이         0.5155 ~ 0.5168   ← 쿠킹된 convexDecomposition 헐
계획이 겨누는 높이            0.4970  (= 0.355 × 1.4, 출처 불명 상수)
```

**책이 놓이면 16 mm 튕겨 오르고, 시각 선반 판 위 18 mm 에 떠 있다.** 영상에 보인다.
`convexDecomposition` 이 얇은 판을 복셀로 근사하면서 두꺼워진 것이다.

- 근사를 **`none`** 으로 바꾸면 면이 메시와 같아진다. 서가는 정적이므로 `none` 이 합법이다.
- 런타임 대안: `SIM_SHELF_COLL=none` + `SIM_SHELF_ROW_MEASURE=1` (기본 조합에 들어가 있음).

## 3. 서가 낱권 책 — 콜리전이 하나도 없다

```
/World/books/forthFloor   프림 176   CollisionAPI 0
/World/books/thirdFloor   프림 169   CollisionAPI 0
```

순전히 장식이다. **꽉 찬 칸에 꽂아도 물리로는 안 막힌다.** 그래서 겹침을 기하로 재고 있다
(`jam_report`, 임계 5 mm). 붙이면 물리가 막아 주지만 **책 340권이 동적 강체가 되므로
성능과 안정성을 봐야 한다** — 시연 전에 바꾸는 것은 권하지 않는다.

## 우선순위

```
1. 트레이 책 boundingCube   — 예비 0.4 → 3.7 mm. 런타임 스위치로 이미 커버됨
2. 서가 콜라이더 none        — 책이 18 mm 뜨는 것. 런타임 스위치로 이미 커버됨
3. 서가 낱권 책              — 시연 뒤. 340권을 동적으로 만드는 일이다
4. 나머지 서가 30개          — 다른 칸을 쓸 때만
```

**1·2 는 런타임 스위치가 이미 하고 있으므로 레벨을 안 고쳐도 시연은 된다.**
레벨에서 고치면 스위치를 지울 수 있고, 근사가 파일에 박혀 다음 사람이 안 헤맨다.
