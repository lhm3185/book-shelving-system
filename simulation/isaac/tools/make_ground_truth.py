"""비전팀 정답지 생성 — 9/23 레벨 실측에서 `ground_truth_slots.yaml` 을 만든다.

## 왜 이런 모양인가

**서가에 빈칸은 원래 없다.** `secondFloor` 43권은 빈틈 없이 꽂혀 있다 —
폭 1.267 m 에 두께 합 1.372 m 가 들어가 있고, 이웃 간격이 전부 음수
(−0.7 ~ −4.6 mm)다. 그러니 "빈칸을 검출한다" 가 성립하려면 우리가 **책을 빼서
빈칸을 만들어야** 한다 (`SIM_SHELF_GAP`, 런타임 프림 비활성).

그래서 정답지는 "원래 있던 구멍의 목록" 이 아니라 **"어떤 지시를 주면 어떤 구멍이
생기는가" 의 표**다. 비전팀은 시험 단계마다 다른 폭을 받게 된다.

## 폭 기준인 이유

책 두께가 7.7 mm(잡지)부터 44.8 mm(하드커버)까지 **6배** 차이 난다.
"몇 권 빼기"로 정의하면 같은 지시가 전혀 다른 난이도가 되므로,
**열 폭**을 말하고 필요한 최소 권수를 빼는 방식으로 정의한다.

난이도 사다리 (`t` = 우리가 꽂는 책 두께 35.3 mm):

    L1  t + 20 mm = 55.3   첫 성공. 좌우 여유 10 mm
    L2  t + 10 mm = 45.3   실전 난이도. 좌우 여유 5 mm
    L3  t +  5 mm = 40.3   한계. 여기서 실패하는 게 정상

이 사다리가 **비전 검출 정확도 요구와 같은 축**이다 — L2 에서 좌우 여유가 5 mm 이므로
검출 오차 σ 가 5 mm 를 넘으면 실패가 나오기 시작한다.

## 쓰기

    python3 simulation/isaac/tools/make_ground_truth.py \\
        docs/doyoon-kim/measurements/20260923_gt_slots.txt \\
        -o simulation/isaac/config/ground_truth_slots.yaml

Isaac 이 필요 없다 — 실측 파일만 읽는다.
"""
import argparse
import os
import re
import sys

ap = argparse.ArgumentParser()
ap.add_argument("measurement", help="20260923_gt_slots.txt (권별 실측)")
ap.add_argument("-o", "--out", default="simulation/isaac/config/ground_truth_slots.yaml")
ap.add_argument("--floor", default="secondFloor", help="정답지를 만들 층")
ap.add_argument("--board-z", type=float, default=1.042, help="그 층의 선반판 월드 z")
ap.add_argument("--book-thick-mm", type=float, default=35.3,
                help="우리가 꽂는 책 두께 (mm). 사다리는 여기에 +20/+10/+5")
ap.add_argument("--ladder-mm", type=float, nargs="+", default=[20.0, 10.0, 5.0])
a = ap.parse_args()

# 실측 파일 한 줄:
#   0   decorative_..._cover21   1.9108   1.9207  0.0100  0.2707  1.044   X   X  (앞과 간격 ...)
ROW = re.compile(r"^\s*(\d+)\s+(\S+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+"
                 r"([-\d.]+)\s+([-\d.]+)\s+(\S+)\s+(\S+)")

books, cur = [], None
for line in open(a.measurement, encoding="utf-8"):
    m = re.match(r"^=== (\w+)\s", line.strip())
    if m:
        cur = m.group(1)
        continue
    if cur != a.floor:
        continue
    g = ROW.match(line)
    if g:
        books.append({"index": int(g.group(1)), "name": g.group(2),
                      "x_min": float(g.group(3)), "x_max": float(g.group(4)),
                      "thickness": float(g.group(5)), "height": float(g.group(6)),
                      "floor_z": float(g.group(7)),
                      "rigid": g.group(8) == "O", "collision": g.group(9) == "O"})

if not books:
    sys.exit(f"'{a.floor}' 층을 실측 파일에서 못 찾았다: {a.measurement}")
books.sort(key=lambda b: b["x_min"])
for i, b in enumerate(books):
    b["index"] = i                      # x 순서로 다시 매긴다 (SIM_SHELF_GAP 이 쓰는 번호)

t = a.book_thick_mm / 1000.0
gaps = []
for level, extra in enumerate(a.ladder_mm, start=1):
    want = t + extra / 1000.0
    for start in range(1, len(books) - 1):          # 양 끝은 이웃이 없어 폭이 정의되지 않는다
        removed, acc, i = [], 0.0, start
        while i < len(books) - 1 and acc < want:
            acc += books[i]["thickness"]
            removed.append(i)
            i += 1
        if acc < want:
            continue
        x_lo = books[start - 1]["x_max"]
        x_hi = books[i]["x_min"]
        opened = x_hi - x_lo
        if opened < want - 0.002:       # 겹침 때문에 두께 합만큼 안 열린 경우
            continue
        gaps.append({
            "level": f"L{level}", "switch": f"{start},{round(want*1000)}",
            "start_index": start, "removed_index": removed,
            "removed_count": len(removed),
            "requested_width_m": round(want, 4),
            "opened_width_m": round(opened, 4),
            "side_clearance_m": round((opened - t) / 2, 4),
            "gap_x_min": round(x_lo, 4), "gap_x_max": round(x_hi, 4),
            "gap_center_x": round((x_lo + x_hi) / 2, 4),
        })

# 층별로 대표 하나씩만 남긴다 (전부 쓰면 표가 수백 줄이 된다)
pick = []
for level in range(1, len(a.ladder_mm) + 1):
    same = [g for g in gaps if g["level"] == f"L{level}"]
    if not same:
        continue
    # 서가 가운데 쪽, 가장 적은 권수로 열리는 것
    mid = len(books) / 2
    same.sort(key=lambda g: (g["removed_count"], abs(g["start_index"] - mid)))
    pick.append(same[0])

os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
with open(a.out, "w", encoding="utf-8") as f:
    w = f.write
    w("# 비전팀 정답지 — 서가 빈칸 검출용\n")
    w("#\n")
    w("# **빈칸은 원래 없다.** 이 층 책들은 빈틈 없이 꽂혀 있다(이웃 간격이 전부 음수).\n")
    w("# 빈칸은 SIM_SHELF_GAP 으로 책을 빼서 만들고, 폭은 우리가 지정한다.\n")
    w("# 그러니 검출 대상은 '원래 있던 구멍' 이 아니라 **'우리가 낸 구멍'** 이고,\n")
    w("# 시험 단계마다 폭이 달라진다.\n")
    w("#\n")
    w("# 좌표는 전부 **world 기준**, 단위 m. 자동 생성: make_ground_truth.py\n")
    w(f"# 원자료: {os.path.basename(a.measurement)} (2026-09-23 실측)\n\n")

    w("shelf:\n")
    w("  prim: /World/bookshelves/shelf_brown__book_shelf_01\n")
    w(f"  floor: {a.floor}\n")
    w(f"  board_z_world: {a.board_z}\n")
    w("  note: >-\n")
    w("    낱권 책이 있는 서가는 이것 하나뿐이다. 나머지 15개는 책이\n")
    w("    book_cube 통짜 메시 하나라서 빈칸 검출 시험이 되지 않는다.\n")
    w("    이 층의 책에는 RigidBodyAPI 도 CollisionAPI 도 없다 — 순전히 장식이라\n")
    w("    검출 결과를 물리로 확인할 수 없다.\n")
    w(f"  book_count: {len(books)}\n")
    w(f"  x_span_world: [{books[0]['x_min']:.4f}, {books[-1]['x_max']:.4f}]\n")
    th = [b["thickness"] for b in books]
    w(f"  thickness_m: {{min: {min(th):.4f}, max: {max(th):.4f}, "
      f"mean: {sum(th)/len(th):.4f}}}\n")
    ov = [books[i]["x_min"] - books[i - 1]["x_max"] for i in range(1, len(books))]
    w(f"  neighbor_gap_m: {{min: {min(ov):+.4f}, max: {max(ov):+.4f}}}   "
      f"# 전부 음수 = 겹쳐 있다\n\n")

    w("# 난이도 사다리. t = 우리가 꽂는 책 두께\n")
    w(f"inserted_book_thickness_m: {t:.4f}\n")
    w("ladder:\n")
    for g in pick:
        w(f"  - level: {g['level']}\n")
        w(f"    switch: \"SIM_SHELF_GAP={g['switch']}\"\n")
        w(f"    removed_index: {g['removed_index']}\n")
        w(f"    opened_width_m: {g['opened_width_m']}\n")
        w(f"    side_clearance_m: {g['side_clearance_m']}    "
          f"# 좌우 여유 — 검출 오차가 이보다 크면 실패가 난다\n")
        w(f"    gap_x_world: [{g['gap_x_min']}, {g['gap_x_max']}]\n")
        w(f"    gap_center_x_world: {g['gap_center_x']}\n")
    w("\n")

    w("# 층의 모든 책 (x 순서). index 가 SIM_SHELF_GAP 의 시작 인덱스다\n")
    w("books:\n")
    for b in books:
        w(f"  - {{index: {b['index']:2d}, x: [{b['x_min']:.4f}, {b['x_max']:.4f}], "
          f"thickness: {b['thickness']:.4f}, height: {b['height']:.4f}, "
          f"name: {b['name']}}}\n")

print(f"{a.out} 을 만들었다 — {a.floor} {len(books)}권, 사다리 {len(pick)}단")
for g in pick:
    print(f"  {g['level']}  SIM_SHELF_GAP={g['switch']}  "
          f"{g['removed_count']}권 빼서 {g['opened_width_m']*1000:.1f} mm 열림 "
          f"(좌우 여유 {g['side_clearance_m']*1000:.1f} mm)")
