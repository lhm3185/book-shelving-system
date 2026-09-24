"""서가의 **빈칸**을 찾아 꽂을 x 를 정한다 — Isaac 없이 도는 순수 기하.

왜 필요한가 (2026-09-24 새벽): 꽂을 x 가 상수(`goal_x = -0.3497`)다. 그런데 이웃
책들은 레벨에 있고 레벨은 계속 바뀐다. 그래서 성공 판정이 난 판도 실제로는 옆 책을
**4.5 mm 파고들고** 있었다. 임계 5 mm 를 사이에 두고 판정만 갈렸을 뿐, 두 판 다
같은 곳을 파고들었다.

고치는 방향은 임계가 아니다. **빈칸의 한가운데를 겨누면 된다.** 빈칸은 재면 된다 —
`book_scene.shelf_book_boxes()` 가 이웃들의 월드 AABB 를 준다.

`tray_delivery` 와 같은 병이다: "레벨에서 잰 값" 을 상수로 박으면 레벨이 바뀔 때
조용히 썩는다. 그리고 판단이 Isaac 의존 파일 안에 있으면 GPU 없이 시험할 수 없다.

주의 — 이 파일이 **문턱을 낮추는 것이 아니다**. 겹침 임계(5 mm)는 그대로 둔다.
겨누는 자리를 옮겨 겹침 자체를 없애는 것이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


def span_for(thickness: float, depth: float, skew_deg: float) -> float:
    """비뚤게 선 책이 칸 방향(x)으로 차지하는 폭.

    책을 칸 안에서 `skew_deg` 만큼 돌리면 AABB 가 **두께가 아니라 대각선**만큼
    벌어진다. 두께 35 mm 짜리가 3.6° 만 돌아도 45 mm 를 먹는다 — 책등→앞마구리
    치수(163 mm)가 지렛대이기 때문이다.
    """
    t = math.radians(float(skew_deg))
    return float(thickness) * math.cos(t) + float(depth) * math.sin(t)


def skew_deg_from_span(span: float, thickness: float, depth: float) -> float:
    """찍힌 x 폭에서 기울기를 되돌린다 (0~90°). 못 풀면 0.

    왜 필요한가: 겹침 로그는 "폭이 10.1 mm 부풀었다" 고 말하는데, 그것만으로는
    얼마나 돌아간 것인지 감이 안 온다. 각도로 바꾸면 **삽입 yaw 허용치(0.10 rad
    = 5.7°)와 같은 단위**가 되어 바로 견줄 수 있다 (2026-09-24).
    """
    lo, hi = 0.0, 90.0
    if float(span) <= float(thickness):
        return 0.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if span_for(thickness, depth, mid) < float(span):
            lo = mid
        else:
            hi = mid
    return lo


@dataclass(frozen=True)
class Gap:
    """빈칸 하나. 전부 월드 x (m)."""

    lo: float
    hi: float
    left: str = ""        # 왼쪽 이웃 이름 ("" 면 서가 끝)
    right: str = ""       # 오른쪽 이웃 이름

    @property
    def width(self) -> float:
        return self.hi - self.lo

    @property
    def center(self) -> float:
        return (self.lo + self.hi) / 2.0

    def clearance(self, thickness: float) -> float:
        """한가운데 꽂았을 때 **한쪽** 여유 (m). 음수면 안 들어간다."""
        return (self.width - thickness) / 2.0

    def max_skew_deg(self, thickness: float, depth: float) -> float:
        """이 빈칸이 견디는 **최대 기울기** (도). 한가운데 꽂았다고 볼 때.

        옆 여유를 각도로 바꾼 값이다. 여유가 12.3 mm 라도 책이 8° 넘게 돌면
        들어가지 않는다 — 여유를 mm 로만 보면 이걸 놓친다.
        """
        lo, hi = 0.0, 90.0
        if span_for(thickness, depth, 0.0) > self.width:
            return 0.0
        for _ in range(60):
            mid = (lo + hi) / 2.0
            if span_for(thickness, depth, mid) <= self.width:
                lo = mid
            else:
                hi = mid
        return lo


def merge_boxes(boxes: Sequence[Tuple[str, float, float]],
                join_below: float = 0.0) -> List[Tuple[float, float, str, str]]:
    """겹치거나 붙어 있는 책들을 하나의 덩어리로 묶는다 → [(x최소, x최대, 왼쪽이름, 오른쪽이름)].

    왜 묶나: 서가 책들끼리도 AABB 가 이미 −0.7~−4.6 mm 겹쳐 있다(2026-09-23 실측).
    묶지 않으면 그 음수 간격들이 전부 '빈칸' 으로 잡힌다. `join_below` 보다 좁은
    틈은 빈칸으로 치지 않고 이어 붙인다.

    더 중요한 것: **눕혀 쌓인 큰 책**은 AABB 가 0.4~0.6 m 를 덮는다. 이웃한 두 권만
    보고 세면 그 안에 81 mm 짜리 빈칸이 있는 것처럼 보이는데, 겹침 판정은 AABB 로
    재므로 거기 꽂으면 그대로 겹친다. 묶어야 그 허깨비가 사라진다.

    양쪽 이름은 **덩어리의 가장자리에 실제로 있는 책**의 것이다 — 빈칸을 보고할 때
    "무엇 옆인가" 가 맞아야 하기 때문이다.
    """
    items = sorted(((float(lo), float(hi), str(n)) for n, lo, hi in boxes))
    out: List[Tuple[float, float, str, str]] = []
    for lo, hi, name in items:
        if out and lo - out[-1][1] <= join_below:
            p_lo, p_hi, n_lo, n_hi = out[-1]
            if hi >= p_hi:
                p_hi, n_hi = hi, name
            out[-1] = (p_lo, p_hi, n_lo, n_hi)
        else:
            out.append((lo, hi, name, name))
    return out


def gaps(boxes: Sequence[Tuple[str, float, float]],
         shelf_lo: Optional[float] = None, shelf_hi: Optional[float] = None,
         join_below: float = 0.005) -> List[Gap]:
    """책 사이(와 서가 양 끝)의 빈칸 목록. 넓은 순이 아니라 **왼쪽부터**."""
    blocks = merge_boxes(boxes, join_below)
    out: List[Gap] = []
    if not blocks:
        if shelf_lo is not None and shelf_hi is not None and shelf_hi > shelf_lo:
            out.append(Gap(shelf_lo, shelf_hi))
        return out
    if shelf_lo is not None and blocks[0][0] - shelf_lo > 0:
        out.append(Gap(shelf_lo, blocks[0][0], "", blocks[0][2]))
    for (_llo, lhi, _ln0, ln1), (rlo, _rhi, rn0, _rn1) in zip(blocks, blocks[1:]):
        if rlo - lhi > 0:
            out.append(Gap(lhi, rlo, ln1, rn0))
    if shelf_hi is not None and shelf_hi - blocks[-1][1] > 0:
        out.append(Gap(blocks[-1][1], shelf_hi, blocks[-1][3], ""))
    return out


def choose_gap(all_gaps: Sequence[Gap], want_x: float, thickness: float,
               clearance: float = 0.005,
               max_move: float = 0.15) -> Tuple[Optional[Gap], str]:
    """겨누던 x 에 **가장 가까운, 들어가는** 빈칸 → (빈칸 또는 None, 사유).

    넓은 칸을 고르지 않는다. 명령이 가리키던 자리를 지키는 것이 먼저다 — 제일 넓은
    칸을 고르면 엉뚱한 칸에 꽂고도 '빈칸에 잘 넣었다' 고 보고하게 된다.

    `max_move` 보다 멀리 가야 하면 **손대지 않는다.** 그때는 어느 칸을 겨냥한
    명령인지 알 수 없다 — 지금까지의 동작을 그대로 두고 판정에 맡긴다.
    """
    need = thickness + 2 * clearance
    fits = [g for g in all_gaps if g.width >= need]
    if not fits:
        widest = max((g.width for g in all_gaps), default=0.0)
        return None, (f'들어가는 빈칸이 없다 (필요 {need*1000:.1f} mm = 두께 '
                      f'{thickness*1000:.1f} + 여유 {clearance*1000:.1f}×2, '
                      f'가장 넓은 빈칸 {widest*1000:.1f} mm)')
    best = min(fits, key=lambda g: abs(g.center - want_x))
    move = best.center - want_x
    if abs(move) > max_move:
        return None, (f'가장 가까운 빈칸이 {move*1000:+.1f} mm 떨어져 있다 — '
                      f'{max_move*1000:.0f} mm 를 넘으면 어느 칸을 겨냥한 명령인지 '
                      f'알 수 없다. 손대지 않는다')
    return best, (f"빈칸 [{best.lo:.4f}, {best.hi:.4f}] 폭 {best.width*1000:.1f} mm "
                  f"(왼쪽 '{best.left or '서가끝'}' 오른쪽 '{best.right or '서가끝'}') "
                  f"→ 꽂을 x {want_x:+.4f} → {best.center:+.4f} ({move*1000:+.1f} mm), "
                  f"한쪽 여유 {best.clearance(thickness)*1000:.1f} mm")


def side_clearances(bb, boxes):
    """꽂은 책의 **좌우 실측 여유** (m). `(왼여유, 왼이름, 오른여유, 오른이름)`.

    이웃이 없는 쪽은 `None` 이다.

    왜 필요한가: 겹침 판정은 `겹쳤다 / 안 겹쳤다` 만 말한다. 그런데 **"겹침 0" 이
    "여유가 있다" 를 뜻하지 않는다** — 2026-09-24 에 겹침 0 으로 통과한 판의 실제
    여유가 한쪽 4.9 mm 였다(임계 5 mm). 통과와 아슬아슬함을 가르려면 **숫자**가
    있어야 하고, 지금 로그에는 그 숫자가 없다.

    **y·z 가 겹치는 책만 이웃으로 센다.** 다른 칸이나 뒤쪽에 있는 책은 옆에 있는
    것이 아니다 — x 만 보면 아래 칸 책이 이웃으로 잡힌다.
    """
    left = right = None
    left_name = right_name = ""
    for name, nb in boxes:
        if nb[4] <= bb[1] or nb[1] >= bb[4]:      # y 가 안 겹친다
            continue
        if nb[5] <= bb[2] or nb[2] >= bb[5]:      # z 가 안 겹친다 (다른 칸)
            continue
        if nb[3] <= bb[0]:                        # 왼쪽에 있다
            d = bb[0] - nb[3]
            if left is None or d < left:
                left, left_name = d, name
        elif nb[0] >= bb[3]:                      # 오른쪽에 있다
            d = nb[0] - bb[3]
            if right is None or d < right:
                right, right_name = d, name
    return left, left_name, right, right_name
