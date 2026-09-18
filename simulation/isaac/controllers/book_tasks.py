"""책 파지·도서관식 꽂기 작업의 경유점 계산. 순수 계산 — Isaac 없이 테스트한다.

입력은 전부 **AABB 기준**이다 (좌표 계약: 에셋 원점을 쓰지 않는다. 원점이 책에서 37cm 떨어진 사고가 있었다).

동작 (2026-09-17 검증)
  홈(트레이 위) → 책등 위 접근 → 내려가 파지 → 들기
  → 선반 앞으로 운반하며 손목 90° 세우기 → 앞마구리부터 틈에 끼우기 → 놓기
  → 손끝이 책등 뒤로 완전히 빠짐 → 그리퍼 닫기 → 책등 밀기 → 후퇴 → 홈
"""

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

# Franka Hand 실측 (Isaac Sim 5.1.0)
FINGER_THICK = 0.0264        # 손가락 한 개의 벌림축 두께
TIP_DOWN = 0.035             # 책등 윗면에서 손끝이 내려가 잡는 깊이
GRIP_CLEAR = 0.005           # 벌렸을 때 책 표면과 손가락 사이 한쪽 여유
BACKOFF_BEHIND_SPINE = 0.025 # 놓은 뒤 손끝이 책등 뒤로 빠지는 거리 (닫기 전에)


@dataclass
class BookBox:
    """책의 월드 AABB. 데크·트레이에 책등이 위로 선 상태: 두께=X, 길이=Y, 폭(책등→앞마구리)=Z"""
    lo: np.ndarray
    hi: np.ndarray

    @property
    def center(self): return (np.asarray(self.lo) + np.asarray(self.hi)) / 2
    @property
    def thick(self): return float(self.hi[0] - self.lo[0])
    @property
    def length(self): return float(self.hi[1] - self.lo[1])
    @property
    def width(self): return float(self.hi[2] - self.lo[2])


@dataclass
class ShelfSlot:
    """서가 칸 위의 꽂을 위치"""
    x: float                 # 책 두께 방향 중심
    front_y: float           # 서가 앞면 (로봇 쪽)
    floor_z: float           # 칸 바닥
    spine_inset: float = 0.02  # 최종 책등이 앞면에서 들어가는 거리


@dataclass
class SpineOutJob:
    segments: List[Tuple[str, List[Tuple[np.ndarray, str]]]]   # (구간 이름, [(위치, 자세이름)])
    speeds: Dict[str, float]
    grip_open: float          # 트레이·서가에서 벌리는 폭 (손가락 하나당)
    grip_hold: float          # 파지 폭
    spine_final_y: float
    points: Dict[str, np.ndarray] = field(default_factory=dict)


def grip_open_width(book: BookBox) -> float:
    """옆 칸 책을 치지 않을 만큼만 벌린다 (4cm 로 벌리면 바깥폭 13.3cm → 옆 책 충돌)"""
    return book.thick / 2 + GRIP_CLEAR


def finger_outer_half_width(open_per_finger: float) -> float:
    return open_per_finger + FINGER_THICK


def min_neighbor_pitch(book: BookBox, margin: float = 0.005) -> float:
    """책 사이 간격이 이보다 좁으면 파지할 때 손가락이 옆 책에 닿는다"""
    return finger_outer_half_width(grip_open_width(book)) + book.thick / 2 + margin


def spine_out_job(book: BookBox, slot: ShelfSlot, home_tip) -> SpineOutJob:
    c = book.center
    grasp = np.array([c[0], c[1], book.hi[2] - TIP_DOWN])
    pre = grasp + [0, 0, 0.13]
    lift = grasp + [0, 0, 0.17]

    L, W = book.length, book.width
    grip_z = slot.floor_z + L / 2 + 0.004                 # 세우면 길이가 세로
    y_pre = slot.front_y - (W - TIP_DOWN) - 0.03          # 앞마구리가 서가 앞 3cm 밖
    transfer = np.array([slot.x, y_pre - 0.02, grip_z + 0.06])
    pre_ins = np.array([slot.x, y_pre, grip_z + 0.01])
    wedge = np.array([slot.x, slot.front_y + 0.10 - (W - TIP_DOWN), grip_z])   # 앞마구리 10cm 끼움
    back = wedge - [0, TIP_DOWN + BACKOFF_BEHIND_SPINE, 0]
    push_z = slot.floor_z + L / 2
    spine_final = slot.front_y + slot.spine_inset
    touch = np.array([slot.x, wedge[1] - TIP_DOWN - 0.008, push_z])
    push = np.array([slot.x, spine_final + 0.002 - 0.010, push_z])   # 닫은 손끝 접촉점은 기준점보다 약 1cm 앞
    retreat = np.array([slot.x, slot.front_y - 0.13, push_z])
    home = np.asarray(home_tip, float)

    segs = [
        ("approach", [(home, "DOWN"), (pre, "DOWN")]),
        ("down", [(pre, "DOWN"), (grasp, "DOWN")]),
        ("lift", [(grasp, "DOWN"), (lift, "DOWN")]),
        ("carry_rotate", [(lift, "DOWN"), (transfer, "DOWN"), (pre_ins, "HORIZ")]),
        ("wedge", [(pre_ins, "HORIZ"), (wedge, "HORIZ")]),
        ("back", [(wedge, "HORIZ"), (back, "HORIZ")]),
        ("touch", [(back, "HORIZ"), (touch, "HORIZ")]),
        ("push", [(touch, "HORIZ"), (push, "HORIZ")]),
        ("retreat", [(push, "HORIZ"), (retreat, "HORIZ")]),
        ("return", [(retreat, "HORIZ"), (home, "DOWN")]),
    ]
    speeds = {"approach": 0.5, "down": 0.25, "lift": 0.25, "carry_rotate": 0.35, "wedge": 0.35,
              "back": 0.3, "touch": 0.3, "push": 0.12, "retreat": 0.35, "return": 0.5}
    return SpineOutJob(segs, speeds, grip_open_width(book), max(0.0, book.thick / 2 - 0.004), spine_final,
                       points={"grasp": grasp, "wedge": wedge, "back": back, "touch": touch,
                               "push": push, "retreat": retreat, "pre_ins": pre_ins})
