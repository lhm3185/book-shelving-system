"""서가 스캔 자세 계획 — 카메라를 어디에 두고 어디를 볼지 정한다.

**베이스를 움직이지 않는다.** 반납할 그 자리에서 팔만 접어 선반을 훑는다.
그래야 비전이 기억한 빈칸 좌표를 `arm_base_link` 그대로 쓸 수 있다 —
베이스가 움직이면 그 사이에 좌표계가 틀어진다 (2026-09-21: 주행 뒤 8.1 cm·4.37°).

## 좌표와 실측값 (2026-09-22, Isaac 5.1.0 / Franka + RSD455)

손 ↔ 카메라는 고정이다. 손 기준으로:

    위치   (-0.0400, +0.0015, +0.0164) m
    시선   손의 +Z   (= 그리퍼가 접근하는 방향)
    위쪽   손의 +X

즉 **카메라는 그리퍼가 가는 쪽을 본다.** 스캔 자세는 "선반을 향해 손을 뻗는 자세"와
같은 계열이라, 파지 자세에서 크게 벗어나지 않는다.

## 어디까지 볼 수 있나 (IK 실측, 팔 베이스 월드 z 0.28)

| 선반판 z | 결과 |
| --- | --- |
| 0.498 | **내려다보면**(-10°) 정면 포함 전부 풀린다. 수평·올려보기로는 정면에 해가 없다 |
| 1.042 | 자유롭게 (수평, 거리 0.55~0.75 전부) |
| 1.581 | **30° 이상 올려다보면** 잡힌다 |
| 2.058 | **어떤 각도·거리로도 해가 없다** — 팔 길이의 한계 |

0.498 이 수평에서 정면만 안 되는 이유: 카메라를 낮게 두고 수평으로 보려면 팔을 접어야 하는데
정면에서는 팔꿈치가 걸린다. **내려다보면 팔을 펴고 볼 수 있어** 풀린다.
처음에는 올려다보는 각만 시험해서 "정면은 불가" 라고 판단했다 — 아래쪽도 봤어야 했다.

**스캔 범위가 반납 범위보다 한 단 넓다.** 1.581 은 보이지만 꽂을 수는 없다
(삽입은 손을 0.75 m 앞으로 뻗어야 해서 1.1~1.2 m 가 한계).
따라서 "보이는데 못 넣는 칸"이 실제로 생기고, 그 판정은 로봇팔이 해야 한다.
"""

import math

import numpy as np

# 손 기준 카메라 (실측)
T_HAND_CAM = np.array([-0.0400, 0.0015, 0.0164])
# 열 = 손 기준 카메라 축 (camX, camY, camZ)
R_HAND_CAM = np.array([[0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])

#: **팔 기준 서가 뒷면** 까지의 거리 (m). 이름이 오래 `SHELF_FACE_Y` 였는데 **틀린
#: 이름이었다** — 앞면이 아니라 뒷면이다. 그 이름을 믿고 비전팀이 로봇을 0.29 m
#: 과도하게 뒤로 물렸다 (2026-09-23).
#:
#: 내력으로 갈랐다. 0.749 는 `7a1dc65`(09-22 15:26)에 들어왔고, 깨진 파지 자리
#: `−3.324` 는 `6254948`(09-22 22:43)에 처음 생겼다 — **7시간 뒤**다. 그 시점
#: 저장소에는 `−3.324` 가 존재하지 않았고 검증된 베이스는 `−3.019` 였다:
#:
#:     서가 앞면 world y −2.5745 · 뒷면 −2.2695 · 베이스 −3.019
#:       → 앞면까지 0.4445 · **뒷면까지 0.7495**   ← 0.749 와 0.5 mm 일치
#:
#: 앞면 기준 해석도 산술로는 똑같이 맞는다(베이스 −3.324 에서 앞면이 0.7495). 두
#: 해석이 겹치는 이유는 **파지 자리 보정량 0.305 와 서가 깊이 0.305 가 정확히
#: 같기 때문**이다. 그래서 숫자로는 영영 안 갈리고 내력으로만 갈린다.
#:
#: **이 상수는 오래 두지 말 것.** 두 움직이는 것(서가·팔 베이스) 사이의 *관계*를
#: 상수로 박은 형태이고, 베이스는 오늘만 −3.324 → −3.019 → −3.0695 로 세 번 바뀌었다.
#: `measure_shelf_back_y()` 로 실측에서 파생시키는 쪽이 옳다.
SHELF_BACK_Y = 0.749
SHELF_FACE_Y = SHELF_BACK_Y   # 옛 이름 (호환). 새 코드는 쓰지 말 것 — 앞면이 아니다
BOOK_CENTER_H = 0.12        # 선반판 위 책 중심 높이 (대략)
ARM_BASE_Z = 0.28           # 팔 베이스 월드 높이 — 선반판 z 를 팔 기준으로 바꿀 때 쓴다

#: 자세표가 **서 있는 자리에 대해 무엇을 가정하는가.** 주석이 아니라 자료로 둔다 —
#: 표를 다시 만들 때 이 값도 같이 고치게 되므로 가정이 표와 함께 이동한다.
#: `check_assumptions()` 가 시작 전에 실측과 대조하고, 어긋나면 **거부**한다.
SCAN_TABLE_ASSUMES = {
    "shelf_back_y_arm": 0.749,   # 팔 기준 서가 **뒷면**
    "shelf_x_center_arm": 0.0,   # 서가 좌우 중심이 팔 x=0
    "tol_y": 0.05,
    "tol_x": 0.15,
    "frame": "arm_base_link",
    "valid_at": "검증된 파지 자리(베이스 world y −3.019 계열). 주차 자리에서는 무효",
}


def measure_shelf_back_y(shelf_aabb_arm):
    """실측 서가 AABB(팔 기준)에서 뒷면 y 를 뽑는다 — 상수 대신 이걸 쓸 것.

    `shelf_aabb_arm` 은 `[xmin, ymin, zmin, xmax, ymax, zmax]` (팔 기준).
    서가는 팔 앞(+Y)에 있으므로 **뒷면 = ymax** 다.
    """
    import numpy as _np
    b = _np.asarray(shelf_aabb_arm, float)
    return float(b[4])


def check_assumptions(shelf_aabb_arm, assumes=None):
    """자세표의 전제가 지금 자리에서 성립하는가. 어긋나면 **사유 문자열**을 돌려준다.

    이건 **합격 문턱이 아니라 전제 조건**이다. 넣으면 '성공' 보고가 **줄어든다**
    (지금 성공으로 세어지던 무의미한 스캔이 거부로 바뀐다). 그래서 "문턱을 올려
    통과시키지 않는다" 와 같은 편이다.
    """
    import numpy as _np
    a = dict(SCAN_TABLE_ASSUMES if assumes is None else assumes)
    b = _np.asarray(shelf_aabb_arm, float)
    back = float(b[4])
    xc = (float(b[0]) + float(b[3])) / 2.0
    bad = []
    if abs(back - a["shelf_back_y_arm"]) > a["tol_y"]:
        bad.append(f"서가 뒷면 팔기준 y {back:+.3f} (표 전제 {a['shelf_back_y_arm']:+.3f} "
                   f"± {a['tol_y']:.3f}) — {abs(back - a['shelf_back_y_arm'])*100:.1f} cm 어긋남")
    if abs(xc - a["shelf_x_center_arm"]) > a["tol_x"]:
        bad.append(f"서가 좌우 중심 팔기준 x {xc:+.3f} (표 전제 {a['shelf_x_center_arm']:+.3f} "
                   f"± {a['tol_x']:.3f}) — {abs(xc - a['shelf_x_center_arm'])*100:.1f} cm 어긋남")
    if not bad:
        return ""
    return ("스캔 자세표의 전제가 지금 자리에서 성립하지 않는다: " + " / ".join(bad)
            + f". 표는 '{a['valid_at']}' 에서만 유효하다")

#: 판별 스캔 자세. (선반판 월드 z, 좌우 위치 cx, 카메라–목표 거리 d, 위로 드는 각 deg)
#: 전부 **IK 해가 있는 것으로 실측된 조합**이다 (2026-09-22).
#:
#: 아래 판은 **내려다본다**(-10°). 수평·올려보기로는 정면(cx=0)에 해가 없었는데,
#: 내려다보면 풀린다 — 카메라를 낮게 두고 수평으로 보려면 팔을 접어야 해서
#: 정면에서 팔꿈치가 걸리기 때문이다. 내려다보면 팔을 펴고 볼 수 있다.
#:
#: 순서도 관절 변화를 줄이려고 고른 것이다. 이웃 자세 사이 최대 변화:
#:   수평 + 앞 해만 씨앗    254.6°   ← 처음
#:   다중 씨앗              172.4°
#:   **아래판 내려다보기     83.3°**  ← 채택
#: 좌우를 지그재그로 도는 '뱀 순서' 는 오히려 102.7° 로 나빴다.
SCAN_TABLE = (
    # 아래 판 — 내려다본다. 정면도 풀린다
    (0.498, -0.30, 0.75, -10.0),
    (0.498, 0.00, 0.75, -10.0),
    (0.498, +0.30, 0.75, -10.0),
    # 가운데 판 — 수평. 가장 자유롭다
    (1.042, +0.30, 0.65, 0.0),
    (1.042, 0.00, 0.65, 0.0),
    (1.042, -0.30, 0.65, 0.0),
    # 위 판 — 40° 올려다봐야 잡힌다. 꽂을 수는 없지만 **보이는 것은 보고한다**
    (1.581, -0.30, 0.60, 40.0),
    (1.581, 0.00, 0.60, 40.0),
    (1.581, +0.30, 0.60, 40.0),
)

#: IK 를 풀 때 쓸 씨앗 수. 앞 자세 하나만 쓰면 가지가 튀어 관절이 크게 돈다.
#: joint_1·3·5·7 을 ±0.6/±1.2 rad 흔든 변형까지 풀어 **앞 자세와 가장 가까운 해**를 고른다.
SEED_JOINTS = (0, 2, 4, 6)
SEED_DELTAS = (-1.2, -0.6, 0.6, 1.2)

#: 이 값을 넘는 관절 변화가 이웃 자세 사이에 생기면 경고한다 (rad)
MAX_STEP_RAD = 1.6

#: 팔이 꽂을 수 있는 선반판. 스캔 범위보다 좁다 (위 문서 참조)
REACHABLE_BOARDS = (0.498, 1.042)


def camera_rotation(tilt_rad):
    """팔 기준 카메라 회전 — +Y 를 보되 위로 `tilt_rad` 만큼 든다.

    열이 카메라 축이다: camX = 팔 +X, camY(위) 와 camZ 는 tilt 만큼 돌아간다.
    카메라는 자기 **-Z** 를 보므로 camZ = -시선이다.
    """
    c, s = math.cos(tilt_rad), math.sin(tilt_rad)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, -s, -c],
                     [0.0, c, -s]])


def camera_pose(board_z, cx, distance, tilt_deg, arm_base_z=ARM_BASE_Z):
    """스캔 한 자세의 **카메라** 위치·회전 (팔 기준).

    목표점은 선반 **뒷면**(`SHELF_BACK_Y`)의 책 중심 높이다. 카메라는 그 점에서
    시선 방향으로 `distance` 만큼 물러난 자리에 놓는다.
    """
    tilt = math.radians(tilt_deg)
    target_z = board_z + BOOK_CENTER_H - arm_base_z
    view = np.array([0.0, math.cos(tilt), math.sin(tilt)])   # 시선 (팔 기준)
    pos = np.array([cx, SHELF_BACK_Y, target_z]) - distance * view
    return pos, camera_rotation(tilt)


def hand_pose(board_z, cx, distance, tilt_deg, arm_base_z=ARM_BASE_Z):
    """같은 자세를 **손** 기준으로 바꾼다 — IK 는 손을 푼다.

    카메라는 손에 고정돼 있으므로 손 자세는 카메라 자세에서 그 고정 변환만큼 뺀 것이다.
    """
    cam_p, R_arm_cam = camera_pose(board_z, cx, distance, tilt_deg, arm_base_z)
    R_arm_hand = R_arm_cam @ R_HAND_CAM.T
    return cam_p - R_arm_hand @ T_HAND_CAM, R_arm_hand


def scan_poses(table=SCAN_TABLE, arm_base_z=ARM_BASE_Z):
    """스캔 순서대로 (이름, 손 위치, 손 회전, 메타) 를 내놓는다.

    순서는 **아래에서 위로** 간다. 팔을 접었다 펴는 큰 이동을 줄이기 위해서다.
    """
    out = []
    for board_z, cx, distance, tilt_deg in table:
        p, R = hand_pose(board_z, cx, distance, tilt_deg, arm_base_z)
        side = "L" if cx < 0 else ("R" if cx > 0 else "C")
        name = f"scan_{board_z:.3f}_{side}"
        out.append((name, p, R, {"board_z": board_z, "cx": cx,
                                 "distance": distance, "tilt_deg": tilt_deg,
                                 "reachable": is_reachable_board(board_z)}))
    return out


def is_reachable_board(board_z, boards=REACHABLE_BOARDS, tol=0.01):
    """이 선반판에 **꽂을 수** 있나. 스캔은 되지만 못 꽂는 판이 있다."""
    return any(abs(board_z - b) <= tol for b in boards)


def unreachable_slots(slots, boards=REACHABLE_BOARDS, tol=0.01, arm_base_z=ARM_BASE_Z):
    """비전이 준 빈칸 중 **팔이 못 닿는 것**을 가려낸다 (1차 거름).

    높이 문턱은 1차일 뿐이다. 경계 근처는 계획기로 실제 풀어 봐야 확정된다 —
    도달 한계는 높이만이 아니라 수평 거리와 손 방향에도 걸리기 때문이다.
    `slots` 는 팔 기준 (x, y, z) 목록이고, z 를 선반판 월드 높이로 되돌려 비교한다.
    """
    out = []
    for i, s in enumerate(slots):
        board = float(s[2]) + arm_base_z - BOOK_CENTER_H
        if not is_reachable_board(board, boards, tol):
            out.append((i, board))
    return out


def seeds_for(q_prev, home):
    """IK 씨앗 목록 — 앞 자세, 홈, 그리고 앞 자세를 흔든 변형들.

    앞 자세 하나만 씨앗으로 쓰면 Lula 가 자세마다 다른 가지를 골라 관절이 크게 돈다
    (실측 254.6°). 여러 씨앗으로 풀어 **앞 자세와 가장 가까운 해**를 고르면 줄어든다.
    """
    out = [np.asarray(q_prev, float), np.asarray(home, float)]
    for j in SEED_JOINTS:
        for d in SEED_DELTAS:
            v = np.asarray(q_prev, float).copy()
            v[j] += d
            out.append(v)
    return out


def pick_closest(candidates, q_prev):
    """해 후보 중 앞 자세와 **가장 가까운** 것. 없으면 None."""
    if not candidates:
        return None
    return min(candidates, key=lambda c: float(np.max(np.abs(np.asarray(c, float)
                                                            - np.asarray(q_prev, float)))))


def step_size(q_a, q_b):
    """두 자세 사이 **가장 크게 도는 관절**의 변화량 (rad)."""
    return float(np.max(np.abs(np.asarray(q_b, float) - np.asarray(q_a, float))))
