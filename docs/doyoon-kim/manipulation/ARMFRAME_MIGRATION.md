# 배치 계산을 팔 기준으로 바꾸기 — 남은 기술부채

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-21 새벽 |
| 상태 | **계획만.** 코드는 안 고쳤다 |
| 지금 어떻게 넘어가고 있나 | `set_robot_yaw.py` 로 레벨에서 로봇 yaw 를 0 으로 맞춘다 (임시 조치) |
| 언제 할 일인가 | 1차 시연 이후. **9/30 최종 발표 준비 구간** |

---

## 1. 무엇이 문제인가

`plan_job` 의 삽입 경로가 **월드 축을 직접 쓴다.**

```python
place_x = place_center_world[0]                      # 칸은 월드 x 를 따라 늘어선다고 가정
y_front = place_center_world[1] - W/2 - MEASURED_INSET   # 서가는 월드 +y 에 있다고 가정
transfer = [place_x, y_pre - 0.02,  grip_z + 0.06]
pre_ins  = [place_x, y_pre,         grip_z + 0.01]
wedge    = [place_x, y_front + 0.10 - (W - TIP_DOWN), grip_z]
push     = [place_x, spine_final + 0.002 - 0.010, push_z]
retreat  = [place_x, y_front - 0.13, push_z]
```

`x` 는 칸이 늘어선 방향, `y` 는 서가를 향하는 방향이라고 **고정**돼 있다.
로봇이 다른 방향을 보고 있으면 이 가정이 깨지고, `carry_rotate` 에서 IK 가 안 풀린다.

받은 에셋의 팔이 yaw 90° 로 서 있어서 **실제로 그 일이 났다** (2026-09-20).

## 2. 지금 어떻게 피하고 있나

`isaac_sim/isaac/tools/set_robot_yaw.py` 로 **레벨에서 로봇을 돌려** yaw 0 으로 만든다.
`place_robot_at_shelf.py` 는 yaw 가 0 이 아니면 아예 멈춘다.

로봇 배치는 우리 담당이라 "시연 구성의 선택"으로 넘어갈 수 있었지만,
**주행이 들어오면 못 쓴다.** AMR 이 서가 앞에 어떤 각도로 서든 꽂을 수 있어야 한다.

## 3. 이미 팔 기준으로 바꾼 것 (참고할 패턴)

같은 문제를 **파지 쪽에서는 이미 풀었다.** 그 방식을 그대로 쓰면 된다.

```python
# book_scene.py 389~393
_cy, _sy = math.cos(self.tray_yaw), math.sin(self.tray_yaw)
_arm_x_w = [_cy,  _sy, 0.0]     # 칸이 늘어선 방향 (팔 기준 x 를 월드로)
_arm_y_w = [-_sy, _cy, 0.0]     # 서가를 향하는 방향 (팔 기준 y 를 월드로)
self.DOWN  = self.orientation([0, 0, -1], _arm_x_w)
self.HORIZ = self.orientation(_arm_y_w, _arm_x_w)
```

트레이도 이미 팔 기준으로 놓는다 (`_tray_w = _bp + _Ry0 @ _tray_rel`).
**남은 건 삽입 경유점뿐이다.**

## 4. 할 일

### 4-1. 경유점을 팔 기준으로 만들고 마지막에 월드로 변환

지금 월드 좌표로 조립하는 것을, **팔 기준으로 조립한 뒤 한 번에 변환**한다.
`to_world()` 가 이미 있다 (`self.l0p + self.Rl0 @ p_arm`).

```python
# 지금:  월드 x/y 를 직접 조립
wedge = np.array([place_x, y_front + 0.10 - (W - TIP_DOWN), grip_z])

# 바꾼 뒤: 팔 기준으로 조립 → 마지막에 변환
wedge_arm = np.array([place_x_arm, y_front_arm + 0.10 - (W - TIP_DOWN), grip_z_arm])
wedge = self.to_world(wedge_arm)
```

바꿀 대상: `transfer`, `pre_ins`, `wedge`, `back`, `touch`, `push`, `retreat`,
그리고 `y_front` / `spine_final` / `place_x` 의 유도.

### 4-2. `shelf_front_y` 를 팔 기준 거리로

```python
self.shelf_front_y = float(shelf[1])     # 월드 y. 서가가 +y 에 있다고 가정
```
→ 서가 앞면을 **팔 기준 y 거리**로 바꾼다. 팔 베이스에서 서가 앞면까지의 거리는
yaw 와 무관하게 정의된다.

### 4-3. 북엔드도 같이

`fit_bookends` 가 `place_x`(월드 x)로 칸을 찾고 월드 y·z 로 큐브를 놓는다.
경유점과 같은 방식으로 팔 기준 → 월드 변환을 태운다.

### 4-4. `set_robot_yaw.py` 는 남겨 둔다

바꾼 뒤에도 **레벨을 yaw 0 으로 만드는 것 자체는 유효**하다
(비교 기준이 있으면 회귀를 잡기 쉽다). 다만 **필수가 아니게 된다.**

## 5. 어떻게 검증하나

**yaw 를 바꿔 가며 같은 결과가 나오는지 본다.** 이게 이 작업의 합격 판정이다.

```bash
# 같은 레벨을 yaw 0 / 45 / 90 으로 만들고
for Y in 0 45 90; do
  ARM_ROBOT=m0609 ISAAC_ENTRY=isaac_sim/isaac/tools/set_robot_yaw.py \
    ./scripts/run_isaac_tool.sh --usd <원본> --out /tmp/lv_$Y.usd --yaw $Y
done
# 각각에서 같은 팔 기준 좌표로 꽂아 본다. 셋 다 같은 결과여야 한다
```

지금 코드로 돌리면 **yaw 0 만 통과**한다. 그게 현재 상태의 증거이기도 하다.

단위 테스트도 붙일 수 있다 — `plan_job` 의 경유점 계산만 떼어내
yaw 0 / 45 / 90 에서 **팔 기준 경유점이 동일한지** 확인하는 식이다.
(`isaac_sim/isaac/tests/` 는 Isaac 없이 도는 테스트다)

## 6. 왜 지금 안 하나

- **시연 전날 밤에 할 작업이 아니다.** 삽입 경로 전체를 건드리는 일이고,
  검증하려면 Isaac 에서 여러 yaw 로 돌려 봐야 한다
- 지금은 `set_robot_yaw.py` 로 충분히 넘어간다
- **주행이 붙기 전까지는 급하지 않다.** 급해지는 시점은 AMR 이 임의 각도로 도킹할 때다

## 7. 관련

- 임시 조치: `isaac_sim/isaac/tools/set_robot_yaw.py` (주석에 같은 내용이 있다)
- 파지 쪽 선례: `book_scene.py` 389~393, `_tray_w` 계산
- 변환 도우미: `book_scene.to_world()`
