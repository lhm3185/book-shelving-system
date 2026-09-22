# 컨트롤러 점검에서 남긴 것 — 동작을 바꾸는 항목

| 항목 | 값 |
| --- | --- |
| 점검 | 2026-09-21 새벽, 제3자 시선 |
| 이미 고친 것 | 커밋 `1824352` (조용히 틀리던 것들) |
| 이 문서 | **동작이 바뀌어서 시연 전에 못 넣은 것.** Isaac 으로 확인한 뒤 넣는다 |

---

## 1. 🔴 검증된 `poses.home` 이 런타임에서 **한 번도 안 쓰인다**

`book_scene.py` 의 `q_home` 은 YAML 이 아니라 **매번 IK 로 다시 푼다.** 그 시드가 문제다.

```python
seed[0] = math.atan2(tray_center_w[1] - l0p[1], tray_center_w[0] - l0p[0])
self.q_home, ok = self.ik_joints(self.home_tip, self.DOWN, seed)
```

`atan2(...)` 는 **월드 방위각**인데 그것을 **관절각** `joint_1` 에 넣는다.

| | 값 |
| --- | --- |
| 시드 `joint_1` | `atan2(0.0788, -0.4748)` = **+2.977 rad (170.6°)** |
| `measure_poses.py` 로 실측·검증한 home `joint_1` | **-0.1513 rad** |
| 차이 | **3.13 rad ≈ 179°** |

`ik_joints` 의 `ik_seed_limit`(1.6 rad)은 **이 시드 기준**으로 잰다. 그래서
**뒤집힌 가지는 한도 안에 들고 검증된 가지는 한도 밖으로 밀려난다** — 걸러내려고 만든 필터가
거르려던 것을 고르고 있다. `book_scene.py` 주석에 적힌 "joint_1 이 3.22 rad = 185° 로 나와
approach 에서 멈췄다" 가 정확히 이 증상이다.

실제 어제 실행에서도 approach 목표 `joint_1` 이 **2.79~3.02** 였다 — 시드 쪽 가지다.

### 왜 안 고쳤나

`--start-home snap` 이 이 `q_home` 으로 스냅하고, **먼 배치의 파지는 그 상태로 성공한다.**
지금 바꾸면 내일 (나)안이 쓰는 유일하게 되는 경로를 검증 없이 건드리는 것이 된다.

### 고칠 때

```python
seed = np.asarray(self.conf["poses"]["home"], float)   # 월드 방위각 대신 검증된 자세
```
그리고 `ok` 가 False 면 조용히 `q_home = seed` 로 넘어가지 말고 멈출 것.

**시험 방법**: 고친 뒤 `--start-home snap` 으로 띄우고 로그의
`시작 자세를 홈으로 고정 (오차 …)` 와 첫 `approach` 목표의 `joint_1` 을 본다.
`joint_1` 이 −0.15 근처면 검증된 가지를 쓰는 것이다.

> **어제 approach 가 막힌 것과 관계있을 수 있다.** 다만 먼 배치에서도 같은 시드를 쓰는데
> 거기선 됐으므로 **이것만으로는 설명되지 않는다.** 확정 시험(`SIM_TRAY_COLLIDER=0`)과
> **둘 다** 해 볼 것.

---

## 2. 트레이 위치에 yaw 만 쓰고, 명령 좌표는 전체 회전을 쓴다

```python
_tray_w = _bp + _Ry0 @ _tray_rel      # 위치까지 yaw 만
...
def to_world(self, p_arm): return self.l0p + self.Rl0 @ p_arm   # 전체 회전
```

`measure_poses.py`(칸 좌표를 만든 도구)도 **전체 회전**을 쓴다. 팔 베이스에 뒤집힘·기울기가
섞여 있다면 **트레이가 실제로 놓인 자리와 명령이 찾는 자리가 다르다.**
그러면 `book_on_tray_near` 가 실패(M411)하거나 **엉뚱한 책을 집는다.**

지금은 yaw 0 이라 잠복 상태다. 고칠 때는 **자세만 수평으로, 위치는 전체 회전**으로:

```python
_tray_w = _bp + _BR @ _tray_rel                    # _BR 은 이미 계산돼 있고 안 쓰이고 있다
_tray_q = [cos(yaw/2), 0, 0, sin(yaw/2)]           # 그대로
```

---

## 3. `verify()` 가 "명령이 틀렸을 때"를 못 잡는다

`y_front` / `place_x` 는 **명령에서 유도**한 값인데, `verify()` 가 그 값과 비교한다.
그래서 **팔이 시킨 대로만 하면 항상 통과**한다 — 허공에 놓으라고 시켜도 통과한다.
(9/20 에 실제로 허공에 놓고도 x 는 `True` 로 나왔다)

독립된 사실이 이미 있다: `self.shelf_front_y = float(shelf[1])` — **실제 서가 AABB** 다.
그런데 북엔드 y 범위에만 쓰고 명령과 대조하지 않는다.

고칠 때: `plan_job` 에서 `abs(y_front - self.shelf_front_y) > 0.03` 이면 거절하거나,
`floor_z` 에 이미 하듯 **실측 앞면으로 스냅**한다.

---

## 4. 테스트가 **실행되지 않는 사본**을 검사하고 있다

| 테스트가 검사하는 것 | 실제로 도는 것 |
| --- | --- |
| `arm_planning.plan_path` | `book_scene.plan_path` (복사본) |
| `book_tasks.spine_out_job` | `book_scene.plan_job` (복사본) |

**이미 벌어졌다**: `book_tasks.py` 에는 아직 `0.13 / 0.17` 이 박혀 있는데
`book_scene` 은 그 값을 YAML 로 뺐다. "테스트 54개 통과"가 `book_scene.py` 에 대해
말해 주는 것은 보기보다 적다.

고칠 때: 복사본을 지우고 `book_scene` 이 `arm_planning` / `book_tasks` 를 쓰게 한다.
**시연 뒤에 할 일이다** — 실행 경로를 통째로 바꾸는 작업이다.

---

## 5. 작은 것들 (시연에 영향 없음)

- `home_tip` z 가 검증된 home 보다 ~6 cm 높다. 1번을 고치면 같이 사라진다
- `.get("pre_lift_m", 0.13)` / `.get("carry_lift_m", 0.17)` 의 기본값이 **Franka 값**이다.
  YAML 오타 하나면 "6축에선 안 된다"고 적어 둔 값으로 조용히 되돌아간다
- `manipulation_executor.py` 의 `dev > 0.03`(M406), `ratio.max() > 0.8`(M403) 이 코드에 박혀 있다
- `run_simulation.py` 가 USD 를 **두 번 연다** (`open_world` → `BookScene.__init__`).
  4분 걸리는 로드를 두 번 하는 셈이고, 검증한 stage 핸들은 버려진다
- `--headless` 인자가 파싱만 되고 안 쓰인다
- `robot_profiles.check()` 가 `root` 와 `base_link` 만 보고, 런타임에서 호출되지 않는다.
  `hand_link` / `finger_links` / `ee_frame` / `camera_prim` 은 안 본다
- `base_idx` 가 `dummy_base` 로 시작하는 관절을 찾는데 **Nova Carter 에는 없다.**
  그래서 `q[s.base_idx] = s.base_hold` 가 조용한 no-op 이고, AMR 베이스를 붙들지 않는다
