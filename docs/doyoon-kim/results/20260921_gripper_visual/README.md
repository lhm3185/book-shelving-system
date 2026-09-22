# 그리퍼가 화면에 안 보인다 (물리는 정상)

| 항목 | 값 |
| --- | --- |
| 확인 | 2026-09-21 새벽 |
| 영향 | **시연 화면**. 물리·파지는 정상 |
| 상태 | 원인 확정. **고치지 않았다** — 에셋이 AMR 담당 소유라 협의가 필요하다 |

---

## 1. 증상

팔이 **가느다란 막대로 끝난다.** 그리퍼 조립체(퀵체인저 · 앵글브래킷 · 그리퍼 몸체 ·
손가락 2개 · RealSense D455)가 통째로 안 보인다.

| 그림 | |
| --- | --- |
| `arm_overview.png` | 팔 전체. 끝에 아무것도 없다 |
| `arm_end_no_gripper.png` | 팔 끝 확대. 막대만 튀어나와 있다 |

출처는 **GPU PC 에서 2026-09-20 에 찍은 시연 녹화본** (`level_shelf01.usd`, M0609).
집 데스크탑에서도 같은 경고가 나므로 **두 PC 다 같은 상태**다.

## 2. 물리는 멀쩡하다

시각만 빠진 것이다. 근거:

- 파지가 **실제로 성공**했다 (책이 손을 따라 움직였다)
- PhysX 겹침 조회에 그리퍼 콜라이더가 이름으로 잡힌다
  (`onrobot_rg2ft/left_inner_finger`, `right_inner_finger`, `gripper_body`, `angle_bracket`)
- 관절도 정상 — `left_inner_knuckle_joint` / `right_inner_knuckle_joint` 로 여닫힌다

**즉 시연 동작에는 문제가 없고, 화면에서만 팔 끝이 비어 보인다.**
책이 저절로 움직이는 것처럼 보이므로 **발표 영상에서는 티가 난다.**

## 3. 원인 — USD 참조가 없는 경로를 가리킨다

```
In </World/Nova_Carter_ROS/m0609/onrobot_rg2ft/world/visuals>:
  Unresolved reference prim path @…/onrobot_rg2_physics.usd@</visuals/world>
  introduced by @…/onrobot_rg2_base.usd@</onrobot_rg2ft/world/visuals>
```

두 파일을 열어 본 결과:

| 파일 | 사실 |
| --- | --- |
| `onrobot_rg2_base.usd` | `/onrobot_rg2ft/world/visuals` 가 **자체 메시 0개**. 유일한 내용이 아래 참조다 |
| `onrobot_rg2_physics.usd` | `/visuals` 아래에 **메시 9개가 멀쩡히 있다** (quick_changer, gripper_body, 손가락 등) |
| | 그런데 참조가 가리키는 **`/visuals/world` 는 존재하지 않는다** |

**메시는 있는데 참조 경로가 한 단계 어긋나 있다.** `world` 가 붙을 자리가 아닌 곳에 붙었다.
`base.usd` 를 단독으로 열면 자기 자신을 향한 같은 모양의 깨진 참조가 하나 더 나온다 —
**변환 단계(URDF → USD)의 버그로 보인다.**

> 디렉터리 이름이 `onrobot_rg2` 라 다른 에셋으로 오해하기 쉬운데, 그 안의 프림 이름이
> `onrobot_rg2ft` 다. **우리가 쓰는 바로 그 그리퍼가 맞다.**

## 4. 고칠 수 있는 방법 — **아직 적용하지 않았다**

참조 대상을 `</visuals/world>` → `</visuals>` 로 바꾸면 될 것으로 보인다.
다만 **`world` 가 왜 붙었는지**를 모르는 상태라 추측으로 에셋을 고치지 않았다.

### (가) 에셋 수정 — AMR 담당자와 협의 필요

원본을 고치는 것이 근본 해결이다. 변환 스크립트에 같은 버그가 있으면
다음 산출물에서도 반복된다. **이동준님께 전달할 것.**

### (나) 실행 시점 보완 — 우리가 쓰던 방식

파일을 안 고치고 실행할 때만 참조를 다시 거는 방법이 있다.
`camera_bridge.apply_amr_test_overrides()` 가 라이다·TF 에 쓰던 것과 같은 패턴이다.

```python
# **미검증.** Isaac 이 있는 PC 에서 시험해 볼 것
from pxr import Sdf
p = stage.GetPrimAtPath(f"{robot_path}/m0609/onrobot_rg2ft/world/visuals")
refs = p.GetReferences()
refs.ClearReferences()
refs.AddReference(<physics.usd 경로>, Sdf.Path("/visuals"))
```

레퍼런스는 리스트 연산이라, 약한 레이어(base.usd)의 의견을 강한 레이어에서 덮어야 한다.
**한 번에 될지 확실하지 않아 시연 경로에는 넣지 않았다.**

## 5. 시연에서 어떻게 할 것인가

- **동작 시연에는 영향 없다.** 그대로 진행해도 된다
- 화면을 보여줄 때 **"그리퍼 비주얼 에셋의 참조가 깨져 있어 안 보인다. 물리는 정상"** 이라고
  한 줄 말하면 된다. 숨기면 심사자가 먼저 알아채고 더 큰 의문이 된다
- 고치려면 (가)가 맞고, 그건 **오늘 아침에 할 일이 아니다**

## 6. 확인 방법

Isaac 을 띄운 뒤 경고에 `Unresolved reference prim path` 가 있는지 보면 된다.
없으면 고쳐진 것이다.
