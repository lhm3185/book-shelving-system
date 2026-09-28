# 교육장 꾸러미 — 지금 상태와 남은 일 (09/29 01:50 실측)

## 1. 잰 것 — `UsdUtils.ComputeAllDependencies`

```
Final_Level_Shaded/Final_Level_Shaded_robot.usdc   레이어 3 · 에셋 92 · 미해결 3 · 원격 0 · 폴더 밖 0
Final_Level_Library/Final_Level_RnD.usd            레이어 2 · 에셋 88 · 미해결 3 · 원격 0 · 폴더 밖 0
```
텍스처 92/88 장은 **전부 레벨 폴더 안**에 있다. 폴더 밖·원격 참조 0. 여기까지는 깨끗하다.

**미해결 3개는 둘 다 같다** — `franka_camera.usd` 안의 NVIDIA 원격 참조다:
```
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd
https://…/Isaac/Sensors/Intel/RealSense/rsd455.usd
https://…/Isaac/Sensors/NVIDIA/Example_Rotary.usda
```
(맨 python 에서는 옴니버스 리졸버가 없어 "미해결" 로 나온다. Isaac 안에서는 받아서 쓴다 — 즉 **인터넷이 있으면 돈다**.)

## 2. 함정 — `robot_assets/` 는 지금 상태로는 못 쓴다

09/28 에 위 3개 파일을 `Final_Level_Shaded/robot_assets/` 로 받아 뒀는데, **그 3개가 또 남을 참조한다**:
```
ridgeback_franka.usd → ./Props/panda_link0~7 · panda_hand · panda_leftfinger …  (13개, 안 받아져 있음)
rsd455.usd          → OmniPBR.mdl · OmniGlass.mdl · Materials/Base/… (5개, 안 받아져 있음)
```
**참조를 robot_assets/ 로 바꿔 끼우면 로봇이 통째로 사라진다.** 09/28 에 "로봇이 없어졌는데?" 를 겪은 것과 같은 꼴이다.
→ 이 폴더는 **지금 상태로 커밋하지도, 참조를 바꿔 끼우지도 않는다.**

## 3. `~/.cache/ov` 도 답이 아니다
`~/.cache/ov/client/https` 는 파일 29개가 **해시 이름으로 납작하게** 들어 있다(폴더 구조 없음). 통째로 복사해도
리졸버가 같은 URL 을 같은 해시로 찾아 준다는 보장이 없다. 이식 수단으로 쓰지 않는다.

## 4. 남은 일 — Isaac 안에서 `Collect As` 한 번
원격 참조를 실제로 풀 수 있는 것은 옴니버스 리졸버뿐이고, 그건 Isaac 이 떠 있어야 있다.
판 ② 가 끝나면 Isaac 을 띄워 두 레벨 각각 **File ▸ Collect As** 로 자기완결 폴더를 만든다. 그 다음:
```
① ComputeAllDependencies 재측정 → 미해결 0 · 원격 0 · 폴더 밖 0 인지 확인
② du -sh 로 용량 기록 (지금 레벨 폴더만 269M / 272M)
③ NVIDIA 에셋이 들어간 꾸러미는 **공개 저장소에 올리지 않는다** — USB 로만 옮긴다
④ 저장소에는 텍스처·레벨(우리 것)만: simulation/assets/level/Final_Level_Shaded/ 에서 robot_assets/ 제외
```

## 5. 지금 당장의 안전판
교육장에 인터넷이 있으면 아무것도 안 해도 돈다(처음 한 번 받느라 느릴 뿐). Collect As 는 **인터넷이 없을 때를
위한 보험**이다. 교육장 망 상태는 랩탑이 확인 중.
