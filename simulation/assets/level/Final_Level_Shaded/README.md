# Final_Level_Shaded — 시연 레벨 (2026-09-29 아침 · **트레이 네 권**)

태그 **`freeze-20260929-shaded-r2`** 가 재현하는 레벨이다. 도윤님이 블렌더 5.2.2 에서 바닥·벽·서가를
셰이딩해 뽑은 것에 로봇 참조를 붙였다.

## 이 폴더에 있는 것
```
Final_Level_Shaded_robot.usdc   15.9 MB   레벨 본체 + 로봇 참조. 이것을 SIM_LEVEL 로 준다
                                md5 8fc1e9f50d05d7385ecfaeb181f0fcd4
env_base_collider.usd            2.2 KB   바닥 콜라이더. 아래 [왜 따로 있나] 참고
textures.md5                     9.1 KB   텍스처 92장의 md5 — 받은 것이 맞는지 대조한다
```

## **텍스처 92장(216 MB)은 이 저장소에 없다**
`.gitattributes` 가 png 를 LFS 로 보내는데, 216 MB 를 올리면 저장소 LFS 가 383 MB → 630 MB 가 되고
**대역폭 월 한도(1 GB)를 팀 전체가 같이 쓴다.** pull 한 번이 630 MB 라 한도를 넘기면 팀원들 pull 이
막힌다. 그래서 텍스처는 USB 로 옮긴다.

받는 법:
```bash
# USB 의 b1_demo_bundle/level_shaded/textures 를 이 폴더 옆에 푼다
cp -r /media/<USB>/b1_demo_bundle/level_shaded/textures  simulation/assets/level/Final_Level_Shaded/
# 맞게 왔는지 대조 (92줄 전부 OK 여야 한다)
cd simulation/assets/level/Final_Level_Shaded/textures && md5sum -c ../textures.md5
```
usdc 가 텍스처를 `./textures/…` 상대경로로 가리키므로 **usdc 옆에 textures 폴더만 있으면 열린다.**
없으면 흰색/자홍색으로 뜬다 (레벨이 깨진 게 아니다).

## 로봇은 원격 참조다
`franka_camera.usd` 안에서 NVIDIA 에셋 셋을 https 로 참조한다(ridgeback_franka · rsd455 ·
Example_Rotary). **인터넷이 있으면 Isaac 이 받아서 쓴다.** 없으면 `Collect As` 로 만든 꾸러미가
필요하다 — 자세한 것은 `cli_exchange/handoff_0929/PORT_BUNDLE.md`.
NVIDIA 에셋은 배포 조건 때문에 **공개 저장소에 올리지 않는다.**

## 왜 콜라이더가 따로 있나
도윤님의 바닥은 큐브에서 면 하나를 떼어 만든 **면 한 장**이다. 부피가 0 이라
`convexDecomposition` 이 무너지고, 그 위에 놓은 트레이가 로봇을 밀어낸다 (09/28 실측).
그래서 바닥 메시는 `physics:approximation = none` 으로 두고, 검증된 `env_Base` 상자를
**보이지 않는 콜라이더**로 따로 넣었다. 레벨을 다시 뽑을 때 이 두 가지를 같이 챙겨야 한다.

## 이 레벨에서 바뀐 이름 (옛 레벨과 다름)
```
반납기   /World/return_machine_final      (옛: /World/return_machine)
서가     /World/bookshelves_main/shelf_brown__book_shelf_11  = shelf_01
         /World/bookshelves_main/shelf_brown__book_shelf_01  = shelf_02
```

## 여는 법
```bash
source cli_exchange/handoff_0929/demo_env.sh     # SIM_LEVEL 등 동결 설정
bash scripts/demo/full_cycle.sh
```


## 2026-09-29 아침 — 트레이를 네 권으로 바꿨다
트레이의 `decorative_book_set_01_2k__book_hardcover_01_cover41_01` **한 권을 뺐다**
(월드 x +5.7352 · 팔기준 y +0.077 · 두께 44.3 mm — 트레이에서 가장 두꺼운 책).

**왜**: 다섯 권일 때 4권째 검출 신뢰도가 0.55~0.63 으로 문턱(0.55) 바로 위에 붙어 있어
네 판 중 세 판이 `411 NOT_READY` 로 멈췄다. 이 책을 빼니 4권째가 **0.82~0.83** 으로 올라왔고
두 판 연속 4/4 로 섰다 (`runs/0637_4books_1` · `runs/0648_4books_2`).
~~혼자 남아서 안 보였던 게 아니라 이 두꺼운 책이 옆 책을 가리고 있었던 것으로 본다.~~
**정정 (2026-09-29 교육장)**: 가림이 아니었다. 이 책(h41)을 되살리고 카탈로그를 뺀 판에서 h41 이 혼자 남자 60장 전부 검출 0,
그림에서 가린 것 없음(`cli_exchange/handoff_0929/runs/1052_site1_swap41_411`). **h41 자체가 혼자 남으면 문턱 위로 안 나온다.**
태그 `freeze-20260929-shaded-r2` 의 메시지에 같은 옛 문장이 있다 — 태그는 고치지 않고 여기서 정정한다.

**뺀 방법**: `Sdf.Layer` 에서 `nameChildren` 의 스펙을 지웠다. 이 prim 이 루트 레이어에만
정의돼 있는 것을 `GetPrimStack()` 으로 먼저 확인했다(참조가 아니라 직접 정의).
비활성(`active=false`)이 아니라 **실제로 지웠다** — 블렌더·Isaac 에서 열었을 때 헷갈리지 않게.

다섯 권 레벨은 태그 `freeze-20260929-shaded-r1` 이고, 파일은 데스크탑
`~/levels/Final_Level_Shaded/*.5books_0929.usdc` 에 남겨 뒀다 (저장소에는 없다).
