# 교육장 점검표 — 데스크탑에만 있는 것이 없는가 (2026-09-29 아침 마감)

아침 판 `0637_4books_1` · `0648_4books_2` 가 **실제로 읽은 것**을 전부 적었다.

| 무엇 | 어디에 | 확인값 |
|---|---|---|
| 코드 | 저장소 `feature/grasp_advanced` (옛 이름 `work/demo_0928`) | 판이 돈 HEAD **`86ebe79`** · 태그 `freeze-20260929-shaded-r2` |
| `demo_env.sh` | 저장소 `cli_exchange/handoff_0929/` | 꾸러미 원본과 **`diff` 결과 같음** |
| 레벨 usdc (네 권) | 저장소 `simulation/assets/level/Final_Level_Shaded/` | md5 **`8fc1e9f50d05d7385ecfaeb181f0fcd4`** (판이 돈 파일과 같음) |
| 바닥 콜라이더 | 저장소 같은 폴더 `env_base_collider.usd` | 2,213 B |
| `franka_camera.usd` | 저장소 `simulation/assets/book_dataset/assets/` | md5 `dd26950f…` — 레벨 폴더의 것과 **같은 파일** |
| 텍스처 92장 (216 MB) | **저장소에 없다** · USB `b1_demo_bundle/level_shaded/textures/` | 저장소에는 `textures.md5`(92줄) |
| 로봇 NVIDIA 에셋 | 원격(https) 참조 · 오프라인은 USB `level_collect/` | 공개 저장소에 올리지 않는다 |
| 비전 가중치 | 저장소 LFS `ros2_ws/src/shelving_perception/resource/` | `best.pt` · `book_tray_best.pt` |
| 좌표·지도·책 규격 | 저장소 `cli_exchange/config/` | `waypoints_measured.yaml` · `shelf_map_measured.yaml` · `book_profiles_measured.yaml` |
| DDS 프로파일 | 저장소 `config/fastdds_local.xml` | `demo_env.sh` 가 가리킨다 |

## git — 남은 것이 없다
```
git status --short                         →  (아래 커밋 뒤) 비어 있음
git log origin/feature/grasp_advanced..HEAD   →  비어 있음      (옛 이름 work/demo_0928 — BRANCHES_0929.md)
git stash list                             →  2건 (2026-09-23 `04441f6` 시절, 오늘 작업과 무관)
다른 브랜치                                 →  전부 origin 을 따라가거나 뒤처져 있음. 앞선 것 없음
```

## 판이 읽은 환경변수 — `demo_env.sh` 만 source 하면 전부 나온다
```
SIM_LEVEL=$BUNDLE/level_shaded/Final_Level_Shaded_robot.usdc
SIM_D_SAG_MAX_M=0.03                 SIM_SPINE_FROM_NEIGHBOURS=1
SIM_SHELF_PRIM / SIM_SHELF_PRIMS     SIM_BOOK_IDS / SIM_RFID_TAGS / SIM_CLASS_CODES
SIM_TRAY_FROM / SIM_TRAY_TO          VISION_ROI_MIN=[-0.40,-0.2,0.0] / VISION_ROI_MAX=[-0.26,0.40,0.32]
MAN_EXTRA=-p vision_grasp_region_max:=[-0.15,0.40,0.30]    REC_EVERY=0
ROS_DOMAIN_ID=129  RMW_IMPLEMENTATION=rmw_fastrtps_cpp  FASTRTPS_DEFAULT_PROFILES_FILE=…/config/fastdds_local.xml
```
**셸에만 살아 있는 설정은 없다.** 아침 판은 `SIM_LEVEL` 만 사본 경로로 덮어 돌렸는데, 지금은 그 사본이
꾸러미의 원래 이름 자리에 들어가 있으므로 `demo_env.sh` 를 그대로 source 하면 같은 파일을 읽는다.

## 파이썬 (비전이 쓰는 것)
```
ultralytics 8.4.157 · torch 2.14.0+cu130 · numpy 2.5.3 · cv2 5.0.0
```
교육장 GPU PC 는 2026-09-16 에 `uv` 로 torch 2.11.0+cu128 · ultralytics 를 넣었다. **버전이 다르다** —
가중치는 같으니 대개 돌지만, 첫 판에서 검출이 이상하면 이 차이를 먼저 본다.

## USB 꾸러미
```
/dev/sda1 (FAT32, 라벨 UBUNTU) 의 b1_demo_bundle/   524 MB · 파일 233개 · md5 232/232 OK
level_shaded(네 권) · level_collect(네 권) · clips · shots_shaded · demo_env.sh · README.md · MD5SUMS
```

## 교육장 첫 판
```bash
git clone <저장소> ~/b1_arm && cd ~/b1_arm
git checkout freeze-20260929-shaded-r2
cp -r /media/<USB>/b1_demo_bundle ~/
vi ~/b1_demo_bundle/demo_env.sh      # 맨 위 박스의 세 줄만 (BUNDLE · FASTRTPS 경로 · DISPLAY=:1)
source ~/b1_demo_bundle/demo_env.sh && bash scripts/demo/full_cycle.sh
```
인터넷이 없으면 `SIM_LEVEL` 을 `$BUNDLE/level_collect/Final_Level_Shaded_robot.usdc` 로 바꾼다.
