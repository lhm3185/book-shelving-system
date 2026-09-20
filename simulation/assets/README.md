# B-1 레벨·에셋 모음 (2026-09-18)

시뮬 레벨과 그 레벨이 참조하는 에셋 전부입니다. **폴더 구조를 그대로 두고 통째로 복사**하세요.
레벨이 `../book_dataset/...` 로 참조하므로 구조가 깨지면 열리지 않습니다.

```
assets/
├── level/
│   ├── ing_library_env_v5.usd   ← 지금 쓰는 레벨 (v4 + 도서관 바닥)
│   ├── ing_library_env_v4.usd   ← 로봇(카메라·라이다) 포함
│   └── ing_library_env_v3.usd   ← 로봇 없음
└── book_dataset/
    ├── usd_v2/                  책 111종 + 텍스처 (325 MB)
    ├── assets/
    │   ├── franka_camera.usd    로봇 (AMR + 로봇팔 + RealSense + 라이다)
    │   ├── shelf/usd/           서가 (갈색 / 짙은갈색 채움)
    │   ├── env_wall_floor/      벽·바닥
    │   └── tray/tray_v1.usdc    트레이
    └── dataset/env_wall_floor/  바닥 (floor_01)
```

## 쓰는 법

복사한 뒤 `level/ing_library_env_v5.usd` 를 열면 됩니다. 예를 들어 홈 아래에 두면:

```bash
cp -r assets ~/b1_assets
# Isaac 실행기
SIM_USD=~/b1_assets/level/ing_library_env_v5.usd \
SIM_TRAY=~/b1_assets/book_dataset/assets/tray/tray_v1.usdc \
./scripts/run_isaac_sim.sh --gui
```

## 확인한 것

| 레벨 | 참조 | 없는 파일 | 메시 |
| --- | --- | --- | --- |
| v5 | 160 | **0** | 157 |
| v4 | 159 | **0** | 156 |
| v3 | 158 | **0** | 156 |

## 알아두실 점

- **인터넷 참조가 하나 있습니다.** v5 는 Omniverse 서버의 사람 캐릭터(`female_adult_police_02`)를 하나 참조합니다. 인터넷이 없으면 그 캐릭터만 안 보이고 나머지는 정상입니다
- 총 362 MB 입니다. 대부분(325 MB)이 책 텍스처입니다
- 서가 채움용 책 111종이 들어 있습니다. 파지·인식에 쓰는 책 6종도 여기 포함됩니다
- 저장소(git)에는 아직 넣지 않았습니다. LFS 용량 때문이며, 넣을지는 팀 결정 사항입니다
