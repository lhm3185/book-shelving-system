# b1 시연 꾸러미 — 2026-09-29 밤 정리

**아침에 하실 일: 이 폴더를 통째로 USB 에 담아 주세요.** (약 523 MB)

```
level_shaded/     도윤님 레벨 + 텍스처 92장 · 바닥 콜라이더            232 MB
level_collect/    같은 레벨을 자기완결로 모은 것 — **인터넷 없이 열린다**  262 MB
demo_env.sh       동결 설정 (저장소 사본과 같다)
clips/            녹화 영상 세 편 (auto · closeA · closeB)              20 MB
shots_shaded/     스틸 아홉 장 + 영상 한 편
```

## 교육장에서
```bash
git clone <저장소> ~/b1_arm && cd ~/b1_arm
git checkout freeze-20260929-shaded-r1          # ← 반드시 -r1. 옛 태그는 시작하자마자 죽는다
cp -r /media/<USB>/b1_demo_bundle ~/           # 이 폴더를 홈에 푼다
vi ~/b1_demo_bundle/demo_env.sh                 # 맨 위 박스의 세 줄만 고친다
source ~/b1_demo_bundle/demo_env.sh
bash scripts/demo/full_cycle.sh
```

## level_shaded 와 level_collect 중 무엇을 쓰나
- **인터넷이 나가면** `level_shaded` (기본값). 로봇 에셋을 NVIDIA 에서 받는다. 첫 로드만 느리다.
- **인터넷이 없으면** `level_collect`. `demo_env.sh` 의 `SIM_LEVEL` 을 이렇게 바꾼다:
  ```
  export SIM_LEVEL=$BUNDLE/level_collect/Final_Level_Shaded_robot.usdc
  ```
  이 꾸러미로 09/29 05:03 에 네 권 완주를 확인했다 (`runs/0452_collect_4of4`).
  의존 검사: 미해결 0 · 원격 0 · 폴더 밖 0.

## 주의
- `level_collect` 안에는 **NVIDIA 로봇 에셋**이 들어 있다. 공개 저장소에 올리지 않는다. USB 로만 옮긴다.
- 4권째에서 `411 NOT_READY` 로 멈추면 그 판을 접고 다시 돌린다 (대략 넷에 하나).
  자세한 것은 저장소의 `cli_exchange/handoff_0929/MORNING_0929.md` 3-3 절.
