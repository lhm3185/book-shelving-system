# 데스크탑 세션이 켜지면 가장 먼저 읽을 것 — 2026-09-29 00:55

노트북 세션이 메시지로 보내려 했으나 데스크탑 Claude 세션이 목록에 없어 파일로 남긴다.
도윤님 지시: "데스크탑 다시 켰어, 연결 확인하고 이어서 진행해줘."

지시서는 `cli_exchange/handoff_0928/NIGHT_PLAN_0929.md` 그대로다 (계획 v1 · 블록 0·1·2·3·S·F).
노트북의 오프라인 몫과 근거는 `cli_exchange/handoff_0929/LAPTOP_NIGHT_0929.md`.

## 받는 법

```bash
cd ~/b1_arm && git fetch && git checkout work/night_0929     # = work/demo_0928(86c6827) + 노트북 커밋
cd ros2_ws && colcon build --packages-select shelving_manipulation && cd ..   # 파라미터 하나가 늘었다 (기본 꺼짐)
```
동결 태그 `freeze-20260928-scan-once` 는 그대로다. 데스크탑 커밋은 `cli_exchange/handoff_0929/` 아래로만,
push 전에 `git pull --rebase`.

## 순서 (한 번에 하나만)

0. **LEDGER 첫 줄** — 계획 v1 · 블록 0·1·2·3·S·F · 브랜치 · 커밋 · 두 레벨 md5
1. **판을 태우기 전에 2분** — 4권째가 서가 책을 문 판의 `vision.log` 가 남아 있으면
   ```bash
   python3 simulation/isaac/tools/vision_log_report.py <판>/vision.log \
       --slots=-0.188,-0.004,0.180,0.364 --since <4권째 검출 시작 HH:MM:SS>
   ```
   출력 전체를 `cli_exchange/handoff_0929/` 에 저장하고 노트북에 보낸다. 볼 것 셋:
   트레이 칸3 무리가 ROI y 상한까지 몇 mm 남았는가 / 끼어든 무리의 y 범위 / [5] 에서 y 보다 넓게 갈리는 축
2. **ROI 0.40 판을 끝까지 1판** (스위치 끔). 끝나면 같은 도구를 그 판 로그에 돌린다
3. 통과하면 같은 설정으로 2판 더. 3/3 이면 태그 `freeze-20260929-shaded`
4. 4권째가 **410** 으로 끝나면 그때 `MAN_BOOK_PREFILTER=true` 로 1판.
   **411** 로 끝나면 스위치는 소용없다 — 도구 출력을 노트북에 보내고 같이 정한다
5. 옛 레벨 회귀 1판 (스위치 끔, `tray_match_tol` 0.05 상태) — 동결본과 같은가

## 알아둘 것

- `MAN_BOOK_PREFILTER` 는 ④의 해법이 아니다. 범위 **밖** 검출이 합의를 가로채 410 으로 끝나는 길만 막는다
- 옛 레벨 로그(9/25)로 본 y 축 틈은 20~30 mm (추정). z 는 트레이 책과 9 mm 차이뿐이라 못 쓴다
- **ROI 를 0.40 아래로 내리지 않는다** — 칸3 검출이 0.386 까지 올 수 있다 (칸 중심 0.364 + 흔들림 22 mm)
- NVIDIA 에셋: 명시 허용 외 재배포 금지, 오프라인 사용 허용. 공개 저장소 커밋 금지, Collect 본은 `.gitignore`
- 판마다 로그를 즉시 복사 · 판 띄우기 전 `ps` 로 런처 겹침 확인 · 종료는 PID 로만

## 노트북과 연결

노트북 세션 이름은 `rokey-68`. Remote Control 이 붙으면 `ListAgents` 에 보인다.
판 하나가 끝날 때마다 결과 한 단락(권별 중심오차·기울기·겹침, 실패면 오류 코드와 로그 줄 원문)을 보낸다.
