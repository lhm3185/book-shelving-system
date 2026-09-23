# LEDGER — 2026-09-23 밤 (계획 v7)

23:2x 계획 v7 적용 · 블록 0·2L·4·6·A·F (랩탑 트랙) + 1·2D·3·5·7 (데스크탑 트랙)
      · 브랜치 work/motion_api_0924 · 도메인 130
      · 실행 주체 분리: 랩탑=설계/배선/문서, 데스크탑=GPU 실행
      · **데스크탑 이미 열림** — 키 등록·레벨 전송(241MB, 체크섬 일치)·재개 지시 완료
      · 불변식② 기준선 = 1
        (grep -rc 'self[.]lula[.]compute_inverse_kinematics(' simulation/isaac/controllers/)

## 블록 0 — 착수

- 브랜치 생성 완료
- **비전 커밋 fetch 불가** — `work/slot-scan-merged` 가 원격에 없고 교육장 망이 끊겼다.
  낮에 읽은 요약으로 대체하고 **미확인** 으로 명시한다:
  1. `c7a711a` perception `target_frame` world → **arm_base_link**
  2. `d6039e9` 스캔 작업이 RUNNING 에 영원히 남던 것 수정 + 자세별 검출 평가 도구
  3. `c5aedd6` **arm_kinematics(FK/IK/moveJ/moveL)** + 수평 스윕 + 1차 시연 방식 출발·도착
  → 빈칸 좌표의 **반환 형태·프레임·칸 폭·법선·측정 시점 베이스 자세는 전부 미확인**
- 블록 A 발송 완료 (push 요청 두 줄 포함)
