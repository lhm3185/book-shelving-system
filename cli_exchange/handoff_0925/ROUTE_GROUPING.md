# 네 권을 **서가별로 묶어서** 돌린다 (도윤님 지시, 2026-09-25)

> "4권 삽입 과정이 매번 책을 한권씩 삽입하고 주행을 반복하는데 매번 주행할 필요없이
> 책장 B 두 권 반납 후에 A까지 바로 주행하고 남은 두 권 삽입 후 홈위치 복귀"

데스크탑이 레벨 격자에서 잰 값과, 그래서 무엇을 고쳐야 하는지. 랩탑 검토 바람.

## 1. 지금 왜 매번 주행하는가 — 원인 두 개 (코드에서 확인)

**(가) FSM 은 작업 순서를 건드리지 않는다.**
`job_planner.py` 가 `normalized_book_ids / _rfid_tags / _classification_codes` 를 그냥 `zip`
한다. 즉 **넣어 준 순서 그대로** 돈다. 서가별 묶기는 정렬 코드가 아니라 환경변수 한 줄이다.

    SIM_BOOK_IDS="['book_003','book_004','book_001','book_002']"
    SIM_CLASS_CODES="['512.3','611.0','005.7','006.3']"     # 5,6 → shelf_02(B) · 0 → shelf_01(A)

(분류 접두사는 `shelf_map_measured.yaml`: shelf_01 = 0~4, shelf_02 = 5~9.)

**(나) "이미 앞에 서 있다 — 주행 생략" 이 shelf_01 에만 안 걸린다.**
`nav_manager._command_for_goal` 의 생략 판정이 `if route is not self.patrol_route:` 안에 있다
(주석: "library_loop(shelf_01)는 검증된 흐름 그대로 둔다"). 그래서 A 의 두 권째도 32 m 를 다시 돈다.

이 가드는 **지금은 불필요하다.** `library_loop` 의 마지막 점이 `patrol_start` 로 다시 돌아오게
적혀 있어서 (`waypoints_measured.yaml`) `route[-1] == patrol_start == (2.535, -3.019)` =
서가 A 작업 자리다. B 쪽 경로와 판정 기준이 똑같다.

## 2. B → A 를 어떻게 가나 — 실측 (카트 반폭 0.45 m, 서가 16개 AABB 대조)

    ① B → A 곧장                     5.25 m   관통 5개 (_01,_02,_06,_07,_11)   ✗
    ② B → 통로 → A  (제안)          10.26 m   관통 0                          ✓
    ③ 지금 (B 에서 library_loop)     37.28 m   관통 9회 (첫 다리 5 + 끝 다리 4) ✗

②의 네 다리:

    (-0.548,+1.225) → (-0.548,+0.760)  0.47 m   [B 주차 자리에서 물러남]
    (-0.548,+0.760) → (+4.000,+0.760)  4.55 m   깨끗
    (+4.000,+0.760) → (+4.000,-3.019)  3.78 m   깨끗
    (+4.000,-3.019) → (+2.535,-3.019)  1.47 m   [A 주차 자리로 들어감]

첫·끝 다리에 검사기가 플래그를 띄우지만 **주차 자리 자체**다 — 서가 앞면에서 0.444 m 물러난
자리라 반폭 0.45 가 AABB 를 스친다. 끝 다리는 `library_loop` 의 `patrol_start↔patrol_wp1` 과
같은 선분이고 이미 검증된 구간이다. 실제 관통은 0.

②의 앞 세 다리는 이미 있는 `home_from_shelf_02` (wp3→wp2→wp1) 와 같은 통로다 — 새로 만드는
게 아니라 끝점만 `patrol_wp1(4.0,-3.019) → patrol_start` 로 바꾼 것이다.

## 3. 제안 — 고칠 곳 두 군데 (둘 다 랩탑 소관)

**(A) `waypoints_measured.yaml` — 경로 추가**

    to_shelf_01_from_shelf_02:
      - shelf_02_wp3      # (-0.5476, +0.760)
      - shelf_02_wp2      # (+4.000,  +0.760)
      - patrol_wp1        # (+4.000,  -3.019)
      - patrol_start      # (+2.535,  -3.019)   ← A 작업 자리. shelf_01 웨이포인트(2.229,-3.010,yaw90°) 아님

**(B) `nav_manager._command_for_goal` — 두 줄**

    # 1) 어디서 오는지 아는 경로를 먼저 본다 (_last_shelf_id 대입 **전에** 읽는다)
    _from = getattr(self, '_last_shelf_id', None)
    route = (self._routes or {}).get(f'to_{request.target_id}_from_{_from}') \
            or (self._routes or {}).get(f'to_{request.target_id}') or self.patrol_route

    # 2) 생략 판정의 `if route is not self.patrol_route:` 가드를 뺀다
    #    (library_loop[-1] == patrol_start == A 작업 자리라 같은 기준이 그대로 성립)

## 4. 이렇게 하면 한 판이 이렇게 돈다

    홈 → to_shelf_02 (검증됨)        → B 1권
       → 생략                        → B 2권
       → to_shelf_01_from_shelf_02   → A 1권
       → 생략                        → A 2권
       → goto(홈, 출발 yaw)          → 복귀        ← 9/9 로 검증된 A→홈 그대로

주행이 **4번 → 3번**, 거리는 대략 **37 m 를 10 m 로**. 그리고 B 가 먼저이므로 마지막 복귀가
가장 많이 검증된 A→홈 경로가 된다 — 지금 순서(A 먼저)보다 안전하다.

## 5. 우리가 확인 못 한 것

- `_last_shelf_id` 는 지금 shelf 목표에서만 갱신된다. A 두 권째에서 `to_shelf_01_from_shelf_01`
  을 찾다 없으면 `to_shelf_01`(없음) → `library_loop` 로 떨어지는데, (B)의 2)로 생략이 걸리니
  경로가 실제로 실행되지는 않는다. 그래도 로그에는 "library_loop 로 patrol" 이 찍힐 수 있다.
- `home_from_shelf_01` 은 없다 → 평범한 goto. 의도한 대로다(검증된 경로).
- FSM 이 `shelf_id` 를 안 채워 주는 문제는 그대로다 (조작 쪽이 거리로 고른다). 이번 변경과 무관.

---

## 부기 — 이 커밋(d2487bf)에 `nav_manager.py` 가 같이 들어갔다

의도한 게 아니다. `git add -A` 로 묶으면서, 그동안 **일부러 커밋 안 하고 있던** 다목표 주행
패치가 함께 올라갔다. 돌려놓으려 했으나 이력 고쳐쓰기는 막혀 있어 그대로 둔다 — 대신 무엇이
들어갔는지 여기 적어 둔다. (R&D 브랜치이고, 위 3절의 (B)가 바로 이 파일이라 랩탑이 이어서
고치기에는 오히려 낫다. 다만 **검증 브랜치로 옮기지 말 것.**)

그 패치가 담은 것:

- `_yaw_of(waypoint)` — 사원수에서 yaw(도) 를 낸다
- 홈 복귀가 `yaw_deg` 를 같이 보낸다. `home_root`(루트 자리) 를 `home`(팔 베이스) 보다 먼저 쓴다
- `_load_patrol_route` 가 yaml 의 `routes:` 를 `self._routes` 로 읽는다
- 서가 목표에서 `to_<shelf_id>` 경로를 쓰고, 그 앞에 서 있으면 주행을 생략한다
  (지금은 `route is not self.patrol_route` 로 shelf_01 을 빼 둔 상태 — 3절 (B)에서 뺄 가드)
- 홈 목표에서 `home_from_<shelf_id>` 를 `pre_route` 로 먼저 돌린다

검증 상태: 서가 A 9/9 회귀 통과, 서가 B 2권 삽입 성공(-1.2 / -2.6 mm, 겹침 없음),
복귀 자리·각도는 출발과 매번 동일. 네 권 완주는 아직 없다.
