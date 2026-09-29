# 프랍 레벨(v3) 첫 판 — 2026-09-29 23:06 · 데스크탑

레벨 `~/b1_level_props_collect_0929_v3/Final_Level_Shaded_robot.light1_props.usdc`
(USB 에서 복사, 313 MB · 파일 143개, `md5sum -c MD5SUMS` **142/142 OK**)
코드 `21b8b61` (detached) · 설정 `~/demo_env_props.sh` · `REC_EVERY=0`

## 두 판 다 4/4
```
합격  2306_props_v3   배치 4/4 · 중심 -3.5 ~ +0.8 mm · 기울기 +0.13 ~ +0.34° · 428 s   (무녹화)
합격  2320_props_rec  배치 4/4 · 중심 -3.3 ~ +0.5 mm · 기울기 +0.09 ~ +0.35° · 436 s   (토픽 녹화)
오류 코드 없음 — 네 권 모두 code=0(OK) · 겹침 없음 · 복귀 yaw +90.00°
d(reorient) 처짐 두 판 모두 0 mm
```

## 트레이 검출 — **바닥 채도 +20 %·벽 +15 % 뒤에도 여유가 늘었다**
```
권  정한 시각   좌표(팔기준)                신뢰도       관측
1   23:08:54   (-0.3617, -0.0030, +0.2118)  0.86~0.92   2개 (x 흔들림 0 mm)
2   23:10:59   (-0.3435, -0.1046, +0.2122)  0.66~0.68   2개 (0 mm)
3   23:13:53   (-0.3699, -0.1873, +0.1876)  0.59~0.65   2개 (3 mm)
4   23:15:57   (-0.3363, +0.1512, +0.2129)  0.70~0.81   2개 (3 mm)
```
3권째 c60 이 **0.59~0.65** — 교육장에서 본 0.56~0.66 과 같은 폭이고 문턱(0.55) 위다.
색을 올린 것이 검출을 깎지는 않았다. 두 판 다 같은 순서로 집었다.

## 새 경고 — 정적 로봇 둘, **무해**
```
[Warning] isaacsim.sensors.physics.plugin  No valid parent for /World/props/ridgeback_franka_0X/…   6회
[Warning] omni.hydra  Mesh '/World/props/ridgeback_franka_02/panda_link7/visuals/panda_…'          1회
```
프랍 로봇이 센서 prim 을 달고 있는데 관절체(articulation) 부모가 없어서 나는 것이다. 판에는 영향이 없다.
`[Error] omni.physx.tensors.plugin  Pattern '/World/ridgeback_franka/panda_hand/rsd455/RSD455' did not match any rigid bodies`
는 **새 것이 아니다** — 어제 판(`0648_4books_2`)에도 같은 줄이 있다.
코드가 쓰는 로봇 prim `/World/ridgeback_franka` 와 프랍 `/World/props/ridgeback_franka_01·02` 는 경로가 달라 겹치지 않는다.
실제 로봇 자리 `(+4.9859, -5.6067)` yaw +90° 도 그대로다(복귀 로그로 확인).

## 책상 — 경로에서 **3.2 m 넘게 떨어져 있다**
주행 경로(홈 → 서가B → 서가A → 홈)의 선분에서 각 프랍의 AABB 모서리까지 잰 최단거리:
```
chair_01   3,245 mm   ← 가장 가깝다
table_01   3,301 mm
chair_02   3,757 mm
그 밖 테이블·의자   4.8 ~ 7.0 m
정적 로봇 둘        9.1 m 이상
```
카트 대각 반지름 622 mm 를 빼도 **여유 2.6 m** 다. 닿을 여지가 없다.
**다만 그림으로는 못 보였다** — 손목 카메라는 주행 중 트레이를 내려다보고 있어서 책상이 화면에 안 들어온다
(`desk_*.png` 세 장이 그 증거다). 눈으로 보려면 뷰포트 녹화(`REC_EVERY`)를 켜야 하는데 그러면 시뮬이 느려진다.

## 녹화 — 토픽을 직접 받아서 (cv_bridge 안 씀)
새 도구 둘:
```
simulation/isaac/tools/topic_rec.py         토픽 여럿을 받는 즉시 PNG 로 + 받은 시각을 index.csv 에
simulation/isaac/tools/make_topic_clip.py   index.csv 로 실제 간격대로 이어 mp4 (고정 fps 출력)
```
구간은 로그 기준 **첫 `NAV_TO_SHELF -> DETECT_TARGET_SLOT` (23:21:48) ~ 두 번째 `PlaceBook 성공` (23:26:30)**,
즉 서가 B 의 스캔부터 두 권 삽입까지다.
```
파일                      장수    실제 길이   출력 길이   해상도    크기
shelfB_debug_rgb.mp4       33장   163.0 s     40.3 s     640x640   0.5 MB
shelfB_depth.mp4            8장    26.1 s     15.5 s     640x640   0.1 MB
shelfB_rgb.mp4          1,233장   286.2 s    286.3 s     640x640  11.9 MB   (원본)
shelfB_depth_raw.mp4    1,027장   286.5 s    286.5 s     640x640   3.0 MB   (원본)
```
**검출 화면(`/perception/debug_image`)은 33장뿐이다** — 검출이 도는 순간에만 발행되기 때문이다.
뎁스 디버그(`/perception/depth_debug_image`)는 8장으로 더 적다. 영상으로 쓰기엔 얇으니,
발표에는 **원본 `/rgb`(1,233장)** 쪽이 낫고 검출 화면은 스틸로 끼워 넣는 편이 낫겠다.
저장소에는 작은 둘만 넣었다(`rec_props/`). 원본 둘과 프레임 원본(1.4 GB)은 데스크탑 `~/b1_rec_0929/` 에 있다.

깊이는 **0 ~ 2 m 고정 범위**를 TURBO 컬러맵으로 칠했다(판마다 대비가 달라지지 않게). 값 없는 화소는 검정.
**그림으로 절대값을 읽지 말 것.**

## 토픽 구독이 시뮬을 늦췄나
네 토픽(검출 RGB·뎁스 디버그·원본 RGB·원본 뎁스)을 동시에 받았는데 **판 길이는 428 s → 436 s**, 8초(1.9%) 차이다.
뷰포트 녹화(2.5배 느려짐)와 달리 **토픽 구독은 사실상 공짜**다. 다음부터 이 방식을 쓰면 된다.
