# 새 반납기 — 이름·자리·회전 여유 다시 잼 (09/29 02:10, USD 실측)

레벨 `~/levels/Final_Level_Shaded/Final_Level_Shaded_robot.usdc` · `UsdGeom.BBoxCache` (default+render, extentsHint)

## 1. Prim 이름이 바뀌었다
```
옛 레벨  /World/return_machine
새 레벨  /World/return_machine_final          ← 이것 때문에 시연 로그에 `선택 Prim 없음: 무인반납기` 가 찍혔다
```
`world_loader.py` 의 `OPTIONAL_PRIMS` 는 **있는지만 알리는 목록**이고 쓰는 곳이 없어서 동작에는 영향이 없었다.
그래도 시연 로그에 남으면 오해를 사므로 서가·책과 같은 방식(둘 중 하나)으로 고쳤다.

## 2. AABB (월드)
```
/World/return_machine_final   x +5.419 ~ +7.518 · y -6.103 ~ -5.228 · z -0.011 ~ +1.726
                              크기 2.099 x 0.875 x 1.736 m
/World/tray_books             x +5.602 ~ +6.048 · y -5.828 ~ -5.491 · z +0.354 ~ +0.558
```
트레이 책 다섯 권이 반납기 x 범위 안(5.602~6.048)에 얹혀 있다 — 데크 위에 제대로 올라가 있다는 뜻이다.

## 3. 회전 여유 — 옛 값과 나란히
카트 대각 반지름 622 mm (Ridgeback 0.96 x 0.79) 기준. `navigation_executor.py` 의 상수와 같은 값을 썼다.
```
                      반납기까지      여유(= 거리 − 622)      옛 레벨 기록
홈 자리 (+4.986, -5.607)    433 mm      -189 mm  ✗           431 mm (같다, 오차 2 mm)
회전 자리 (+4.660, -5.228)  759 mm      +137 mm  ✓           775 mm → +153 mm
```
- **홈 자리에서 제자리 회전하면 189 mm 긁는다** — 09/28 에 도윤님이 육안으로 두 번 지적하신 그대로다.
  `SIM_TURN_STANDOFF_M=0.50` 이 그것을 막고 있고, **새 레벨에서도 그대로 유효**하다.
- 새 레벨의 여유는 **137 mm** 로 옛 레벨(153 mm)보다 **16 mm 얇다**. 반납기가 그만큼 앞으로 나왔다.
  통과 여유이지만 `SIM_TURN_STANDOFF_M` 을 0.50 아래로 내리면 안 된다 (0.35 면 옛 레벨에서도 50 mm 뿐이었다).
- 홈 자리 거리가 431 → 433 mm 로 사실상 같다 — **새 반납기는 옛 것과 거의 같은 자리에 있다.**

## 4. 재는 법 (되풀이용)
```bash
L=~/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
cd ~/isaacsim && PYTHONPATH=$L LD_LIBRARY_PATH=$L/bin ./python.sh <스크립트> <레벨.usdc> <x> <y>
```
맨 `python.sh` 에는 `pxr` 가 없다. 위처럼 `omni.usd.libs` 를 붙이면 Isaac 을 안 띄우고도 잰다.
단, 그러면 옴니버스 리졸버가 없어 **로봇(원격 참조)은 AABB 가 안 나온다** — 로봇 관련 치수는 Isaac 안에서 재야 한다.
