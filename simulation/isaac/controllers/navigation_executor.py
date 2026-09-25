"""AMR 주행 실행기 — 받은 waypoint 를 따라 로봇 몸통을 옮긴다.

설계 원칙 (로봇팔 실행기와 같다)
- **`world.step()` 을 부르지 않는다.** 스텝은 로봇팔 실행기(또는 메인 루프)가 한 번만 부른다.
  여기서는 매 스텝 **자세만** 갱신한다. 둘 다 부르면 시뮬이 두 배로 돈다.
- **블로킹하지 않는다.** 도착할 때까지 기다리는 함수를 만들지 않는다.
- 제한 시간은 **스텝 수**로 센다 (벽시계로 재면 렌더가 느린 PC 에서 멀쩡한 주행이 실패한다).

## 1단계: 회전 없이 옮긴다

Ridgeback 은 전방향(홀로노믹)이라 몸통을 돌리지 않고 옆으로도 간다.
**yaw 를 고정한 채 평행이동만** 한다 — 로봇팔과 트레이가 같은 방향을 유지하므로,
한 바퀴 돌아온 뒤 곧바로 파지할 수 있다. 회전은 이것이 되고 나서 붙인다.

## 통신

    수신 /navigation/sim/command   std_msgs/String (JSON)
         {"type": "patrol", "route": [[x, y], ...], "speed": 0.4}
         {"type": "goto",   "x": 1.0, "y": 2.0, "yaw_deg": 0.0}
         {"type": "cancel"}
         `yaw_deg` 를 주면 도착해서 그 방향으로 돈다 (없으면 방향을 건드리지 않는다).
    발행 /navigation/sim/state     std_msgs/String (JSON)
         {"status": "running"|"succeeded"|"failed"|"idle",
          "leg": 2, "legs": 5, "remaining": 3.21, "pose": [x, y, yaw_deg]}
         **pose 의 yaw 는 도(度)다** — 명령의 `yaw_deg` 와 같은 단위라 그대로 되돌려
         줄 수 있다. 예전에는 pose 에 x, y 만 실려서 출발 방향을 아무도 몰랐고,
         복귀가 자리만 맞추고 방향은 틀린 채 끝났다 (2026-09-23 수정).
"""

import json
from nav_mode import read_nav_mode, teleport_allowed
import math
import os
import time

import numpy as np

from isaacsim.core.prims import SingleXFormPrim
from std_msgs.msg import String

from robot_profiles import profile

BOT = profile()

ARRIVE_TOL_M = 0.05          # 이 안에 들면 그 점은 도착으로 본다
DEFAULT_SPEED = 0.4          # m/s. Nav2 기본 속도대
# 보정을 포기하는 문턱. **루트 대비 처짐**을 재므로 목적지와는 무관하다 —
# 예전에는 '출발 자리로 돌아왔나' 를 재느라 편도 주행에서 3.5 m 로 오판했다.
# 실측 처짐은 주행 거리·속도에 따라 수십 cm 까지 나온다 (v5, 3.56 m 주행에서 67.8 cm).
# 이보다 크면 처짐이 아니라 무언가 잘못된 것이다.
RETURN_SNAP_M = float(os.environ.get("SIM_RETURN_SNAP_M", "1.5"))
SETTLE_STEPS = 60            # 도착 뒤 기다리는 스텝 수
#: 도착 뒤 팔 베이스를 맞추는 보정. **기본 꺼짐.**
#: 9/21 순회(제자리 복귀)를 위해 만든 것인데, 편도 주행에서는 해롭다 —
#: 트레이를 그 자리에 고정해 버려 로봇만 떠나고 트레이가 남는다 (2026-09-22 실측).
#: 도착 뒤에는 `refresh_base()` 가 실제 팔 베이스를 다시 읽으므로 파지 좌표는 맞는다.
RETURN_CORRECTION = os.environ.get("SIM_RETURN_CORRECTION", "0") != "0"
MAX_YAW_FIX = math.radians(30)   # 이보다 큰 자세 차이는 '처짐' 이 아니라 측정 오류로 본다
#: 도착해서 방향을 맞추는 데 쓰는 시간 (초). 트레이가 튕기지 않게 부드럽게 돈다
TURN_S = float(os.environ.get("SIM_TURN_S", "3.0"))
DIAG = os.environ.get("SIM_DIAG_M406") == "1"   # 계측만 켠다 — 동작은 바뀌지 않는다
REALIGN_EVERY = 12           # 이 스텝마다 남은 차이를 다시 잰다 → 60/12 = 5회
REALIGN_DONE_M = 0.001       # 이 안쪽이면 맞은 것으로 본다


def _yaw(q):
    """Isaac 쿼터니언 (w, x, y, z) 에서 yaw(rad) 를 뽑는다."""
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _yaw_quat(yaw):
    """z 축 회전만 담은 쿼터니언 (w, x, y, z)."""
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def _quat_mul(a, b):
    """쿼터니언 곱 a*b — 둘 다 (w, x, y, z)."""
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


class NavigationExecutor:
    """waypoint 를 따라 로봇 몸통을 옮긴다. 매 시뮬 스텝 `spin()` 이 불린다."""

    def __init__(self, scene, node, say,
                 command_topic="/navigation/sim/command",
                 state_topic="/navigation/sim/state",
                 speed=DEFAULT_SPEED):
        self.scene = scene
        self.node = node
        self.say = say
        self.speed = float(speed)
        self.root = SingleXFormPrim(BOT.root)
        # **루트 XForm 을 옮긴다.** articulation 자세를 대신 설정해 봤더니(2026-09-22)
        # 루트 XForm 이 그대로 있어 **도착 판정과 트레이 추종이 둘 다 깨졌다** —
        # 로봇은 움직이는데 시뮬은 "안 움직였다" 고 보고, 트레이는 제자리에 남는다.

        self.inbox = []
        node.create_subscription(String, command_topic, lambda m: self.inbox.append(m.data), 10)
        self.pub = node.create_publisher(String, state_topic, 10)

        self.route = []          # 남은 목표들 [(x, y), ...]
        self.legs = 0            # 전체 구간 수
        self.status = "idle"
        self.steps = 0
        self.limit = 0
        self.settle = 0
        self.debug = os.environ.get("SIM_NAV_DEBUG", "1") != "0"
        self._last_report = 0.0

        # **루트와 팔 베이스의 제자리 관계를 기억해 둔다.** 파지·삽입 좌표는 모두 팔
        # 기준이라, 도착했을 때 맞아야 하는 것은 루트 XForm 이 아니라 **팔 베이스**다.
        # 루트를 순간이동시켜도 관절로 붙은 몸통은 그대로 따라오지 않아, 주행하면
        # 둘 사이가 벌어진다 (2026-09-21: 루트는 제자리인데 꽂는 x 가 8.8 cm 밀렸다).
        #
        # **출발 자리가 아니라 '관계' 를 기억한다.** 출발 자리로 잡으면 편도 주행에서
        # "3.5 m 떨어졌다" 고 오판한다 (2026-09-22 실측). 관계로 잡으면 어디로 가든 맞는다.
        self.arm_base = SingleXFormPrim(f"{BOT.root}/{BOT.base_link}")
        _hp, _hq = self.arm_base.get_world_pose()
        _rp, _rq = self.root.get_world_pose()
        self.home_arm = np.asarray(_hp, float).copy()
        self.home_arm_q = np.asarray(_hq, float).copy()
        #: 루트 → 팔 베이스 (월드 기준 차이). yaw 가 0 이 아니어도 주행은 평행이동만 하므로
        #: 이 차이는 그대로 유지돼야 한다
        self.arm_offset = self.home_arm - np.asarray(_rp, float)
        #: 루트 대비 팔 베이스의 **상대 yaw**. 월드 yaw 를 그대로 비교하면 로봇이
        #: 돌아 있는 레벨에서 140° 씩 틀어진 값이 나온다 (2026-09-22 실측, v5 는 yaw +90°)
        self.home_rel_yaw = _yaw(self.home_arm_q) - _yaw(np.asarray(_rq, float))
        #: **루트의 출발 yaw.** 복귀할 때 이 방향으로 되돌린다. 자리만 맞추고 방향을
        #: 안 돌리면 서가를 볼 때의 각도(0°)로 선 채 끝난다 — 처음 트레이를 받던
        #: 자세와 90° 어긋난다 (2026-09-24 도윤님 지적, 이 레벨은 출발 yaw +90°).
        self.home_root_yaw = _yaw(np.asarray(_rq, float))
        #: **루트의 출발 자리.** 복귀 목표는 이것이어야 한다. 주행기는 `goto` 의 x, y 에
        #: 루트를 올려놓는데, 여태 복귀 목표로 팔 베이스 자리(`home`)를 받았다. 둘은
        #: `arm_offset` 만큼(이 레벨은 y +0.300 m) 다르므로 로봇이 그만큼 못 미쳐 섰고,
        #: 데크가 트레이에서 30 cm 떨어져 트레이를 다시 받을 수 없었다
        #: (2026-09-24 도윤님 지적 → 로그로 확인: 루트 출발 -5.607, 복귀 도착 -5.307).
        self.home_root = np.asarray(_rp, float).copy()

        p, _ = self.root.get_world_pose()
        self.say(f"주행 실행기 준비: {BOT.root} 현재 위치 "
                 f"({float(p[0]):+.3f}, {float(p[1]):+.3f}), 속도 {self.speed} m/s")
        self.say(f"  팔 베이스 출발 자리 ({self.home_arm[0]:+.4f}, {self.home_arm[1]:+.4f}) "
                 f"yaw {math.degrees(self.home_root_yaw):+.2f}° — 돌아왔을 때 여기로 맞춘다")
        self.say(f"  루트 출발 자리 ({self.home_root[0]:+.4f}, {self.home_root[1]:+.4f}) "
                 f"— **복귀는 여기로 간다** (팔 베이스와 "
                 f"{float(np.hypot(*self.arm_offset[:2])):.3f} m 다르다)")

    # ---------------------------------------------------------------- 발행
    def publish(self, **extra):
        # **yaw 를 같이 실어 보낸다.** 계약(머리말)은 처음부터 `[x, y, yaw]` 였는데
        # 코드는 x, y 만 보내고 있었다. 그래서 full_cycle 이 **출발 방향을 알 길이
        # 없었고**, ⑤ 복귀가 자리로는 돌아오는데 방향은 파지할 때 각도(yaw 0) 그대로
        # 남았다 — "복귀했는데 처음 자세가 아니다" 의 원인이다 (2026-09-23).
        # `home` 은 출발 자리(팔 베이스) — 작업 끝에 nav_manager 가 여기로 되돌린다 (비전 브랜치).
        p, q = self.root.get_world_pose()
        state = {"status": self.status, "legs": self.legs,
                 # **[x, y, yaw]** — 머리말의 계약대로 셋을 다 싣는다. 예전에는 x, y 만
                 # 실어서 복귀가 자리만 맞고 방향은 파지할 때 각도로 남았다.
                 "home": [round(float(self.home_arm[0]), 4), round(float(self.home_arm[1]), 4),
                          round(math.degrees(self.home_root_yaw), 3)],
                 # **복귀 목표는 이쪽이다.** `home` 은 팔 베이스라 주행 목표로 쓰면 안 된다
                 "home_root": [round(float(self.home_root[0]), 4), round(float(self.home_root[1]), 4),
                               round(math.degrees(self.home_root_yaw), 3)],
                 "leg": self.legs - len(self.route),
                 "pose": [round(float(p[0]), 4), round(float(p[1]), 4),
                          round(math.degrees(_yaw(q)), 3)],
                 "stamp": time.time()}
        state.update(extra)
        self.pub.publish(String(data=json.dumps(state)))

    # ---------------------------------------------------------------- 명령
    def handle(self, text):
        try:
            cmd = json.loads(text)
        except (TypeError, ValueError):
            self.say(f"주행 명령 형식 오류: {text[:80]}")
            return
        kind = cmd.get("type")
        # **주행 모드 스위치** (nav_mode.py). nav2 모드면 방 단위 주행(goto/patrol)은 Nav2 몫이라
        # 여기서 거절한다 — 루트를 쓰면 OmniGraph 와 싸운다. 도킹 보정은 manipulation_executor 라 무관.
        mode, note = read_nav_mode()
        if note and not getattr(self, "_mode_noted", False):
            self._mode_noted = True
            self.say(f"[주행모드] {note}")
        if not teleport_allowed(mode, kind):
            self.say(f"[주행모드] SIM_NAV_MODE=nav2 — 순간이동 주행기는 {kind!r} 를 받지 않는다 "
                     f"(Nav2 가 /cmd_vel 로 굴린다). nav_manager 를 띄웠다면 그게 잘못이다")
            self.status = "failed"
            self.publish(message=f"nav2 모드: {kind} 거절")
            self.status = "idle"
            return
        if kind == "cancel":
            self.route = []
            self.status = "idle"
            self.say("주행 취소")
            self.publish(message="취소")
            return
        if kind == "goto":
            route = [[float(cmd["x"]), float(cmd["y"])]]
            # **도착해서 바라볼 방향** (월드 yaw, 도). 없으면 지금처럼 방향을 안 바꾼다.
            # 서가 앞면 법선이 ±Y 라, 팔 +Y(=루트 yaw+90°)가 서가를 보려면 루트 yaw=0° 다
            # (2026-09-22 실측: 레벨 시작 yaw 90°, 팔 +Y 는 -180° 를 보고 있었다).
            self.goal_yaw = cmd.get("yaw_deg")
        elif kind == "patrol":
            route = [[float(x), float(y)] for x, y in cmd.get("route", [])]
        else:
            self.say(f"모르는 주행 명령: {kind}")
            return
        if not route:
            self.say("주행 경로가 비어 있다")
            return

        self.speed = float(cmd.get("speed", self.speed))
        self.route = route
        self.legs = len(route)
        self.status = "running"
        self.steps = 0
        # 제한 시간: 전체 거리를 속도로 나눈 값의 3배 + 여유. 안전장치이지 성능 목표가 아니다
        p, _ = self.root.get_world_pose()
        pts = [[float(p[0]), float(p[1])]] + route
        total = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        dt = max(1e-4, float(self.scene.world.get_physics_dt()))
        self.limit = int((total / max(self.speed, 1e-3)) * 3.0 / dt) + int(10.0 / dt)
        self.say(f"주행 시작: {self.legs}개 구간, 총 {total:.2f} m, "
                 f"속도 {self.speed} m/s (제한 {self.limit} 스텝)")
        self.publish(message="시작", total_m=round(total, 3))

    # ---------------------------------------------------------------- 루트 쓰기
    def _write_root(self, pos, quat, why):
        """루트 XForm 자세를 쓰는 **유일한 통로**.

        작업(파지·운반) 중에 루트를 옮기면 손 안의 책이 밀린다. 그런 쓰기가
        실제로 일어나는지 세려면 쓰기가 한 곳을 지나야 한다 (계측 H-a).
        """
        if getattr(self.scene, "job_active", False):
            self.scene.root_writes_after_job = getattr(
                self.scene, "root_writes_after_job", 0) + 1
            if DIAG:
                self.say(f"[DIAG] 작업 중 루트 쓰기: {why} "
                         f"(누적 {self.scene.root_writes_after_job})")
        self.root.set_world_pose(pos, quat)

    # ---------------------------------------------------------------- 진단
    def _snapshot(self, tag):
        """트레이와 책이 팔 기준 어디에 있는지 찍는다 (복귀 보정 추적용).

        왜: 보정으로 팔 베이스는 출발 자리로 돌아가는데, 그 뒤에도 책이 기준보다
        2.9 cm 남아 운반 중 M406 이 났다. 트레이 추종(follow_tray)이 보정을
        따라오는지 보려면 **보정 앞뒤로 같은 값을 찍어 비교**해야 한다.
        SIM_NAV_DEBUG=0 으로 끌 수 있다.
        """
        if not self.debug:
            return
        scene = self.scene
        try:
            tray_w = np.asarray(SingleXFormPrim(scene.tray).get_world_pose()[0], float)
            line = f"[추적:{tag}] 트레이 월드 {np.round(tray_w, 4).tolist()}"
            if hasattr(scene, "to_arm"):
                line += f" 팔기준 {np.round(scene.to_arm(tray_w), 4).tolist()}"
            self.say(line)
            for book in list(getattr(scene, "books", []))[:3]:
                c = scene.center(book)
                arm = scene.to_arm(c) if hasattr(scene, "to_arm") else None
                self.say(f"[추적:{tag}]   {book.rsplit('/', 1)[-1]} 월드 "
                         f"{np.round(c, 4).tolist()}"
                         + (f" 팔기준 {np.round(arm, 4).tolist()}" if arm is not None else ""))
        except Exception as exc:      # noqa: BLE001 - 진단이 주행을 막으면 안 된다
            self.say(f"[추적:{tag}] 못 찍었다: {type(exc).__name__}: {exc}")

    def _log_residual(self, when):
        """**팔 베이스**의 출발 대비 잔차를 적는다 (루트가 아니다).

        두 번 찍는다: 보정 직후와 정착 뒤. 보정 직후 값은 물리가 따라오기 전의
        USD 값일 수 있어 0 에 가깝게 나와도 믿기 어렵다. 파지가 실제로 보는 것은
        **정착 뒤** 값이다.
        """
        if not DIAG:
            return
        p, q = self.arm_base.get_world_pose()
        root_now, _ = self.root.get_world_pose()
        res = (np.asarray(root_now, float) + self.arm_offset)[:2] - np.asarray(p, float)[:2]
        _, _rq_now = self.root.get_world_pose()
        ryaw = self.home_rel_yaw - (_yaw(np.asarray(q, float)) - _yaw(np.asarray(_rq_now, float)))
        ryaw = math.atan2(math.sin(ryaw), math.cos(ryaw))
        self.say(f"[DIAG] return_residual({when})=[{float(np.hypot(*res)):.5f}, "
                 f"{math.degrees(ryaw):+.3f}]")

    # ---------------------------------------------------------------- 복귀 보정
    def _realign_arm_base(self):
        """제자리로 돌아왔으면 **팔 베이스**를 출발 자리에 정확히 맞춘다.

        왜 필요한가: 파지·삽입 좌표는 전부 팔 기준이고, 그 좌표를 월드로 바꾸는 기준이
        팔 베이스다. 루트 XForm 을 목표점에 정확히 올려놔도, 관절로 매달린 몸통은
        그만큼 따라오지 않아 팔 베이스가 밀린 채 남는다. 그 밀림이 그대로
        파지·삽입 오차가 된다 (2026-09-21 실측: 루트는 제자리, 꽂는 x 는 8.8 cm 밀림
        → 책이 선반에 못 들어가고 바닥으로 떨어졌다).

        **출발 자리 근처로 돌아온 경우에만** 맞춘다. 다른 곳으로 가는 주행까지
        출발점으로 당기면 그게 더 큰 사고다.
        """
        cur_p, cur_q = self.arm_base.get_world_pose()
        cur_p = np.asarray(cur_p, float)
        # **도착점 기준으로 있어야 할 자리**를 만든다 (출발 자리가 아니다).
        # 주행은 평행이동만 하므로 루트 → 팔 베이스 차이는 그대로여야 한다
        root_now, _ = self.root.get_world_pose()
        want = np.asarray(root_now, float) + self.arm_offset
        off = want[:2] - cur_p[:2]
        drift = float(np.hypot(off[0], off[1]))
        # **상대 yaw 로 비교한다** (루트 기준). 절대 yaw 를 쓰면 로봇이 돌아 있는 레벨에서
        # 통째로 틀어진 값이 나온다 — v5(yaw +90°)에서 140° 를 돌리려 했다
        _, root_q_now = self.root.get_world_pose()
        rel_now = _yaw(np.asarray(cur_q, float)) - _yaw(np.asarray(root_q_now, float))
        dyaw = self.home_rel_yaw - rel_now
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        if abs(dyaw) > MAX_YAW_FIX:
            # 이만큼 돌았다는 것은 처짐이 아니라 **재는 방법이 틀린 것**이다.
            # 돌려 버리면 로봇을 엉뚱한 방향으로 세운다 — 재지 말고 알린다
            self.say(f"[주행] 자세 차이가 {math.degrees(dyaw):+.1f}° 로 너무 크다 "
                     f"— 자세는 보정하지 않는다 (위치만 맞춘다)")
            dyaw = 0.0
        if drift < REALIGN_DONE_M and abs(dyaw) < 1e-4:
            return
        if drift > RETURN_SNAP_M:
            # **조용히 넘어가지 않는다.** 보정을 건너뛰면 좌표계가 틀어진 채 파지에 들어가고,
            # 2026-09-21 의 406 기제로 그대로 돌아간다 — 주행은 "도착" 이라 보고하고
            # 엉뚱한 단계에서 터진다. 처짐은 주행 **시간**에 비례하므로(≈0.155 cm/s,
            # 9/22 실측: 0.6 m/s 8.1 cm / 0.3 m/s 17.1 cm) 느린 주행이나 긴 경로에서
            # 이 문턱을 넘을 수 있다. 0.15 m/s 면 34 cm 로 넘는다
            self.status = "failed"
            msg = (f"팔 베이스가 있어야 할 자리에서 {drift*100:.1f}cm 떨어져 있다 — "
                   f"{RETURN_SNAP_M*100:.0f}cm 를 넘어 보정하지 않는다. "
                   f"이대로 파지하면 좌표계가 틀어진 채 집는다")
            self.say(f"[주행] **{msg}**")
            self.publish(message=msg)
            return

        # **자세까지 되돌린다.** 위치만 맞추면 팔 기준 좌표계가 돌아간 채로 남아,
        # 칸마다 다른 만큼 어긋난다 (2026-09-21 실측: 책들은 월드에서 y 가 모두 같은데
        # 팔 기준 y 는 0.0916→0.1030 으로 벌어졌다 = 팔 베이스가 4.3° 돌아감).
        # 루트를 dyaw 만큼 돌린 뒤, 돌아간 상태에서 남는 위치 차이를 다시 메운다.
        self._snapshot("보정전")
        # **보정 전 트레이 자세를 붙잡아 둔다.** 보정은 루트를 옮기므로 트레이 앵커(차체)가
        # 함께 끌려가고, 추종이 트레이를 따라 옮겨 칸 중심에 있던 책이 밀린다.
        # 보정 전 트레이는 이미 출발 팔 기준 칸 중심에 있으니 그 자리에 두게 한다
        try:
            _tp, _tq = SingleXFormPrim(self.scene.tray).get_world_pose()
            self.scene.rebase_tray_to(_tp, _tq)
        except Exception as exc:      # noqa: BLE001 - 보정이 이것 때문에 멈추면 안 된다
            self.say(f"[주행] 트레이 자세를 못 붙잡았다: {type(exc).__name__}: {exc}")
        root_p, root_q = self.root.get_world_pose()
        root_p = np.asarray(root_p, float)
        root_q = np.asarray(root_q, float)
        if abs(dyaw) > 1e-4:
            self._write_root(root_p, _quat_mul(_yaw_quat(dyaw), root_q), "복귀 보정(자세)")
            # 회전은 루트를 중심으로 돌기 때문에 팔 베이스 위치가 함께 움직인다.
            # 남은 위치 차이는 회전을 반영해 다시 계산한다
            arm_off = cur_p[:2] - root_p[:2]
            c, sn = math.cos(dyaw), math.sin(dyaw)
            turned = np.array([c * arm_off[0] - sn * arm_off[1],
                               sn * arm_off[0] + c * arm_off[1]])
            off = self.home_arm[:2] - (root_p[:2] + turned)
            root_p, root_q = self.root.get_world_pose()
            root_p = np.asarray(root_p, float)
        root_p[0] += off[0]
        root_p[1] += off[1]
        self._write_root(root_p, np.asarray(root_q, float), "복귀 보정(위치)")
        self.say(f"[주행] 팔 베이스 복귀 보정: 위치 {drift*100:.1f}cm, "
                 f"자세 {math.degrees(dyaw):+.2f}° — 파지 기준을 출발 때와 같게 맞췄다")
        self._log_residual("보정직후")
        self._snapshot("보정직후")

    # ---------------------------------------------------------------- 매 스텝
    def spin(self):
        """한 시뮬 스텝. **world.step() 은 부르지 않는다.**"""
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.0)
        while self.inbox:
            self.handle(self.inbox.pop(0))

        # **이송이 끝나기 전에는 한 발짝도 움직이지 않는다.**
        # 트레이는 월드 좌표로 밀려가는 중인데 로봇이 떠나면 둘이 어긋나고,
        # 이송이 끝나는 순간 `rebase_tray_to` 가 그 어긋난 상대 위치를 **영구히 기록**한다
        # (2026-09-22 실측: 차이가 (+0.007,-0.052) → (+0.145,-0.198) 로 20 cm 튄 뒤 고정,
        #  트레이가 데크 모서리에 걸린 채 주행했다).
        # 녹화를 켜면 시뮬 시간이 실제보다 느려서 "충분히 기다렸다" 는 감이 틀린다 —
        # 시간으로 재지 말고 **상태로** 기다린다.
        if getattr(self.scene, "_deliver", None) and self.status == "running":
            self._deliver_waits = getattr(self, "_deliver_waits", 0) + 1
            if self._deliver_waits % 60 == 1:
                self.say("[주행] 트레이 이송이 끝나기를 기다린다")
                self.publish(message="이송 대기")
            return

        if self.status == "turning":
            # **부드럽게 돈다.** 한 번에 돌리면 트레이가 원심력처럼 튕긴다 —
            # 추종은 앵커 상대 자세를 따르므로 트레이·책은 같이 돌아간다.
            self._turn_t += 1
            u = min(1.0, self._turn_t / self._turn_n)
            u = u * u * (3.0 - 2.0 * u)
            d = (self._turn_to - self._turn_from + math.pi) % (2 * math.pi) - math.pi
            pos, _ = self.root.get_world_pose()
            self._write_root(pos, _yaw_quat(self._turn_from + d * u), "도착 회전")
            if u >= 1.0:
                self.goal_yaw = None
                self.status = "settling"
                self.settle = SETTLE_STEPS
                _p, _q = self.root.get_world_pose()
                self.say(f"[주행] 회전 끝 — **실측** yaw {math.degrees(_yaw(_q)):+.2f}° "
                         f"(목표 {math.degrees(self._turn_to):+.2f}°)")
            return

        if self.status == "settling":
            self.settle -= 1
            # 절반쯤에서 보정한다. **보정 뒤에도 스텝이 남아야 한다** — 트레이와 책은
            # follow_tray 가 매 스텝 따라 붙이는데, 보정하자마자 '도착' 을 알리면
            # 파지가 시작되며 job_active 로 추종이 멈춰, 트레이가 밀린 채 굳는다
            # (2026-09-21 실측: 책이 기준보다 2.9 cm 남아 운반 중 3.1 cm 미끄러짐).
            # **여러 번 나눠 맞춘다.** 트레이는 물리 링크(Cube/articulation_root)에 붙어 있고
            # 우리가 옮기는 것은 루트 XForm 이다. 둘은 강체가 아니라서 한 번에 계산한 보정이
            # 그대로 들어맞지 않는다. 매번 실제 팔 베이스를 다시 읽어 남은 차이만 줄이면
            # 몇 번 만에 수렴한다 (한 번만 맞췄을 때 2.9 cm 가 남았다).
            if RETURN_CORRECTION and self.settle > 0 and self.settle % REALIGN_EVERY == 0:
                self._realign_arm_base()
            if self.settle <= 0:
                # 보정 뒤 남은 스텝 동안 follow_tray 가 따라왔는지 — '보정직후' 와 비교한다
                self._snapshot("도착시")
                # **정착 뒤 잔차를 다시 잰다.** 보정 직후에 읽은 값은 물리가 따라오기 전
                # USD 값일 수 있다 (같은 이유로 '보정직후' 스냅샷이 '보정전' 과 같았다).
                # 파지가 실제로 보는 것은 이 값이다
                self._log_residual("정착후")
                self.status = "succeeded"
                p, _ = self.root.get_world_pose()
                self.say(f"[주행] 도착 ({float(p[0]):+.3f}, {float(p[1]):+.3f})")
                self.publish(message="도착")
            return

        if self.status != "running" or not self.route:
            return

        self.steps += 1
        if self.steps > self.limit:
            self.status = "failed"
            self.say(f"[주행] 제한 시간 초과 ({self.steps} 스텝) — 남은 구간 {len(self.route)}")
            self.publish(message="제한 시간 초과")
            self.route = []
            return

        pos, quat = self.root.get_world_pose()
        pos = np.asarray(pos, float)
        tx, ty = self.route[0]
        dx, dy = tx - pos[0], ty - pos[1]
        dist = math.hypot(dx, dy)

        if dist <= ARRIVE_TOL_M:
            # **목표점에 정확히 올려놓고 넘어간다.** 허용 오차 안이라고 그냥 멈추면
            # 최대 5 cm 가 남는데, 그 오차는 베이스 기준 좌표를 쓰는 로봇팔에
            # 그대로 전달돼 삽입을 깨뜨린다 (2026-09-21: 4.5 cm 남고 배치 검증 실패).
            # 여기서는 자세를 직접 쓰므로 정확히 맞추는 데 드는 비용이 없다.
            pos[0], pos[1] = tx, ty
            self._write_root(pos, quat, "경유점 정렬")
            self.route.pop(0)
            done = self.legs - len(self.route)
            if not self.route:
                # **바로 '도착' 이라고 하지 않는다.** 루트를 방금 옮겼을 뿐이라
                # 관절로 매달린 몸통(=팔 베이스)은 아직 따라오지 않았다. 지금 읽으면
                # 낡은 값으로 보정하게 된다. 몇 스텝 가라앉힌 뒤 맞추고 알린다 —
                # 파지는 '도착' 을 보고 시작하므로 보정이 먼저 끝나야 한다.
                if getattr(self, "goal_yaw", None) is not None:
                    self.status = "turning"
                    self._turn_from = _yaw(quat)
                    self._turn_to = math.radians(float(self.goal_yaw))
                    self._turn_n = max(1, int(TURN_S / max(1e-4, float(
                        self.scene.world.get_physics_dt()))))
                    self._turn_t = 0
                    self.say(f"[주행] 마지막 점 도달 — 이제 {math.degrees(self._turn_from):+.1f}° "
                             f"→ {self.goal_yaw:+.1f}° 로 돈다 ({TURN_S:.1f}초)")
                    return
                self.status = "settling"
                self.settle = SETTLE_STEPS
                self.say(f"[주행] 마지막 점 도달 ({done}/{self.legs}) "
                         f"({pos[0]:+.3f}, {pos[1]:+.3f}) — 자세가 가라앉기를 기다린다")
            else:
                self.say(f"[주행] 경유점 {done}/{self.legs} 통과 "
                         f"({pos[0]:+.3f}, {pos[1]:+.3f})")
                self.publish(message=f"경유점 {done}")
            return

        # **yaw 는 건드리지 않는다** (1단계). 평행이동만 한다
        dt = max(1e-4, float(self.scene.world.get_physics_dt()))
        step = min(self.speed * dt, dist)
        pos[0] += dx / dist * step
        pos[1] += dy / dist * step
        self._write_root(pos, quat, "주행")

        # 1초에 한 번쯤 상태를 낸다 (매 스텝 내면 토픽이 넘친다)
        now = time.time()
        if now - self._last_report > 1.0:
            self._last_report = now
            self.publish(remaining=round(dist, 3))
