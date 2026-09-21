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
         {"type": "goto",   "x": 1.0, "y": 2.0}
         {"type": "cancel"}
    발행 /navigation/sim/state     std_msgs/String (JSON)
         {"status": "running"|"succeeded"|"failed"|"idle",
          "leg": 2, "legs": 5, "remaining": 3.21, "pose": [x, y, yaw]}
"""

import json
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
RETURN_SNAP_M = 0.30         # 이 안쪽이면 '제자리로 돌아온 것' 으로 보고 팔 베이스를 맞춘다
SETTLE_STEPS = 60            # 도착 뒤 기다리는 스텝 수 (그 사이에 여러 번 나눠 맞춘다)
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

        # **팔 베이스의 출발 자리를 기억해 둔다.** 파지·삽입 좌표는 모두 팔 기준이라,
        # 돌아왔을 때 맞아야 하는 것은 루트 XForm 이 아니라 **팔 베이스**다.
        # 루트를 순간이동시켜도 관절로 붙은 몸통은 그대로 따라오지 않아, 한 바퀴 뒤
        # 둘 사이가 벌어진다 (2026-09-21: 루트는 제자리인데 꽂는 x 가 8.8 cm 밀렸다).
        self.arm_base = SingleXFormPrim(f"{BOT.root}/{BOT.base_link}")
        _hp, _hq = self.arm_base.get_world_pose()
        self.home_arm = np.asarray(_hp, float).copy()
        self.home_arm_q = np.asarray(_hq, float).copy()

        p, _ = self.root.get_world_pose()
        self.say(f"주행 실행기 준비: {BOT.root} 현재 위치 "
                 f"({float(p[0]):+.3f}, {float(p[1]):+.3f}), 속도 {self.speed} m/s")
        self.say(f"  팔 베이스 출발 자리 ({self.home_arm[0]:+.4f}, {self.home_arm[1]:+.4f}) "
                 f"— 돌아왔을 때 여기로 맞춘다")

    # ---------------------------------------------------------------- 발행
    def publish(self, **extra):
        p, _ = self.root.get_world_pose()
        state = {"status": self.status, "legs": self.legs,
                 "leg": self.legs - len(self.route),
                 "pose": [round(float(p[0]), 4), round(float(p[1]), 4)],
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
        if kind == "cancel":
            self.route = []
            self.status = "idle"
            self.say("주행 취소")
            self.publish(message="취소")
            return
        if kind == "goto":
            route = [[float(cmd["x"]), float(cmd["y"])]]
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
        off = self.home_arm[:2] - cur_p[:2]
        drift = float(np.hypot(off[0], off[1]))
        dyaw = _yaw(self.home_arm_q) - _yaw(np.asarray(cur_q, float))
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        if drift < REALIGN_DONE_M and abs(dyaw) < 1e-4:
            return
        if drift > RETURN_SNAP_M:
            self.say(f"[주행] 팔 베이스가 출발 자리에서 {drift*100:.1f}cm 떨어져 있다 "
                     f"— {RETURN_SNAP_M*100:.0f}cm 를 넘어 손대지 않는다 (제자리 복귀가 아니다)")
            return

        # **자세까지 되돌린다.** 위치만 맞추면 팔 기준 좌표계가 돌아간 채로 남아,
        # 칸마다 다른 만큼 어긋난다 (2026-09-21 실측: 책들은 월드에서 y 가 모두 같은데
        # 팔 기준 y 는 0.0916→0.1030 으로 벌어졌다 = 팔 베이스가 4.3° 돌아감).
        # 루트를 dyaw 만큼 돌린 뒤, 돌아간 상태에서 남는 위치 차이를 다시 메운다.
        self._snapshot("보정전")
        root_p, root_q = self.root.get_world_pose()
        root_p = np.asarray(root_p, float)
        root_q = np.asarray(root_q, float)
        if abs(dyaw) > 1e-4:
            self.root.set_world_pose(root_p, _quat_mul(_yaw_quat(dyaw), root_q))
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
        self.root.set_world_pose(root_p, np.asarray(root_q, float))
        self.say(f"[주행] 팔 베이스 복귀 보정: 위치 {drift*100:.1f}cm, "
                 f"자세 {math.degrees(dyaw):+.2f}° — 파지 기준을 출발 때와 같게 맞췄다")
        self._snapshot("보정직후")

    # ---------------------------------------------------------------- 매 스텝
    def spin(self):
        """한 시뮬 스텝. **world.step() 은 부르지 않는다.**"""
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.0)
        while self.inbox:
            self.handle(self.inbox.pop(0))

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
            if self.settle > 0 and self.settle % REALIGN_EVERY == 0:
                self._realign_arm_base()
            if self.settle <= 0:
                # 보정 뒤 남은 스텝 동안 follow_tray 가 따라왔는지 — '보정직후' 와 비교한다
                self._snapshot("도착시")
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
            self.root.set_world_pose(pos, quat)
            self.route.pop(0)
            done = self.legs - len(self.route)
            if not self.route:
                # **바로 '도착' 이라고 하지 않는다.** 루트를 방금 옮겼을 뿐이라
                # 관절로 매달린 몸통(=팔 베이스)은 아직 따라오지 않았다. 지금 읽으면
                # 낡은 값으로 보정하게 된다. 몇 스텝 가라앉힌 뒤 맞추고 알린다 —
                # 파지는 '도착' 을 보고 시작하므로 보정이 먼저 끝나야 한다.
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
        self.root.set_world_pose(pos, quat)

        # 1초에 한 번쯤 상태를 낸다 (매 스텝 내면 토픽이 넘친다)
        now = time.time()
        if now - self._last_report > 1.0:
            self._last_report = now
            self.publish(remaining=round(dist, 3))
