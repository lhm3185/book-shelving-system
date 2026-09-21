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
import time

import numpy as np

from isaacsim.core.prims import SingleXFormPrim
from std_msgs.msg import String

from robot_profiles import profile

BOT = profile()

ARRIVE_TOL_M = 0.05          # 이 안에 들면 그 점은 도착으로 본다
DEFAULT_SPEED = 0.4          # m/s. Nav2 기본 속도대


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
        self._last_report = 0.0

        p, _ = self.root.get_world_pose()
        self.say(f"주행 실행기 준비: {BOT.root} 현재 위치 "
                 f"({float(p[0]):+.3f}, {float(p[1]):+.3f}), 속도 {self.speed} m/s")

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

    # ---------------------------------------------------------------- 매 스텝
    def spin(self):
        """한 시뮬 스텝. **world.step() 은 부르지 않는다.**"""
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.0)
        while self.inbox:
            self.handle(self.inbox.pop(0))

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
            self.route.pop(0)
            done = self.legs - len(self.route)
            if not self.route:
                self.status = "succeeded"
                self.say(f"[주행] 도착 ({done}/{self.legs}) "
                         f"({pos[0]:+.3f}, {pos[1]:+.3f})")
                self.publish(message="도착")
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
