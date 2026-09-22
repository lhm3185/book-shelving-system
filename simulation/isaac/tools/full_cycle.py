"""전체 순환 — 트레이 안착 → 주행 → 파지 → 반납 → 복귀.

    ROS_DOMAIN_ID=130 python3 full_cycle.py
    ROS_DOMAIN_ID=130 python3 full_cycle.py --speed 0.6 --no-return

트레이 안착(①)은 **시뮬이 알아서** 한다 (`SIM_TRAY_DELIVERY`). 이 도구는 그 뒤를 잇는다:

    ② 파지 장소로 주행      /navigation/sim/command
    ③ 비전 좌표로 파지      pick_from_vision 과 같은 경로
    ④ 반납 (같은 작업에 포함)
    ⑤ 홈(반납기 앞)으로 복귀

**홈과 출발점은 같은 자리다.** 반납기 앞에서 시작해 반납기 앞으로 돌아온다.
출발 자리는 시뮬에게 물어보지 않고 **주행 상태 토픽의 첫 보고**에서 읽는다 —
레벨이 바뀌어도 따라간다.
"""
import argparse
import json
import os
import math
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#: 파지·반납을 하는 자리 (월드). 여기 서면 검증된 팔 기준 좌표가 그대로 맞는다
#: **검증된 거리를 맞춘 자리.** 서가 앞면은 y = −2.575 이고, 팔 기준 서가 앞면은
#: `scan_planner.SHELF_FACE_Y = 0.749` 로 검증돼 있다. 팔 베이스는 루트보다 +0.30 앞(팔 +X)에
#: 있으므로, 루트 y = −2.575 − 0.749 + 0.30 = **−3.024**... 가 아니라 팔 베이스 y 가
#: −2.575 − 0.749 = −3.324 여야 하고, 루트는 거기서 팔 오프셋을 뺀 자리다.
#: 예전 값 (2.535, −3.019) 은 팔 베이스–서가 앞면이 0.444 m 로 **30.5 cm 너무 가까웠다**
#: (2026-09-22 실측). 가까우면 팔이 접힐 공간이 없어 책이 선반에 부딪히고,
#: 그 충돌이 키네마틱 책을 통해 로봇을 들어 올린다.
#: **(2026-09-23 야간 정정) 위 계산은 영상 없이 한 것이고 틀렸다.** −3.324 로 옮기면서 삽입 목표
#: (pick_from_vision --goal-y 0.5495, 팔 기준)를 같이 고치지 않아 책을 서가 앞 약 20 cm 허공에서
#: 놓았다(v05: 놓기 직전 책 y −2.836, 서가 앞면 −2.575). 검증된 쌍 −3.019 ↔ GOAL_Y 0.5495 로 되돌린다.
#: 로봇을 옮기려면 GOAL_Y = 0.5495 + (−3.019 − PICK_Y) 로 같이 옮길 것 (야간 최선: −3.049 / 0.5795).
PICK_SPOT = (2.535, float(os.environ.get("SIM_PICK_Y", "-3.019")))
#: 거기서 바라볼 방향 (월드 yaw, 도).
#: 팔 +Y 축이 서가 앞면을 향해야 한다. 팔 +Y = 루트 yaw + 90° 이고 서가 앞면 법선은 +Y 이므로
#: **루트 yaw = 0°** 이다 (2026-09-22 실측: 레벨 시작 yaw 90°, 팔 +Y 는 -180° 를 보고 있었다).
PICK_YAW_DEG = 0.0


class Cycle(Node):
    """순환 한 바퀴를 돌린다. 각 단계는 **앞 단계가 끝나야** 시작한다."""

    def __init__(self, args):
        super().__init__("full_cycle")
        self.args = args
        self.nav_pub = self.create_publisher(String, "/navigation/sim/command", 10)
        self.nav_state = []
        self.create_subscription(String, "/navigation/sim/state",
                                 lambda m: self.nav_state.append(m.data), 20)
        self.home = None          # 출발 자리 — 첫 주행 보고에서 읽는다

    # ------------------------------------------------------------------ 도움
    def wait_for(self, pub, seconds=15.0):
        start = self.get_clock().now()
        while pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > seconds:
                return False
        return True

    def drive_to(self, x, y, speed, label, timeout=180.0, yaw_deg=None):
        """한 점으로 주행하고 도착까지 기다린다. `yaw_deg` 를 주면 도착해서 그 방향으로 돈다."""
        self.nav_state.clear()
        _cmd = {"type": "goto", "x": float(x), "y": float(y), "speed": float(speed)}
        if yaw_deg is not None:
            _cmd["yaw_deg"] = float(yaw_deg)
        self.nav_pub.publish(String(data=json.dumps(_cmd)))
        print(f"[{label}] ({x:+.3f}, {y:+.3f}) 로 주행")
        start = self.get_clock().now()
        last = None
        while (self.get_clock().now() - start).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            while self.nav_state:
                st = json.loads(self.nav_state.pop(0))
                if self.home is None and st.get("pose"):
                    self.home = tuple(st["pose"][:2])
                    print(f"       출발 자리 기억: ({self.home[0]:+.3f}, {self.home[1]:+.3f})")
                key = (st.get("status"), st.get("message"))
                if key != last:
                    last = key
                    p = st.get("pose", [0, 0])
                    extra = f" 남은 {st['remaining']:.2f}m" if "remaining" in st else ""
                    print(f"       {st.get('status'):10s} ({p[0]:+.2f}, {p[1]:+.2f}){extra}"
                          f"  {st.get('message', '')}")
                if st.get("status") == "succeeded":
                    return True
                if st.get("status") == "failed":
                    print(f"       **주행 실패** {st.get('message', '')}")
                    return False
        print("       **주행 시간 초과**")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.6, help="주행 속도 (m/s)")
    ap.add_argument("--pick-x", type=float, default=PICK_SPOT[0])
    ap.add_argument("--pick-y", type=float, default=PICK_SPOT[1])
    ap.add_argument("--pick-yaw", type=float, default=PICK_YAW_DEG,
                    help="파지 자리에서 바라볼 월드 yaw (도)")
    ap.add_argument("--no-return", action="store_true", help="반납 뒤 복귀하지 않는다")
    ap.add_argument("--no-pick", action="store_true", help="주행만 본다 (파지 건너뜀)")
    ap.add_argument("--settle", type=float, default=2.0, help="도착 뒤 기다릴 시간 (초)")
    a = ap.parse_args()

    rclpy.init()
    c = Cycle(a)
    if not c.wait_for(c.nav_pub):
        print("**시뮬이 안 보인다** — Isaac 이 떠 있는지, 도메인이 같은지 볼 것")
        return 2

    t0 = time.time()
    print("=" * 60)
    print("① 트레이 안착은 시뮬이 시작할 때 이미 끝난다 (SIM_TRAY_DELIVERY)")

    print("=" * 60)
    if not c.drive_to(a.pick_x, a.pick_y, a.speed, "② 주행", yaw_deg=a.pick_yaw):
        return 1
    # **도착 직후 바로 집지 않는다.** 복귀 보정과 트레이 추종이 자리를 잡을 시간을 준다
    for _ in range(int(a.settle * 10)):
        rclpy.spin_once(c, timeout_sec=0.1)

    ok = True
    if not a.no_pick:
        print("=" * 60)
        print("③④ 비전 좌표로 파지·반납")
        # **출발 자리를 노드 밖에 보관한다.** 아래에서 노드를 껐다 다시 만들면
        # c.home 이 None 이 되고, 그 상태로는 ⑤ 복귀가 '출발 자리를 못 읽었다' 로
        # 건너뛴다 (2026-09-23 take2 에서 확인). 주행 보고는 움직일 때만 오므로
        # 새 노드는 이 값을 다시 받지 못한다.
        _home = c.home
        c.destroy_node()
        rclpy.shutdown()
        import subprocess
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        r = subprocess.run([sys.executable, os.path.join(here, "pick_from_vision.py")])
        ok = r.returncode == 0
        print("       " + ("파지·반납 성공" if ok else f"**파지 실패 (코드 {r.returncode})**"))
        rclpy.init()
        c = Cycle(a)
        c.wait_for(c.nav_pub)
        if c.home is None:
            c.home = _home

    if not a.no_return:
        print("=" * 60)
        home = c.home or (0.0, 0.0)
        if c.home is None:
            print("       출발 자리를 못 읽었다 — 복귀를 건너뛴다")
        else:
            ok = c.drive_to(home[0], home[1], a.speed, "⑤ 복귀") and ok

    print("=" * 60)
    print(f"{'**한 바퀴 끝**' if ok else '**중간에 실패했다**'}  ({time.time() - t0:.0f}초)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
