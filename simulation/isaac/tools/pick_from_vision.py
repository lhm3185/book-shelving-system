"""비전이 본 책을 그대로 집게 한다 — `/perception/books` → `PlaceBook`.

## 무엇을 하나

1. `/perception/detect_request` 로 검출을 요청한다
2. `/perception/books` 로 오는 **신뢰도 1등 책의 좌표**를 받는다
   (비전이 `publish_best_only=true` 면 오는 것이 곧 1등이다)
3. 그 좌표를 `PlaceBook.grasp.top_center` 에 실어 보낸다
4. 로봇팔이 계약 검사(5절)를 통과시킨 뒤 집어서 꽂는다

**비전 좌표는 "보이는 윗면 중심"이다.** 로봇팔이 책 폭의 절반을 빼 AABB 중심으로
바꾼다 — 유도는 치수를 아는 쪽이 한다 (`grasp_planner.grasp_pick_center`).

    ROS_DOMAIN_ID=129 python3 pick_from_vision.py
    ROS_DOMAIN_ID=129 python3 pick_from_vision.py --goal-x -0.4297 --dry-run

`--dry-run` 은 **좌표만 확인하고 보내지 않는다.** 처음 볼 때 쓰면 좋다.
"""
import argparse
import os
import sys

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shelving_interfaces.action import DetectTargetSlot, PlaceBook
from std_msgs.msg import Bool

# 검증된 Franka 꽂을 좌표 (팔 기준). test_contract_coords.py 가 지키는 값이다
FRANKA_SLOTS = [-0.3497, -0.4297, -0.5097, -0.2697]
#: **규격이다. 실측이 아니다.** 2026-09-24 실측에서 손가락이 13.7 mm 에서
#: 멈췄다 — 실제 반두께가 13.7 mm 라는 뜻이고 두께는 27.4 mm 다. 규격보다
#: 7.8 mm 얇다. 그래서 파지 지령(13.6 mm)이 책에 닿지도 않아 조임량이 0.1 mm 다.
#: 다섯 권 실측이 들어오면 여기와 config/book_profiles.yaml 을 함께 고친다.
def _load_book(name='default'):
    """책 치수를 **조작 노드가 읽는 바로 그 파일**에서 읽는다.

    여기 상수로 적어 두면 갈라진다. 실제로 갈라져 있었다 — 2026-09-24 에 yaml 은
    실측으로 갱신됐는데(`width` 0.1631 → 0.1517) 이 파일은 v3 레벨 값에 멈춰
    있었다. 그리고 **410 높이 검사는 클라이언트가 보낸 치수를 쓰므로** 그 검사가
    옛 폭으로 돌았다. 둘 중 어느 것이 진짜인지 모르는 채로 두 값이 돌았다.

    설치본(`share/`)을 먼저 본다 — 노드가 그것을 읽기 때문이다. 저장소 파일과
    다르면 **말한다**: 빌드를 안 한 것이고, 파일값과 런타임값이 다르다는 뜻이다.
    """
    import yaml
    paths = []
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.append(os.path.join(get_package_share_directory('shelving_manipulation'),
                                  'config', 'book_profiles.yaml'))
    except Exception:      # noqa: BLE001 - 설치본이 없으면 저장소 것으로 간다
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.normpath(os.path.join(here, '..', '..', '..', 'ros2_ws', 'src',
                                         'shelving_manipulation', 'config',
                                         'book_profiles.yaml'))
    paths.append(repo)
    loaded = []
    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path, encoding='utf-8') as f:
            p = yaml.safe_load(f)['profiles'][name]
        loaded.append((path, {k: float(p[k]) for k in ('thickness', 'height', 'width')}))
    if not loaded:
        raise SystemExit(f'book_profiles.yaml 을 못 찾았다: {paths}')
    used_path, dims = loaded[0]
    print(f'책 치수 [{name}] {used_path}')
    print(f'  두께 {dims["thickness"]*1000:.1f} · 높이 {dims["height"]*1000:.1f} · '
          f'폭 {dims["width"]*1000:.1f} mm')
    for path, other in loaded[1:]:
        if other != dims:
            print(f'  **주의: {path} 의 값이 다르다** {other} — 빌드를 안 했다. '
                  f'노드는 설치본을 읽으므로 **파일값과 런타임값이 다르다**')
    return dims


BOOK = _load_book()

#: **짝 규칙의 기준점.** 베이스를 월드 y 로 Δ 옮기면 팔 기준 목표를 −Δ 옮겨야
#: 월드 삽입 지점이 고정된다. 이 두 값이 서로 다른 파일(full_cycle 의 PICK_SPOT,
#: 여기의 --goal-y)에 흩어져 있어서 **한쪽만 옮기는 사고가 실제로 났다**
#: (2026-09-22: PICK_Y 를 −3.019 → −3.324 로만 옮겨 책을 서가 앞 20 cm 허공에
#: 놓았다). 그래서 이제 GOAL_Y 를 PICK_Y 에서 **만든다** — 따로 적지 않는다.
PAIR_PICK_Y, PAIR_GOAL_Y = -3.019, 0.5495


def _goal_y_default():
    """SIM_GOAL_Y 가 있으면 그대로, 없으면 SIM_PICK_Y 에서 짝 규칙으로 만든다"""
    if os.environ.get('SIM_GOAL_Y'):
        return float(os.environ['SIM_GOAL_Y'])
    pick_y = float(os.environ.get('SIM_PICK_Y', PAIR_PICK_Y))
    return PAIR_GOAL_Y + (PAIR_PICK_Y - pick_y)


def reclaim(node, handle, why, wait_s=10.0):
    """떠나기 전에 **남긴 목표를 거둬들인다.**

    안 하면 이 프로세스가 끝나도 목표는 조작 노드에서 계속 돈다. 그리고 바깥
    사이클이 "파지 실패" 로 판단해 로봇을 집으로 보낸 **뒤에**, 살아 있던 그 목표의
    회전 명령이 나간다. 2026-09-24 VD2 가 그랬다 — 키오스크에 선 채로 서가 자리를
    맞추려 차체를 2.676 m 끌고 가려 했고, 그 판은 418초를 먹었다.

    **거두는 데 실패해도 이 프로세스는 끝난다.** 여기서 막히면 사이클이 더 오래
    멈춘다. 그래서 짧게 기다리고, 됐는지 안 됐는지를 **로그에 남긴다** —
    "취소했다" 와 "취소를 보냈다" 는 다른 사실이다.
    """
    if handle is None:
        return
    print(f'  남긴 목표를 거둔다 ({why})')
    try:
        fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, fut, timeout_sec=wait_s)
        r = fut.result()
        if r is None:
            print('  **취소 응답이 없다** — 목표가 아직 돌고 있을 수 있다')
        elif getattr(r, 'goals_canceling', None):
            print('  취소 받아들여짐')
        else:
            print(f'  **취소가 거절됐다** (return_code={getattr(r, "return_code", "?")}) '
                  '— 목표가 아직 돌고 있을 수 있다')
    except Exception as exc:      # noqa: BLE001 - 거두기 실패가 종료를 막으면 안 된다
        print(f'  **취소 실패** {type(exc).__name__}: {exc}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal-x', type=float,
                    default=float(os.environ.get('SIM_GOAL_X', FRANKA_SLOTS[0])),
                    help='꽂을 칸 x (팔 기준)')
    ap.add_argument('--goal-y', type=float, default=_goal_y_default(),
                    help='꽂을 칸 y (팔 기준). 비우면 SIM_PICK_Y 에서 짝 규칙으로 만든다')
    ap.add_argument('--goal-z', type=float, default=float(os.environ.get('SIM_GOAL_Z', 0.3399)))
    ap.add_argument('--book-id', default='book_0')
    ap.add_argument('--wait', type=float, default=20.0, help='비전 좌표를 기다릴 시간(초)')
    ap.add_argument('--dry-run', action='store_true', help='좌표만 보고 보내지 않는다')
    ap.add_argument('--detect-slot', action='store_true',
                    help='꽂을 칸을 **비전에게 묻는다** (DetectTargetSlot 먼저). '
                         '안 주면 --goal-x/y/z 상수를 쓴다')
    ap.add_argument('--slot-wait', type=float, default=120.0,
                    help='빈칸 검출을 기다릴 시간(초). 스캔이 들어가 오래 걸린다')
    a = ap.parse_args()

    rclpy.init()
    node = Node('pick_from_vision')
    seen = []
    node.create_subscription(PointStamped, '/perception/books', seen.append, 10)
    trigger = node.create_publisher(Bool, '/perception/detect_request', 10)

    def _wait_book():
        # 검출 요청을 반복해 보낸다 — 비전은 요청이 있을 때만 검출한다.
        # 한 번만 보내면 디스커버리가 늦어 놓치는 일이 있다 (2026-09-21 실측).
        seen.clear()
        start = node.get_clock().now()
        while (node.get_clock().now() - start).nanoseconds / 1e9 < a.wait:
            trigger.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.3)
            if seen:
                return seen[-1]
        return None

    # **빈칸을 먼저, 책을 나중에.** 순서가 중요하다 (`--detect-slot` 일 때).
    #
    # `DetectTargetSlot` 은 스캔 전에 차체를 서가 중심으로 **266 mm 옮긴다**
    # (`align_base_before_work`). 책 좌표는 `arm_base_link` 기준이므로, 옮기기 **전**에
    # 받아 두면 옮긴 뒤에는 엉뚱한 데를 가리킨다 — `goal_x` 상수가 26 cm 어긋난 것과
    # **똑같은 병**이다 (2026-09-24). 그래서 옮긴 뒤에 받는다.
    #
    # 이것이 비전팀에 요청해 둔 `TargetSlot.base_pose` 가 필요한 이유이기도 하다:
    # 좌표에 "잴 때 어디 서 있었는지" 가 없으면 이런 순서를 사람이 외워야 한다.
    print(f'꽂을 곳(상수)  ({a.goal_x:+.4f}, {a.goal_y:+.4f}, {a.goal_z:+.4f})')

    # **꽂을 칸을 비전에게 묻는다** (`--detect-slot`).
    #
    # 왜 필요한가: 아래 `--goal-x/y/z` 는 **상수**다. 그리고 그 값은 "주행 경유점에
    # 선 자세" 에 묶여 있다 — 2026-09-24 이전 열다섯 판을 그 자세에서 쟀다.
    # 그런데 스캔은 서가 전체를 보려고 차체를 **서가 중심으로 266 mm 옮긴다**
    # (`align_base_before_work`). 옮긴 뒤 같은 팔 기준 상수를 쓰면 26 cm 떨어진
    # 데를 가리키고, 실제로 남의 자리(책이 꽉 찬 칸)에 35.2 mm 겹쳐 꽂았다.
    #
    # 검출값을 받으면 **옮긴 자세 기준으로 나오므로 자동으로 맞는다.** 상수를
    # 옮긴 자세에 맞춰 다시 재는 길도 있지만, 그러면 또 하나의 썩을 상수가 는다.
    detected = None
    if a.detect_slot:
        det = ActionClient(node, DetectTargetSlot, '/detect_target_slot')
        if not det.wait_for_server(timeout_sec=10.0):
            print('**/detect_target_slot 액션 서버가 없다** — '
                  'manipulation_node 의 enable_perception_bridge 를 볼 것')
            return 2
        dg = DetectTargetSlot.Goal()
        dg.job_id = f'slot_{node.get_clock().now().nanoseconds // 10**9}'
        dg.book_id = a.book_id
        dg.book_width, dg.book_height = BOOK['width'], BOOK['height']
        dg.book_thickness = BOOK['thickness']
        print('빈칸을 찾는다 (스캔이 들어가 오래 걸린다) …')
        _s = det.send_goal_async(dg, feedback_callback=lambda f: print(
            f'  {f.feedback.phase} 후보 {f.feedback.candidate_count}'))
        rclpy.spin_until_future_complete(node, _s, timeout_sec=30.0)
        if not (_s.done() and _s.result() and _s.result().accepted):
            print('**빈칸 검출 목표가 거부됐다**')
            return 2
        _handle = _s.result()
        _r = _handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, _r, timeout_sec=a.slot_wait)
        except KeyboardInterrupt:
            reclaim(node, _handle, '중단됨')
            raise
        if not _r.done():
            print(f'**빈칸 검출이 {a.slot_wait:.0f}초 안에 안 끝났다**')
            # **이쪽도 거둔다.** 빈칸 검출도 베이스 회전을 낸다 — 살려 두면
            # PlaceBook 과 똑같이 철 지난 회전 명령이 뒤늦게 나간다
            reclaim(node, _handle, '빈칸 검출 시간 초과')
            return 2
        res = _r.result().result
        if not res.success:
            print(f'**빈칸 검출 실패** {res.error_code} {res.message}')
            return 2
        detected = res.target_slot
        _pp = detected.pose.position
        print(f'빈칸 검출: frame={detected.header.frame_id} '
              f'({_pp.x:+.4f}, {_pp.y:+.4f}, {_pp.z:+.4f}) '
              f'폭 {detected.available_width*1000:.1f} mm · '
              f'높이 {detected.available_height*1000:.1f} mm · '
              f'후보 {res.candidate_count} · 신뢰도 {detected.confidence:.2f}')
        print(f'  상수와의 차이  x {(_pp.x - a.goal_x)*1000:+.1f} · '
              f'y {(_pp.y - a.goal_y)*1000:+.1f} · z {(_pp.z - a.goal_z)*1000:+.1f} mm '
              f'(상수 {a.goal_x:+.4f}, {a.goal_y:+.4f}, {a.goal_z:+.4f})')
        # **폭을 꼭 본다.** 겹침 예비는 빈칸 폭에서 나온다 — 어제 잰 3.7 mm 는
        # 폭 59.9 mm 기준이고, 더 좁은 칸이 잡히면 그만큼 깎인다.
        _need = BOOK['thickness'] + 0.010
        if detected.available_width > 0 and detected.available_width < _need:
            print(f'  **주의: 검출 폭 {detected.available_width*1000:.1f} mm 가 '
                  f'책 두께 + 여유 {_need*1000:.1f} mm 보다 좁다**')

    msg = _wait_book()
    if msg is None:
        print(f'**비전 좌표가 {a.wait:.0f}초 안에 오지 않았다.** '
              f'vision_manager 가 떠 있는지, 도메인이 같은지 볼 것')
        return 2
    p = msg.point
    print(f'비전 좌표(윗면 중심)  frame={msg.header.frame_id}  '
          f'({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f})'
          + ('   ← 빈칸 검출(차체 이동) **뒤에** 받은 값이다' if detected is not None else ''))
    print(f'→ 로봇팔이 쓸 AABB 중심  ({p.x:+.4f}, {p.y:+.4f}, '
          f'{p.z - BOOK["width"] / 2:+.4f})   (책 폭 {BOOK["width"]} 의 절반을 뺀 값)')
    if a.dry_run:
        print('--dry-run: 보내지 않는다')
        return 0

    client = ActionClient(node, PlaceBook, '/place_book')
    if not client.wait_for_server(timeout_sec=10.0):
        print('**/place_book 액션 서버가 없다** — manipulation_node 를 볼 것')
        return 2

    goal = PlaceBook.Goal()
    goal.job_id = f'vision_{msg.header.stamp.sec}'
    goal.book_id = a.book_id
    if detected is not None:
        # 검출값을 **그대로** 싣는다. 우리가 고쳐 쓰면 무엇이 검출이고 무엇이
        # 우리 보정인지 못 가린다 — 오늘 그 구분을 못 해 두 번 헤맸다.
        goal.target_slot = detected
    else:
        goal.target_slot.header.frame_id = 'arm_base_link'
        goal.target_slot.pose.position.x = a.goal_x
        goal.target_slot.pose.position.y = a.goal_y
        goal.target_slot.pose.position.z = a.goal_z
        goal.target_slot.pose.orientation.z = 0.7071068      # yaw +90° (서가 방향)
        goal.target_slot.pose.orientation.w = 0.7071068
        goal.target_slot.confidence = 1.0
    goal.book_thickness = BOOK['thickness']
    goal.book_height = BOOK['height']
    goal.book_width = BOOK['width']
    # 비전 관측을 싣는다 — **다만 치수는 관측이 아니다.** `top_center` 만 비전이
    # 실제로 본 값이고, thickness·width 는 아래 BOOK 상수다. 이 구분을 안 적어 둬서
    # 2026-09-24 에 "관측이 치수를 실어 온다" 고 믿고 하루를 썼다. 그 값으로 재는
    # 검사(410 높이, 헛쥠 판정, 파지 지령)는 전부 **규격을 규격과 견주고 있었다.**
    goal.has_grasp = True
    goal.grasp.header = msg.header
    goal.grasp.top_center = p
    goal.grasp.thickness = BOOK['thickness']     # ← 상수. 비전이 잰 값이 아니다
    goal.grasp.width = BOOK['width']             # ← yaml 프로파일
    goal.grasp.confidence = 1.0
    print(f"  ※ 치수는 관측이 아니라 **yaml 프로파일**이다 (두께 {BOOK['thickness']*1000:.1f} · "
          f"폭 {BOOK['width']*1000:.1f} mm). 실물과 다르면 파지 지령과 검사가 함께 틀린다")

    print('보냄 — 진행 상황:')
    send = client.send_goal_async(
        goal, feedback_callback=lambda f: print(f'  {f.feedback.phase}'))
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print('**목표가 거절됐다**')
        return 2
    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=180.0)
    except KeyboardInterrupt:
        # 정리 스크립트가 우리를 죽일 때도 목표는 거둬야 한다
        reclaim(node, handle, '중단됨')
        raise
    res = result_future.result()
    if res is None:
        print('**결과를 못 받았다 (시간 초과)**')
        reclaim(node, handle, '결과 시간 초과')
        return 2
    r = res.result
    print(f"결과: success={r.success} code={r.error_code} "
          f"phase={r.failed_phase} verified={r.placement_verified}")
    print(f"  {r.message}")
    return 0 if r.success else 1


if __name__ == '__main__':
    sys.exit(main())
