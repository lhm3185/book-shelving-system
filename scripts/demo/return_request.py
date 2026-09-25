#!/usr/bin/env python3
"""입력 "N번 서가에 반납할 책 K권" → `TrayJob` 한 건 — 기존 `/return_machine/tray_job` 그대로 쓴다.

    python3 scripts/demo/return_request.py --shelf shelf_02 --count 2 --shelf shelf_01 --count 2
    python3 scripts/demo/return_request.py --shelf shelf_02 --count 2 --print-env     # 발행 없이 SIM_* 만

왜 이렇게: 서가는 분류 코드 접두어로 정해진다(job_planner → shelf_map.shelves.*.classification_prefixes).
그러니 "shelf_02 에 2권" 은 **그 서가 접두어를 가진 분류 코드 둘**을 TrayJob 에 싣는 것이고, 2026-09-25
네 권 3/3 이 정확히 그 경로로 돌았다. 새 인터페이스를 만들지 않는다 — 진짜 반납기 로직이 오면 발행자만
바뀐다(웹 클로드 v44 회신 §2). 순서는 준 순서 그대로 돈다(서가별로 묶어 주면 주행이 준다).

full_cycle.sh 를 `SIM_JOB_INPUT=manual` 로 띄우면 return_machine 이 자동 발행하지 않으므로, 그 뒤 이
스크립트로 입력을 준다. 책 id 는 트레이 칸 순서(book_001…)를 그대로 쓴다 — 칸 배정은 아직 순서뿐이다.
"""
import argparse
import os
import sys
import uuid

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHELF_MAP = os.path.join(REPO, "cli_exchange", "config", "shelf_map_measured.yaml")
TOPIC = "/return_machine/tray_job"
#: 접두어 하나에 대한 대표 분류 코드 — 접두어 뒤는 아무 값이나 된다(job_planner 는 접두어만 본다)
CODE_TAIL = "00.0"


def shelf_prefixes(path=SHELF_MAP):
    """shelf_map → {shelf_id: [접두어…]}."""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    out = {}
    for sid, shelf in (data.get("shelves") or {}).items():
        pre = [str(p) for p in (shelf.get("classification_prefixes") or [])]
        if pre:
            out[str(sid)] = pre
    return out


def job_from_request(requests, prefixes, first_book=1):
    """`[(shelf_id, count), …]` → `(book_ids, rfid_tags, classification_codes)`. 모르는 서가면 ValueError."""
    books, rfids, codes = [], [], []
    n = first_book
    for sid, count in requests:
        if sid not in prefixes:
            raise ValueError(f"shelf_map 에 없는 서가: {sid!r} (있는 것: {sorted(prefixes)})")
        for _ in range(int(count)):
            books.append(f"book_{n:03d}")
            rfids.append(f"rfid_{n:03d}")
            codes.append(f"{prefixes[sid][0]}{CODE_TAIL}")
            n += 1
    if not books:
        raise ValueError("책이 0권이다")
    return books, rfids, codes


def env_lines(books, rfids, codes):
    q = lambda xs: "[" + ",".join(f"'{x}'" for x in xs) + "]"      # noqa: E731
    return [f'SIM_BOOK_IDS="{q(books)}"', f'SIM_RFID_TAGS="{q(rfids)}"', f'SIM_CLASS_CODES="{q(codes)}"']


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shelf", action="append", required=True, help="서가 id (shelf_01·shelf_02…). 반복 가능")
    ap.add_argument("--count", action="append", type=int, required=True, help="그 서가에 꽂을 권수 (--shelf 와 짝)")
    ap.add_argument("--shelf-map", default=SHELF_MAP)
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--tray-id", default="tray_01")
    ap.add_argument("--print-env", action="store_true", help="발행하지 않고 full_cycle 용 SIM_* 만 찍는다")
    a = ap.parse_args(argv)
    if len(a.shelf) != len(a.count):
        raise SystemExit("--shelf 와 --count 의 수가 다르다")
    books, rfids, codes = job_from_request(list(zip(a.shelf, a.count)), shelf_prefixes(a.shelf_map))
    print("입력 →", " · ".join(f"{s} {c}권" for s, c in zip(a.shelf, a.count)))
    print("책   ", books)
    print("분류 ", codes)
    if a.print_env:
        print("\n".join(env_lines(books, rfids, codes)))
        return 0

    import rclpy
    from shelving_interfaces.msg import TrayJob
    rclpy.init()
    node = rclpy.create_node("return_request")
    pub = node.create_publisher(TrayJob, a.topic, 10)
    msg = TrayJob()
    msg.job_id = f"job_{uuid.uuid4().hex[:12]}"
    msg.tray_id = a.tray_id
    msg.created_at = node.get_clock().now().to_msg()
    msg.book_ids = books
    msg.rfid_tags = rfids
    msg.classification_codes = codes
    # 구독자가 붙을 때까지 잠깐 기다린다 — 발행 직후 종료하면 첫 메시지가 안 갈 수 있다
    for _ in range(50):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        # **조용히 성공처럼 끝나지 않는다.** 2026-09-25 데스크탑: 셸의 ROS_DOMAIN_ID(130)가 스크립트의
        # 기본(129)과 달라 구독자 0 인 채 발행하고 끝났다 — 아무 일도 안 일어난다. 도메인·RMW·프로파일을
        # full_cycle 과 같게 맞춰야 붙는다.
        node.destroy_node(); rclpy.shutdown()
        raise SystemExit(f"구독자가 없다 — {a.topic} 을 듣는 노드가 안 보인다. full_cycle 과 같은 "
                         f"ROS_DOMAIN_ID / RMW_IMPLEMENTATION / FASTRTPS_DEFAULT_PROFILES_FILE 인지 확인 "
                         f"(지금 ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(없음)')})")
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print(f"발행 {a.topic}: job_id={msg.job_id} · {len(books)}권 (구독자 {pub.get_subscription_count()})")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
