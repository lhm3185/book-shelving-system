#!/usr/bin/env python3
"""한 판의 로그를 읽어 **합격인지 말한다** — 손으로 읽지 않게.

왜: 2026-09-24 에 스무 판 넘게 돌리면서 매번 로그를 눈으로 훑어 판정했다. 그러다
`code=0` 만 보고 "통과" 라고 했는데 여유가 0.000 인 판이 섞여 있었고, "겹침 0" 을
안전으로 읽었는데 임계까지 예비가 −0.1 mm 였다. **판정 기준이 머릿속에만 있으면
매번 조금씩 달라진다.**

기준 (2026-09-24 합의):

    합격 = code 0  ·  min_margin ≥ 0.15  ·  겹침 0 mm

그리고 **통과해도 얇으면 말한다** — 임계까지의 예비, 꽂힌 기울기, 떠오름.

    python3 judge_run.py ~/b1_arm/night/runs/V8
    python3 judge_run.py ~/b1_arm/night/runs/*/ --brief
"""
import argparse
import glob
import os
import re
import sys

#: 합격선 (rad). 계획 거부선 0.03 과 다르다 — 그쪽은 "재려던 것을 못 재게 막지 않는" 선이다
MIN_MARGIN = 0.15
#: 겹침은 **0** 이다. 5 mm 는 서가 책들끼리의 관측 바닥값이지 우리가 써도 되는 예산이 아니다
JAM_MM = 0.0
#: 통과해도 **이만큼 안쪽이면 말한다** — "합격" 과 "여유가 있다" 는 다르다
THIN_MARGIN = 0.05


#: 기동 성공의 **유일한 표시**. Isaac 이 명령 대기까지 왔다는 뜻이고,
#: `run_vision.sh` 가 재시도 여부를 판단할 때 쓰는 바로 그 줄이다.
BOOTED = "준비 완료"


def load(d):
    """판 하나의 로그를 **파일별로** 읽는다.

    붙여서 읽으면 안 된다 — "Isaac 이 떴는가" 는 `sim.log` 에만 답이 있는데
    합쳐 놓으면 다른 로그의 글자가 답을 바꾼다. 2026-09-24 에 실제로 그랬다.
    """
    logs, names = {}, ("sim.log", "cyc.log", "man.log", "vis.log")
    for name in names:
        p = os.path.join(d, name)
        if os.path.exists(p):
            logs[name] = open(p, errors="replace").read()
    #: 재시도로 밀려난 기동 로그 (`sim.boot1.log` ...). 있으면 한 번에 뜨지 않았다는 뜻
    logs["#boot_attempts"] = sorted(glob.glob(os.path.join(d, "sim.boot*.log")))
    return logs


def incidents(logs):
    """**런타임 사고를 종류별로 가른다** — 완주율을 세려면 "왜 못 갔나" 가 갈려야 한다.

    합격/불합격과 다른 축이다. 사고는 검출·배치의 품질이 아니라 **끝까지 갔는가**를
    말한다. 2026-09-24 현재 셋이 관측됐고 셋 다 배치와 무관하다.

    여기 있는 규칙은 전부 한 번씩 틀렸던 것을 고친 것이다:

    * `previous crash` 로 기동 크래시를 세지 않는다. 그건 크래시리포터가 시작할 때
      **과거 덤프를 나열하는 줄**이다. 덤프가 39개 쌓여 있어서 멀쩡히 완주한 판에도
      40줄씩 찍혔고, 그래서 16/16 이 전부 "기동 크래시" 로 찍혔다. 덤프 대부분은
      종료(`kill -INT`) 때 남은 것이라 기동과는 더더욱 상관이 없다.
    * 기동의 판단은 **`준비 완료` 가 있는가** 하나다. 110판 실패율 6.4% 를 낼 때 쓴
      기준이고 `run_vision.sh` 의 대기 루프가 쓰는 기준과 같다. 자를 통일한다.
    * 재시도해서 뜬 판은 **사고가 아니다** — 감싸는 데 성공한 것이다. 다만 "한 번에
      떴다" 와 "세 번 만에 떴다" 는 다른 사실이라 따로 센다.
    """
    sim = logs.get("sim.log")
    man = logs.get("man.log", "")
    cyc = logs.get("cyc.log", "")
    inc, note = [], {}

    # 기동 — sim.log 가 있는데 `준비 완료` 가 없으면 못 뜬 것이다
    if sim is not None and BOOTED not in sim:
        inc.append("boot_crash")
    tries = len(logs.get("#boot_attempts") or [])
    if tries:
        note["boot_retry"] = tries + 1        # 밀려난 로그 수 + 성공한 판

    # PhysX NaN — 관절값이 깨진다. 28,202줄 난 판이 있었다(떠 있는 베이스 + hold_base_tick).
    # **몇 줄인지 같이 센다** — 종료 때 몇 줄 나는 것과 판 전체가 깨진 것은 다른 사실이다.
    if sim:
        n = len(re.findall(r"Invalid PhysX transform", sim))
        if n:
            inc.append("physx_nan")
            note["physx_lines"] = n

    # manipulation_node 사망 — 액션 실행 중 rclpy 핸들이 죽는다.
    # **정상 종료와 가르기 위해** 사이클이 결과를 못 낸 판만 센다. 끝난 뒤의 종료
    # 잡음까지 세면 멀쩡한 판이 사고로 찍힌다 — `previous crash` 와 같은 실수다.
    if re.search(r"RCLError|publish_feedback|feedback publisher is invalid", man):
        if "cycle rc=0" not in cyc:
            inc.append("node_died")

    # rotate_base 무응답 — 명령은 갔는데 완료가 안 온다 (VD2 는 418초 멈췄다).
    #
    # **"완료가 하나도 없는가" 로 보면 안 된다.** 한 판에 회전이 **두 번** 있다
    # (스캔 전·꽂기 전). VD2 는 첫 번째가 완료되고 두 번째가 멈췄는데, 그 조건으로는
    # "완료가 있으니 괜찮다" 가 되어 배치 실패로 잘못 찍혔다. **수를 맞춰 본다.**
    all_txt = "".join(v for k, v in logs.items() if isinstance(v, str))
    cmd_n = all_txt.count("베이스 회전 명령 전송")
    done_n = all_txt.count("베이스 회전 완료")
    if cmd_n > done_n:
        inc.append("rotate_hang")
        note["rotate_counts"] = (cmd_n, done_n)
        # **어디서 멈췄는지 같이 들고 나온다.** VD2 는 느려서 멈춘 게 아니라
        # 키오스크 자리에서 90° 를 통째로 돌라는 명령을 받았다 — 다른 판은 0° 다.
        starts = re.findall(r"베이스 회전 시작:[^\n]*", all_txt)
        if starts:
            note["rotate_last"] = starts[-1].strip()[:160]
    return sorted(set(inc)), note


def read(d):
    """판 하나의 로그를 긁어 숫자로."""
    logs = load(d)
    # **숫자를 읽는 덩어리는 세 로그뿐이다.** `vis.log` 는 사고 판정에만 쓴다 —
    # 덩어리에 넣으면 비전 로그의 글자가 `code=` 같은 정규식에 걸릴 수 있다.
    txt = "".join(logs.get(n, "") for n in ("sim.log", "cyc.log", "man.log"))
    g = {}
    inc, note = incidents(logs)
    if inc:
        g["incidents"] = inc
    g.update(note)
    m = re.search(r"code=(\d+)", txt)
    if m:
        g["code"] = int(m.group(1))
    m = re.search(r"verified=(True|False)", txt)
    if m:
        g["verified"] = m.group(1) == "True"
    # 경로검사 구간별 여유 중 최소
    marg = [float(v) for v in re.findall(r"한계 최소여유 ([\d.]+) rad", txt)]
    if marg:
        g["min_margin"] = min(marg)
        g["segments"] = len(marg)
    # 겹침
    m = re.search(r"\[겹침\] 꽂은 책이 '([^']*)' 와 ([\d.]+) mm 겹친다", txt)
    if m:
        g["jam_mm"], g["jam_with"] = float(m.group(2)), m.group(1)
    elif "옆 책과 겹치지 않음" in txt:
        g["jam_mm"] = 0.0
    # 꽂은 자세
    m = re.search(r"가로 ([\d.]+) x ([\d.]+) mm.*?기울기 ([+-][\d.]+)", txt, re.S)
    if m:
        g["span_mm"], g["tilt_deg"] = float(m.group(1)), float(m.group(3))
    # 떠오름 (놓기직전 → 놓음)
    d_mm = re.findall(r"밑면−칸바닥 ([+-][\d.]+) mm", txt)
    if len(d_mm) >= 2:
        g["rise_mm"] = float(d_mm[-1]) - float(d_mm[0])
    # 꽂기 전 차체 옆이동 — 검출된 틈을 검증된 place_x 로 가져오는 양.
    # 판마다 다르고(0.171~0.288 m), 그만큼 팔 자세가 달라져 min_margin 이 흔들린다.
    m = re.search(r"꽂기 전 차체를 옆으로 ([+-]?[\d.]+) m", txt)
    if m:
        g["shift_m"] = float(m.group(1))
    # 빈칸 깊이 파생이 얼마나 끌어왔나 (관측이 얼마나 뒤를 읽었나)
    dep = [float(v) for v in re.findall(r"빈칸 깊이:.*?차이 ([+-][\d.]+) mm", txt)]
    if dep:
        g["depth_pull_mm"] = (min(dep), max(dep))
    # 레벨·코드 출처
    lv = os.path.join(d, "level.txt")
    if os.path.exists(lv):
        for ln in open(lv):
            if ln.startswith(("md5=", "git=")):
                g[ln.split("=")[0]] = ln.split("=", 1)[1].strip()
    return g


#: 사고 이름표. **한 곳에서만 적는다** — 두 군데 적어 두면 하나만 고친다
INCIDENT_NAMES = {
    "boot_crash": "Isaac 기동 실패 (`준비 완료` 없음)",
    "physx_nan": "PhysX 변환 깨짐 (관절 NaN)",
    "node_died": "manipulation_node 사망",
    "rotate_hang": "rotate_base 무응답",
}


def verdict(g):
    """합격 여부와 사유들."""
    bad, warn = [], []
    inc = g.get("incidents") or []
    names = INCIDENT_NAMES
    for k in inc:
        tail = f" ({g['physx_lines']:,}줄)" if k == "physx_nan" and g.get("physx_lines") else ""
        bad.append(f"런타임 사고: {names.get(k, k)}{tail} — **배치와 무관하다**")
    if g.get("code") is None and not inc:
        bad.append("code 를 못 읽었다")
    elif g.get("code") is None:
        pass                      # 사고로 못 갔으면 code 가 없는 게 당연하다
    elif g.get("code") is not None and g["code"] != 0:
        bad.append(f"code {g['code']}")
    if g.get("verified") is False:
        bad.append("verified=False")
    mm = g.get("min_margin")
    if mm is None:
        warn.append("min_margin 을 못 읽었다")
    elif mm < MIN_MARGIN:
        bad.append(f"min_margin {mm:.3f} < {MIN_MARGIN}")
    jm = g.get("jam_mm")
    if jm is None:
        warn.append("겹침을 못 읽었다")
    elif jm > JAM_MM:
        bad.append(f"겹침 {jm:.1f} mm" + (f" ({g['jam_with']})" if g.get("jam_with") else ""))
    # 통과해도 얇으면 말한다
    if mm is not None and MIN_MARGIN <= mm < MIN_MARGIN + THIN_MARGIN:
        warn.append(f"min_margin {mm:.3f} — 합격선에서 {mm - MIN_MARGIN:.3f} 뿐")
    if g.get("rotate_last"):
        bad.append(f"멈춘 자리: {g['rotate_last']}")
    if g.get("boot_retry"):
        warn.append(f"기동이 {g['boot_retry']}번째 시도에 떴다 — 감싸서 넘겼지만 한 번에 뜬 것은 아니다")
    if g.get("tilt_deg") is not None and abs(g["tilt_deg"]) > 1.0:
        warn.append(f"기울기 {g['tilt_deg']:+.2f}° — 1° 를 넘으면 옆 여유를 먹는다")
    if g.get("rise_mm") is not None and abs(g["rise_mm"]) > 5.0:
        warn.append(f"놓을 때 {g['rise_mm']:+.1f} mm 움직였다 — 콜라이더 면을 볼 것")
    return (not bad), bad, warn


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--brief", action="store_true", help="한 줄씩만")
    a = ap.parse_args()

    dirs = [d for pat in a.dirs for d in sorted(glob.glob(pat)) if os.path.isdir(d)]
    if not dirs:
        print("판 디렉터리가 없다")
        return 2
    n_ok, n_inc, tally = 0, 0, {}
    for d in dirs:
        g = read(d)
        ok, bad, warn = verdict(g)
        n_ok += int(ok)
        n_inc += int(bool(g.get("incidents")))
        for k in (g.get("incidents") or []):
            tally[k] = tally.get(k, 0) + 1
        name = os.path.basename(d.rstrip("/"))
        head = f"{'합격' if ok else '불합격'}  {name:<16}"
        nums = (f"code {g.get('code', '?')} · min_margin {g.get('min_margin', float('nan')):.3f} · "
                f"겹침 {g.get('jam_mm', float('nan')):.1f} mm")
        if g.get("span_mm"):
            nums += f" · 가로 {g['span_mm']:.1f} mm {g.get('tilt_deg', 0):+.2f}°"
        if g.get("shift_m") is not None:
            nums += f" · 옆이동 {g['shift_m']:+.3f} m"
        print(f"{head} {nums}")
        if a.brief:
            continue
        for b in bad:
            print(f"    ✘ {b}")
        for w in warn:
            print(f"    ! {w}")
        if g.get("depth_pull_mm"):
            lo, hi = g["depth_pull_mm"]
            print(f"    빈칸 관측이 앞면보다 {lo:+.0f}~{hi:+.0f} mm 뒤였다 (파생으로 끌어옴)")
        if g.get("md5") or g.get("git"):
            print(f"    레벨 {g.get('md5', '?')[:8]} · 코드 {g.get('git', '?')[:8]}")
    if len(dirs) > 1:
        # **세 칸으로 나눈다.** "불합격" 과 "사고로 못 감" 을 한 칸에 넣으면
        # 완주율도 합격률도 안 나온다 — 산출물에 넣을 표는 이 세 칸이다.
        n_done = len(dirs) - n_inc
        n_fail = n_done - n_ok
        print(f"\n판 {len(dirs)}개")
        print(f"  합격            {n_ok}")
        print(f"  불합격 (배치)   {n_fail}")
        print(f"  사고로 못 감    {n_inc}")
        pct = 100.0 * n_done / len(dirs)
        print(f"→ 완주율 {n_done}/{len(dirs)} = {pct:.0f}%"
              + (f" · 완주한 판의 합격 {n_ok}/{n_done}" if n_done else ""))
        if tally:
            print("런타임 사고 (배치와 무관):")
            for k, v in sorted(tally.items(), key=lambda kv: -kv[1]):
                print(f"  {INCIDENT_NAMES.get(k, k):<28} {v}/{len(dirs)}")
    return 0 if n_ok == len(dirs) else 1


if __name__ == "__main__":
    sys.exit(main())
