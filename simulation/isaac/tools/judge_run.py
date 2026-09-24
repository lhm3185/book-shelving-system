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


def read(d):
    """판 하나의 로그를 긁어 숫자로."""
    txt = ""
    for name in ("sim.log", "cyc.log", "man.log"):
        p = os.path.join(d, name)
        if os.path.exists(p):
            txt += open(p, errors="replace").read()
    g = {}
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
    # 레벨·코드 출처
    lv = os.path.join(d, "level.txt")
    if os.path.exists(lv):
        for ln in open(lv):
            if ln.startswith(("md5=", "git=")):
                g[ln.split("=")[0]] = ln.split("=", 1)[1].strip()
    return g


def verdict(g):
    """합격 여부와 사유들."""
    bad, warn = [], []
    if g.get("code") is None:
        bad.append("code 를 못 읽었다")
    elif g["code"] != 0:
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
    n_ok = 0
    for d in dirs:
        g = read(d)
        ok, bad, warn = verdict(g)
        n_ok += int(ok)
        name = os.path.basename(d.rstrip("/"))
        head = f"{'합격' if ok else '불합격'}  {name:<16}"
        nums = (f"code {g.get('code', '?')} · min_margin {g.get('min_margin', float('nan')):.3f} · "
                f"겹침 {g.get('jam_mm', float('nan')):.1f} mm")
        if g.get("span_mm"):
            nums += f" · 가로 {g['span_mm']:.1f} mm {g.get('tilt_deg', 0):+.2f}°"
        print(f"{head} {nums}")
        if a.brief:
            continue
        for b in bad:
            print(f"    ✘ {b}")
        for w in warn:
            print(f"    ! {w}")
        if g.get("md5") or g.get("git"):
            print(f"    레벨 {g.get('md5', '?')[:8]} · 코드 {g.get('git', '?')[:8]}")
    if len(dirs) > 1:
        print(f"\n{n_ok}/{len(dirs)} 합격")
    return 0 if n_ok == len(dirs) else 1


if __name__ == "__main__":
    sys.exit(main())
