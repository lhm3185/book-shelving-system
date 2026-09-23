#!/usr/bin/env python3
"""night/runs/<ID> 들을 한 줄씩 요약한다 — 합격 기준은 code==0 && verified && min_margin>=0.15 && 겹침 없음."""
import re, sys, os
for d in sys.argv[1:]:
    p = f"night/runs/{d}"
    if not os.path.isdir(p):
        print(f"{d:8} (없음)"); continue
    cyc = open(f"{p}/cyc.log", errors="ignore").read() if os.path.exists(f"{p}/cyc.log") else ""
    sim = open(f"{p}/sim.log", errors="ignore").read() if os.path.exists(f"{p}/sim.log") else ""
    env = open(f"{p}/env.txt", errors="ignore").read() if os.path.exists(f"{p}/env.txt") else ""
    ms = [float(m) for m in re.findall(r"한계 최소여유 ([0-9.]+) rad", sim)]
    mm = min(ms) if ms else None
    ov = "겹치지 않음" in sim
    ok = ("code=0" in cyc) and ("verified=True" in cyc) and ov and mm is not None and mm >= 0.15
    pick = re.search(r"SIM_PICK_Y=(-?[\d.]+)", env)
    print(f"{d:8} pick={pick.group(1) if pick else '?':9} 합격={ok} min_margin={mm} 겹침없음={ov}")
