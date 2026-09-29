#!/usr/bin/env python3
"""index.csv(프레임별 받은 시각)로 **실제 간격대로** mp4 를 만든다.

    python3 mkclip.py <raw/<이름>> <시작 HH:MM:SS> <끝 HH:MM:SS> <나갈.mp4> [fps]

프레임은 발행될 때만 있으므로 간격이 들쭉날쭉하다. ffmpeg concat демuxer 에
프레임마다 `duration` 을 줘서 실제 시간대로 잇고, 출력만 고정 fps 로 만든다.
검출 화면은 검출 중에만 발행되니 **끊기는 구간이 있는 게 정상**이다.
"""
import sys, os, csv, datetime, subprocess

d, t0s, t1s, out = sys.argv[1:5]
fps = sys.argv[5] if len(sys.argv) > 5 else "24"
d = os.path.expanduser(d)
rows = []
with open(os.path.join(d, "index.csv")) as f:
    for r in csv.DictReader(f):
        rows.append((int(r["index"]), float(r["wall_time"])))
if not rows:
    sys.exit("프레임이 없다")
base = datetime.datetime.fromtimestamp(rows[0][1])


def at(s):
    h, m, sec = (int(v) for v in s.split(":"))
    return base.replace(hour=h, minute=m, second=sec, microsecond=0).timestamp()


t0, t1 = at(t0s), at(t1s)
sel = [(i, t) for i, t in rows if t0 <= t <= t1]
if not sel:
    sys.exit(f"{t0s}~{t1s} 구간에 프레임이 없다 (전체 {len(rows)}장, "
             f"{datetime.datetime.fromtimestamp(rows[0][1]):%H:%M:%S}~"
             f"{datetime.datetime.fromtimestamp(rows[-1][1]):%H:%M:%S})")

lst = os.path.join(d, "concat.txt")
with open(lst, "w") as f:
    for k, (i, t) in enumerate(sel):
        nt = sel[k + 1][1] if k + 1 < len(sel) else t + 1.0 / float(fps)
        dur = max(1.0 / float(fps), min(nt - t, 3.0))   # 3초 넘는 공백은 3초로 줄인다
        f.write(f"file '{os.path.join(d, 'f%06d.png' % i)}'\nduration {dur:.3f}\n")
    f.write(f"file '{os.path.join(d, 'f%06d.png' % sel[-1][0])}'\n")

out = os.path.expanduser(out)
os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0",
       "-i", lst, "-vsync", "cfr", "-r", fps, "-c:v", "libx264", "-preset", "slow",
       "-crf", "24", "-pix_fmt", "yuv420p", "-movflags", "+faststart", out]
subprocess.run(cmd, check=True)
span = sel[-1][1] - sel[0][1]
print(f"{os.path.basename(d):10s} {len(sel):5d}장 · 실제 {span:6.1f}s · "
      f"{os.path.getsize(out)/1048576:.1f} MB → {out}")
