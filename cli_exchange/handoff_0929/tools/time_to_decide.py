import re,glob,os
SL={-0.18:'c60',-0.10:'s64',-0.005:'s58',0.077:'h41',0.158:'s31'}
def slot(y):
    k=min(SL,key=lambda s:abs(s-y)); return SL[k]
T=re.compile(r'\[(\d+\.\d+)\]')
rows={}
for d in sorted(x for x in glob.glob('*/') if os.path.exists(x+'manipulation.log') and os.path.exists(x+'vision.log')):
    last_rot=None; out=[]; left=None
    five = d[:4] < '0637'
    left = set(SL.values()) if five else set(SL.values())-{'h41'}
    for l in open(d+'manipulation.log',errors='replace'):
        m=T.search(l)
        if not m: continue
        t=float(m.group(1))
        if '베이스 회전 완료' in l: last_rot=t
        g=re.search(r'책 좌표 정함 \(([-+\d.]+), ([-+\d.]+)',l)
        if g and last_rot:
            s=slot(float(g.group(2))); 
            out.append('%s %.1fs [남은 %s]'%(s,t-last_rot,'+'.join(sorted(left))))
            rows.setdefault((s,'+'.join(sorted(left)), 'old' if 'old' in d else ('site' if 'site' in d else 'home')),[]).append(round(t-last_rot,1))
            left.discard(s)
    print(d, ' | '.join(out))
print()
for k in sorted(rows): print(k, rows[k])
