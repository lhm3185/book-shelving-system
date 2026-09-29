import re,sys,glob,os
SL={-0.18:'c60',-0.10:'s64',-0.005:'s58',0.077:'h41',0.158:'s31'}
def slot(y):
    k=min(SL,key=lambda s:abs(s-y)); return SL[k] if abs(k-y)<0.03 else 'etc(%.2f)'%y
P=re.compile(r'\[(\d+\.\d+)\].*Book picked\(best\): xyz=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\).*conf=([\d.]+).*후보 (\d+)권')
D=re.compile(r'\[(\d+\.\d+)\].*책 좌표 정함 \(([-+\d.]+), ([-+\d.]+)')
F=re.compile(r'\[(\d+\.\d+)\].*(411|책을 찾지 못|검출 실패|timeout)')
for d in sorted(glob.glob('*/')):
    v=open(d+'vision.log',errors='replace').read().splitlines()
    m=open(d+'manipulation.log',errors='replace').read().splitlines() if os.path.exists(d+'manipulation.log') else []
    picks=[(float(a),float(y),float(c),int(n)) for a,x,y,z,c,n in (g.groups() for g in map(P.search,v) if g)]
    dec=[(float(g.group(1)),float(g.group(3))) for g in map(D.search,m+v) if g]
    dec=sorted(set(dec))
    print('==',d, '정함', [slot(y) for _,y in dec])
    edges=[0]+[t for t,_ in dec]+[9e18]
    for i in range(len(edges)-1):
        seg=[p for p in picks if edges[i]<p[0]<=edges[i+1]]
        by={}
        for t,y,c,n in seg: by.setdefault(slot(y),[]).append(c)
        print('   구간',i+1,'(→',slot(dec[i][1]) if i<len(dec) else '끝/실패',')', {k:'%d회 conf %.2f~%.2f'%(len(c),min(c),max(c)) for k,c in by.items()}, '후보수', sorted(set(n for *_,n in seg)))
