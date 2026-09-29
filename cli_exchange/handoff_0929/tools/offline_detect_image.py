# 실패 직후 손목 카메라 그림에 같은 가중치를 걸어 "문턱 아래에서 몇 점이 나오는가" 를 본다 (시스템 문턱은 안 건드린다)
import sys, cv2, numpy as np
from ultralytics import YOLO
img=cv2.imread(sys.argv[1]); m=YOLO('ros2_ws/src/shelving_perception/resource/book_tray_best.pt')
print('이름표', m.names, '그림', img.shape)
def run(tag, im, off=(0,0), sc=1.0):
    r=m.predict(im, conf=0.01, verbose=False, device='cpu')[0]
    rows=[]
    for b in r.boxes:
        x1,y1,x2,y2=[float(v) for v in b.xyxy[0]]
        rows.append((float(b.conf[0]), m.names[int(b.cls[0])], (x1/sc+off[0], y1/sc+off[1], x2/sc+off[0], y2/sc+off[1])))
    rows.sort(reverse=True)
    print('==',tag, im.shape[:2])
    for c,n,bx in rows[:8]:
        cx,cy=(bx[0]+bx[2])/2,(bx[1]+bx[3])/2
        tray = 150<cx<400 and 210<cy<420
        print('   conf %.2f %-8s 원본 좌표 상자 (%.0f,%.0f)-(%.0f,%.0f)%s'%(c,n,*bx,'  ← 트레이 안' if tray else ''))
run('원본 640', img)
# 트레이 부분만 잘라 크게 (책이 화면에서 커지면 점수가 오르는가)
for (x0,y0,x1,y1) in [(90,160,480,450),(200,200,420,430)]:
    crop=img[y0:y1,x0:x1]; sc=640/max(crop.shape[:2])
    big=cv2.resize(crop,None,fx=sc,fy=sc,interpolation=cv2.INTER_CUBIC)
    run('잘라서 %.1f배 (%d,%d)-(%d,%d)'%(sc,x0,y0,x1,y1), big, (x0,y0), sc)
