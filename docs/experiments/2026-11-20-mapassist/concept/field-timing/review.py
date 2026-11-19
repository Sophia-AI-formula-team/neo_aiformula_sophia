"""Review the last Nov 19 ROS 2 field session: python review.py EXTRACTED_BAG.db3.
Requires NumPy, OpenCV (cv2), Pillow and a CJK font file (--font on Linux).
Writes the summary, timeline and contact sheet alongside this script.
"""
import argparse
from datetime import datetime,timedelta,timezone
import json
import math
from pathlib import Path
import sqlite3
import struct

import cv2
import numpy as np

BASE=Path(__file__).resolve().parent
TZ=timezone(timedelta(hours=9))
def hh(ns):
    return datetime.fromtimestamp(ns/1e9,TZ).strftime('%H:%M:%S.%f')[:-3]
def hns(b):
    s,n=struct.unpack_from('<iI',b,4)
    return s*10**9+n
def jsns(st):
    return st['sec']*10**9+st['nanosec']
def ros_str(b):
    n,=struct.unpack_from('<I',b,4)
    return json.loads(b[8:8+n-1])
def after_header(b,align):
    length,=struct.unpack_from('<I',b,12)
    return 4+((12+length+align-1)//align)*align
def readbag(bag):
    f=next(bag.glob('concept*_170817_*.db3')) if bag.is_dir() else bag
    c=sqlite3.connect(f.resolve().as_uri()+'?mode=ro&immutable=1',uri=True)
    def topic(name):
        return c.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=? ORDER BY m.timestamp,m.id',(name,))
    d=[(t,ros_str(b)) for t,b in topic('/concept/decision')]
    jpeg={hns(b):b[b.index(b'\xff\xd8\xff'):] for t,b in topic('/concept/perception/overlay/compressed')}
    auto={hns(b):(t,*struct.unpack_from('<6d',b,after_header(b,8))[::5]) for t,b in topic('/concept/auto_cmd')}
    mot=[(t,*struct.unpack_from('<6d',b,4)[::5]) for t,b in topic('/concept/internal/motor_cmd')]
    can=[(hns(b),t,*struct.unpack_from('<ii',b,len(b)-8)) for t,b in topic('/concept/internal/motor_can')]
    wheel=[(hns(b),t,*struct.unpack_from('<ii',b,len(b)-8)) for t,b in topic('/aiformula_sensing/vehicle_info') if struct.unpack_from('<I',b,after_header(b,4))[0]==0x711]
    imu=[(hns(b),t,struct.unpack_from('<d',b,after_header(b,8)+8*15)[0]) for t,b in topic('/aiformula_sensing/vectornav/imu')]
    states=[ros_str(b) for _,b in topic('/concept/vehicle_state')]
    gates=[(t,ros_str(b)) for t,b in topic('/concept/gate/status')]
    topics=[r[0] for r in c.execute('SELECT name FROM topics')]
    c.close()
    return d,jpeg,auto,mot,can,wheel,imu,states,gates,topics

# Manual outer edge of the left boundary at the 5 m image scanline. Every
# frame whose forward-reachable displayed centreline reaches 5 m up to the
# first reversal is included. The green apron is on the left of the first
# outer paint; a separate parallel marking farther right is not an outer edge.
OUTER5={0:259,14:264,20:275,21:277,22:280,23:284,24:286,25:290,26:296,
        27:306,28:314,29:324,30:332,31:339,32:349,33:357,34:365,35:372,36:382}
# These are the VIRTUAL camera constants used to draw the retained overlay.
# On Nov 19 the Honda profile used the measured 0.55 m optical height (Honda
# nominal height 0.54 m) and live CameraInfo to warp flat ground into this view.
# Its 0.655 m height applies only to projection on the warped picture.
# Sources: honda-ground.json, honda-v1.4.0-ground-projection-prior.json and
# HondaMountProjection in the Nov 19 archive (linked in the accompanying report).
FX,CX,CY,VIRTUAL_HEIGHT_M=254.391622,330.01302,181.14964,.655
def ppx(z,x):
    return (int(round(CX+FX*x/z)),int(round(CY+FX*VIRTUAL_HEIGHT_M/z)))
def integrate(series,start,end):
    rel=[((t-start)/1e9,v) for t,v in series]; a=np.asarray(rel)
    ts=np.unique(np.r_[0.,a[(a[:,0]>0)&(a[:,0]<(end-start)/1e9),0],(end-start)/1e9])
    vs=np.interp(ts,a[:,0],a[:,1])
    return float(np.sum(.5*(vs[1:]+vs[:-1])*np.diff(ts)))

def analyze(data):
    ds,jpeg,auto,motor,can,wheel,imu,states,gates,topics=data
    frames=[]
    for i,(arrival,v) in enumerate(ds[:44]):
        ns=jsns(v['stamp']); sent=jsns(v['emitted_stamp']);at,lin,yaw=auto[ns]
        assert np.allclose([lin,yaw],list(v['command'].values()))
        outs=[t for t,V,W in motor if t>=at and abs(V-lin)<1e-10 and abs(W-yaw)<1e-10]
        road=np.asarray(v['road_rows']);path=np.asarray(v['selected_path'])
        x=None
        if len(path) and path[:,0].min()<=5<=path[:,0].max():
            x=float(np.interp(5.,path[:,0],path[:,1]))
        lm=float(np.interp(3.,road[:,0],road[:,1])) if len(road) else None
        direction=v['map']['pose']
        row=dict(frame=i,photo_ns=ns,photo_jst=hh(ns),auto_ns=at,auto_jst=hh(at),
                 gate_ns=outs[0] if outs else None,gate_jst=hh(outs[0]) if outs else None,
                 emitted_ns=sent,decision_bag_ns=arrival,linear=lin,yaw_ros=yaw,
                 left_at_1m=float(np.interp(1.,road[:,0],road[:,1])) if len(road) else None,
                 left_at_3m=lm,confirmed_edge=v['edges'],pose_forward_right_heading=direction,
                 measured_body_speed_mps=v['speed_estimate_mps'],five_m_photo_edge_px=OUTER5.get(i),
                 five_m_planned_center_px=None if x is None else float(CX+FX*x/5),
                 phase_ms=dict(v['camera_timing_ms'],driver_callback=v['processing_ms'],
                               capture_to_auto_packet=(at-ns)/1e6,capture_to_driver_emit=(sent-ns)/1e6))
        frames.append(row)
    used=[f for f in frames[:37] if f['five_m_planned_center_px'] is not None]
    assert all(f['five_m_photo_edge_px'] is not None for f in used)
    crossings=[f['frame'] for f in used if f['five_m_planned_center_px']+7<f['five_m_photo_edge_px']]
    assert crossings[0]==27 and 28 not in crossings and all(i in crossings for i in range(29,37))
    a,r,stop=frames[27]['photo_ns'],frames[36],frames[37]
    right=r['gate_ns']
    lo,hi=right-3_000_000_000,right+1_100_000_000
    gaps={}
    for name,ts in [('camera',[x['photo_ns'] for x in frames]),('wheel',[x[0] for x in wheel]),
                    ('raw_imu',[x[0] for x in imu]),('gate_motor_cmd',[x[0] for x in motor])]:
        times=np.sort([int(t) for t in ts if lo<=t<=hi]);gap=np.diff(times)/1e6;at=int(np.argmax(gap))
        gaps[name]=dict(median_ms=float(np.median(gap)),max_ms=float(gap[at]),
                        longest_interval=[hh(int(t)) for t in times[at:at+2]],
                        samples_straddling_gate=[hh(int(times[times<right][-1])),hh(int(times[times>=right][0]))])
    speeds=[(t,(R+L)*np.pi*.127/60) for t,ar,R,L in wheel]
    rates=[(t,v) for t,ar,v in imu]
    checks=[]
    for s in states:
        if (s.get('valid') and '17:08:26'<=hh(round(s['t']*1e9))<'17:08:30'):
            j=min(imu,key=lambda sample:abs(sample[0]/1e9-s['gyro_t']))
            if abs(j[0]/1e9-s['gyro_t'])<1e-6:
                checks.append(abs(j[2]-s['gyro']))
    assert checks and max(checks)<1e-9
    prev=next(x for x in reversed(can) if x[0]<right)
    changed=next(x for x in can if x[0]>=right and x[2:]!=prev[2:])
    return dict(stage='11/19 deployed online output, not a policy replay',five_m_manual_reading_uncertainty_px=7,
                first_outer_visible_image=hh(frames[0]['photo_ns']),first_clear_5m_planned_crossing_frame=crossings[0],
                archived_decisions_with_path=sum(bool(v.get('selected_path')) for _,v in ds),
                horizon_to_right_command=dict(between_photo_s=(r['photo_ns']-a)/1e9,
                    wheel_integral_between_photos_m=integrate(speeds,a,r['photo_ns']),
                    wheel_integral_during_processing_and_gate_m=integrate(speeds,r['photo_ns'],right),
                    wheel_integral_while_right_gate_m=integrate(speeds,right,stop['gate_ns']),
                    measured_imu_net_while_right_gate_rad=integrate(rates,right,stop['gate_ns']),
                    gate_right_duration_ms=(stop['gate_ns']-right)/1e6),
                raw_imu_near_right=[dict(source_jst=hh(t),receipt_jst=hh(ar),yaw_ros_rad_s=v) for t,ar,v in imu
                                    if right-70_000_000<=t<=stop['gate_ns']+50_000_000],
                wheel_first_below_left=next([hh(t),R,L] for t,ar,R,L in wheel if t>right and R<L),
                wheel_first_both_zero_after_stop=next(hh(t) for t,ar,R,L in wheel if t>stop['gate_ns'] and R==L==0),
                internal_can_before=[hh(prev[0]),*prev[2:]],internal_can_changed=[hh(changed[0]),*changed[2:]],
                final_can_topic_recorded='/aiformula_control/motor_controller/reference_signal' in topics,
                gate_modes=sorted(set(g['mode'] for t,g in gates if lo<=t<=hi)),sample_window=[hh(lo),hh(hi)],sample_gaps=gaps,
                imu_source_matched_vehicle_state_samples=len(checks),frames=frames)

def plot(data,result,fontname):
    from PIL import Image,ImageDraw,ImageFont
    from functools import lru_cache
    @lru_cache(None)
    def font(s):return ImageFont.truetype(fontname,s)
    ds,images,auto,motor,can,wheel,imu,states,gates,topics=data
    frames=result['frames']
    out=Image.new('RGB',(1440,940),'#f2f5f4'); dr=ImageDraw.Draw(out)
    def text(x,y,s,z=22,color='#153c44'):
        dr.text((x,y),s,font=font(z),fill=color,anchor='lt')
    text(24,16,'实车何时开始回正？把照片、指令和车身转动对到同一时钟',36)
    text(24,73,'11 月 19 日末场 · 日本时间 17:08    橙色：车线识别　粉色：当时选的中心路线　绿点：前方 5 m 处手读的缓冲区一侧边缘',20)
    for col,(i,title,caption) in enumerate([(0,'首帧已有可辨认的左外缘','当时选的短路线未达到前方 5 m。'),
                   (27,'预测中心越过前方 5 m 的左外缘','中心约 296 px，边缘约 306 px。'),
                   (35,'贴边时仍在请求微左转','中心约 330 px，边缘约 372 px。')]):
        f=frames[i];v=ds[i][1];ns=f['photo_ns']
        bgr=cv2.imdecode(np.frombuffer(images[ns],np.uint8),cv2.IMREAD_COLOR)
        pts=[ppx(z,x) for z,x in v['selected_path'] if z>.96]
        if len(pts)>1:
            cv2.polylines(bgr,[np.array(pts,np.int32)],False,(74,18,67),5,cv2.LINE_AA)
            cv2.polylines(bgr,[np.array(pts,np.int32)],False,(209,65,234),3,cv2.LINE_AA)
        g=(OUTER5[i],ppx(5,0)[1]);cv2.circle(bgr,g,5,(35,49,35),-1,cv2.LINE_AA);cv2.circle(bgr,g,3,(124,255,130),-1,cv2.LINE_AA)
        # Crop a fixed view to retain the full lateral extent and all near-road pixels.
        x=24+col*468; y=150
        text(x,115,f['photo_jst']+'  '+title,19)
        out.paste(Image.fromarray(cv2.cvtColor(bgr[139:],cv2.COLOR_BGR2RGB)).resize((456,157)),(x,y))
        text(x,319,caption,18)
    text(24,372,'门控向电机节点发出的转向与陀螺仪实测',27)
    text(26,415,'单位 rad/s，向左为正；零线下方向右。',19)
    dr.line((735,414,777,414),fill='#9c467d',width=4);text(785,403,'门控发给电机节点的指令',18)
    dr.line((1069,414,1111,414),fill='#147d7d',width=3);text(1120,403,'陀螺仪原始测量',18)
    base=frames[36]['photo_ns']//10**9*10**9
    l,r,t,b=100,1396,468,652
    xmin,xmax=-2.3,1.65; ymin,ymax=-.055,.235
    X=lambda n: l+(r-l)*((n-base)/1e9-xmin)/(xmax-xmin)
    Y=lambda v:b-(b-t)*(v-ymin)/(ymax-ymin)
    for val in [-.05,0.,.1,.2]:
        y=Y(val);dr.line((l,y,r,y),fill='#bac9c7' if val!=0 else '#617775',width=2 if val==0 else 1);text(25,y-8,'%.2f'%val,16)
    for sec in (-2,-1,0,1):
        x=X(base+sec*10**9);dr.line((x,t,x,b),fill='#d7dddc',width=1);text(x-28,b+9,hh(base+sec*10**9)[:8],17)
    for rows,color,step in [(motor,'#9c467d',True),(imu,'#147d7d',False)]:
        source=[(X(z),Y(value)) for z,_,value in rows if l<=X(z)<=r]
        for a,k in zip(source,source[1:]):
            dr.line([a,(k[0],a[1]),k] if step else [a,k],fill=color,width=3 if step else 2)
        if not step:
            for x,y in source:dr.ellipse((x-1.5,y-1.5,x+1.5,y+1.5),fill=color)
    for ns,txt,xoff,yoff in [(frames[27]['photo_ns'],'5 m 处首次明确越线',-70,-32),(frames[36]['gate_ns'],'首个向右指令',-106,-32),
                              (frames[37]['gate_ns'],'改发停车',8,-8)]:
        x=X(ns);dr.line((x,t,x,b),fill='#627569',width=1);text(x+xoff,t+yoff,txt,17)
    text(25,712,'产生第一条向右指令的这一帧',26)
    text(25,756,'28.415  相机时间戳',22)
    text(370,756,'28.680  自动指令入包',22)
    text(786,756,'28.701  门控指令入包',22)
    text(1130,756,'28.900  停车指令入包',22)
    text(25,802,'从相机时间戳到门控指令入包约 286 ms；按轮速积分，车在此期间向前走了约 0.64 m。',23)
    text(25,845,'但画面中 5 m 处已有明确风险，到产生右转指令的那张图，两次拍摄相隔 1.85 s，车已经走了约 4.21 m。',21)
    text(25,894,'照片看不到近车轮胎接地点。图中越线指候选路线在前方穿过外缘，无法据此判罚车轮当时已完全越线。',19)
    out.save(BASE/'外缘与回正时间线.png')

def contact_sheet(data):
    from PIL import Image,ImageDraw
    ds,images,*_=data
    canvas=Image.new('RGB',(1080,1015),'white')
    draw=ImageDraw.Draw(canvas)
    for j,i in enumerate([0,8,17,20,22,24,26,28,29,30,31,32,33,34,35,36,37,39]):
        v=ds[i][1]; ns=jsns(v['stamp'])
        b=cv2.imdecode(np.frombuffer(images[ns],np.uint8),cv2.IMREAD_COLOR)
        pts=np.asarray([ppx(z,x) for z,x in v['selected_path'] if z>.98],dtype=np.int32)
        if len(pts)>1:cv2.polylines(b,[pts],False,(232,66,226),3)
        x,y=j%3*360,j//3*169
        canvas.paste(Image.fromarray(cv2.cvtColor(b[142:],cv2.COLOR_BGR2RGB)).resize((360,138)),(x,y+21))
        draw.text((x+2,y),str(i)+' '+hh(ns),fill='black')
    canvas.save(BASE/'contact.jpg',quality=92)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path,help='Extracted last-session .db3 or the folder directly containing it')
    p.add_argument('--font',default='C:/Windows/Fonts/msyh.ttc')
    args=p.parse_args();data=readbag(args.bag); detail=analyze(data)
    (BASE/'events.json').write_text(json.dumps(detail,indent=2,ensure_ascii=False),encoding='utf-8')
    plot(data,detail,args.font)
    contact_sheet(data)
    print('frames',len(detail['frames']),'first right',detail['frames'][36]['gate_jst'])

if __name__=='__main__':
    main()
