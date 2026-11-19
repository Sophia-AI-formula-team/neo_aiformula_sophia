"""Recover full observations from recorded static filter updates and check horizontal parallax."""
from pathlib import Path
import sqlite3, json, numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
dbfile=next((Path.home()/'rosbags/concept').glob('concept_20261119_163533*/*.db3'))
db=sqlite3.connect('file:'+str(dbfile)+'?mode=ro',uri=True)
prev=None
fixes={}
new=[]
reject={}
def mat(a):return np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]])
rows=db.execute('select data from messages join topics on topics.id=messages.topic_id where topics.name="/concept/decision" order by timestamp')
for frame,(raw,) in enumerate(rows):
    d=json.loads(deserialize_message(raw,String).data)
    if not d.get('map'):continue
    pose=np.array(d['map']['pose']);now=d['map']['t']; session=d['session']
    tracks={t['id']:(t,np.asarray(o)) for t,o in zip(d['tracks'],d['objects'])}
    if prev and session==prev[0]:
        _,old,then,prior=prev
        rot=mat(pose[2]-old[2]);shift=mat(old[2])@(pose[:2]-old[:2]);dt=now-then
    for uid,(t,o) in tracks.items():
        if t['source']=='new':
            point=o[:2]
        elif t['source']=='complete_box_update' and prev and session==prev[0] and uid in prior:
            _,old_o=prior[uid]
            mean=rot@(old_o[26:28]-shift)
            pcov=rot@old_o[30:46].reshape(4,4)[:2,:2]@rot.T
            travel=old_o[73]+np.linalg.norm(shift)
            angle=old_o[74]+abs(pose[2]-old[2]);radius=max(old_o[75],np.linalg.norm(mean))
            extra=.02**2*dt+max(0,(.01*travel+.02*angle*radius)**2-old_o[76]**2)
            pcov+=np.eye(2)*extra
            cov=o[30:46].reshape(4,4)[:2,:2]
            gain=np.eye(2)-cov@np.linalg.inv(pcov)
            point=mean+np.linalg.solve(gain,o[26:28]-mean)
        else:
            continue
        z,x=point*(1.-.24/np.linalg.norm(point));u=330.01302+254.391622*x/z
        # Black bases finish on an integer row. Small shell masks add one pixel,
        # large masks add 5% of integer component height.
        possibilities=[]
        for h in [.57875,.605]:
            v=181.14964+254.391622*h/z
            grid=20 if h==.57875 else 1
            possibilities.append((abs(v-round(v*grid)/grid),v,h))
        err,v,h=min(possibilities)
        if abs(u*2-round(u*2))>.03 or err>.01:
            reject['not_on_pixel_grid']=reject.get('not_on_pixel_grid',0)+1
            continue
        corrected=z*.655/h
        if t['source']=='new':
            new.append(dict(frame=frame,id=uid,uv=[round(u,2),round(v,2)],old_center=point.tolist(),old_contact=z,field_contact=corrected,field_right=x*.655/h))
        # Cross-check horizontal bearing with odometry from different views.
        direction=mat(-pose[2])@np.array([1.,x/z]);n=np.array([-direction[1],direction[0]])/np.linalg.norm(direction)
        fixes.setdefault((session,uid),[]).append((frame,pose,z,corrected,n,u,v))
    prev=session,pose,now,tracks
results=[]
parallax=[]
for (s,uid),lst in fixes.items():
    if len(lst)<8:continue
    A=np.array([q[4] for q in lst]);b=np.array([np.dot(q[4],q[1][:2]) for q in lst])
    target=np.linalg.lstsq(A,b,rcond=None)[0]
    real=np.array([(mat(q[1][2])@(target-q[1][:2]))[0] for q in lst]);raw=np.array([q[2] for q in lst]);field=np.array([q[3] for q in lst]);p=np.array([q[1][:2] for q in lst]);svals=np.linalg.svd(A,compute_uv=False)
    for j,q in enumerate(lst):
        parallax.append(dict(id=uid,frame=q[0],pose=q[1].tolist(),u=q[5],v=q[6],tri=float(real[j]),nominal=float(field[j])))
    results.append(dict(id=uid,frames=[lst[0][0],lst[-1][0]],n=len(lst),condition=float(svals[0]/svals[1]), trans_m=np.ptp(p,axis=0).tolist(), median_abs_cross_bearing_m=float(np.median(abs(A@target-b))),median_signed_old_minus_bearing_m=float(np.median(raw-real)),median_signed_field_minus_bearing_m=float(np.median(field-real))))
print(json.dumps(dict(range_check=new,bearing_diagnostics=results,observations=parallax,rejected=reject),indent=2))
