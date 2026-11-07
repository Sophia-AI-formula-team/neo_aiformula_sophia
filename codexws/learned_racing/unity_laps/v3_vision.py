"""A4/RGB to V3 local road + controller, with no map, pose or station input."""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import struct
import sys
import cv2
import numpy as np
from vision import receive,FOCAL,CX,CY,HEIGHT
from v3_model import Planner,Observation,LocalTracker

class VisibleRoad:
    def __init__(self):self.pavement=None
    def read(self,image,lane_mask):
        lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB).astype(float)
        if self.pavement is None:self.pavement=np.median(lab[265:310,275:385].reshape(-1,3),axis=0)
        delta=lab-self.pavement
        road=(np.linalg.norm(delta[:,:,1:],axis=2)<12)&(abs(delta[:,:,0])<52)
        road|=lane_mask>0
        sections=[];pixels=[]
        for z in np.linspace(1.2,10.,27):
            v=round(CY+HEIGHT*FOCAL/z)
            if not 1<=v<image.shape[0]-1:continue
            mask=(road[v-1:v+2].mean(axis=0)>.66).astype(np.uint8)
            mask=cv2.morphologyEx(mask[None,:],cv2.MORPH_CLOSE,np.ones((1,5),np.uint8))[0]
            indices=np.flatnonzero(mask)
            groups=[g for g in np.split(indices,np.flatnonzero(np.diff(indices)>1)+1) if len(g)>8]
            if not groups:continue
            g=min(groups,key=lambda g:max(g[0]-CX,CX-g[-1],0))
            left=(float(g[0])-CX)*z/FOCAL;right=(float(g[-1])-CX)*z/FOCAL
            if right-left<.3:continue
            # Image-boundary clipping is observation coverage, not road boundary
            # confirmation. The conservative allowed region stays inside it.
            sections.append([float(z),left,right,.06+.006*z])
            pixels.append([int(g[0]),int(g[-1]),int(v)])
        return np.asarray(sections).reshape(-1,4),pixels

def main():
    p=argparse.ArgumentParser();p.add_argument("release",type=Path);p.add_argument("ready",type=Path)
    p.add_argument("--model",type=Path);a=p.parse_args();sys.path.insert(0,str(a.release))
    from yolop_a3_runtime import PerceptionModel
    weights=a.release/"weights/shared_best.pth";sha=hashlib.sha256(weights.read_bytes()).hexdigest()
    if json.loads((a.release/"weights/manifest.json").read_bytes())["sha256"]!=sha:raise ValueError("A4 weight hash")
    model=PerceptionModel(str(weights),device="cuda");model.predict(np.zeros((360,640,3),np.uint8))
    params=json.loads(a.model.read_bytes())["parameters"] if a.model else None
    with socket.socket() as server:
        server.bind(("127.0.0.1",0));server.listen(1)
        control_sha=hashlib.sha256((Path(__file__).parent/"v3_model.py").read_bytes()+Path(__file__).read_bytes()+(a.model.read_bytes() if a.model else b"default")).hexdigest()
        a.ready.write_text(json.dumps(dict(port=server.getsockname()[1],weights=sha,controlSha=control_sha,version="V3.1" if params else "V3.0",sensorContract="camera pixels, wheel proxy RPM, raw gyro")),encoding="utf8")
        while True:
            connection,_=server.accept()
            with connection:
                observer=VisibleRoad();planner=Planner(params);tracker=LocalTracker();last=None;frame=0
                try:
                    while True:
                        n=struct.unpack("<I",receive(connection,4))[0]
                        if n==0:return
                        if n>2048:raise ValueError("Unexpected sensor packet")
                        sensor=json.loads(receive(connection,n))
                        if set(sensor)!={"t","leftRpm","rightRpm","gyro","wheelRadius","source"}:raise ValueError("Forbidden or missing sensor field")
                        n=struct.unpack("<I",receive(connection,4))[0]
                        if n>4000000:raise ValueError("Unexpected image size")
                        image=cv2.imdecode(np.frombuffer(receive(connection,n),np.uint8),cv2.IMREAD_COLOR)
                        prediction=model.predict(image);road,pixels=observer.read(image,prediction.lane_mask)
                        measured=[]
                        for obj in prediction.detections:
                            if obj.class_id!=91:continue
                            x0,y0,x1,y1=obj.box_xyxy
                            if y1<=CY+5:continue
                            z=HEIGHT*FOCAL/(y1-CY);x=(.5*(x0+x1)-CX)*z/FOCAL
                            if z<12:measured.append([z,x])
                        dt=.2 if last is None else float(sensor["t"]-last);last=float(sensor["t"])
                        speed=max(0,(sensor["leftRpm"]+sensor["rightRpm"])*np.pi*sensor["wheelRadius"]/60)
                        objects=tracker.update(measured,speed,sensor["gyro"],dt)
                        command=planner.step(Observation(road,objects,speed,sensor["gyro"],dt))
                        decision=dict(**planner.last,road=road.tolist(),objects=objects.tolist(),speedEstimate=speed,roadSources="current RGB pavement and A4 lane pixels only",sensor=sensor)
                        payload=dict(control=command.tolist(),decision=decision)
                        overlay=prediction.combined_overlay.copy()
                        for x0,x1,v in pixels:cv2.line(overlay,(x0,v),(x1,v),(220,180,70),1)
                        frames=a.ready.parent/"frames";frames.mkdir(exist_ok=True)
                        cv2.imwrite(str(frames/f"{frame:05d}.jpg"),overlay);frame+=1
                        msg=json.dumps(payload,allow_nan=False).encode();connection.sendall(struct.pack("<I",len(msg))+msg)
                except (EOFError,ConnectionResetError,BrokenPipeError):pass
                except Exception as e:
                    msg=json.dumps(dict(error=repr(e))).encode();connection.sendall(struct.pack("<I",len(msg))+msg);raise
if __name__=="__main__":main()
