"""Single-frame RGB inference for the Unity lab. Each release runs in its own Python process."""
import argparse
import hashlib
import json
import socket
import struct
import sys
from pathlib import Path

import cv2
import numpy as np

FOCAL=254.391622
CX,CY=330.01302,181.14964
HEIGHT=.655
DEPTHS=(2.,3.5,5.5,8.)
HALF_LANE=1.715255


class LaneSections:
    """Associate visible lane-marking runs along camera rays, preserving curved geometry."""
    def __init__(self):
        self.last=np.zeros(4)

    def read(self,mask):
        line=[];marks=[]
        for i,z in enumerate(DEPTHS):
            v=round(CY+HEIGHT*FOCAL/z)
            active=np.flatnonzero(mask[v-1:v+2].max(axis=0))
            runs=np.split(active,np.flatnonzero(np.diff(active)>3)+1)
            centers=np.array([(.5*(g[-1]+g[0])-CX)*z/FOCAL for g in runs if 1<=len(g)<=36])
            local=self.last[i]
            if i and line[i-1] is not None:
                local=.65*line[i-1]+.35*local
                if i>1 and line[i-2] is not None:
                    local+=.7*(line[i-1]-line[i-2])*(z-DEPTHS[i-1])/(DEPTHS[i-1]-DEPTHS[i-2])
            edges=[]
            for sign in (-1,1):
                if not centers.size:continue
                expected=local+sign*HALF_LANE
                j=int(np.argmin(abs(centers-expected)))
                if abs(centers[j]-expected)<1.20+.055*z:
                    edges.append(float(centers[j])-sign*HALF_LANE)
            x=float(np.median(edges)) if edges else None
            if x is not None:
                self.last[i]=x
                marks.append((round(CX+x*FOCAL/z),v))
            line.append(x)
        return line,marks


def receive(sock,n):
    data=bytearray()
    while len(data)<n:
        chunk=sock.recv(n-len(data))
        if not chunk:raise EOFError
        data.extend(chunk)
    return data


def main():
    p=argparse.ArgumentParser();p.add_argument("release",type=Path);p.add_argument("ready",type=Path)
    args=p.parse_args();root=args.release.resolve();args.ready.parent.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(root))
    from yolop_a3_runtime import PerceptionModel
    weight=root/"weights/shared_best.pth";digest=hashlib.sha256(weight.read_bytes()).hexdigest()
    manifest=json.loads((root/"weights/manifest.json").read_bytes())
    if manifest["sha256"]!=digest:raise ValueError("Release hash mismatch")
    model=PerceptionModel(str(weight),device="cuda")
    model.predict(np.zeros((360,640,3),np.uint8))
    samples=args.ready.parent/(root.name+"_samples");samples.mkdir(exist_ok=True)
    with socket.socket() as server:
        server.bind(("127.0.0.1",0));server.listen(1)
        meta=dict(port=server.getsockname()[1],release=root.name,sha256=digest,mode="single_frame",device="cuda",sampleStride=1)
        tmp=args.ready.with_suffix(".tmp");tmp.write_text(json.dumps(meta),encoding="utf8");tmp.replace(args.ready)
        print("Ready",meta,flush=True)
        while True:
            connection,_=server.accept()
            with connection:
                sections=LaneSections();frame=0
                try:
                    while True:
                        size=struct.unpack("<I",receive(connection,4))[0]
                        if size==0:return
                        if size>4000000:raise ValueError("Unexpected camera packet")
                        image=cv2.imdecode(np.frombuffer(receive(connection,size),np.uint8),cv2.IMREAD_COLOR)
                        out=model.predict(image);lane,marks=sections.read(out.lane_mask);objects=[]
                        for obj in out.detections:
                            if obj.class_id!=91:continue
                            x0,y0,x1,y1=obj.box_xyxy
                            if y1<=CY+5:continue
                            z=HEIGHT*FOCAL/(y1-CY);x=(.5*(x0+x1)-CX)*z/FOCAL
                            if z<12 and abs(x)<5.5:objects.append([z,x])
                        overlay=out.combined_overlay.copy()
                        for u,v in marks:cv2.circle(overlay,(u,v),3,(219,225,47),-1)
                        cv2.imwrite(str(samples/f"{frame:05d}.jpg"),overlay,[cv2.IMWRITE_JPEG_QUALITY,85])
                        msg=json.dumps(dict(lane=lane,objects=objects),allow_nan=False).encode()
                        connection.sendall(struct.pack("<I",len(msg))+msg);frame+=1
                except (EOFError,ConnectionResetError,BrokenPipeError):
                    print("Closed",frame,flush=True)
                except Exception as ex:
                    msg=json.dumps(dict(error=repr(ex))).encode()
                    connection.sendall(struct.pack("<I",len(msg))+msg)
                    print("Error",repr(ex),flush=True)

if __name__=="__main__":main()
