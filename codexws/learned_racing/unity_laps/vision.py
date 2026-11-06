"""Single-frame RGB inference for the Unity lab. Each release runs in its own Python process."""
import argparse
import hashlib
import json
import os
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


class OuterEdgeSections:
    """Anchor the reference lane to a painted pavement/verge boundary.

    The first image initializes a pavement chroma sample under the forward
    camera. The RGB values of the verge itself are never configured. A dashed
    divider normally has pavement on both sides: it cannot stand in for the
    outer road edge while the car overtakes across it.
    """
    scan_depths = np.linspace(1.5, 10., 21)

    def __init__(self):
        self.pavement = None
        self.fallback = LaneSections()
        self.sources = []

    def read(self, mask, image):
        chroma = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)[:, :, 1:]
        if self.pavement is None:
            h, w = mask.shape
            ground = chroma[3*h//4:7*h//8, 2*w//5:3*w//5]
            self.pavement = np.median(ground, axis=(0, 1))
        hits = {-1: [], 1: []}
        for z in self.scan_depths:
            v = round(CY + HEIGHT*FOCAL/z)
            active = np.flatnonzero(mask[v-1:v+2].max(axis=0))
            for run in np.split(active, np.flatnonzero(np.diff(active) > 3) + 1):
                if not 1 <= len(run) < 48:
                    continue
                left, right = run[[0, -1]]
                pad = max(3, round(.12*FOCAL/z))
                if left < 2*pad or right + 2*pad >= image.shape[1]:
                    continue
                strip = chroma[v-1:v+2]
                before = np.median(strip[:, left-2*pad:left-pad], axis=(0, 1))
                after = np.median(strip[:, right+pad:right+2*pad], axis=(0, 1))
                a, b = np.linalg.norm(before-self.pavement), np.linalg.norm(after-self.pavement)
                if min(a, b) > 6 or abs(a-b) < 6:
                    continue
                # +1: pavement on the left, outer boundary; -1: inner boundary.
                side = 1 if a < b else -1
                hits[side].append((z, (.5*(left+right)-CX)*z/FOCAL))

        legacy, _ = self.fallback.read(mask)
        estimates = []
        for side in (1, -1):
            points = np.array(hits[side])
            if len(points) < 4 or np.ptp(points[:, 0]) < 1.4:
                continue
            depths, lateral = points.T
            degree = 2 if np.ptp(depths) > 3.0 else 1
            fit = np.polyfit(depths, lateral, degree)
            residual = abs(np.polyval(fit, depths)-lateral)
            keep = residual <= max(.22, 2.5*np.median(residual))
            if keep.sum() < 4 or np.ptp(depths[keep]) < 1.4:
                continue
            fit = np.polyfit(depths[keep], lateral[keep], degree)
            if np.median(abs(np.polyval(fit, depths[keep])-lateral[keep])) > .27:
                continue
            # Offset perpendicular to the observed boundary, including during
            # a swerve. Express that offset at each fixed forward depth.
            distances = np.array(DEPTHS)
            slope = np.polyval(np.polyder(fit), distances)
            offset = -HALF_LANE if side == 1 else 3*HALF_LANE
            centers = np.polyval(fit, distances) + offset*np.hypot(1, slope)
            low, high = depths[keep].min(), depths[keep].max()
            estimates.append((centers, low, high))
        line, marks = [], []
        self.sources = []
        for i, z in enumerate(DEPTHS):
            value = legacy[i]
            source = "marking" if value is not None else "missing"
            for centers, low, high in estimates:
                if low-2.5 <= z <= high+1.5 and abs(centers[i]) < 11:
                    # Preserve precise lane-mask intersections when they agree
                    # with the edge identity. Use the boundary fit to resolve
                    # a different lane or supply a missing section.
                    if value is None or abs(value-centers[i]) > 1.1:
                        value = float(centers[i])
                        source = "edge_fit" if low <= z <= high else "edge_extrapolated"
                    break
            if value is not None:
                self.fallback.last[i] = value
                marks.append((round(CX+value*FOCAL/z), round(CY+HEIGHT*FOCAL/z)))
            line.append(value)
            self.sources.append(source)
        return line, marks


def receive(sock,n):
    data=bytearray()
    while len(data)<n:
        chunk=sock.recv(n-len(data))
        if not chunk:raise EOFError
        data.extend(chunk)
    return data


def main():
    p=argparse.ArgumentParser();p.add_argument("release",type=Path);p.add_argument("ready",type=Path);p.add_argument("--no-samples",action="store_true");p.add_argument("--ignore-objects",action="store_true")
    p.add_argument("--lane-observer",choices=("legacy","outer-edge"),default=os.environ.get("AIFORMULA_LANE_OBSERVER","legacy"))
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
        meta=dict(port=server.getsockname()[1],release=root.name,sha256=digest,mode="single_frame",device="cuda",sampleStride=1,
                  objectObservations="muted" if args.ignore_objects else "enabled",laneObserver=args.lane_observer)
        tmp=args.ready.with_suffix(".tmp");tmp.write_text(json.dumps(meta),encoding="utf8");tmp.replace(args.ready)
        print("Ready",meta,flush=True)
        while True:
            connection,_=server.accept()
            with connection:
                sections=OuterEdgeSections() if args.lane_observer=="outer-edge" else LaneSections();frame=0
                try:
                    while True:
                        size=struct.unpack("<I",receive(connection,4))[0]
                        if size==0:return
                        if size>4000000:raise ValueError("Unexpected camera packet")
                        image=cv2.imdecode(np.frombuffer(receive(connection,size),np.uint8),cv2.IMREAD_COLOR)
                        out=model.predict(image)
                        if args.lane_observer=="outer-edge":
                            lane,marks=sections.read(out.lane_mask,image)
                        else:
                            lane,marks=sections.read(out.lane_mask)
                        objects=[]
                        for obj in out.detections:
                            if obj.class_id!=91:continue
                            x0,y0,x1,y1=obj.box_xyxy
                            if y1<=CY+5:continue
                            z=HEIGHT*FOCAL/(y1-CY);x=(.5*(x0+x1)-CX)*z/FOCAL
                            if z<12 and abs(x)<5.5:objects.append([z,x])
                        if not args.no_samples:
                            overlay=out.combined_overlay.copy()
                            for u,v in marks:cv2.circle(overlay,(u,v),3,(219,225,47),-1)
                            cv2.imwrite(str(samples/f"{frame:05d}.jpg"),overlay,[cv2.IMWRITE_JPEG_QUALITY,85])
                        payload=dict(lane=lane,objects=[] if args.ignore_objects else objects)
                        if args.lane_observer=="outer-edge":
                            payload["laneSources"]=sections.sources
                        msg=json.dumps(payload,allow_nan=False).encode()
                        connection.sendall(struct.pack("<I",len(msg))+msg);frame+=1
                except (EOFError,ConnectionResetError,BrokenPipeError):
                    print("Closed",frame,flush=True)
                except Exception as ex:
                    msg=json.dumps(dict(error=repr(ex))).encode()
                    connection.sendall(struct.pack("<I",len(msg))+msg)
                    print("Error",repr(ex),flush=True)

if __name__=="__main__":main()
