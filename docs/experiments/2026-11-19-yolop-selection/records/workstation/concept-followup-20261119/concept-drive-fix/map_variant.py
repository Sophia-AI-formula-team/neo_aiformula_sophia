"""CPU-only repeatable camera projection/overlay experiment."""
import argparse
import json
import time
import cv2
import numpy as np
from concept_yolop.geometry import HondaMountProjection

def fixed_remap(warp, w, h):
    # OpenCV's warpPerspective uses 5 fraction bits with linear interpolation.
    y, x = np.ogrid[:h, :w]
    inv = cv2.invert(warp)[1]
    denom = inv[2, 0] * x + inv[2, 1] * y + inv[2, 2]
    scale = np.divide(32., denom, where=(denom != 0), out=np.zeros_like(denom))
    ix, iy = [np.rint((row[0]*x + row[1]*y + row[2])*scale).astype(np.int32) for row in inv[:2]]
    xy = np.stack((ix>>5, iy>>5), axis=-1).clip(-32768,32767).astype(np.int16)
    alpha = (((iy & 31)<<5) + (ix & 31)).astype(np.uint16)
    return xy, alpha

def bench(fn, n=100):
    times = []
    for _ in range(n):
        tick=time.monotonic_ns(); fn(); times.append((time.monotonic_ns()-tick)*1e-6)
    return [round(np.median(times),3),round(np.percentile(times,95),3)]
parser=argparse.ArgumentParser()
parser.add_argument('params')
parser.add_argument('image')
args=parser.parse_args()
cv2.setNumThreads(1)
mount=HondaMountProjection(json.load(open(args.params)))
im=cv2.imread(args.image)
road=im[:,:,0] > 100
lane=im[:,:,1] > 245
maps=cv2.initUndistortRectifyMap(np.eye(3),None,mount.warp,np.eye(3),(640,360),cv2.CV_16SC2)
warp=lambda:cv2.warpPerspective(im,mount.warp,(640,360),flags=cv2.INTER_LINEAR)
remap=lambda:cv2.remap(im,*maps,interpolation=cv2.INTER_LINEAR)
lut=(.57*np.arange(256)[:,None,None]+.43*np.array([218,132,42])).astype(np.uint8)
def old_overlay():
    out=im.copy()
    out[road]=.57*out[road]+.43*np.array([218,132,42])
    out[lane]=(30,143,255)
    return out
def new_overlay():
    out=im.copy()
    cv2.copyTo(cv2.LUT(im,lut),road.view(np.uint8),out)
    out[lane]=(30,143,255)
    return out
for typ in ['zed','noise']:
    if typ=='noise':
        im=np.random.default_rng(3023).integers(0,256,im.shape,dtype=np.uint8)
    print(json.dumps(dict(image=typ, cv2=cv2.__version__, projection_differences=np.count_nonzero(warp()!=remap()).item() if hasattr(np.count_nonzero(warp()!=remap()),'item') else int(np.count_nonzero(warp()!=remap())), overlay_differences=int(np.count_nonzero(old_overlay()!=new_overlay())), warp=bench(warp), remap=bench(remap), old_overlay=bench(old_overlay), new_overlay=bench(new_overlay), road_fraction=round(np.mean(road),3))),flush=True)
