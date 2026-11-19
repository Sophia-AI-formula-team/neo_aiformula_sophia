"""Run on archived frames, with one CPU thread, without ROS subscriptions."""
import json, time
from pathlib import Path
import cv2
import numpy as np
from concept_yolop.node import draw_overlay

cv2.setNumThreads(1)
base=Path.home() / 'aiformula_runtime/concept-field-20261119'
fixtures=base/'deployed-src/vehicle_concept/src/concept_drive/test/data/moving-lap-start'
times={'baseline':[], 'patched':[]}
lut=(.57*np.arange(256)[:,None,None]+.43*np.array([218,132,42])).astype(np.uint8)
def patched(im,road,lane):
    out=im.copy()
    cv2.copyTo(cv2.LUT(im,lut),road.view(np.uint8),out)
    out[lane]=(30,143,255)
    return out
empty=dict(boxes=[],scores=[],labels=[])
for rgb in sorted(fixtures.glob('*_rgb.png')):
    im=cv2.imread(str(rgb))
    with np.load(str(rgb).replace('_rgb.png','_perception.npz')) as packet:
        road,lane=packet['raw'].astype(bool),packet['lane'].astype(bool)
    for k in range(10):
        t=time.monotonic_ns(); old=draw_overlay(im,road,lane,empty); times['baseline'].append((time.monotonic_ns()-t)/1e6)
        t=time.monotonic_ns(); new=patched(im,road,lane); times['patched'].append((time.monotonic_ns()-t)/1e6)
        np.testing.assert_array_equal(old,new)
print(json.dumps(dict(samples=len(times['baseline']),exact=True,cv2=cv2.__version__,p50_p95_ms={k:list(np.percentile(v,[50,95])) for k,v in times.items()})))
