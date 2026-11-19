"""Compare two-class GPU mask extraction on six archived inputs, no ROS output."""
import argparse
import inspect
import json
import os
from pathlib import Path
import textwrap
import time

import numpy as np
import cv2
import torch
from concept_yolop.inference import Predictor
from concept_yolop.model import B14
from concept_yolop.weights import default_path
from concept_yolop.geometry import load_projection

def bool_argmax(x):
    return x.argmax(0).bool()

def binary_argmax(x):
    return x[1] > x[0]

def binary_exact_argmax(x):
    return (x[1] > x[0]) | (torch.isnan(x[1]) & ~torch.isnan(x[0]))

params=Path.home()/'.config/concept/honda-ground.json'
projection=load_projection(params)
base=Path.home()/'aiformula_runtime/concept-field-20261119'
images=[projection.rectify(cv2.imread(str(next(base.glob('marks-*/left/camera.png')))))]
images.extend(cv2.imread(str(path)) for path in sorted(base.glob('deployed-src/vehicle_concept/src/concept_drive/test/data/moving-lap-start/*_rgb.png')))
predictor=Predictor(default_path(),'cuda')
base_forward=B14.forward
argmax=base_forward
source=textwrap.dedent(inspect.getsource(base_forward))
namespace=dict(base_forward.__globals__,mask=None)
source=source.replace('self.restore(self.lane_head(features), batch, native).argmax(0)','mask(self.restore(self.lane_head(features), batch, native))').replace('self.restore(self.road_head(features), batch, native).argmax(0)','mask(self.restore(self.road_head(features), batch, native))')
exec(source,namespace)
fast_forward=namespace['forward']
with torch.no_grad():
    for i in range(6):
        predictor.predict(images[i])
    outputs=[predictor.predict(image) for image in images]
    modes=[('base',None),('comparison',binary_argmax),('nan_equal',binary_exact_argmax),('bool',bool_argmax)]
    elapsed={k:[] for k,v in modes}
    for i in range(10):
        for j in range(4):
            mode,fn=modes[(i+j)%4]
            B14.forward=argmax if fn is None else fast_forward
            namespace['mask']=fn
            for k in range(3):
                index=(3*i+k)%len(images)
                t=time.perf_counter(); out=predictor.predict(images[index]); elapsed[mode].append(1000*(time.perf_counter()-t))
                ref=outputs[index]
                for a,b in zip(out[1:3],ref[1:3]):
                    assert np.array_equal(a,b),mode
                assert all(np.array_equal(out[3][key],ref[3][key]) for key in out[3]),mode
    print(json.dumps(dict(percentiles=['median','p95'],device=torch.cuda.get_device_name(),torch=torch.__version__,n_per_variant=len(elapsed['base']),images=len(images),all_masks_and_detections_equal=True,ms={k:[round(float(x),2) for x in np.percentile(v,[50,95])] for k,v in elapsed.items()})))
