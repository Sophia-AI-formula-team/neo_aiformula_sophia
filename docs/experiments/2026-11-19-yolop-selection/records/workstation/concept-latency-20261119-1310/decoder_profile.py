"""Attribute the GPU time inside one B1.4 segmentation decoder."""
import json
import sys
import time

import cv2
import torch

from concept_yolop.inference import Predictor
from concept_yolop.weights import default_path


predictor = Predictor(default_path())
image = cv2.imread(sys.argv[1])
predictor.predict(image)
decoder = predictor.model.lane_head
events = {}
previous = [None]


def timing(name):
    def begin(_, args):
        start = torch.cuda.Event(enable_timing=True)
        end = torch.cuda.Event(enable_timing=True)
        start.record()
        events[name] = start, end, previous[0], tuple(args[0].shape), str(args[0].dtype)
    def finish(*_):
        end = events[name][1]
        end.record()
        previous[0] = end
    return begin, finish


for name, layer in decoder.named_modules():
    if name and not list(layer.children()):
        begin, finish = timing(name)
        layer.register_forward_pre_hook(begin)
        layer.register_forward_hook(finish)

start = time.monotonic_ns()
predictor.predict(image)
print('wall ms', (time.monotonic_ns() - start) / 1e6)
for name, (a,b,prev,shape,dtype) in events.items():
    print(json.dumps(dict(layer=name, shape=shape, dtype=dtype, ms=round(a.elapsed_time(b), 3),
                          gap_before_ms=round(prev.elapsed_time(a), 3) if prev is not None else None)))
