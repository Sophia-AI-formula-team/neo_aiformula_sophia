"""Verify the image's GPU operators, native planner extensions and model identities."""
import hashlib
import importlib
import json
import os
from pathlib import Path

import scipy
import torch
import torchvision
from scipy.optimize import linear_sum_assignment
from torchvision.ops import nms
from concept_yolop.weights import verify

weight = Path(os.environ.get('CONCEPT_WEIGHTS', '/weights/B1.4.pth'))
with weight.open('rb') as file:
    verify(file)
h12 = Path(importlib.import_module('concept_ranker').__file__).with_name('model-h12.json')
extensions = ['concept_road._memory', 'concept_road._markings', 'concept_candidates._progress',
              'concept_feasibility._motion', 'concept_feasibility._tread', 'concept_feasibility._risk']
native = [importlib.import_module(n).__file__ for n in extensions]
assert all(p.endswith('.so') for p in native)
assert linear_sum_assignment([[3, 0], [0, 5]])[1].tolist() == [1, 0]
boxes = torch.tensor([[0., 0., 10., 10.], [1., 1., 10., 10.]], device='cuda')
indices = nms(boxes, torch.tensor([.98, .9], device='cuda'), .5)
assert indices.is_cuda and indices.tolist() == [0]
print(json.dumps(dict(gpu=torch.cuda.get_device_name(), cuda=torch.version.cuda, torch=torch.__version__,
                      torchvision=torchvision.__version__, nms=indices.tolist(), scipy=scipy.__version__,
                      native=native, b14=hashlib.sha256(weight.read_bytes()).hexdigest(),
                      h12=hashlib.sha256(h12.read_bytes()).hexdigest()), indent=2))
