from unittest.mock import patch

import pytest
import torch

from concept_yolop.inference import native_fusion_channels


@pytest.mark.parametrize('version, capability, name, expected', [
    (8600, (8, 7), 'Orin', (112,)),
    (8700, (8, 9), 'NVIDIA GeForce RTX 4070 Ti', (112, 160)),
    (8700, (8, 9), 'another Ada GPU', ()),
    (8900, (8, 9), 'NVIDIA GeForce RTX 4070 Ti', ()),
])
def test_auto_uses_only_measured_device_paths(version, capability, name, expected):
    with patch('torch.backends.cudnn.version', return_value=version), \
            patch('torch.cuda.get_device_capability', return_value=capability), \
            patch('torch.cuda.get_device_name', return_value=name):
        assert native_fusion_channels(torch.device('cuda'), 'auto') == expected


def test_cpu_and_explicit_comparisons_need_no_gpu_discovery():
    with patch('torch.cuda.get_device_capability', side_effect=AssertionError):
        assert native_fusion_channels(torch.device('cpu'), 'auto') == ()
        assert native_fusion_channels(torch.device('cpu'), 'native112_160') == ()
        assert native_fusion_channels(torch.device('cuda'), 'stock') == ()
        assert native_fusion_channels(torch.device('cuda'), 'native112') == (112,)
        assert native_fusion_channels(torch.device('cuda'), 'native112_160') == (112, 160)
