"""Exercise production image methods offline, without ROS, YOLOP or a GPU.

Only ROS message conversion and the inference boundary are doubled. The actual
image_callback/publish_result bodies, NumPy and OpenCV execute unchanged. This
is not a model-inference or ROS transport test.
"""

import ast
from contextlib import nullcontext
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest

import cv2
import numpy as np


_SOURCE = (
    Path(__file__).resolve().parents[1] / 'road_detector' / 'road_detector.py'
)


class _Tensor:
    """Small NumPy-backed double for the callback's inference boundary."""

    def __init__(self, values):
        self.values = np.asarray(values)

    @property
    def shape(self):
        return self.values.shape

    def to(self, unused_device):
        return self

    def half(self):
        return self

    def float(self):
        return self

    def ndimension(self):
        return self.values.ndim

    def unsqueeze(self, dimension):
        return _Tensor(np.expand_dims(self.values, dimension))

    def __getitem__(self, key):
        return _Tensor(self.values[key])

    def __gt__(self, other):
        return _Tensor(self.values > other)

    def int(self):
        return _Tensor(self.values.astype(np.int32))

    def squeeze(self):
        return _Tensor(self.values.squeeze())

    def cpu(self):
        return self

    def numpy(self):
        return self.values


def _header(frame_id='zed_left_camera_optical_frame'):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=123, nanosec=987654321), frame_id=frame_id
    )


class _Bridge:
    """Capture requested encodings and pixels at the ROS conversion boundary."""

    def imgmsg_to_cv2(self, message, encoding):
        if encoding != 'bgr8':
            raise AssertionError('Unexpected input encoding')
        return message.pixels.copy()

    def cv2_to_imgmsg(self, pixels, encoding):
        return SimpleNamespace(
            pixels=pixels.copy(), encoding=encoding, header=_header('')
        )


def _production_methods():
    tree = ast.parse(_SOURCE.read_text(encoding='utf-8'), filename=str(_SOURCE))
    detector = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'RoadDetector'
    )
    # Execute the production copy import as well: removing it must fail tests.
    imports = [
        node for node in tree.body
        if isinstance(node, ast.ImportFrom) and node.module == 'copy'
    ]
    methods = [
        node for node in detector.body
        if isinstance(node, ast.FunctionDef)
        and node.name in ('publish_result', 'image_callback')
    ]
    module = ast.Module(body=imports + methods, type_ignores=[])

    def interpolate(tensor, scale_factor, mode, align_corners):
        if (scale_factor, mode, align_corners) != (1, 'bilinear', False):
            raise AssertionError('Unexpected interpolation contract')
        return tensor

    torch = SimpleNamespace(
        no_grad=nullcontext,
        sigmoid=lambda tensor: _Tensor(1.0 / (1.0 + np.exp(-tensor.values))),
        max=lambda tensor, axis: (
            _Tensor(np.max(tensor.values, axis=axis)),
            _Tensor(np.argmax(tensor.values, axis=axis)),
        ),
        nn=SimpleNamespace(functional=SimpleNamespace(interpolate=interpolate)),
    )
    namespace = {'np': np, 'cv2': cv2, 'torch': torch, 'CvBridgeError': ValueError}
    exec(compile(module, str(_SOURCE), 'exec'), namespace)
    return namespace


class TestOutputContract(unittest.TestCase):
    """Keep source headers and the existing full-frame pixel contract intact."""

    def setUp(self):
        self.methods = _production_methods()
        self.annotated = []
        self.masks = []
        self.node = SimpleNamespace(
            cv_bridge=_Bridge(),
            annotated_mask_image_pub=SimpleNamespace(
                publish=self.annotated.append
            ),
            lane_mask_image_pub=SimpleNamespace(publish=self.masks.append),
        )
        for name in ('image_callback', 'publish_result'):
            setattr(self.node, name, MethodType(self.methods[name], self.node))
        self.image = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)

    def test_publish_preserves_full_mask_and_encodings(self):
        mask = np.array(
            [[1, 0, 2, 0], [0, 255, 0, 1], [0, 0, 1, 0]], dtype=np.uint8
        )
        image_before = self.image.copy()
        mask_before = mask.copy()
        source_header = _header()
        self.node.publish_result(self.image, mask, source_header)
        self.assertEqual(len(self.masks), 1)
        self.assertEqual(len(self.annotated), 1)
        published = self.masks[0]
        self.assertEqual(published.encoding, 'mono8')
        self.assertEqual(published.pixels.dtype, np.uint8)
        np.testing.assert_array_equal(published.pixels, (mask > 0) * 255)
        self.assertEqual(self.annotated[0].encoding, 'bgr8')
        overlay = np.zeros_like(self.image)
        overlay[mask > 0] = (0, 255, 0)
        np.testing.assert_array_equal(
            self.annotated[0].pixels,
            cv2.addWeighted(self.image, 1.0, overlay, 0.5, 0),
        )
        np.testing.assert_array_equal(self.image, image_before)
        np.testing.assert_array_equal(mask, mask_before)

    def test_headers_are_full_independent_deep_copies(self):
        source_header = _header()
        self.node.publish_result(self.image, np.ones((3, 4)), source_header)
        mask_header = self.masks[0].header
        annotated_header = self.annotated[0].header
        for output_header in (mask_header, annotated_header):
            self.assertEqual(output_header, source_header)
            self.assertIsNot(output_header, source_header)
            self.assertIsNot(output_header.stamp, source_header.stamp)
        self.assertIsNot(mask_header, annotated_header)
        self.assertIsNot(mask_header.stamp, annotated_header.stamp)
        source_header.frame_id = 'source_changed'
        source_header.stamp.sec = 999
        self.assertEqual(mask_header, _header())
        self.assertEqual(annotated_header, _header())
        mask_header.stamp.nanosec = 0
        mask_header.frame_id = 'mask_changed'
        self.assertEqual(annotated_header, _header())

    def test_empty_source_frame_is_not_fabricated(self):
        source_header = _header('')
        source_header.stamp.sec = 0
        source_header.stamp.nanosec = 0
        self.node.publish_result(self.image, np.zeros((3, 4)), source_header)
        self.assertEqual(self.masks[0].header, source_header)
        self.assertEqual(self.annotated[0].header, source_header)

    def test_callback_passes_full_header_for_all_model_output_forms(self):
        expected_mask = np.array(
            [[0, 1, 0, 1], [1, 0, 1, 0], [0, 0, 1, 1]], dtype=np.uint8
        )
        self.node.architecture = 'cpu'
        self.node.padding_image = lambda pixels: (pixels, 1.0, 0, 0, 0, 0)
        self.node.image_to_tensor = lambda pixels: _Tensor(
            pixels.transpose(2, 0, 1)
        )
        for channels in (1, 2):
            for container in (lambda value: value, tuple, list):
                for half_precision in (False, True):
                    with self.subTest(
                        channels=channels, container=container,
                        half_precision=half_precision,
                    ):
                        self.annotated.clear()
                        self.masks.clear()
                        if channels == 1:
                            scores = np.where(expected_mask, 10.0, -10.0)
                            tensor = _Tensor(scores[None, None, :, :])
                        else:
                            scores = np.stack((1 - expected_mask, expected_mask))
                            tensor = _Tensor(scores[None, :, :, :])
                        output = (
                            container([tensor])
                            if container in (tuple, list) else container(tensor)
                        )
                        self.node.model = lambda unused_input: output
                        self.node.use_half_precision = half_precision
                        message = SimpleNamespace(
                            header=_header(), pixels=self.image.copy()
                        )
                        self.node.image_callback(message)
                        self.assertEqual(len(self.masks), 1)
                        self.assertEqual(self.masks[0].header, message.header)
                        self.assertIsNot(self.masks[0].header, message.header)
                        self.assertEqual(self.annotated[0].header, message.header)
                        np.testing.assert_array_equal(
                            self.masks[0].pixels, expected_mask * 255
                        )


if __name__ == '__main__':
    unittest.main()
