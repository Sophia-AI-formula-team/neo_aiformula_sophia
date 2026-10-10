"""Verify the actually loaded extension, its source identity and real calls."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import struct
import sys

import _lane_mapping_lya_native as native


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--install-prefix", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    binary = Path(native.__file__).resolve(strict=True)
    if args.install_prefix:
        binary.relative_to(args.install_prefix.resolve(strict=True))
    expected = hashlib.sha256(args.source.read_bytes()).hexdigest()
    assert native.API_VERSION == 1, "incompatible graph API"
    assert native.RADIUS_GRAPH_API_VERSION == 1, "incompatible radius graph API"
    assert native.LOCAL_SUPPORT_API_VERSION == 1, "incompatible local support API"
    assert native.NEAREST_API_VERSION == 1, "incompatible nearest API"
    assert native.CROSS_SECTION_WEIGHTS_API_VERSION == 1, "incompatible cross-section API"
    assert native.LOCAL_MOMENTS_API_VERSION == 1, "incompatible local moments API"
    assert native.LOCAL_GEOMETRY_API_VERSION == 1, "incompatible local geometry API"
    assert native.SOURCE_SHA256 == expected, "loaded extension does not match the C source"
    assert native.graph_forest(b"\x03\x03\x04", 3) == ([0, 0, 2], [-1, 0, -1])
    assert native.connected_subsets(b"\x03\x03\x04", 3, b"\x07\x05", [0, 1]) == b"\x03\x00"
    assert native.radius_graph(struct.pack("=dddd", 0., 0., .22, 0.),
                               struct.pack("=qqqq", 0, 0, 1, 0), 2, .22 ** 2) == b"\x03\x03"
    graph = native.prepare_graph(b"\x03\x03\x04", 3)
    support, counts = native.local_support(graph, 3, struct.pack("=qqq", 0, 1, 2),
        struct.pack("=qq", 0, 1), b"\x01\x01\x01\x01\x00\x01",
        struct.pack("=qqq", -1, 0, -1))
    assert support == b"\x01\x01\x00\x00\x00\x00"
    assert struct.unpack("=qq", counts) == (2, 0)
    nearest = native.nearest_grouped(struct.pack("=dd", 0., 0.),
        struct.pack("=dddd", 1., 0., -1., 0.), struct.pack("=qq", 0, 1),
        struct.pack("=q", 0), struct.pack("=qq", 0, 2), struct.pack("=qq", 1, 0),
        b"\x01\x01", 1.0)
    assert struct.unpack("=q", nearest) == (1,), "first tie must preserve candidate order"
    weights = native.cross_section_weights(graph, 3,
        struct.pack("=dddddd", 0., 0., 0., .1, 2., 0.),
        struct.pack("=dddddd", 0., 1., 0., 1., 0., 1.),
        struct.pack("=qqq", 0, 0, 1), .05, 1.35 ** 2)
    assert struct.unpack("=ddd", weights) == (1.0 / 3.0,) * 3
    means, covariance = native.local_moments(
        struct.pack("=dddddd", -1., 0., 0., 1., 1., 0.), b"\x01\x00\x01", 1, 3)
    assert struct.unpack("=dd", means) == (0., 0.)
    assert struct.unpack("=dddd", covariance) == (1., 0., 0., 0.)
    counts, means, covariance = native.local_geometry(graph, 3,
        struct.pack("=dddddd", 0., 0., 1., 0., 3., 0.),
        struct.pack("=qqq", 0, 0, 2), struct.pack("=qqq", -1, 0, -1),
        struct.pack("=qq", 0, 1), struct.pack("=q", 0),
        struct.pack("=qq", 0, 3), struct.pack("=qqq", 0, 1, 2), 1.0, 2)
    assert struct.unpack("=q", counts) == (2,)
    assert struct.unpack("=dd", means) == (.5, 0.)
    assert struct.unpack("=dddd", covariance) == (.25, 0., 0., 0.)
    result = dict(backend="native", api_version=native.API_VERSION,
        radius_graph_api_version=native.RADIUS_GRAPH_API_VERSION,
        local_support_api_version=native.LOCAL_SUPPORT_API_VERSION,
        nearest_api_version=native.NEAREST_API_VERSION,
        cross_section_weights_api_version=native.CROSS_SECTION_WEIGHTS_API_VERSION,
        local_moments_api_version=native.LOCAL_MOMENTS_API_VERSION,
        local_geometry_api_version=native.LOCAL_GEOMETRY_API_VERSION,
        source_sha256=expected, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        binary_name=binary.name, python=sys.version, platform=platform.platform(),
        installed_prefix_verified=args.install_prefix is not None, real_calls_passed=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
