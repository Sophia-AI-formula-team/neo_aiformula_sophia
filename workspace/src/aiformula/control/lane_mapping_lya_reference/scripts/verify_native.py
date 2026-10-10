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
    assert native.SOURCE_SHA256 == expected, "loaded extension does not match the C source"
    assert native.graph_forest(b"\x03\x03\x04", 3) == ([0, 0, 2], [-1, 0, -1])
    assert native.connected_subsets(b"\x03\x03\x04", 3, b"\x07\x05", [0, 1]) == b"\x03\x00"
    assert native.radius_graph(struct.pack("=dddd", 0., 0., .22, 0.),
                               struct.pack("=qqqq", 0, 0, 1, 0), 2, .22 ** 2) == b"\x03\x03"
    result = dict(backend="native", api_version=native.API_VERSION,
        radius_graph_api_version=native.RADIUS_GRAPH_API_VERSION,
        source_sha256=expected, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        binary_name=binary.name, python=sys.version, platform=platform.platform(),
        installed_prefix_verified=args.install_prefix is not None, real_calls_passed=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
