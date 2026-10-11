"""Private stdio RGB inference adapter, with optional read-only LYA candidates."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import sys
import time

PACKAGE = Path(__file__).resolve().parent / "a3-v14"
WEIGHT_SHA256 = "a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c"
MAX_LINE = 24 * 1024 * 1024


def verify_package():
    manifest = json.loads((PACKAGE / "weights/manifest.json").read_text(encoding="utf-8"))
    weights = PACKAGE / "weights/shared_best.pth"
    with weights.open("rb") as source:
        digest = hashlib.file_digest(source, "sha256").hexdigest()
    if digest != WEIGHT_SHA256 or manifest["sha256"] != digest:
        raise ValueError("a3-v14 checkpoint SHA256 mismatch")
    if manifest["provenance"]["tag"] != "a3-v14" or manifest["provenance"]["step"] != 16384:
        raise ValueError("Wrong A3 model provenance")
    return weights


def decode_request(line):
    import cv2
    import numpy as np
    if len(line) > MAX_LINE:
        raise ValueError("Input frame exceeds transport limit")
    request = json.loads(line)
    sequence = request.get("sequence")
    if type(sequence) is not int or sequence < 1:
        raise ValueError("Invalid frame sequence")
    png = base64.b64decode(request["png"], validate=True)
    # Validate dimensions before OpenCV allocates the decoded image.
    if png[:8] != b"\x89PNG\r\n\x1a\n" or len(png) < 24:
        raise ValueError("Expected PNG input")
    width, height = int.from_bytes(png[16:20], "big"), int.from_bytes(png[20:24], "big")
    if not (2 <= width <= 4096 and 2 <= height <= 4096):
        raise ValueError("Unsupported RGB dimensions")
    image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (height, width):
        raise ValueError("Invalid PNG pixels")
    return sequence, image, hashlib.sha256(png).hexdigest()


def encode_png(image):
    import cv2
    ok, pixels = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise RuntimeError("PNG encoding failed")
    return base64.b64encode(pixels).decode("ascii")


def infer(model, line, pipeline_state=None):
    sequence, image, digest = decode_request(line)
    request = json.loads(line)
    began = time.perf_counter()
    output = model.predict(image)
    payload = output.detections_payload()
    overlay = output.combined_overlay
    if request.get("signal_panel_rgb") is True and pipeline_state is not None:
        from signal_panel_rgb import SignalPanelRGBTracker, draw_panel_overlay
        tracker = pipeline_state.setdefault("signal_panel", SignalPanelRGBTracker())
        evidence = tracker.update(image, output.traffic_light_state, output.detections, sequence)
        payload.update(traffic_light_original=output.traffic_light_state,
                       traffic_light_state=evidence["state"], traffic_light_source=evidence["source"],
                       signal_panel=evidence)
        overlay = draw_panel_overlay(overlay, evidence)
    reply = dict(type="frame", sequence=sequence, width=image.shape[1], height=image.shape[0],
                input_sha256=digest, inference_ms=(time.perf_counter() - began) * 1000,
                overlay=encode_png(overlay), lane=encode_png(output.lane_mask),
                obstacles=encode_png(output.obstacle_mask), **payload)
    control = request.get("lane_control")
    if control is not None and pipeline_state is not None:
        try:
            from upstream_adapter import Pipeline
            if pipeline_state.get("epoch") != control["epoch"]:
                pipeline_state.update(epoch=control["epoch"], pipeline=Pipeline())
            reply["lane_control"] = pipeline_state["pipeline"].process(output.lane_mask, dict(control, sequence=sequence))
        except Exception as error:
            reply["lane_control"] = dict(valid=False, supported=False, error=f"{type(error).__name__}: {error}")
    return reply


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    args = parser.parse_args()
    protocol = sys.stdout
    sys.stdout = sys.stderr  # Dependency diagnostics cannot corrupt protocol stdout.
    def send(value):
        protocol.write(json.dumps(value, separators=(",", ":"), allow_nan=False) + "\n")
        protocol.flush()
    try:
        weights = verify_package()
        sys.path.insert(0, str(PACKAGE))
        sys.path.insert(0, str(PACKAGE.parent.parent / "VisionLaneFollower"))
        import torch
        from yolop_a3_runtime import PerceptionModel
        torch.set_num_threads(min(4, os.cpu_count() or 1))
        if os.environ.get("AIFORMULA_PERCEPTION_DETERMINISTIC") == "1":
            torch.use_deterministic_algorithms(True)
            torch.backends.cudnn.benchmark = False
        model = PerceptionModel(weights, device=args.device, score_threshold=.5)
        send(dict(type="ready", model="a3-v14", sha256=WEIGHT_SHA256,
                  device=str(model.device), torch=torch.__version__))
        pipeline_state = {}
        while True:
            line = sys.stdin.buffer.readline(MAX_LINE + 1)
            if not line:
                return 0
            try:
                send(infer(model, line, pipeline_state))
            except Exception as error:
                send(dict(type="error", message=f"{type(error).__name__}: {error}"))
                return 1
    except Exception as error:
        send(dict(type="error", message=f"{type(error).__name__}: {error}"))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
