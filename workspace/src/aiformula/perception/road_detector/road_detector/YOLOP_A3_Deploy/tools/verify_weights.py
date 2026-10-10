"""Verify the bundled model checksum and inference-only checkpoint schema."""
import hashlib
import json
from pathlib import Path
import sys

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yolop_a3_runtime.shared_fpn import CONTRACT_ARCHITECTURE, INPUT_CONTRACT


def main():
    manifest = json.loads((ROOT / "weights/manifest.json").read_text(encoding="utf-8"))
    path = ROOT / manifest["file"]
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    checksum = digest.hexdigest()
    if checksum != manifest["sha256"]:
        raise ValueError("Checkpoint checksum mismatch")
    payload = torch.load(path, map_location="cpu", weights_only=True)
    if set(payload) != {"architecture", "model", "input_contract", "provenance"}:
        raise ValueError("Expected model-only checkpoint without training state")
    if payload["architecture"] != manifest["architecture"] or payload["architecture"] != CONTRACT_ARCHITECTURE:
        raise ValueError("Architecture mismatch")
    if payload["input_contract"] != INPUT_CONTRACT or manifest["input_contract"] != INPUT_CONTRACT:
        raise ValueError("V14 input contract mismatch")
    if payload["provenance"] != manifest["provenance"]:
        raise ValueError("Provenance mismatch")
    if path.stat().st_size != manifest["bytes"] or len(payload["model"]) != manifest["tensors"]:
        raise ValueError("Checkpoint size or tensor count mismatch")
    if any(not torch.isfinite(value).all() for value in payload["model"].values()):
        raise ValueError("Nonfinite model tensor")
    print(json.dumps(dict(sha256=checksum, bytes=path.stat().st_size,
                          step=payload["provenance"]["step"], tag="a3-v14", inference_only=True), indent=2))


if __name__ == "__main__":
    main()
