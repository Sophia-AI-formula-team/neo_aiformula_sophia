"""Verify the bundled model checksum and inference-only checkpoint schema."""
import hashlib
import json
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]


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
    if set(payload) != {"architecture", "model", "provenance"}:
        raise ValueError("Expected model-only checkpoint without training state")
    if payload["architecture"] != manifest["architecture"]:
        raise ValueError("Architecture mismatch")
    if payload["provenance"]["epoch"] != manifest["epoch"]:
        raise ValueError("Selected epoch mismatch")
    print(json.dumps(dict(sha256=checksum, bytes=path.stat().st_size,
                          epoch=manifest["epoch"], inference_only=True), indent=2))


if __name__ == "__main__":
    main()
