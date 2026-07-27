from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from typing import Any, Dict


@dataclass
class SemanticConstraints:
    red_light_active: bool = False
    red_forward_forbidden: bool = False
    allow_lane_follow: bool = True
    diversion_active: bool = False
    mode: str = "NORMAL"

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, payload: str | Dict[str, Any] | None) -> "SemanticConstraints":
        if payload is None:
            return cls()
        if isinstance(payload, str):
            if not payload.strip():
                return cls()
            data = json.loads(payload)
        else:
            data = dict(payload)

        allowed = {field.name for field in cls.__dataclass_fields__.values()}
        return cls(**{key: value for key, value in data.items() if key in allowed})
