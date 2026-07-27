from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .constraints import SemanticConstraints


def is_command_safe(cmd: Any, costmap: np.ndarray | None, constraints: SemanticConstraints) -> bool:
    if costmap is None or np.asarray(costmap).size == 0:
        return False

    arr = np.asarray(costmap)
    if bool(np.any(arr >= 255)) and _recent_path_had_lethal(cmd):
        return False

    linear_x = _get_value(cmd, "linear_x", "linear", "x", default=0.0)
    angular_z = _get_value(cmd, "angular_z", "angular", "z", default=0.0)

    if constraints.red_forward_forbidden and linear_x > 0.08 and angular_z < 0.15:
        return False

    return True


def _recent_path_had_lethal(cmd: Any) -> bool:
    if isinstance(cmd, Mapping):
        return bool(cmd.get("recent_path_had_lethal", False))
    return bool(getattr(cmd, "recent_path_had_lethal", False))


def _get_value(cmd: Any, flat: str, group: str, attr: str, default: float) -> float:
    if isinstance(cmd, Mapping):
        if flat in cmd:
            return float(cmd[flat])
        nested = cmd.get(group)
        if isinstance(nested, Mapping):
            return float(nested.get(attr, default))
        return default
    if hasattr(cmd, flat):
        return float(getattr(cmd, flat))
    nested = getattr(cmd, group, None)
    if nested is not None and hasattr(nested, attr):
        return float(getattr(nested, attr))
    return default
