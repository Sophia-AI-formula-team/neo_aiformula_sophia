"""Keep the two independently owned vehicle speed settings across process starts."""
import json
import math
import os
from pathlib import Path
import tempfile


class SpeedSetting:
    def __init__(self, node, name, upper):
        directory = node.declare_parameter('speed_settings_directory', '').value
        self.path = Path(directory) / (name + '.json') if directory else None
        self.name, self.upper = name, upper
        saved = 4.0
        load_error = None
        if self.path is not None:
            try:
                saved = json.loads(self.path.read_text(encoding='utf8'))
                self.validate(saved)
            except FileNotFoundError:
                pass
            except (OSError, ValueError) as exc:
                # An explicit override can still recover a malformed setting.
                saved, load_error = None, exc
        # An explicitly supplied ROS parameter takes priority over the saved default.
        self.initial = node.declare_parameter(name, saved).value
        if self.initial is None and load_error is not None:
            raise ValueError(f'Invalid {self.path}: {load_error}') from load_error
        self.validate(self.initial)

    def validate(self, value):
        if type(value) is not float or not math.isfinite(value) or not 0 < value <= self.upper:
            raise ValueError(f'{self.name} must be a double in (0, {self.upper}] m/s')

    def save(self, value):
        self.validate(value)
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=self.path.parent, mode='w', delete=False) as tmp:
            try:
                tmp.write(json.dumps(value) + '\n')
                tmp.flush()
                os.fsync(tmp.fileno())
                os.replace(tmp.name, self.path)
            finally:
                if os.path.exists(tmp.name):
                    os.unlink(tmp.name)
