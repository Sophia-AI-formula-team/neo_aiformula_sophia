"""Arrival-time voting for traffic-light observations, independent of ROS."""

from collections import deque
import math
from typing import Deque, Dict, Optional, Tuple


class TrafficLightFilter:
    """Require a strict majority of real messages in a sliding time window.

    The caller supplies monotonic arrival times, not image/ROS timestamps.  The
    first decision waits for a complete observation window.  A message gap
    longer than the window starts that observation period again.  ``unknown``
    results count in the denominator but never release a stop by themselves;
    retaining a red-light stop belongs to the calling state machine.
    """

    def __init__(self, window_sec: float = 0.5, min_samples: int = 3):
        try:
            window_sec = float(window_sec)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("window_sec must be finite and positive")
        if not math.isfinite(window_sec) or window_sec <= 0.0:
            raise ValueError("window_sec must be finite and positive")
        if isinstance(min_samples, bool) or not isinstance(min_samples, int) or min_samples < 2:
            raise ValueError("min_samples must be an integer of at least 2")
        self.window_sec = window_sec
        self.min_samples = min_samples
        self._samples: Deque[Tuple[float, str]] = deque()
        self._started_at: Optional[float] = None
        self._last_message_at: Optional[float] = None
        self._last_now: Optional[float] = None

    def clear(self) -> None:
        """Discard the current observations and require a fresh full window."""
        self._samples.clear()
        self._started_at = None
        self._last_message_at = None
        self._last_now = None

    def add(self, state: str, now: float) -> str:
        """Add exactly one received observation and return the current vote."""
        now = self._advance(now)
        if self._started_at is None:
            self._started_at = now
        state = state.strip().lower() if isinstance(state, str) else "unknown"
        if state not in ("red", "green"):
            state = "unknown"
        self._samples.append((now, state))
        self._last_message_at = now
        return self._decision(self._snapshot(now))

    def evaluate(self, now: float) -> str:
        """Read a current vote without inventing another detector message."""
        return self._decision(self.snapshot(now))

    def snapshot(self, now: float) -> Dict[str, object]:
        """Return JSON-compatible diagnostics without adding a vote."""
        now = self._advance(now)
        return self._snapshot(now)

    def _advance(self, now: float) -> float:
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("now must be a finite monotonic time")
        if not math.isfinite(now):
            raise ValueError("now must be a finite monotonic time")
        if self._last_now is not None and now < self._last_now:
            raise ValueError("now must not move backwards")
        self._last_now = now
        if self._last_message_at is not None and now - self._last_message_at > self.window_sec:
            self._samples.clear()
            self._started_at = None
        cutoff = now - self.window_sec
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        return now

    def _snapshot(self, now: float) -> Dict[str, object]:
        red_count = sum(state == "red" for _, state in self._samples)
        green_count = sum(state == "green" for _, state in self._samples)
        count = len(self._samples)
        ready = (self._started_at is not None
                 and now - self._started_at >= self.window_sec
                 and count >= self.min_samples)
        return {
            "window_sec": self.window_sec,
            "min_samples": self.min_samples,
            "ready": ready,
            "sample_count": count,
            "red_count": red_count,
            "green_count": green_count,
            "unknown_count": count - red_count - green_count,
            "last_message_age_sec": (None if self._last_message_at is None
                                     else now - self._last_message_at),
        }

    @staticmethod
    def _decision(snapshot: Dict[str, object]) -> str:
        if not snapshot["ready"]:
            return "unknown"
        count = snapshot["sample_count"]
        if 2 * snapshot["red_count"] > count:
            return "red"
        if 2 * snapshot["green_count"] > count:
            return "green"
        return "unknown"
