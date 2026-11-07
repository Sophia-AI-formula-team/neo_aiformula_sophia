"""Observe the frozen planner's return frame without changing its calculation.

The Python profile hook copies local arrays AFTER the actual step has returned.
The returned command is never recomputed, patched, filtered or overridden.
These records explain the finite candidate search, not a collision probability.
"""
import sys
import numpy as np
from v3_model import Planner


class AuditedPlanner(Planner):
    def step(self, obs):
        captured = {}
        previous = self.command.copy()
        old_profile = sys.getprofile()
        if old_profile is not None:
            raise RuntimeError("Acceptance instrumentation requires its own profile hook")

        def capture(frame, event, arg):
            if event == "return" and frame.f_code is Planner.step.__code__:
                captured.update(frame.f_locals)

        try:
            sys.setprofile(capture)
            command = super().step(obs)
        finally:
            sys.setprofile(old_profile)
        if "pick" not in captured:
            self.last["audit"] = {"selected": None, "table": [], "paths": []}
            return command
        c = captured
        progress = c["z"] / c["horizon"]
        heading = .8 * c["v"] * (c["h"] - c["road_heading"]) ** 2
        smooth = self.p["smoothness"] * ((c["ww"] - previous[1]) ** 2 + .15 * c["ww"] ** 2)
        risk = .15 * np.maximum(0, .5 - c["robust"]) ** 2
        table = np.c_[c["uu"], c["ww"], np.repeat(c["targets"], len(c["speeds"])),
                      c["clearance"], c["uncertainty"], c["robust"], c["feasible"],
                      c["score"], progress, heading, smooth, risk]
        self.last["audit"] = dict(
            columns=["speed", "yaw", "target_x", "geometric_clearance", "sigma",
                     "robust_clearance", "feasible", "score", "progress_score",
                     "heading_penalty", "smooth_penalty", "margin_penalty"],
            selected=int(c["pick"]), horizon=float(c["horizon"]),
            threshold=self.p["clearance"], uncertainty_scale=self.p["uncertainty"],
            previous_command=previous.tolist(), table=table.round(6).tolist(),
            paths=np.asarray(c["predicted"]).transpose(1, 0, 2).round(4).tolist(),
            rule="max score among robust-clearance feasible candidates; if none, max 100*margin+score-0.3*speed",
        )
        return command


def parity(parameters):
    from v3_model import Observation
    rng = np.random.default_rng(90208118)
    count = 0
    for params in (None, parameters):
        a, b = Planner(params), AuditedPlanner(params)
        for i in range(256):
            z = np.linspace(1.2, rng.uniform(5, 10), 24)
            center = rng.uniform(-.03, .03) * z ** 2
            road = np.c_[z, center-rng.uniform(1.4, 3), center+rng.uniform(1.4, 3), .06+.006*z]
            objects = np.array([[rng.uniform(1, 9), rng.uniform(-2.4, 2.4), 0,
                                 rng.uniform(-1, 1), .34, .06] for _ in range(i % 5)]).reshape(-1, 6)
            if i % 31 == 0:
                road = road[:2]
            obs = Observation(road, objects, rng.uniform(0, 4.5), rng.uniform(-.6, .6))
            ca, cb = a.step(obs), b.step(obs)
            if not np.array_equal(ca, cb):
                raise AssertionError((i, ca, cb))
            assert a.last == {k: v for k, v in b.last.items() if k != "audit"}
            if b.last["audit"]["selected"] is not None:
                audit = b.last["audit"]
                assert np.array_equal(cb, np.asarray(a.command))
                assert audit["selected"] == int(np.argmax(np.array(audit["table"])[:, 7]))
            count += 1
    return dict(comparisons=count, command_max_error=0, original_decision_fields_identical=True)
