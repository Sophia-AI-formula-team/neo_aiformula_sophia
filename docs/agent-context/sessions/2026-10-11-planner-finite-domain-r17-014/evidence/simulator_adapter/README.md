# R17 Simulator adapter source archive

These 12 inert `.py.txt` files are byte-exact afterimages of the listed
`Simulator/Tools/LaneTeachRepeat` files. Every original and archive SHA256 is
in `manifest.json`. No content redaction or numerical edit was necessary;
no personal absolute paths or credential literals were found in the reviewed
files. Root and an independent agent reviewed the publication contents.

Six files changed for R17: `lane_backend.py`,
`replay_observed_keyframe.py`, `test_replay_certified_dependencies.py`,
`test_replay_raw_imu.py`, `test_boundary_replay.py`, and
`test_boundary_adapter.py`. The remaining six are import dependencies,
not claimed as new R17 work. There is no trustworthy immediate pre-edit
snapshot for every file, so this archive is **not** represented as an exact
before/after diff.

R17 adapter changes add explicit boundary-v3 certificate identity, preserve
old v1/v2 source-selected replay behavior, and load/hash/archive
`finite_support` only when the selected paint AST imports it. Historical
manifests do not silently gain a new helper. Current runtime capability,
recorded representation, candidate overrides and hash equality stay distinct.
The tests exercise malformed/mixed versions and missing helper identity.

## Restore and reproduce

Restore reviewed afterimages into the corresponding relative paths of a
compatible Simulator checkout, removing the final `.txt` suffix. Review
local differences before replacing anything. This document performs no copy,
server start, scene change or driving action.

This is **not a complete Unity project**. The existing Simulator Frontend
package, scene/bridge, other tests, and platform-specific native build are
separate dependencies. `upstream_teacher.py` retains its normal relative
Frontend lookup; it is not rewritten to a fake path. Captured replay inputs
remain private and are not included. A replay needs its exact input fixture
and matching core/native source identities.

For focused checks, use the original relative command from
`Simulator/Tools/LaneTeachRepeat`, with `LANE_TEST_REPO` pointing to the
reviewed neo checkout and the required four package roots on `PYTHONPATH`:

```text
python -B -m pytest test_replay_certified_dependencies.py test_replay_raw_imu.py test_boundary_replay.py test_boundary_adapter.py -q -ra -p no:cacheprovider
```

NumPy/Pillow and the selected interpreter are required. Some original tests
require accessible historical Git blobs or private recording fixtures; do not
fabricate them or interpret a skip as equivalent evidence. Existing recorded
commands/results, including the earlier collection failure, are summarized in
`../reviewed_summary.json`. This archive itself has not been executed after
copying, and it does not provide new runtime/ROS/vehicle acceptance.
