# Experiment and verification archives

This directory is excluded from ROS package discovery by `COLCON_IGNORE`.
Its source overlays and installers require an explicit selection; merging an
archive does not install it into Unity or enable it on the vehicle.

| Archive | Source | Purpose |
| --- | --- | --- |
| [unity_semantic_integration](unity_semantic_integration/README.md) | PR #17 | Frozen final63 red/green wiring and evidence |
| [unity_semantic_left_turn_20261012](unity_semantic_left_turn_20261012/README.md) | PR #18 | Historical left-turn failures and default-disabled calibrated candidate |
| [unity_semantic_official_matching_20261012](unity_semantic_official_matching_20261012/README.md) | PR #21 and #22 | Official-map diagnostics and opt-in bbox-distance experiment |
| [unity_neo_topo_20261015](unity_neo_topo_20261015/RESULTS.md) | PR #26 | Stop/wait, odometry-reference closure and checkpoint restart verification |

PR #18 remains in its original dated experiment path, with its complete
before/overlay/evidence payload and original checksums. The selected follow-up
uses the official map; the calibrated candidate is not selected for runtime.
No archive establishes physical-vehicle or official-map left-turn acceptance.
