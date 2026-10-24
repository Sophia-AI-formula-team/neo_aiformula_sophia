# Exact live fusion snapshot audit

Exact live candidate samples replayed once with original core and parameters; additional seed/turn diagnostics only

- Source files checked: 73; unchanged after audit: true.
- Raw candidate samples: 1014; excluded: 2.
- Original full-fusion replay exactly equal: True; numeric agreement: True; maximum absolute delta: 0.
- Actual fusion result: {'available': False, 'mode': 'odom_only', 'metric_source': 'odometry', 'recorded_length_m': 286.5017321447541, 'official_size_used': False, 'field_validation_required': True, 'reason': 'rmse_exceeds_limit', 'final_length_m': 286.4885465825291}.
- Required official outer node count: 5.

| Window m | Original search m | Original anchor outcome | Per-node passing turn candidates |
|---:|---:|---|---|
| 1 | 4 | node_turn_unobserved_or_excluded | 10000000000:0, 10000000001:0, 10000000002:0, 10000000003:0, 10000000004:0 |
| 2 | 4 | node_turn_unobserved_or_excluded | 10000000000:0, 10000000001:0, 10000000002:0, 10000000003:0, 10000000004:0 |
| 4 | 4 | node_turn_unobserved_or_excluded | 10000000000:0, 10000000001:0, 10000000002:0, 10000000003:0, 10000000004:16 |
| 8 | 4 | node_turn_inconsistent | 10000000000:27, 10000000001:76, 10000000002:0, 10000000003:1, 10000000004:54 |

These are diagnostic anchors under the recorded transform. They do not change the original fusion availability.

| Window m | Separated measured peaks at 10 degrees | At 20 degrees |
|---:|---:|---:|
| 1 | 0 | 0 |
| 2 | 0 | 0 |
| 4 | 11 | 0 |
| 8 | 5 | 5 |

Peak counts vary with scale and explicit diagnostic thresholds; they are not forced to four or five and are not semantic node identities.

Actual run outcome: {'accepted': False, 'reason': 'Stopped without 0.1 m progress for 15 simulation seconds', 'firstLapLearned': True, 'redScheduled': True, 'detourEntered': False, 'detourCompleted': False, 'leftTurnObserved': False, 'leftTurnValidated': False}

Object identity across a serialized snapshot cannot be asserted; structural/numeric return agreement is audited.
No input parameter or available flag is modified. The same rejected alignment remains rejected in every diagnostic.
A measured bend peak is not proof of an intersection or a safely drivable internal edge.
If fusion is unavailable, _update_traffic stops before any detour node selection; larger anchor windows alone cannot enable a detour.

No control/source edits, map preseed, CAD route input, model inference, Unity operation, or threshold relaxation occurred.

## Concrete outer-run interpretation

The exact coarse alignment RMSE is 3.404463140 m against the unchanged 3.0 m limit. The alternative is 4.408826786 m, so the 1.004363646 m ambiguity gap exceeds the unchanged 0.3 m minimum; this failure is the shape-fit residual gate, not the ambiguity gate.

At the terminal sample, the first lap has been learned and the odometry reference is matched, with fresh ABC point ages of 0.031 s. Current and voted light states are red (4 of 4 votes), OA and branch modes are idle, and the signal fault is not latched (`wait_latched=false`, `fault_sequence=-1`). The actual reason is `red_wait_map_fusion`; official node positions/progress are empty and `detour_node_ids=[]`. Thus this run never reaches node-entry selection. It does not demonstrate a left turn or a failed turning controller.

With the recorded rejected transform used only as a diagnostic seed, the default 2 m feature window has 83 trusted search candidates at each of the five nodes, but maximum same-sign turns are only 6.051, 6.136, 5.544, 3.307 and 6.277 degrees. The required minima are 21.305, 19.951, 24.774, 12.407 and 11.563 degrees. All five fail despite trusted geometry being available; the generic `unobserved_or_excluded` reason here must not be attributed solely to exclusions.

Increasing only the diagnostic feature window to 8 m still fails: node 00's selected turn differs from the official turn by 62.390 degrees (over the unchanged 60 degree consistency limit), and node 02 has no candidate reaching its 24.774 degree minimum (maximum 22.065 degrees). These diagnostics do not justify relaxing RMSE or turn gates, accepting the recorded transform, or declaring official junction identities established.
