# Unity semantic integration: frozen final63 archive

This directory preserves the Simulator wiring used for the four completed
`final63_*` traffic trials on 2026-10-12. The active ROS packages remain in
[`workspace/src/aiformula/perception/semantic_planning_system`](../../workspace/src/aiformula/perception/semantic_planning_system).
`COLCON_IGNORE` deliberately excludes this archival tree from ROS package discovery.

Use [DEPLOYMENT.md](DEPLOYMENT.md) for compatible-Simulator installation and
rollback, and [AGENTS.md](AGENTS.md) for the next agent's handoff.
This is a scoped source overlay requiring an existing Simulator project. It does
not contain a standalone Unity project, checkpoint, videos, or native ROS/DDS
runtime acceptance evidence.

## Source identity and layout

| Item | Identity / role |
| --- | --- |
| Frozen semantic source | `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96` |
| Original semantic source | `5bc52e05329d7fe642f219d32a578d04d2b7f668` |
| Unchanged vision/LLP/LYA upstream | `05f1651c8a14d56ab9a86a9767c52793403dcfdc` |
| `overlay/` | 16 exact final wiring files, including four necessary new `.meta` files |
| `before/` | 10 exact original wiring files, sufficient for their rollback |
| `manifest.json` | 116 managed paths; 100 native semantic paths use pinned Git blobs with explicit LF/CRLF recipes |
| Native semantic completeness | 100 managed + four unchanged prerequisite files = 104 source files |
| `evidence/` | Four result receipts, one shared configuration/hash set, final report and media/geometry QA JSON |
| `install.py` | Offline hash preflight, inspect, apply and rollback; no force option |
| `test_install.py` | Isolated safety tests; no Unity or model required |

The 90 native newline-only changes have recipes instead of duplicate source
copies. The two substantive native commits are read from the local Git object
database, never from mutable working-tree files. `.gitattributes` preserves
exact `before/`, `overlay/` and evidence bytes across Git checkouts. Semantic
changes are reviewed in the normal package location; the archive's source
manifest describes their installed byte representation.

The original source snapshot remains an external provenance record. Neither
installation nor rollback requires it:

- `source_before.zip`: 8,386,101 bytes, SHA256
  `950f4ae464bbf2a8fab25ba4867af48f5618ddee838b29e8f90895cbcdb21ee0`.
- `source_before.json`: 937-entry original inventory, SHA256
  `3935df0e41f45582c1c5335bc7e3e67b328a46654f0205b3b36bbc201c30a300`.
- Frozen original delta manifest SHA256:
  `a0de95abc231dff9ba58ea6596cb71485519cbe0de58d2e437357df0f32d895d`.
- Final source audit SHA256:
  `a09c44a0708fb467092abc35ab6f8431732472810d9a54a0092ece5c6c584503`.

The original audit excluded 10 peer-owned LaneTeachRepeat changes and one
unattributed editor-generated setting. They are not payloads in this archive.
The installer writes only the 116 named source paths and checks 305 unchanged
prerequisites. This is a compatibility check for the selected baseline assets
and dependencies, not a proof that every file of a larger Unity project is
identical. It does not recreate missing scene assets or user configuration.

## Recorded result

All four cases used the same frozen integration hashes and recorded
`accepted=true`. They cover the east straight, northeast bend and north
straight to `x <= 38 m, z >= 35 m`, starting at `z=-10 m`, yaw 0.

| Case | Start x (m) | Red stop front gap (m) | Travel (m) | Recorded duration incl. stopped tail (s) | Min left/right lane-boundary center distance (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| final63_inner_red | 68.76 | 8.635939 | 73.622140 | 39.355972 | 1.8271 / 1.8113 |
| final63_inner_green | 68.76 | n/a | 73.657520 | 32.628222 | 1.8122 / 1.7625 |
| final63_outer_red | 72.53 | 8.602903 | 79.296070 | 39.522116 | 1.6300 / 1.7512 |
| final63_outer_green | 72.53 | n/a | 79.186140 | 34.100755 | 1.6248 / 1.7506 |

The red trials stopped in the requested 8–9 m band, then resumed after green.
All 2022 recorded vehicle-center samples stayed inside their CAD lane; center
crossings of the line's inner paint edge were also zero. These are discrete
center samples, not a whole-body envelope or continuous collision proof.
Initial-green cases are passage tests, not red-stop tests.

Control uses **RGB body-forward depth** at capture time. The reported stop gap
is a separate **evaluation-only fixed-plane front gap**, measured from the most
forward transformed bumper corner (local lateral ±0.3275 m, forward +0.216 m)
to the fixed signal plane `z=15 m`. Lamp truth, lamp world position and the CAD
route are not control inputs. Unity supplies ideal, noiseless odometry.
The selected unchanged camera cannot support a general 2–3 m stop claim.

The recorded chain is Unity RGB → A3-v14 + explicitly sourced RGB-panel
recognition → LLP adapter → pinned native lane-point selection / LYA →
MapAssist gates → Unity vehicle. Python stdio and in-process ROS shims connect
the classes; no native ROS graph or DDS transport was exercised here.
Red means stop/approach in this cold-start MapAssist trial, not a forced left
turn or completed topological-map detour.

Single-side centers are explicitly estimated. RIGHT uses a finite-lived paired
width; LEFT requires a geometric identity check and retains native asymmetric
LEFT offsets. The 1.5 s prior lifetime, source/sequence checks and command
freshness guards remain relevant. The LEFT comparison is not ego-motion
aligned and does not prove physical line identity beyond these trials.

The red runs retain `first_motion_rearm_edge_not_sampled`: the precise first
release edge fell between recorder samples. Later fresh-point/new-command
samples and regression tests do not retroactively fill that sampling gap.
See [the full recorded report](evidence/RUN_REPORT.md) for fault recovery,
unknown-color handling, opt-in focus policy and the evidence boundaries.
That report's PR file counts describe the original core-only handoff, before
this later archive was added to the repository.

## Local media and evidence

The machine-local artifact root is:

```text
C:\Users\PC\OneDrive - Sophia Univ. Students\Main\SOPHIA\Y4T1\Research\Demo_LaneLineTrainer\workspace\diagnostics\semantic_stop_20261012
```

These large artifacts are deliberately not copied into Git:

| Artifact under that root | SHA256 |
| --- | --- |
| `final_semantic_four_cases.mp4` | `1f183b2c70adb2b1e0257310b959a714b8653b60e2b8d105f07084fba2ae62fe` |
| `final_trajectories.png` | `dfd03469232ff6e36645fb2547c350f0f5a157a07545a1c0254cb01277029675` |
| `final_trajectory_samples.csv` | `f7629284e205afc388f9cb1fed1a90df0eca4ee407cc10835a5ced2e15890a56` |
| `final_metrics.json` | `782e5460bb50339c5486a8d0b447fcdd3323e9f1777f5008806894ddf025c517` |

The movie is 1280×960 H.264/yuv420p, 15 fps, 2191 frames, 146.066667 s,
57,116,592 bytes. All frames decoded without an error. Four complete contact
sheets and actual decoded stop/passage/terminal/estimated-LEFT frames were
visually checked. The upper 75% remains the unobscured observer view; lower
panels show perception, LLP/LYA, semantic gates and trajectory. Real wall time,
red waiting and the stopped tail remain present.

`evidence/final_media_manifest.json` and `evidence/final_media_qa.json` identify
the exact checks and external image paths. The four original case directories
hold `trace.jsonl`, `motion.jsonl`, `frames.jsonl`, RGB/overlay frames and
events. Earlier critical failures and partial runs remain in the same local
diagnostic root; they are not substituted for the final four-case receipts.

Recorded package validation: 212 tests and 203 subtests passed; 67 tracked
Python files parsed with Python 3.8 syntax. Separate shim/transport and C# stub
checks are described in the report and are not counted as vehicle or DDS
acceptance. The archive installer has its own isolated tests, independent of
those historical runtime results.
Current archive checks are recorded separately in [VALIDATION.md](VALIDATION.md).
