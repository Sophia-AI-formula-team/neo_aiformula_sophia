# A4 Deployment Handoff

## Release Decision

The user renamed the selected V76 aligned step 1024 package to **A4**.
Release tag: **a4**; package version **4.0.0**. Identical prior model tag:
**a3-v76**; earlier model rollback tag: **a3-v14**. No retraining or weight change.
See weights/manifest.json for exact source/export hashes and input contract.

This release contains the actual temporal module and its learned weights.
The existing ROS image-only node runs single-frame inference; enabling temporal
fusion requires the explicit calibrated frame API in TEMPORAL.md. No vehicle
launch or perception publisher has been switched by this publication.

### Actual Deployment Status

Model and temporal API publication is complete; real-camera temporal input
integration is not complete. Single-frame means each video frame is independently
processed, as in the previous V14 ROS node, using the new base weights rather than
V14. It does not mean only the first image is processed or that GPU is disabled.
The Unity temporal comparisons used accurate camera poses. The image-only ROS
node cannot reproduce their temporal enhancement until calibrated capture-time
geometry is integrated. The A4 rename does not enable that integration.
Chinese explanation: [MODE_GUIDE_ZH.md](MODE_GUIDE_ZH.md).
Next work: [issue #34](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/issues/34).

## Measured Results

900 held-out synthetic sequence frames: 4 spatial anchors, three lighting
conditions (midday, long shadow, soft dusk), dashed/solid/unmarked variants.
The same frames, unchanged native labels and raw threshold 0.5 were used.

| Metric | Frozen single-frame parent | V76 aligned |
|---|---:|---:|
| Dashed-gap recall | 57.150% | 57.598% |
| Lane IoU on dashed scenes | 72.305% | 73.105% |
| Empty-scene false-pixel rate | 0.03418% | 0.03226% |
| Automated false joins | 0 | 0 |

The image model and detector were frozen. The 49-parameter fusion improves
cleanup modestly, but the +0.448-point gap gain is below the experiment's
+2-point direction gate. The source flag remains false; user selection is
recorded separately. These sequence metrics must not be compared directly with
V14's older static splits or different gap definitions.

Paired research-model latency on RTX 4070 Ti: single-frame median 21.63 ms,
temporal median 22.70 ms (+4.96%). This excludes ROS transport and overlay work.
Object raw outputs are unchanged relative to the V73 parent. This is not a new
per-class object-quality comparison against V14.

## Packaging Verification

- Before the A4 rename: all 40 package tests passed; after it: 41 passed,
  including release/manifest/Python/ROS version consistency.
- New unit/contract checks cover actual weights, one encoder traversal, independent
  outputs, native restoration, temporal resets, invalid metadata and ROS headers.
- The unchanged V76 model's 900-frame full-resolution source/export comparison
  remains in verification/runtime_smoke.json. The A4 rename did not rerun this
  experiment; exact weight and inference-source identity were checked instead.
- Wheel/install/CLI results: see verification/package_smoke.json.
- Sanitized experiment summary: verification/training_summary.json.

The publication preserves the newer road_detector full-Header repair.
No ROS/DDS, Jetson or moving-vehicle run was performed in this release task.

## Next Owner

1. Install from the tag and verify the manifest checksum before inference.
2. Run the image demo and all shadow remaps in DEPLOYMENT.md; verify publisher
   count, encodings, original timestamps and frame IDs.
3. For temporal operation supply metric plane geometry at image capture time.
   Validate real-camera ground projection, optical axes, camera extrinsics and
   odometry synchronization. Do not normalize the projection matrix arbitrarily.
4. On AGX Orin measure sustained end-to-end timing and source timestamp age,
   memory and temperatures with representative input. FP32 is the reference.
5. Compare lanes and each object class on the same real footage before changing
   runtime precision or input size. Temporal object tracking is not implemented.

## Rollback And Training

Restore the entire a3-v14 runtime and weight together; do not rename a checkpoint.
The package is inference-only. Full optimizer/RNG/history checkpoints remain in
the separate training project. No training, resume configuration or private data
belongs in this deployment directory.
