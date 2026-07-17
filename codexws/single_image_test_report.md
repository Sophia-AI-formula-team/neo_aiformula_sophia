# Single-image lane-line test

Date: 2026-07-17 JST

Input: `/home/nvidia/Downloads/000750.jpg` (640 x 360 BGR image)

## Baseline result

The image was published repeatedly on
`/aiformula_sensing/zed_node/left_image/undistorted`. The live road detector
produced 640 x 360 `mono8` masks containing 6,697 non-zero pixels. The deployed
lane-line publisher published messages on left, center, and right cloud topics,
but all messages had zero points.

The isolated build of the unmodified local lane-line source reproduced the same
failure. Temporary diagnostics showed that rejection occurred before projection:

- left: `too_few_component_pixels`, 108 path pixels
- right: `too_few_component_pixels`, 74 path pixels
- configured hard-coded minimum: 180 path pixels

The 180-pixel threshold is defined for a 1920-pixel reference width but was not
scaled for the active 640-pixel image. Other nearby thresholds already use the
reference-image scaling helpers.

## Intrinsic and frame audit

The local and remote `intrinsic_nHD.yaml` and `extrinsic.yaml` files are
byte-for-byte identical. The running node uses:

- image: 640 x 360
- fx/fy: 254.391622
- cx: 330.013020833
- cy: 181.149637858
- vehicle frame: `base_footprint`
- camera frame: `zed_left_camera_optical_frame`
- translation: `[0.055, 0.060, 0.540]`
- quaternion xyzw: `[-0.496, 0.496, -0.504, 0.504]`

Applying those active values to the generated mask produced 4,117 positive
ground intersections and 3,764 finite points inside the configured vehicle ROI.
Therefore the observed empty output was not caused by all pixels failing the
intrinsic/TF ground projection.

The launcher selects calibration directory `SN48442725`. The earlier handoff
record says the physically detected ZED X serial was `48797506`; calibration
provenance should be verified after reconnecting the camera. That mismatch may
affect metric accuracy, but it did not cause this empty-cloud result.

## Isolated corrected result

The minimum component support was scaled by `imageWidthScale()`. At 640 pixels,
the effective threshold becomes 60 instead of 180. No deployed file or running
all-nodes process was replaced.

With the same image, mask, intrinsics, TF, and ROI, the isolated corrected node
produced:

| Cloud | Messages observed | Maximum points | Finite non-zero XYZ points |
| --- | ---: | ---: | ---: |
| left | 65 | 9 | 9 |
| center | 65 | 9 | 9 |
| right | 65 | 9 | 9 |

Diagnostic projection spans were 4.816170 m left and 4.444571 m right. The test
acceptance flag was `all_three_clouds_nonempty: true`.

## Safety/state

- The local ROS workspace was read-only throughout.
- All local work and evidence is under `/home/nvidia/Desktop/codexws`.
- The deployed remote workspace and installed executable were not changed.
- The temporary candidate node was stopped after the test.
- The original road detector, lane-line publisher, and RViz processes remained running.
- The injected camera publisher stopped; the camera topic again has zero publishers.

## Authorized deployment

After user authorization, the tested corrected source was deployed to the remote
active package and `COLCON_IGNORE` was moved out of the package. The package was
rebuilt in place with `--symlink-install`. Workspace-wide discovery initially
reported an unrelated duplicate package at
`src/aiformula/perception/lane_line_publisher_new`; that directory was not
changed, and the successful build constrained discovery to the intended active
package.

The bottom-left RViz panel was changed from Dynamic ROI to Vehicle Fit while
preserving its dock geometry. The consolidated launch was restarted with a clean
environment using `/opt/ros/foxy/setup.bash` and
`/home/workspace/install/local_setup.bash`.

Final clean launch PID: `7451` at verification. Child PIDs are historical and
must be rediscovered before signaling:

- road detector: `7501`
- deployed lane-line publisher: `7503`
- RViz: `7828`

RViz had one reliable subscription to
`/aiformula_perception/lane_line_publisher/vehicle_fit_image`.

The final test exercised the deployed node, not the isolated candidate. It
observed 65 cloud messages per side and a maximum of 9 finite, non-zero XYZ
points in each left, center, and right cloud. The generated Vehicle Fit image
contained all three colored lane lines. After the test, the injector stopped and
the camera topic returned to zero publishers. The clean consolidated launch,
road detector, deployed lane publisher, and RViz remained running.

Deployment hashes:

- source: `87a7748b3c363ec144f644dccfabd0f95a7befc599a45efeb3a2a0eaa574b93b`
- RViz config: `cd8eefab537b686e0ce959603029478cb94b3016dd98cf5862efd9476b83f400`
- built executable: `7e3cfd2adaf8a8339b347d5948136a8db033df01e722c2b7e8745afc401bfd31`
- deployed report: `0212a6cb0b2931402ade5203db06b054dd4df65aa8d71d91ae3b695db40c6240`

Rollback archive:

`deployment_backup/lane_line_predeploy_20260717.tar.gz`
