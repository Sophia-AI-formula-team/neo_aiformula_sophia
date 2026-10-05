# Vehicle integration handoff

## Scope and preservation

- Keep the vehicle-sourced `workspace/src/aiformula/perception/semantic_planning_system` contents unchanged during the 2026-10-05 YOLOP integration. It is an exact source snapshot, including its existing tests and documentation.
- `lane_points/lane_points/lane_0927oa.py` is an additional executable. Do not replace `lane_0215`, `lane_0529oa`, or other existing implementations with it.
- Preserve existing detector code and weights before deploying a new version. Use separate directories and reversible launch selection, and record exact backup and rollback paths.
- Do not use bag smoke tests for this integration. Acceptance is live camera input, lane mask Image output, compatible object mask/annotated boxes, then a separately launched obstacle-avoidance node producing its spline output.
- Maintain the integration status, commands, commit/weight hashes, validation results, and rollback instructions in `docs/agent_handoff_20261005.md`. Never describe a planned step or a static check as a passed live test.

## Current interface contracts

- Camera input: `/aiformula_sensing/zed_node/left_image/undistorted` (`sensor_msgs/Image`).
- Lane mask: `/aiformula_perception/road_detector/mask_image` (`sensor_msgs/Image`, `mono8`, source image dimensions, 0/255).
- Obstacle mask: `/cone_mask` (`sensor_msgs/Image`, `mono8`, source dimensions, filled detector bounding boxes).
- Annotated object image: `/cone_bbox_image` (`sensor_msgs/Image`, `bgr8`, source dimensions).
- Avoidance output uses the existing `withoutkalman_0312` switching interface: OA three-point output to `/oa_point_a`, `/oa_point_b`, `/oa_point_c`; selection flag `/point_flag`.
- `allnodes.launch.py` keeps `lane_0215` as its original lane-points implementation. The legacy lane-only road detector is opt-in so a separately launched new detector can be the sole lane-mask publisher.

Read the dated handoff before continuing vehicle work; it records what is actually deployed and what remains pending.
