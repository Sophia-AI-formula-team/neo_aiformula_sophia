# YOLOP_A2 Deployment Handoff

## Publication scope (2026-10-05)

Publish the existing deployment runtime, both stripped checkpoints, ROS2 wrapper,
offline tools, tests, and portable documentation beside the original YOLOP.
No old model, launch file, vehicle source, dataset, or training code is replaced.
The deployment package's existing MIT license notice is retained.

The source training workspace is local YOLOP_A2. Exact optimizer continuation
uses `runs/object_detection/2026-08-13_01-00-30/epoch-20.pth` there; that training
checkpoint is intentionally absent from this deployment.

## Verification

- Before publication: the local deployment suite passed 17 tests.
- The publication copy also passed all 17 tests and the seven-case CUDA smoke
  on 2026-10-05, including weight hashes, separate masks, expected labels, and
  red/green-light adapter checks. Warm median in this run was 87.791 ms.
- Both included weights match the hashes in `context.md`.
- Historical full-lap inference processed 1545 native 640x360 images from
  `rosbag2_2026_06_29-17_48_44`, spanning 123.785 seconds.
- Full-lap outputs had 1544 nonempty lane masks and 881 nonempty obstacle masks.
- The PNG export validated 1545 binary 0/255 masks; frame 1521 was a valid empty
  mask. These counts are not accuracy scores.
- ROS2/vehicle execution remains unverified on the Windows development host.

## Integration

1. Use the install and explicit wrapper build commands in `README.md`.
   Match Python, torch, torchvision, CUDA, and ROS2 on the target first.
   This package requires Python >=3.10; it is not a Python 3.8 deployment.
2. Confirm the ZED topic, node namespace, and absolute checkpoint paths.
3. Select one publisher for lane topics. This addition does not alter launchers.
4. Observe lane/obstacle `mono8`, annotated `bgr8`, JSON, and light-state outputs.
5. Validate the avoidance ROI/count logic, camera-rate throughput, and failure
   handling on the vehicle before claiming integration complete.

Related PR #9 (`integration/live-yolop-20261005`) prepares detector/avoidance
startup and awaits the new model package. This independent publication supplies
the package without modifying that PR.

## Follow-up

Label barriers and tiny cones from the bag, then train and re-evaluate the
person/cone confusion and lane false positives separately. Use real traffic-light
sequences for temporal validation. Live stale-message handling also needs ROS2
verification.

Rollback is selection of the original detector in the integration launch;
the original `../yolop` is unchanged by this package.
