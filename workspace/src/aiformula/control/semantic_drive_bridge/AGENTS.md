# Scope and verification

- This package is real rclpy shadow wiring, version 0.1.0. Do not introduce ROS
  stubs, virtual message modules, AST class extractors or model inference here.
- Preserve the hard `/semantic_drive/shadow_*` output restriction, including
  validation after ROS remapping. This version has no live-enabling parameter.
- Rejected captures never become unknown votes. Do not refresh source sequence,
  color, range, commands or receipt age from cached/duplicate observations.
- Clock rollback requires a full-chain restart. Preserve steady-clock timers
  independently of ROS `/clock` and require a new command after a hold/fault.
- Keep core algorithm changes outside this package and separately reviewed.
  A stamped capture relay does not close the later bridge-to-core DDS age gap.
- Pure policy tests, real rclpy/DDS tests, Simulator trials and physical vehicle
  evidence are separate categories. Report missing runtime checks explicitly.
