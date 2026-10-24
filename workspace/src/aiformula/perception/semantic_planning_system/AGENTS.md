# Agent instructions: semantic planning

Read README.md, DEPLOYMENT.md, AGENT_HANDOFF.md, and INTEGRATION_ISSUES.md first.
Inspect the current branch and dirty state. This subtree contains multiple ROS
packages and two different planning pipelines. Do not combine their launches.

## Change boundaries

- Preserve semantic core revision 63eec2d for the recorded four-case baseline.
  This integration release changes wiring and documentation only. For a critical
  core issue, write a concrete problem report before applying an algorithm fix.
- Keep `enable_signal_approach` opt-in. Package vehicle defaults are 8.5 m,
  0.8 m/s and 1.0 m/s². Unity trial values 8.0/3.0/1.5 are simulation overrides.
- Keep stamped atomic signal input separate from the legacy color String. The
  latter clears the core's range observation. Never connect both to one detector.
- Do not change the camera, light pose, scene, calibrated transforms, LYA gains,
  model contract, source weights, motor state1 math, or another agent's Tools.
- Native ROS uses `control/semantic_drive_bridge` and the dedicated shadow
  launch. Do not route this release to the motor topic or add an automatic arm.
- Unity workers contain transport shims and pinned class extraction. Preserve
  that description. They are not native ROS nodes or DDS validation.

## Verification and coordination

Keep package unit tests, source checks, transport tests, real Unity runs, DDS
tests and physical vehicle tests separately named. Never convert one into a
claim about another. The final63 video is evidence for that exact source tree.

Unity interaction must use Unity MCP, not Computer Use. Obtain exclusive Unity
ownership from the other active conversation before Refresh, Play, or scene
changes. CPU/model tests also require respecting the current shared CPU lease.
Return ownership explicitly and preserve editor/runtime restoration evidence.

Before uploading, review the scoped diff, run relevant tests and diff checks,
record package versions and source hashes, and exclude weights, raw RGB frames,
secrets, caches and unrelated files. Use a feature branch and a draft PR. Preserve
the existing PR stack and immutable tags; do not merge, force-push or create a
release tag without the user's request. Keep support records attributable to the
source commit that produced them, even after newer commits are added.
