# Motor controller versions — 2026-10-19

The vehicle and repository keep these three sibling directories:

| Directory | Version | Build discovery |
| --- | --- | --- |
| `motor_controller` | 0.4.1 | Active ROS package |
| `motor_controller_old_bkup` | 0.2.1 | Source backup only |
| `motor_controller_new_bkup` | 0.4.1 | Source backup only |

The backups contain the complete tracked package snapshots from PR #20
(`b9676e422d405fe49a73d56fd3b0699bf458e71f`) and PR #24
(`67aa5ad3abc0c93559f53306f54334353ba56eb5`). File bytes are preserved.
Only the root build metadata filenames change: `package.xml.bkup`,
`setup.py.bkup`, and `setup.cfg.bkup`. No `COLCON_IGNORE` is used.
Directory suffixes alone do not prevent colcon package discovery.

Only the active directory is discoverable as `motor_controller`. To restore
a backup, first preserve the active package, copy the chosen snapshot into
the active directory, and restore those three original metadata filenames
there. Do not restore their original names inside the backup directories.
The launch package name and Python imports remain `motor_controller`.

0.2.1 is the original USB sigmoid steering controller. It has no independent
command timeout and its zero-command sigmoid path can generate nonzero wheel
RPM. 0.4.1 retains nonzero sigmoid steering and fixed wheel compensation,
adds exact-zero/straight-motion fixes, a 0.5-second command timeout, manual
priority, and optional semantic traffic gating. Both retain CAN ID 0x210
and right/left signed int32 little-endian encoding.

This motor-only integration does not merge the stacked semantic/Unity
experiments from PR #24. Semantic traffic gating requires the matching
MapAssist 0.2.2 traffic-only status and lifecycle wiring described in the
active package README. Those dependencies were already deployed on the
vehicle; importing this motor package alone does not install them on a new
checkout. Existing velocity-command operation does not require semantic.

Validation for this layout uses package discovery, exact snapshot hashes,
static Python checks, and pure motor-gate unit tests. No vehicle node is
started and no driving command is published.
