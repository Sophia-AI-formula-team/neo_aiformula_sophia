# motor_controller 0.2.0

This version replaces neo's selectable 0.1.1 controller with the implementation
requested from `Sophia-AI-formula-team/aiformula_sophia`, commit
`dd55f65ef2fe434512d9f7bf48fff7c17141fac3`, path
`aiformula/control/motor_controller`. `UPSTREAM.json` records source checksums.
The control algorithm file is byte-for-byte identical to that source.

Twist input is `sub_speed_command`; the output is `pub_can`, CAN ID `0x210`,
eight bytes containing two signed int32 little-endian RPM values. The existing
launch maps these to `/aiformula_control/game_pad/cmd_vel` and
`/aiformula_control/motor_controller/reference_signal`. Wheel tread, diameter
and gear ratio continue to come from the vehicle's wheel.yaml. The package's
timer remains 0.01 seconds.

The imported controller uses the upstream fixed empirical compensation. It does
not implement the previous controller_state selector, joystick state switching,
live RPM correction weights, learned-model mode or correction debug topic.
The source's output ordering and compensation constants are preserved exactly.

Integration metadata is versioned 0.2.0, with explicit runtime dependencies,
upstream MIT attribution, and this provenance record. The invalid upstream
motortest entry point is removed because its module is absent. Tracked Python
bytecode is excluded. No control algorithm or empirical constant was changed.

Only motor_controller is built for this deployment. The independently staged
semantic packages remain unbuilt. An already-running Python node keeps its
loaded previous code; restarting its owning launch selects the newly installed
controller. Build and isolated ROS-topic validation do not establish physical
driving performance.

The previous neo implementation remains in Git history at
`05f1651c8a14d56ab9a86a9767c52793403dcfdc`. Vehicle source, build and install
backups and deployment receipts are recorded in the dated deployment handoff.
