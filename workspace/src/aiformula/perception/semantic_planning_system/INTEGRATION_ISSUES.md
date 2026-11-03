# Native integration issues — 2026-10-12

This report precedes the new wiring. The semantic core is unchanged in this
integration release. The following native gaps prevent treating the Unity
result as a deployed physical vehicle result.

| Issue | Evidence in neo | Minimal treatment in this release |
| --- | --- | --- |
| Competing node and velocity owners | `allnodes` already launches detector, points, filter, teleop and motor; `map_drive` adds a different points/LYA chain | New independent shadow launch, private intermediate topics, no motor or teleop |
| Inference interpreter mismatch | A3 DEPLOYMENT requires Python >=3.10; stock Foxy uses Python 3.8 | Do not import A3 into the Foxy bridge. Run a separately compatible perception producer; interpreter/process bridge remains to be commissioned |
| No native metric signal producer deployed | A3 detection JSON has source Header; its plain color String has no stamp or range | Stamped capture receiver validates frame, source age and sequence, then produces MapAssist atomic input. RGB geometry/calibration producer is still required |
| Atomic range can be cleared by legacy input | `MapAssist._on_light_state` clears `signal_observation` | Keep legacy input on an unused private topic |
| ABC does not carry source provenance | `lane_0215` caches three clouds independently and publishes Header-less Pose2D, including default straight fallback | Shadow output only. A synchronized stamped ABC and command lineage adapter is required before live control |
| Extra relay delay | Core atomic input has `age_sec`, but no source Header | Receiver age includes capture-to-bridge latency. Subsequent bridge-to-core DDS latency is not rebased at final consumption; quantify/fix this before live deployment |
| Motor caches commands without input timeout | `motor_controller` repeatedly transmits cached CAN command | Independent shadow watchdog validates receipt age and status. Its process death still cannot stop the motor; motor/controller hardware timeout must be commissioned separately |
| OA owns output when active | MapAssist yields its point and command path to OA | This launch uses an explicit OA-idle configuration, starts no OA, and permits no live motor output; it does not provide a global red-light override for OA |

Real target validation still needs matching ROS/Python ABI, CameraInfo/TF and
road datum, complete source age through DDS, signal/ABC/command lineage, restart
and clock-reset behavior, single command publisher ownership, and physical
braking/watchdog measurements. Simulation LED dimensions and desktop Torch pins
must not silently become vehicle calibration or Jetson installation defaults.

The earlier 21–23 m stop problem and its reported fixes are separate: the old
color-only policy stopped immediately when red votes confirmed. Opt-in distance
approach plus perception-loss recovery achieved 8.635939 m and 8.602903 m in the
final Unity inner/outer red runs. See the frozen integration evidence; those
figures do not resolve the native issues listed above.
