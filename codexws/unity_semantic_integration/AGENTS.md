# Agent handoff: Unity semantic final63 archive

Read README.md, DEPLOYMENT.md, manifest.json and evidence/RUN_REPORT.md before
changing this directory. The user selected both native ROS integration and a
separate Unity source archive. This folder is the archive; the active semantic
packages live under workspace/src/aiformula/perception/semantic_planning_system.
Keep COLCON_IGNORE so archived package names cannot enter a colcon build.

## Frozen boundary

- Preserve core `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`, original baseline
  `5bc52e05329d7fe642f219d32a578d04d2b7f668`, and vision/LYA upstream
  `05f1651c8a14d56ab9a86a9767c52793403dcfdc` as historical pins.
- Do not rewrite final63 evidence to describe new code. A later revision needs
  a distinct source identity, regenerated manifest and separate acceptance.
- Preserve byte attributes on before/overlay/evidence. Native files come from
  Git objects with explicit newline recipes. Do not vendor another copy of
  the 104 native files or regenerate `.meta` GUIDs casually.
- Keep weights, videos, decoded images, build outputs, caches, large original
  ZIPs and intermediate replay fixtures out of this archive. The original
  diagnostic root and artifact hashes are recorded in README and manifests.
- No Tools/LaneTeachRepeat or unrelated settings belong to this change. The
  source baseline was captured while another agent had separate work; the
  original audit explicitly attributes those exclusions.

## Implementation facts to retain

The stdio semantic worker wraps the original Python classes and supplies
in-process ROS message/clock shims. It does not run DDS. Unity ideal odometry
is a declared control input; lamp truth, fixed world signal plane and CAD
route are evaluation-only. RGB distance means current body-forward depth.
Fixed-plane front gap means a separate transformed bumper-corner measurement.
Never call both fields the same distance or label the vehicle center as front.

The optional RGB panel/range adapter, current corridor selection, first-frame
headroom, finite prior lifetime and focus override are explicit simulator
wiring. Preserve disabled defaults and original native LLP/LYA gains. Paired
center is measured; current RIGHT/identity-checked LEFT centers are estimated.
A stale or unsupported candidate must not be relabeled valid to keep moving.

MapAssist's short cold-start traffic experiment is red stop / green resume.
It does not establish forced red-to-left behavior, full topology mapping,
automatic detouring, the complete circuit, native ROS operation or vehicle
safety. The retained first-motion sampling-edge limitation must remain visible.

## Safe next steps

1. Inspect Git state and the current owner of the Simulator and CPU. Do not
   refresh, start Play, compile, test or encode concurrently with another
   agent's control trial. The installer never performs Unity operations.
2. Verify payloads with install.py. Use immutable Git history rather than
   silently substituting current working files when a pin is unavailable.
3. Check a compatible existing Simulator with verify-before/verify-after;
   inspect reports mixed states. A mismatch is evidence to investigate, not
   authorization for force replacement or a whole-project reset.
4. Any applied source change requires a fresh exact source audit and new
   case directories. Preserve original failed and successful trials intact.
5. For tooling changes, run the isolated safety tests and a temporary-project
   roundtrip. Never use the live Simulator as an apply/rollback test fixture.
6. After a real trial, check cleanup errors, workers, speed, actual lane
   geometry and the rendered diagnostics, not only `accepted` or compilation.
   Four final63 runs are recorded evidence; a new installation must earn its
   own functional acceptance.

The installer hashes every managed before/after payload before any mutation
and checks unchanged prerequisites. Writes are limited to named files. A
multi-file interruption can leave a mixed state: retain the JSON completed-path
receipt, stop, inspect and review recovery. Do not add a force flag or recursive
deletion shortcut. Rollback removes only the exact added files after hash checks.

## Acceptance already recorded

Inner/outer red fixed-plane front gaps: 8.635939 m / 8.602903 m. All four
final63 cases reached the requested end region with no sampled center-lane
boundary crossing. The combined video has 2191 decoded frames and genuine
1× wall-time waits, with a 75% unobscured main view. Exact metrics, hashes,
limits and local paths are in README and evidence. These are Unity results;
no native ROS/DDS, Orin throughput or real-car acceptance is implied.

Formal core tests and shim tests are separate from this archive's installer
tests. Read their receipts instead of adding unlike test counts or treating
Python syntax compatibility as proof of a live ROS launch.
