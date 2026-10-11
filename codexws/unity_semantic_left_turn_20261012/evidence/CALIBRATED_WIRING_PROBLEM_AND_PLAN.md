# Explicit simulator calibration wiring: report before repair

2026-10-12. Original semantic core remains commit 63eec2d. This is a simulator-only, opt-in adapter proposal; it does not repair or certify automatic junction learning.

The exact outer trial reproduced all 4321 numerical values from 1014 original candidate samples. Shape RMSE is 3.404463 m against the unchanged 3.0 m gate. All five official nodes are traffic junctions, while the four physical outer bends are between them. Changing the threshold or matching bend apices to junction IDs would misrepresent localization. The current A3 model has lane and detection heads, with no drivable mask. A reliable automatic solution needs new time-paired visual junction observations; absence of lane pixels is insufficient.

## Smallest separate wiring mode

Preserve the official OSM and every core source file. Add a default-disabled simulator adapter with an explicitly versioned calibration JSON. It must label coordinates as prior-map calibration used for control, never as observed junctions or automatic fusion. Store logical official IDs separately from metric edge geometry, including smooth intersection transitions. The logical bypass is 02 -> 06 -> 03: left entry, right internal turn, left exit.

Reuse the actual measured closed first-lap reference, with its original trust mask and unchanged metric scale. Associate configured junctions only by checked projection onto that reference. Validate unique projection, trusted neighborhood, bounded projection error, cyclic order and direction; preserve the failed automatic result in diagnostics. A saved reference, if loaded for bounded repeat testing, must be the exact captured real sample set with verified source/configuration hashes and an explicit warm-start label. It must not be called a fresh learning lap.

Do not fake an accepted similarity. The adapter can provide an explicitly calibrated graph and separate traversal direction to MapAssist; automatic alignment remains unavailable. Preserve real signal votes, odometry/command age, E-STOP, ownership and branch/OA gates. Enter detour only near the configured junction with fresh localization. Every other/default run must follow the original path exactly.

The simulator consumer currently unconditionally stops on missing visual ABC before reading a valid map-detour command. If this is observed in a calibrated run, a scoped ownership repair may allow only a fresh, explicit semantic detour with fresh generated map ABC to own steering. Genuine RGB still runs and expired images, odometry, map command or transport must stop. Normal lane following keeps the original watchdog. No global watchdog weakening is justified.

Dense edge geometry alone is insufficient with the original segment-local tracker: its three preview points all stay on one tiny segment, losing curvature and compressing preview to near-zero distances. The independent right-angle counterexample already records the late segment switch/behind-target failure. The opt-in adapter therefore also needs a continuous arclength preview across consecutive configured segments, with monotonic bounded local projection, cross-track/entry/behind-target rejection, explicit completion and no arbitrary global jump. This is new adapter logic, not an assertion that the original tracker has been fixed. Original core/default tracking remains unchanged; tests must cover overshoot, bends, self-crossings, stale/invalid geometry and terminal behavior before a live run.

## Evidence requirements

First preserve a separate failure video for exact inner and outer trials. Stage adapter and calibration changes for review, use meaningful default/invalid-input tests, and audit configured paths against independent road geometry before enabling them. Actual MCP inner and outer runs must show entry, physical left rotation/displacement, internal traversal and exit. A video or unit test alone does not certify lane/vehicle clearance. Calibration coordinates are visibly disclosed control inputs; geometric evaluation results are separate.

The optional map-mode preference is still pending. Preparation of this disabled wiring is within the requested repair scope; enabling it for live validation will be reported explicitly. Genuine automatic junction learning remains a separate unresolved capability.
