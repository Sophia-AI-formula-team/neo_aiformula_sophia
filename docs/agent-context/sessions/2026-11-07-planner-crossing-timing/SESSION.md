# Local crossing-time experiment

## Objective and scope

Explain why the seven earlier obstacle videos did not demonstrate accelerating to pass before a moving obstacle. Run a matched optional capability experiment inside the existing Unity simulator and record its limitations.

## Input handoff

Continues the obstacle-films handoff at 62515a068f51b44243fe6d89fb2e62d0398738da. The starting worktree was clean. Reviewed the frozen balanced policy, detector timing, the earlier pass-speed guard, rank, paired source videos and command records.

## Source and working tree

Local branch experiment/unity-lap-training, with independent solver and video worktrees. The optional control source was frozen at 6e96acdd7d721216c396bbd8762152b95607fe0f. The held-out test protocol was recorded at ac07f6d. The former driver remains the default in the simulator experiment. No vehicle packages, physical equipment, live editor or remote branch were operated.

## Work performed

Audited each recorded update from first yield to the last injected obstacle station or contact. Ran the existing simulator with a local motion fit, front and rear arrival model, reachable acceleration and robust clearance comparisons. The only change within a scored pair is whether the candidate may select go_first. Both arms receive the same initial approach speed and camera model. Four new crossing phases were predefined before inspection; additional development and diagnostic pairs are labeled separately. The independent referee uses measured car motion and the cone's trajectory to measure temporal overlap. Actual camera images, A4 results and wheel commands were rerun for the selected pair.

## Evidence and result

All 185 yielding commands in the former seven films requested less speed than the current measured speed. The development demonstration starts with identical observations, physical rows and a measured 2.72 m/s at 7.80 s. With optional early passage enabled the car entered the crossing at 4.45 m/s, leaving 0.99 m minimum clearance. With the branch disabled, it entered at 1.83 m/s with 0.33 m clearance. The obstacle zone times were 3.45 and 5.42 s, respectively. The fast car's tail cleared 1.13 s before the cone entered its traversed strip, against 0.10 s in the slower branch. The recorded MP4 has 140 frames at 20 fps with two chase cameras and A4 views synchronized on the same simulation clock. See [primary audit](evidence/primary.json), [frozen phase protocol](evidence/phase_protocol.json), [independent holdout results](evidence/held_out.json) and [historic window audit](evidence/frozen_choices.txt).

## Risks and blockers

The mechanism and confidence thresholds are engineered and this measurement used a single moving object on a straight with a common 2.4 m/s approach target. At full cruise the new branch did not trigger. Both arms passed 3/4 held-out phases; the one held-out positive saved 4.46 s but reduced minimum clearance from 0.93 m to 0.22 m. A different phase and two additional constrained timing diagnostics still contacted the object. The experiment proves a behavior exists, not an overall stability improvement. Shadow-only or hardware validation was outside the scope.

## Next agent actions

Inspect late arrivals vs fitted lateral motion error and revisit the fallback's ability to stop or wait. Freeze additional test sets before learning the timing choice or changing clearance margins. Local files are under codexws/learned_racing/unity_laps/crossing_evidence and timing_revision; the previous gallery contains a separate link to the new movie.
