# V3.2 independent Unity acceptance — 2026-11-08

Acceptance **failed**. The final 30,000-trial / generation-118 model has no completed full lap in the three new full-lap attempts. V3.0 and V3.2 each complete only 2 of 16 declared scenarios under the all-wheel-centers-out rule. V3.2 contacts a cone twice; V3.0 has no cone contact but departs in 14 cases. This is not a useful overall driving improvement.

The [Owen Drive delivery](https://drive.google.com/drive/folders/1XUABULfYf_SegyxFIdqSDcuQ_ye7WJII) contains all 22 files: 18 videos, training plot, report, index and evidence ZIP. Google Drive confirmed 22 completed uploads; all files remained after refresh and the overview played successfully. `upload_manifest.json` records local SHA256 values and UI verification; remote server checksums were not retrieved. Local videos are in `delivery/` (ignored by Git).

## Fixed protocol and honest comparison

`plan.json` was saved before any new outcome. Both versions use identical held-out seeds, static locations and moving-cone phases. Original control source bytes match the training snapshot. No coefficients, perception thresholds or physics were tuned during acceptance. CEM trained nine interpretable controller coefficients in the procedural surrogate; this is not 30,000 Unity runs or an image-to-command network. A4 perception remains the learned image component.

The original referee terminated at **any body corner outside**. Its 32 attempts remain in `runs/`. That rule is stricter than the user's whole-vehicle departure rule, so a separately declared second cohort (`race_rule/protocol.json`) uses all installed wheel centers outside. The installed Honda simulation has two front wheels and one rear caster; it is not a four-wheel simulator. Contact patch width is not modeled by this departure test. The controller receives neither rule nor wheel positions. The default old referee gate remains unchanged.

Across 2,379 common time samples, both cohorts have exactly identical traces before the earlier strict termination. Original strict results are not overwritten or described as passing. This additional cohort was selected after seeing strict failures and is a rule-sensitivity check on the same scenarios, not another blind sample.

Only scenario 16 completed for both versions: V3.0 9.89 s, V3.2 9.49 s. Scenario 12 improves from departure to completed; scenario 15 regresses from completed to departure. Early departure/contact times are never ranked as quicker completion. No valid paired lap-time improvement exists.

## Sensor and decision evidence

Each of 64 runs retains RGB/chase frames, A4 overlays, sensor packets and all decisions locally. The planner input whitelist is camera pixels, time, motor RPM proxy, raw gyro and fixed wheel radius. There is no GNSS, global pose, course station, preloaded map or true obstacle position in its input. The RGB pavement-color extractor plus A4 line mask is not a learned drivable-area segmentation model. Ideal gyro and target/plant RPM proxy are simulation limitations, not independent real encoders.

`acceptance_audit.py` captures local arrays after the original `Planner.step` returns. It never recomputes or alters the issued command. Its 512 comparison steps match original commands and decision fields exactly. All 2,544 candidate-bearing recorded decisions pass command, score and uncertainty-margin consistency checks. The controller uses 60 candidates, a finite prediction horizon and geometric uncertainty margins; these are not calibrated passing probabilities. Its no-feasible branch still issues a least-harm command and is not a verified emergency-stop mechanism.

## Films and verification

- 16 paired, full-attempt films: chase + perception + all local candidate paths + chosen/reference point + geometric margin formula + alternative scores + requested and actual speed. Each starts from rest and retains its terminal failure or success. Both sides share one simulation clock, with the ended side held still.
- A narrated-by-captions overview with excerpts explicitly labeled.
- A separate fixed-coordinate referee movie makes stationary/moving/continuous obstacles clear. Its world geometry is for the viewer only.
- Training curves show trial-population statistics, not final-model validation accuracy.

Source images and inference are 5 Hz. Encoded video is 1080p H.264 at 10 fps, repeating the latest captured frame without synthesizing intermediate camera images. `verification.json` records complete decoding of all 18 films (537.7 seconds), all stream counts, source/model/video hashes and contract checks. `visual_review.jpg` is the 16-case middle/terminal inspection sheet. `delivery/evidence.zip` contains 90 source/model/trace/protocol files with a verified SHA256 manifest. The full image archive remains local; the films and JSON preserve the requested visual/control evidence.

## Reproduction and limits

Use the already owned private Unity copy only. `acceptance_run.py prepare` is a one-time immutable registration; its strict source hash check deliberately rejects the later referee change. To rerun the strict cohort, use the archived `frozen/strict_referee` sources in a separate checkout and new output directory. `acceptance_race.py` resumes only the declared separate cohort without changing the models. Render with `python -B acceptance_media.py`, `python -B acceptance_diagnostics.py`; verify and package with `python -B acceptance_verify.py`.

Six existing V3 contract tests pass. Actual Unity driving acceptance fails. Current road association, prediction-versus-execution mismatch, and no-feasible behavior require a new version and a new held-out evaluation. `prediction_followup.json` compares an open-loop prediction to later re-planned motion, so it must not be misrepresented as a pure plant-model error measurement. No physical vehicle was connected or operated; the owned Unity batch editor was stopped after all runs.
