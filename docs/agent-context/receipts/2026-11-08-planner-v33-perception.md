# 2026-11-08-planner-v33-perception

- Role: planner / Codex; result type: RESULT_READY, **partial, not accepted for training or driving**.
- Input: [2026-11-08-planner-v3-acceptance at 5e8c8e7](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/5e8c8e73a1e8b5f9cc5cb652834e8efeb4dcd76f/docs/agent-context/sessions/2026-11-08-planner-v3-acceptance).
- Incoming checkout: 5e8c8e73a1e8b5f9cc5cb652834e8efeb4dcd76f. Dedicated branch: experiment/driving-v3-3-perception.
- Evidence/code commit: [a508a8267724261b0ac8a2fb13e0b8aee67b1b55](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/a508a8267724261b0ac8a2fb13e0b8aee67b1b55).
- Session: [2026-11-08-planner-v33-perception](../sessions/2026-11-08-planner-v33-perception/SESSION.md).
- [Bundle](../bundles/2026-11-08-planner-v33-perception.zip): 970337 bytes; SHA256 `a472f59f1d3ffc06edd22fd7d3138636b807fe7f66f4ffae5dd3f2e0c6d47bba`.
- Publication destination was verified before push: Sophia-AI-formula-team/neo_aiformula_sophia, immutable repository ID 1303517209. No merge, release, deployment or real-car output.

## Completed and checked

49 Unity trials with all failures retained. The same-speed baseline proves local perception improvements, but no candidate meets the complete registered gates. V3.4.5 passes the three development A/B/C passages; it fails consecutive obstacles and both development full laps. New long-training episode count remains **0**.

Eight new behavioral tests pass (exit 0). The sensor whitelist and actual selected-action audit pass across **49 runs / 9764 decision frames**. The nine 1920x1080 films (396.1 s total) fully decode, and representative comparison/near-field stills were visually inspected. This is not a claim that all films were manually watched second by second. All local HTML/media links resolve.

The new session validates and packages successfully. Sealed ZIP bytes, working files and staged bytes match exactly, as do all 24 retained source snapshot hashes. `git diff --cached --check` passes; the scoped whitespace attributes preserve final empty lines in frozen evidence without altering source hashes.

## Failed and unrun

The required repository-wide `python -m unittest discover -s tools/tests -v` run exits 1: **33 tests, 3 failures, 1 error, 1 skipped**. All failures concern pre-existing sealed records: byte mismatches in 2026-11-06-planner-unity-lap-training, 2026-11-07-planner-driving-v3 and 2026-11-07-planner-longtraining; the error is the earlier training.png invalid image signature. Those records were not changed. The new session passes its own validation and byte comparison. This receipt records the broader test result after session sealing rather than rewriting the sealed package.

Near-field physical-edge acceptance, complete fresh-start qualification, four disturbed laps, sixteen fresh obstacle layouts, new execution-step calibration, V3.2 initializer comparison and long training are not passed or not run. The existing surrogate's short-step fit does not validate the new five-second prediction horizon.

Existing selected subagents were reused after the authorized stale-quota reset, but HTTP 413 and 429/retries yielded no independent review. No further reset, unselected model or paid overflow was used.

**Drive upload is incomplete.** Computer Use timed out on the initial list_apps, retry and post-reset retry. No new folder/upload or sharing change occurred. Nine curated films, the report and their hashes remain under `codexws/learned_racing/unity_laps/v33_evidence/delivery/`; previous Drive evidence is untouched.

## State and next owner

The owned private Unity process was stopped through its queue, and its PID was confirmed absent. All per-trial vision processes exited through their runner. No hardware commands were issued; no background training is running.

Planner retains responsibility: first establish reliable physical-edge correspondence and reconcile current-footprint/recovery constraints, then validate execution predictions and retest failed development cases. Freeze and draw new held-out cases only after development feasibility; start >=10000 procedural episodes only after every gate passes. Resume the authorized Owen Drive upload when its connection is available.

No implementation is promoted as a new accepted baseline by this receipt. Evidence availability is not code acceptance or permission to operate the vehicle.
