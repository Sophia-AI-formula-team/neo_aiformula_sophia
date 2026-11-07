# Agent session: 2026-11-07-planner-driving-v3

Input handoff: 2026-11-07-planner-longtraining

Result is partial: healthy GPU training startup, pending training completion and Unity lap acceptance. No vehicle operation or new external publication.

## Objective and scope

User requests versioned hybrid driving, first-lap sensor-only input, tens of thousands of generated trials, risk/speed optimisation, GPU parallelism, and subagent repair. Stop continual supervision after observing healthy training. No hardware operation.

## Input handoff

Read the prior longtraining SESSION, laboratory README, project context, protocol and runbook. The prior 560 probes did not meet the new training scale. Old reference points extrapolated a fixed outer lane and object tracking used true body pose.

## Source and working tree

Started from clean f5d9a121919f4eb3f75691235a12a3a86b1ac132. New worktree lap_driving_v3_wt, branch experiment/driving-v3. Source changes are confined to the Unity research laboratory and this record. The original integration worktree and vehicle packages were preserved. Router repairs were developed in E:/Tools/CodexRouter-worker-fix and applied to the local service with preexisting changes retained.

## Work performed

Added V3 sensor contract, visible local road extraction, white-box trajectory/footprint prediction, explicit narrow-opening candidates, CEM parameter learning, resumable journals and source snapshots. Fitted dynamics from 45 archived Unity training runs, checked 15 separate runs. Added CUDA Graph batch prediction on one RTX 4070 Ti. The CPU still advances the procedural plant and sensors. Formal command: v3_train.py --out .work/v3_2_long_30000_20261107 --attempts 30000 --population 32 --cases 8 --backend cuda, using the existing CUDA Python environment. Launcher PID 5108. Source bytes are frozen under the run's sources/ directory.

Three actual Unity tests used the previously owned lap_gap_batch_20261107 copy; its editor was stopped afterwards. Added three-panel failure videos. Fixed external-worker oversized tool-output handling and Antigravity cleanup error masking; live checks returned V3_WORKER_RETURN_OK and AGY_WORKER_RETURN_OK. Eight provider request slots used, no paid overflow or model-selection change.

## Evidence and result

Startup evidence: evidence/startup.json. Six contract tests pass. 960 CPU/CUDA command comparisons have maximum error 3.33e-7. Router expanded tests: 48 pass; Antigravity fault-injection test passes. Formal training was healthy at 768 trials / 3 updates, stderr empty, sampled GPU utilisation 47% and memory 2163 MiB. Supervisor stopped continual monitoring at this point as requested; the job continues independently. Details: codexws/learned_racing/unity_laps/v3_evidence/README.md.

Unity V3.0 completed a 33.7 m segment in 8.82 s. Full attempts failed: V3.0 boundary at 52.0 m / 14.19 s; early V3.1 boundary at 48.5 m / 15.11 s. Raw traces, terminal images and full three-panel videos are retained in v3_unity_evidence. Later explicit-opening/GPU changes have not passed fresh Unity lap validation. GPU smoke failures from float32 JSON serialisation and missing optional plotting dependency were retained and fixed before formal launch.

## Risks and blockers

Training is procedural surrogate work, not 30,000 Unity runs. A one-step fit is not long-rollout or real-car validation. Current Unity RPM is a target/plant proxy, not an independent encoder. RGB road continuity and whole-lap reliability remain unresolved; no online battery-adaptation claim. The evaluator still uses the old conservative body-edge departure rule. Current progress/checkpoints and videos remain local; no new Drive upload or repository push occurred. Provider-side Zen free quota exhaustion and Go Flash capacity refusal were not bypassed.

## Next agent actions

On the next user request, inspect terminal training results and retained failures, freeze a candidate without tuning on independent evaluation scenes, then run new A4 Unity evaluations. Training dashboard: codexws/learned_racing/unity_laps/.work/v3_2_long_30000_20261107/index.html; checkpoints, attempts.jsonl, owner.json and sensor/decision replays share that run directory. STOP stops after the current batch. Remove STOP and use the original command plus --resume; changed sources are rejected. Do not start continuous monitoring or vehicle output from this handoff.

