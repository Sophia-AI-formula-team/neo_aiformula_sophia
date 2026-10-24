# Static review before any rendering

Prepared for the neo `stop_wait` Unity validation. No image decode, encoding, model inference, compilation or runtime test has been performed by this media subtask.

## Input contract

- `frames.jsonl`: ascending `wall`, observer `path`, recorded `t`, optional same-frame previews.
- `trace.jsonl`: actual detection rows, `candidate` from original LLP/LYA, `overlay`, `rgb`, `sequence`, `epoch`, `age`.
- `motion.jsonl`: original semantic response and commands, `semantic_running`, `semantic_result_age`, original `signal_truth`, `stage`, `case_kind`, `worker_generation`, `worker_boot_id`, `diagnostic_hold`, `topo_closure`, `persistence`, `traffic_stop_required`, `command_gate`, `restore_event`.
- `events.jsonl`: actual `wall` and `event`; every event receives a 1x context window and decoded QA sample.
- `semantic_ticks.jsonl`: original response ledger, sealed even though the normal display samples `motion.jsonl`.
- `configuration.json` and `result.json`: mandatory provenance and exact recorded outcome. A short source-version JSON supplied by root is separately hashed and shown in the video.

`status.traffic_policy` is `stop_wait`. `status.traffic_mode` is a runtime state such as `normal` or `wait`; it is not used as a policy check. `status.signal_approach.enabled=true` is rejected. The explicit policy file additionally binds actual configuration values for policy, distance, bbox and detour to expected values. Missing fields are errors, not assumed false. The sample policy paths must match finalized harness output before a real plan is sealed.

The exact neo 0.3 closure keys are `state`, `blocker`, `recorded_samples`, `recorded_distance_m`, `distance_to_start_m`. Persistence displays `state`, `saved_samples`, `resume_distance_m`, `error`. `saved` is never relabeled `restored`. The producer's original same-boot `restore_event.observed/tick_sequence` is displayed separately; the renderer does not manufacture it. A transient `restored` state remains in original events/ticks and is not inferred from a later saved checkpoint.

The root explicitly reuses the previously accepted Unity RGB traffic helper (`signal_panel_rgb=true`). The renderer reads that actual configuration and labels the A3 lane overlay + RGB signal helper combination. `detected_light_source` remains visible; the scenario lamp is evaluation-only. Runtime helper override and restoration are in the harness configuration, rather than described as an unchanged A3-only classifier.

## Preservation and display

No source file is rewritten. Every image/table in the plan is checked before and after rendering. Explicit epoch mismatches remain stale; missing clocks/ages remain absent. A same epoch does not itself imply control admission. The final result caption uses recorded `accepted` and `reason` verbatim; renderer success only means media encoding/decoding success.

Only the old renderer's file reading, image containment, drawing primitives, LLP/LYA panel and semantic-age helper are imported. Its old traffic/distance/trajectory panels, thresholds and case list are never invoked. `distance`, `signal_panel`, range fields and SLAM outputs are not used by the new renderer. Candidate geometry is only the genuine recorded output. The top 720 of 960 pixels are never overlaid.

The schedule covers the first through last actual camera wall times in order, at 1x/2x/4x only. Event windows and semantic state changes receive 1x; source still samples retain their real wall timestamps. All source-to-output indices/speed/generation/epoch are in `timeline.jsonl`. Exact final camera sample is appended with one documented output frame, rather than cut away.

## Runtime verification pending

Root must provide the owned process-tree bound and cleanup evidence. The environment guard is only an invocation contract. Render code retains failed partial output, does not overwrite existing destination folders, and never retries. Full decode uses showinfo on every original output frame before sample selection and checks exact rational PTS without forcing decode dimensions/fps/pixel format. Samples are extracted from the encoded video, not substituted from original PIL frames. Actual visual review is a separate required step.

Root should run syntax/import checks before the first real encode, confirm source config paths and short version labels, then inspect decoded frames at red-stop, green-start, closure and restart boundaries. No test pass or functioning video is claimed by this static review.
