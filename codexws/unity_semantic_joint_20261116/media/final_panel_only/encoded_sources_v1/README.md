# 1115 semantic joint experiment video

`joint_media.py` reads completed `AIFormulaJointTopoTrial` recordings. It does not run perception, planning, control, map fitting, or acceptance tests. No Unity/repository source files are changed. The layout is 1920 × 1080 at 15 fps:

- Left top: the existing first-person detection overlay at 1280 × 720, with no added text covering the image. Original `detect.Input` is also visible below. If the recorded overlay is missing, the main image uses the actual input and says so.
- Right: actual recorded semantic map output. Cyan is the driven trajectory, green the current reference, gray the first closed odometry reference, purple the learned samples, and pink the planned detour. **Dashed internal graph edges are estimates, not observed interior road mapping.** An official graph whose `graph_frame` is `official_unaligned` gets a separate inset, never an odometry overlay.
- Bottom: original RGB, LLP clouds/ABC and LYA selected points/target, traffic and command values, map status and the recorded acceptance result. Physical lamp truth is labeled evaluation-only. RGB helper use is labeled separately.

The required files are `configuration.json`, `result.json`, `frames.jsonl`, `trace.jsonl`, `motion.jsonl`, and `events.jsonl`. JSONL rows use the harness's relative `wall` clock. Worker UNIX `wall` timestamps are **not** mixed into that clock. `motion.semantic` contains the original worker response including `map_data`; `trace.candidate` is the recorded LLP output. `frames.source` must be `actual_first_person_detector_input`. Bbox distance and old `neo_stop_wait` must be explicitly disabled in configuration; only `semantic_joint_1115` is accepted.

The planner source/version identity remains in the sealed original configuration and worker receipts. The renderer does not assume the old `stop_wait` validation or old map acceptance applies to the new package.

## Prepare after the run finishes

Use the existing Python environment with Pillow and either PyAV or an existing FFmpeg executable. Python313 on this machine already has Pillow 12.2.0 and PyAV 17.1.0 with libx264/yuv420p encoding and H.264 decoding; no dependency installation is needed. Supply actual absolute paths; `RUN` is the completed recording directory, not the source package.

```powershell
python joint_media.py prepare --source RUN --plan NEW_PLAN.json --name INNER --seconds 30 --backend pyav
python joint_media.py render --plan NEW_PLAN.json --preview NEW_PREVIEW.png --at 30
```

The CLI backend remains available through `--backend ffmpeg --ffmpeg FFMPEG_EXE`. There is no automatic switch between backends. The PyAV plan seals its installed Python extension/DLL hashes and library versions; render-time checks reject a different imported installation.

After the root verifies the live Unity scene matrices, optionally add `--evaluation-surface EVALUATION_SURFACE_JSON`. The JSON must state `live_mcp_verified=true`, `coordinate_frame=unity_world_xz`, and `enters_control=false`. Its exact SHA is pinned in the plan. Recorded green/paint boundary rings are drawn faintly behind the semantic output using only `odom=(world_z,-world_x)`: the exporter has already applied the scene transform. No fitting, forced topology alignment, road-connectivity inference, or control input is introduced. Boundary-only drawing preserves the green islands' asphalt holes; the visual-ground AsphaltApron does not define road boundaries.

Preparation hashes the source tables and all referenced images and seals the result verbatim. It performs no image decode or video encoding. Do not prepare while the recording is still being written. An existing plan/preview/output is never overwritten. Use a separate plan and output for outer, inner, and any pre-fix failure runs.

The schedule retains **the full case from wall zero through the last recorded event/motion/result time**, including startup stalls and rendering transitions. Before the first recorded image, the main panel explicitly says no first-person image exists yet; it never uses a later image to fill an earlier interval. Thereafter images use sample-and-hold with distinct capture, perception and semantic ages and no interpolation. Full-lap portions are compressed to the requested duration; red, closure and turn transitions get slower windows. If `red_detour_failed` or `completed_detour_waiting_at_red` occurs, the preceding recorded 12 simulation seconds are retained at **1× source-wall playback**, even if this exceeds the approximate duration. This duration extension is explicit in the plan. Green recovery is labeled separately and cannot erase the earlier failed red detour. A run ending before two physical laps stays a failed run; the video never manufactures a second lap or an interior turn.

Inspect the plan and one-frame previews before encoding, especially graph alignment, source ages, failure/recovery labels, and text readability. Map scale uses the full recording's bounds for stability, but only geometry and actual trajectory already recorded at the current frame are drawn.

For the user's combined approximately one-minute delivery, prepare the completed inner and outer cases with **`--seconds 30 --backend pyav` each**, then seal them in order with the command below. The combined renderer opens **one encoder** and directly renders both case timelines into a single video; it produces no intermediate encoded clips. The existing single-plan path remains available. See `COMBINED_VIDEO.md` for the narrow design and verification limits.

```powershell
python joint_media.py combine --plans INNER_PLAN.json OUTER_PLAN.json --plan COMBINED_PLAN.json
python run_joint_bounded.py --plan COMBINED_PLAN.json --python PILLOW_PYAV_PYTHON_EXE --attempt TASK_MEDIA_NEW_ATTEMPT_DIRECTORY
```

The `[1/2]` and `[2/2]` labels identify distinct trials, with a direct cut and no implied continuous drive. Per-frame case identity/local time and start/end boundaries are read back during full-video decoding. Both sides of the cut and each case's start/middle/end receive decoded PNGs. Inspect the printed combined duration before encoding. If protected red-wait intervals force a duration above approximately 60 seconds, report the actual total instead of removing or accelerating protected failure observations.

When the recorded opt-in identity adapter is enabled, diagnostics separately show the final control light, raw model light, filtered model light, evaluation-only physical lamp, and the identity-filter reason. The original detection overlay is explicitly labeled unchanged; a rejected raw red model box can remain visible while the effective control state is unknown. This is not hidden or presented as an unfiltered classifier result. All displayed filter values come from the recorded `trace.signal_panel`, not from new detection logic in the renderer.

The display accepts the current configuration name `signal_panel_model_identity_guard` and historical `signal_panel_reject_conflicting_below_horizon_lights`. It shows configured state separately from the observed `signal_panel.model_identity_filter.enabled`; contradictory recorded aliases are labeled `cfg CONFLICT`. These fields never set or change control behavior. Final comparable cases must use the same actual adapter revision; inner05 is historical and is not paired with the newer outer02 revision.

For recordings explicitly declaring both `signal_panel_only=true` and **`signal_policy=calibrated_rgb_panel_only`**, the panel instead states **Signal: calibrated RGB panel; raw model display-only**. `signal_policy` is the actual configuration and helper field; the earlier proposed name `signal_input_policy` is not accepted as a mode selector. The actual control light still comes from recorded aggregate `detected_light`; raw model state remains visible, but an `effective_model_state=unknown` model-exclusion marker is not drawn as a missing RGB-panel observation. The original A4 detection overlay remains unchanged. This is a display policy only. The map statistics explicitly count **OSM nodes**, not newly built intersections. Final planned cases for this policy are outer03 and inner07; read/prepare them only after the root confirms completion.

## Encoding — root's exclusive CPU window only

Do not start during Unity/model work. The root must invoke this under its owned, bounded process-tree supervisor and attach that supervisor's receipt. `run_joint_bounded.py` now supplies the task-specific Windows wrapper. `AIFORMULA_JOINT_MEDIA_BOUNDED=1` alone remains a caller contract, **not proof** that a process is contained.

```powershell
python run_joint_bounded.py --plan NEW_PLAN.json --python PILLOW_PYTHON_EXE --attempt TASK_MEDIA_NEW_ATTEMPT_DIRECTORY
```

The attempt must be a **new subdirectory inside this task's `media` directory**. `render/` is reserved for the renderer's outputs; supervisor receipts and stdout/stderr remain in the new attempt directory. An existing attempt is never resumed or overwritten. A fixed named mutex permits one joint encoding attempt at a time, but does not coordinate with Unity; the root still owns the CPU scheduling decision.

The wrapper starts the Python launcher suspended (`CREATE_SUSPENDED=0x00000004`), assigns it to a private `KILL_ON_JOB_CLOSE` Job, verifies the unique initial thread, then resumes it. This includes children of a Python virtual-environment launcher and FFmpeg descendants. A fixed 300-second primary deadline ends with owned-Job termination, at most 5 seconds of cleanup observation, and no automatic retry. An abandoned wrapper handle also triggers kill-on-close. It never kills by process name or follows an unowned PID. Birth tokens, independently observed Job membership, sampled children and final `ActiveProcesses=0` are recorded. A cleanup failure is explicitly marked `cleanup_unverified_do_not_return_CPU`.

`windows_owned_job.py` is copied byte-for-byte from the now archived `semantic_neo_topo_20261015/staging/media/windows_owned_job.py`, SHA256 `aa6f98f52029502a5ad4452acafc7829f99d1a31e55a7d1b518b21446f98fd24`; the wrapper verifies that hash before importing it. There is no runtime dependency on the old directory. The wrapper has no old five-case policy or retry machinery. Its first actual PyAV encoding attempt, `../../media/inner04_failure/encode_v1/receipt.json`, completed in 83.476 seconds with independently observed interpreter Job membership, complete decode, verified cleanup and zero remaining owned processes. The encoded trial itself failed semantically; machine/media verification does not change that outcome.

Both backends use one encoding thread. Outputs under `render/` include `semantic_joint.mp4`, per-output-frame source indices/times, three composed frame PNGs and a receipt. PyAV additionally decodes the complete video with one decoder thread, checks every decoded frame's exact rational PTS, count, dimensions and pixel format, writes `decoded_frames.jsonl`, and exports three **actual decoded** PNGs. The FFmpeg backend writes its log and performs a complete `ffmpeg -xerror` decode. All sealed source hashes are checked before and after. The composed PNGs are **not decoded-video evidence**. Final visual QA must inspect the decoded images and actual video; the machine receipt deliberately leaves visual QA pending. Initial real-data preview findings are recorded in `../../media/inner_joint_01_preview/PREVIEW_QA.md`; no encoding has run during preparation.

Completed baseline media and visual QA: `../../media/inner04_failure/VIDEO_QA.md`. Exact pre-clarification renderer/wrapper/helper files are preserved in that directory's `encoded_sources_v1/`. Future plans should use the current renderer, which explicitly labels genuinely null LLP packets during the stopped tail. Do not silently replace a null current packet with an older nonnull packet.

The separate original outer01 failure is documented in `../../media/outer01_failure/VIDEO_QA.md`, with the actual decoded false-red trigger, 0-lap result and its own exact encoded-source snapshot. Neither failure video is overwritten by later display updates.
