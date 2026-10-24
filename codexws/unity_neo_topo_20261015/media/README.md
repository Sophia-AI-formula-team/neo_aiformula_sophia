# Neo stop_wait validation media

Status: preparation only. No video, model, image decode, Unity operation or test has run in this directory.

This renderer reads completed Unity recordings. It does not run the planner, infer detections, change commands, import SLAM code, or compute a distance. Bbox-distance recordings are rejected by an explicit policy contract in the sealed render plan. Raw `distance`, `signal_panel` and range fields are never displayed or used.

The 1280 x 720 observer image occupies the top 75% of a 1280 x 960 canvas, with no text drawn over it. Four lower panels show the recorded A3 overlay, original LLP/LYA geometry, traffic/command gate, and semantic topology/persistence. The official topology is not represented as a physical SLAM map. When `signal_panel_rgb=true`, the overlay is explicitly labeled **A3 lanes + RGB signal helper**, retaining the existing Unity traffic wiring; this is not an A3-only traffic classifier. The actual `detected_light_source` is displayed and the scenario lamp is separately labeled evaluation-only. Lamp truth is not supplied to control. Result labels are copied from `result.json`, never derived from a green image or an exit code.

`make_plan.py` seals the completed source tables and every referenced image by SHA256, adds the source version JSON verbatim, and builds an explicit full-coverage time schedule. Default playback is 4x; every recorded event and each traffic-stop / semantic-worker / topology closure transition receives a 1x window. First/last 3 seconds are also 1x. Overlapping windows merge. No source interval is cut; the last image sample is included. The actual playback speed, phase and capture time remain in the lower panel. Set `--speed 1` to retain the complete recording at 1x.

The root must run encoding under its owned, bounded process-tree supervisor. `build_video.py` requires `AIFORMULA_NEO_MEDIA_BOUNDED=1` and `--permit-encoding`. This environment flag is a caller contract, not proof of process containment. The final receipt must additionally include the root's process/birth/cleanup record.

`run_bounded.py` and the pinned `windows_owned_job.py` now provide that Windows supervisor as a static candidate. See `BOUNDED_DRIVER.md` for explicit time bounds, process-birth tokens and invocation. It has not yet been exercised. The v03 display correction and preserved v02 replay instructions are described in `DISPLAY_PROBLEM_03.md`.

For each run:

1. Finalize configuration, events, result and recording files. Do not re-use or overwrite a previous run.
2. Run `make_plan.py --source RUN --name INNER --versions VERSIONS_JSON --policy POLICY_JSON --output-directory NEW_MEDIA_DIR --plan NEW_PLAN_JSON --ffmpeg FFMPEG_EXECUTABLE`. Use OUTER for the other ring. FFmpeg may instead be supplied through `AIFORMULA_FFMPEG` or PATH. The policy JSON is an explicit reviewed binding to configuration fields; see `policy.example.json`.
3. Review the sealed plan, especially source versions, mode checks, final result and 1x windows. `make_plan.py` computes no acceptance verdict.
4. Under the root's bounded supervisor: `build_video.py --plan NEW_PLAN_JSON --permit-encoding`.
5. Inspect decoded event frames plus the contact sheet at readable size. Add a separate visual-QA receipt; don't rewrite the original machine receipt.

Machine checks include before/after hashes of every input, H.264 / 1280 x 960 / yuv420p / 15 fps, a complete `ffmpeg -xerror` decode, every decoded frame index and exact rational PTS, and unforced decoded sample images. The report explicitly leaves visual QA pending. Every output frame has a JSONL mapping back to original observer/detection/motion rows, semantic generation, phase, source wall time and playback speed.

The shared renderer is packaged verbatim at `shared/recording_renderer.py`, read-only and pinned at SHA256 `889a952a9c229d167b8b54100290db98def2f5a8cd40ca88137e8f5c7d1e32af`. Its original `draw_semantic`, `draw_trajectory`, distance panels and old case list are not called. Its `draw_lane` receives only copied display state with a reconstructed display-age field; source rows remain immutable. The previous local source file is untouched. A different path may be passed with `--renderer`, but its bytes must match this reviewed dependency. Pillow must be installed in the selected rendering Python environment; no user-specific Python, FFmpeg or renderer path is required by the new entry points.

Long cases may be rendered as separate videos for traffic, closure and restart tests. Their evidence names and case_kind remain visible. Concatenating separate runs must be explicitly labeled; this renderer never pretends a process restart or separate capture was continuous.
