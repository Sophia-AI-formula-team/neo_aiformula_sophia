# Final 60-second media evidence

The final single video is **60.000 s, 1920×1080, H.264/yuv420p, 15 FPS, 900 frames**. Inner07 occupies 0–30 s and outer03 30–60 s. Each actual run completed two physical laps and red-stop/stationary-green recovery, but official topology and the red interior turn failed; both accepted=false results remain visible. One bounded encoding and complete 900-frame decoding passed, with verified Job cleanup and unchanged source inputs. Actual decoded-frame visual QA is recorded separately from the immutable machine receipt's historical visual-QA-pending field.

Read [VIDEO_QA.md](final_panel_only/VIDEO_QA.md), [media_manifest.json](final_panel_only/media_manifest.json), [event_time_index.json](final_panel_only/event_time_index.json) and the original [25-file inventory](final_panel_only/archive_files.json). `encoded_sources_v1/` contains the exact renderer, wrapper/helper and documentation used. Their historic preparation notes are preserved byte-for-byte; this index and VIDEO_QA describe the final completed state.

The repository intentionally contains only the 25 text artifacts and their inventory: exact sealed plans, source code, Job/decode/QA receipts, source hash tables and 900-row output/decode journals. No video, decoded images or raw simulation logs are copied. The final binary remains at:

`E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116/media/final_panel_only/encode_v1/render/semantic_joint.mp4`

SHA256: `815285ea9da67903df8fc92d67deafb8845aff9159670c8c8a1c2d49a9994c9b` (8,343,024 bytes).

This is a reproducibility index, not a self-contained replay. Sealed plans retain the actual original run/image paths, installed Python313/PyAV/Pillow library paths/hashes and evaluation-surface dependency. The full sources must remain present and unchanged. Use the exact code and original combined plan with a new attempt directory under the original task media root; do not edit old plans or relabel hashes to bypass checks. A migrated environment requires an explicitly new provenance plan. No compact replacements of the final plans or receipts are used here.

Full source time ranges are retained with variable speed: startup is shown honestly, each recorded red-wait window remains approximately 12 s at 1×, and long driving segments are compressed. The per-frame journals record sample-and-hold source timing and asynchronous image/perception/semantic ages. The video cannot expose every original frame or prove instantaneous control robustness. The calibrated-panel policy and raw model overlay are distinguished, and the unaligned official graph is shown separately from the learned odometry path.
