# Recorded closure success followed by checkpoint failure needs a separate caption

Observed in completed `outer_closure_02` before any display edit:

- `result.accepted=false`.
- `result.firstLapClosed=true`.
- Original `result.reason="Production topo closure complete"`.
- `finalShutdownReceipt.clean=false` and `acknowledgement.checkpoint_ok=false`.
- Checkpoint acknowledgement has `persistence.state="save_failed"`, `persistence.error="negative_progress"`.
- `cleanupErrors` records that semantic shutdown/checkpoint close was not confirmed clean.

The v02 renderer's terminal caption showed accepted and original reason only. That could obscure why the whole case failed even though the topology closed. Neither the original reason nor any recorded result will be changed.

Minimal v03 display correction: keep the exact reason, show `firstLapClosed` independently, and add a third lower-panel caption with the recorded checkpoint state/error and cleanup errors whenever cleanup/checkpoint failure exists. It remains below row 720 and preserves the entire observer image. No planner, perception, control, source event or acceptance logic changes.

The previous v02 scripts and pinned shared renderer are retained byte-for-byte under `frozen_v02/`; existing plan01 files stay unchanged. New plans receive unique plan02 directories and the new builder SHA. The v02 plan remains reproducible using the archived builder as `--builder frozen_v02/build_video.py` in the bounded driver.

This is a display-only correction. It does not fix the recorded `negative_progress` failure and does not apply later planner source versions to the old run.
