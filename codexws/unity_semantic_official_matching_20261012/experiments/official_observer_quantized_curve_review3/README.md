# Review3: bounded adaptive use of measured straight-pixel support

Static candidate only. No import, AST, test, image computation, model or Unity
execution occurred during preparation. See
`POSITIVE_FIXTURE_MEASURED_FAILURE_AND_REPAIR.md` for the actual prior failure and
the narrowly scoped repair. Review2 and all original failed receipts are intact.

API unchanged: `observe_lane_geometry(mask, camera, *, enabled=False, parameters=None)`.
The original 64-pixel half-window remains baseline. Candidate windows are only
128/256 pixels, within the same contiguous observed arm, and require all raw
support to satisfy the quantization strip and original chord/arc criterion.
The first candidate with directional quantization <= half the unchanged pair
angle limit may be used. Otherwise the exact original descriptor is retained.

New sample provenance: `baseline_tangent`, `baseline_half_window_px`,
`used_half_window_px`, `tangent_support_order_index_range` (inclusive slice of
the parent's real `ordered_pixel_ids`), `adaptive_tangent_used`,
`adaptive_attempts`, and accepted `straight_strip_evidence`. No observed point,
support gap, physical span or control target is synthesized. Work counters and a
global support-check budget are explicit.

The 27 original test methods are copied byte-for-byte, including the previously
failed positive assertion. `test_adaptive_tangent.py` adds six tests. All 33 are
prepared and unexecuted for this candidate; the previous 26/27 result describes
review2 only. Later verification must use a fresh process in this directory to
avoid the same-named older observer module.

Original K/R/t ray helper bytes are unchanged. Pair angle/width/span/footprint,
main continuation, duplicate and competition criteria remain unchanged. No CAD,
official ID, historical cloud, semantic/core or vehicle-control changes occur.
`junction_recognized=false` and unknown total model error still prohibit an
official-map anchor or driving claim, even if a geometric hypothesis passes.
