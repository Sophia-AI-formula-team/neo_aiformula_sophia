# R22 geometry evidence — reviewed checkpoint, not a driving release

These are offline synthetic-mask, saved-observation and exact-arithmetic checks.
They are not a real-bag accuracy result, live Unity run, ROS validation, runtime
budget pass, completed map, or completed lap. No new estimator run was performed
to prepare this document. Original failed attempts remain in local evidence.

## Fixed 22-mask extraction comparison

One fixed comparison used the original analytic masks. Analytic assignments were
used only for posthoc scoring, never as extractor input. Source/input checks
passed; every original raw field and unaffected output stayed byte-identical.

| Assigned model-root metric | Before guard | After guard |
| --- | ---: | ---: |
| Count | 9987 | 9940 |
| Absolute normal error >30 degrees | 8 | 3 |
| Distance RMSE, m | 0.008207076513800177 | 0.008215398400802475 |
| Maximum distance error, m | 0.04379469343283469 | 0.04379469343283469 |
| Absolute normal-error p95, degrees | 3.858392834201884 | 3.8583928342018714 |
| Maximum absolute normal error, degrees | 89.83611645084531 | 60.02334681886688 |

The guard removed **47** assigned roots: **5** had >30-degree error and **42**
did not. Removed-root distance RMSE was 0.00620102442448969 m; maximum distance
error was 0.01000000000000012 m. Thus distance RMSE did not improve; its slight
increase also compares different admitted populations. Retained coordinates
were not adjusted toward analytic truth. Rejecting a root is not recovering it.

The three remaining >30-degree roots are in the same coarse arc case. Their
signed normal errors are 60.02334681886688, -36.428972244850286, and
32.77993711917406 degrees. Its original contour had no selected fitting cut,
so the new cut-endpoint guard did not apply. A result flag referring to the
three previously selected bad endpoints must not be interpreted as all errors
fixed. This is a partial geometric improvement with remaining failures.

Source result: `r22_corner_masks_01/result_01.json`, SHA256
`cf753f8edf4bed416551529b64ace5a60540cf8cbcd183dc3ccafd69c11af4a8`.

## Three saved competing-direction witnesses

One helper-only run tested the three original selected bad endpoint windows,
without new extraction, PCA, fitting, truth input, mapping, or pose estimation.
All three returned `ambiguous`. Independent exact Fraction checks passed for
every original white/black label, strict seed crossing, the original signed
other-side cone, and strict exclusion from the selected signed cone. Source
and input hashes were unchanged. This proves ambiguity for those tested
families; `clear` would not prove a unique quadratic direction.

Source result: `r22_corner_direction_probe_01/run_01/result.json`, SHA256
`3ea0dbe0fa5d04da0533ced840006ff186d7c1f5051fc908c328c903dd146445`.

## Saved causal continuation differential

This was a saved synthetic-camera geometry differential, not a driving replay.
Thirteen original observations were re-extracted; the original 27 arrays and
stamp matched exactly. Only the twelve accepted past frames were committed.
The current failed frame did not vote. Original token payloads, record/edge/
domain updates, and old 18 reference arrays were unchanged; new continuation
metadata was explicit, not presented as the old representation.

For the original **112 queries**, coverage gained **2**, lost **0**. Original
nearest owner, target and normal were byte-identical: no farther-neighbor
fallback. Each topology hypothesis independently required the original 5-frame
and 1-reliable support. An independent Fraction oracle checked 87 exact event
boundaries/interior samples without an unsound sample. Permission was limited
to observed common subintervals, not a whole-owner exemption; unsupported
endpoints and conservative ULP gaps remained rejected.

The check used 0 engine calls, 0 matcher calls and 2 pure graph queries. It did
not recompute poses or establish that the failed frame would pass all engine
gates. The geometry refers to stored binary64 observations, not a physical
pose/calibration uncertainty bound. A later transactional capacity repair is
covered separately by tests; this differential retains its own source hashes.

Source result: `r22_saved_continuation_01/result_01.json`, SHA256
`b812c7300c897d23e006447d30e5fb415c52b54e0d05705a91582ef6b8e367f5`.

No raw images, sensor sequences, GNSS values, private paths, host identity or
raw record IDs are included in this publication.
