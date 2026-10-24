# R12 quality failures and evidence limits

Draft summary of completed local artifacts, not a new estimator run. Names
below are relative logical names under controlled diagnostics; raw files are
not included here. The root planner reviewed this minimized summary before sealing.

## Short recorded accuracy

Source: r12_recorded_accuracy_01/accuracy_01.json, SHA256
89c6811fd13b697486d747cc63d1c0a8642fb8a440a2ce6b1a8f6fda52094e55.
The common committed sequence is151–247 of the R10A-driven input, with only a
fixed first-TEACH alignment. No later fit/scale/interpolation/extrapolation.
Truth is read after completed output identity/binding checks, not supplied to
the estimator. This is post-hoc development evidence, not blind validation;
it applies to replay04's source, not automatically to later optimizations.

| Method | Lateral RMSE m | Yaw RMSE rad | Final lateral m |
| --- | ---: | ---: | ---: |
| Raw wheel/gyro | 0.004596479 | 0.000646559 | -0.005090501 |
| R10A | 0.220024830 | 0.019233853 | +0.461696130 |
| R11 | 0.063455734 | 0.007308036 | +0.157786389 |
| R12 graph v2 | 0.006386860 | 0.000940494 | +0.014304561 |

R12 reduces added visual drift relative to R10A/R11, but remains worse than raw.
Longitudinal RMSE is about0.28456m for all methods. R12 position RMSE0.284631732m
versus raw0.284596967m does not prove localization improvement. Rejected poses
returned as raw are excluded, not counted as successful corrections.

## Four predeclared development cases: 0/4 quality pass

Aggregate r12_quality_validation_01/r12_development_01.json, SHA256
75b0c491fa42329f88c81238b5583acc758de939403c0eeec91e23cb7db8bfb2.
Independent full-mask circle generator: radius16m, speed2m/s,509 planned frames,
left/right and nominal pitch0/+0.35deg. Empty-map initialization, causal mask/raw
motion only. No restart or gate change. Exit0 means failures were saved, not
that quality passed.

| Cases | Accepted/planned | First rejected frame | Reason | Overlap |
| --- | ---: | ---: | --- | ---: |
| Left0 and left+0.35 | 7/509 each | 7 | match_insufficient_overlap | 0.487603306 |
| Right0 and right+0.35 | 7/509 each | 7 | match_insufficient_overlap | 0.512195122 |

All stop before the pitch ramp while up=[0,0,1], so tilted tracking was not
successfully tested. Original diagnostic budgets were2000ms, refresh0.25s/0.25m;
not runtime timing evidence. Unchanged quality criteria: accepted/planned>=0.95,
final frame accepted, lateralRMSE<=0.10m,yawRMSE<=0.01rad. Eight reused holdout-
family regressions were not run. Wrapper10/10 and self-test passed, not quality.
Loaded binary identity matched the independently exercised seven-API module.
Paint source here was bf14ef4c8e129e89aade7036983d1051bdbf73a118280a047eaf5f8b693493af.

## Freshest past map still rejects

Source r12_stale_reference_audit_01/result_01.json, SHA256
fdf070870aeca3f9a0d4a098a8e8076f092da41f9fe6104ee2f855194d477a38.
One bounded frame0–7 reconstruction matched baseline non-timing outputs exactly.
At left frame7, active covered mass29.5/60.5; freshest committed-past reference
covered32.0/60.5=0.528925620, still below unchanged0.55. Current frame7 was not
committed or used as reference. Faster refresh alone is not an established fix.

The complete JSON was saved before a final stdout formatting error (flush
passed to json.dumps) caused exit1. That failure is preserved; the estimator was
not rerun to fabricate a successful exit. This is diagnostic, not runtime proof.

## Geometry-loss decomposition, not a gate proposal

Source r12_stale_reference_audit_01/gaps_01.json, SHA256
3e024db445eccabaf2781238c009c76b202a3196c1248a560f802687c979ae6e.
One extraction/geometry audit, exit0, no match/pose solver/map commit. All103
representative positions/normals/weights are byte-identical to captured query.
Complete foreground denominator remains60.5.

Primary, non-double-counted uncertified mass10.5:

- 7.0 invalid direction;6.5 lies on six-face closed raster components whose
  seed-relative compatible paths have at most4–5 faces, below existing6.
- 2.0 curve roots outside the actually observed white/black bracket.
- 1.0 raw J above existing reliable0.1 gate.
- 0.5 model normal incompatible with original foreground cross-section.

All27 old-reference finite-support rejections (mass13.25) select true terminal
nodes and lie outside finite chords. None is an internal bad-normal joint,
branch/subset-degree error or numerical two-end ambiguity. Eleven rows/mass5.25
are within1cm of some historical raw face; these faces are audit evidence, not
position truth or an alternative matching model. Other rejections have genuine
gaps. Descriptive distances do not authorize padding, unknown topology merging,
lower votes/overlap, root clamping or a current/future reference.

## Runtime integration is still incomplete

Source-only r12_runtime_entry_audit_01/REPORT.md confirms _gnss worker consumes
full boundary masks in teach/repeat; ordinary reference/fixed launches retain
CommonGroup GPS/INS recorder/follower and legacy bundles, not equivalent R12
consumers. Reference needs LYA safety fallback; fixed needs managed-LYA stop;
_gnss adds endpoint policy. One executable cannot replace all three unchanged.

Raw tilt is opt-in; hardware specific-force semantics remain unverified. ROS
TEACH rejection logs rather than sharing Simulator automatic HOLD. Projection/
extraction precedes the engine stopwatch. These require explicit integration
work. No ROS, hardware or new actual lap was performed for this summary.
