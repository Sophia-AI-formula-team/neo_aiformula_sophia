# Measured positive-fixture failure and bounded support repair

## Before change: actual single-fixture result

Root executed the gated runner03 once (approximately 0.687 seconds overall).
The original `official_observer_quantized_curve/positive_fixture_diagnosis_01`
contains the unmodified review2 fixture and its full diagnostic geometry. Its
summary reports one fixture and one observer call, with source hashes unchanged.
Closest pair 1 fails **only** `quantized_parallelism`:

- Worst estimated disagreement plus quantization bounds: 10.43327853920782 deg,
  against the unchanged 10-degree criterion.
- Conservative observed spans: 1.3797261049130782 / 1.355308774774349 m.
- 216 matched sections; width interval union 2.1310168288504263–2.2297255575243207 m.
- Both main sides have 318 actual pixels before and 67 after the side region.

This measured result rejects the earlier conjecture that this fixture failed
because a closed walk reduced its support below one metre. It does not justify
loosening parallelism, separation, span or footprint checks. Review2, its 27-test
failure, runners and single-fixture diagnostic remain untouched.

## Review3: same observed straight support, larger endpoint baseline

The original 64-pixel half-window remains the baseline. Only a sample whose
baseline directional quantization bound exceeds half the existing pair-angle
criterion (or whose direction is unresolved) may try 128 then 256 pixels. A
maximum-window setting accepts only 64, 128 or 256; no arbitrary fixture-specific
value is introduced. A global support-inspection budget fails closed.

Each larger window must remain within one original arm and must add actual
observed endpoint support. Every adjacent UV step must be a distinct 8-neighbour
step; no absent pixel or projection gap is bridged. All observed pixel centres
must lie in the straight strip intersected by their half-pixel square cells,
around the candidate endpoint chord. The exact allowed perpendicular distance
is `half_pixel * (abs(normal_u) + abs(normal_v))`. The entire candidate window
must also retain the original chord/arclength ratio >=.85. A gap, turnaround or
non-straight strip rejects expansion and stops trying larger windows; the code
does not skip a bend to recover later support.

Use the first larger window whose original endpoint-box/chord angle calculation
meets the half-angle target. If none meets it, retain the original baseline
estimate/uncertainty and let the unchanged pair gates reject it. There is no
partial improvement accepted merely to rescue a fixture. All geometric support
remains estimated under the same uncertain local-plane model, never a junction.

Record baseline descriptors, attempted/used windows, actual endpoint IDs and
inclusive indices into the arm's retained raw pixel-ID list, strip residual and
quantization bound, and rejection reasons. Shared raw-ID slices avoid duplicating
large pixel arrays; they identify actual observations, not new fitted pixels.

## Prepared tests and limits

Copy all 27 original cases unchanged into this independent directory, preserving
the positive assertion and the review2 negative/boundary cases. Add a small set
covering actual extension/provenance, bend rejection, missing-pixel rejection,
insufficient truncated support and capacity/window guards. No threshold changes,
resampling, historical support, control/core changes, official priors, model or
Unity activity is included. This candidate and its tests are prepared statically;
root owns subsequent bounded validation. A passing synthetic result would still
not establish real-mask junction observability.
