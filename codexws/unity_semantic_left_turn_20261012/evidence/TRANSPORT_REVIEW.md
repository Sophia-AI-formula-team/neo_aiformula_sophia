# Calibrated simulator transport candidate

Only staged `calibrated_transport.py` and `semantic_worker.py` are changed by
this part of the work. The exact installed worker is preserved as
`semantic_worker.before.py.txt` (SHA-256
`929c8f533aaf03dd58dbdf841110274f8c7feeaa92cdd284033b26ccc1121da9`).
The semantic core, native LLP/LYA, Unity physics and installed Assets are not
modified here. This remains the existing simulator in-process ROS shim, not
a native rclpy/DDS deployment.

## Opt-in and lifecycle

`--calibrated-map FILE` explicitly enables the topology loader and continuous
detour tracker. Both CLI options default to None. Without this option no new
helper is imported, no builder/MapAssist method/tracker is replaced, and the
existing tick, ready, perception, watchdog and diagnostic methods remain
unchanged. `--recorded-reference FILE` without a calibrated map is rejected.

One module builder wrapper is installed before the existing optional diagnostic
snapshot hook. Attaching a new worker chain does not stack wrappers. Each real
new epoch gets a fresh MapAssist and adapter state. The original builder runs
first, once, with the actual captured samples and options. Its exception still
propagates. Calibration cannot replace a failed builder with synthetic points.

Only a validated calibration returns a new FusionResult. Its reference is the
exact same object returned by the original builder; trust, points, cumulative
progress and length are not edited. The complete original automatic result is
retained in adapter memory. Published `original_auto_result` retains the entire
original status/alignment/node results and a reference SHA, point/trust counts
and length, avoiding repeated thousands-of-points I/O at 20 Hz. An unavailable
automatic result remains unavailable in that nested diagnostic.

MapAssist's instance topology preparation uses the validated explicit graph;
it never constructs OfficialToOdom or fabricates similarity parameters. Actual
official outer IDs must match the configuration exactly. Core reset clears its
reference, which makes the adapter identity check fail closed; reset does not
silently reseed. Old nodes cannot obtain a graph from the next chain.

## Status and sources

Fusion status `mode` is always `calibrated_map` on calibrated success, matching
the staged C# command-owner contract. `reference_mode` is `real_first_lap` or
`warm_start`. `metric_source=explicit_simulation_calibration` and
`automatic_estimate=false` distinguish this experimental prior from automatic
fusion. The alignment object contains only mode, direction and that explicit
automatic-estimate flag. No successful similarity is claimed.

`configuration_file_sha256` fingerprints the exact input JSON bytes;
`configuration_canonical_sha256` fingerprints the private JSON object passed to
the topology loader. The loader's existing `configuration_sha256` is therefore
the same canonical-object hash, not the input-file hash.

## Recorded-reference validation

Warm startup requires one error-free `semantic_route_diagnostics/v1` snapshot,
an exact successful builder capture, unchanged source hash/tree/file manifest
and actual bytes on disk, the actual selected OSM in that source tree, the same
official route and all fusion keyword options. Duplicate JSON keys/nonfinite
numbers and ambiguous multi-snapshot files are rejected.

The captured first-lap state must have no previous reference, at least 30
candidate samples and at most max_recorded_samples+1, a recorded count exactly
one less than the candidate count, and the configured minimum travel. Original
closure parameters must match. The last actual sample must not be excluded,
must equal captured current odometry, and must satisfy the original first/last
closure distance and wrapped heading criteria.

The original builder is recomputed from those captured RouteSamples before
worker ready. The whole original result (including every point, trust flag,
progress, status and alignment) must match the recorded original result; only
numeric roundoff of abs 1e-9 / relative 1e-12 is tolerated. No deserialized
reference is trusted in place of recomputation. Calibration is also checked
before ready, so this work does not run as a fit on the first live camera tick.

The validated reference is applied only when the actual first epoch creates its
chain. It starts at progress 0 and matched false. Samples, odometry, timestamps,
traffic votes, ABC and controller commands remain unseeded. Original freshness,
ownership, traffic and velocity gates still apply. Status includes
`fresh_learning_proof=false`, recorded snapshot SHA, source tree SHA and
recomputation evidence. It is explicitly a reused reference, not a new learning
acceptance or proof of a completed detour.

## Verification boundary

`test_calibrated_transport.py` contains lifecycle, reference identity, original
failure, warm-seed, corrupt capture/source/result, closure and disabled-control
body parity tests using actual core reference classes with explicit fixture
nodes. These tests do not simulate ROS, a vehicle, neural inference or actual
first-lap learning. Test execution and any real recorded fusion replay are
separately authorized by the root agent; installation and Unity runs remain
pending its review.

Executed once after the CPU handback: 16 unit tests PASS; independent before
and candidate default workers each processed three real-clock protocol ticks.
Every output field matched except the explicitly excluded monotonic receipt
ages (odom_age_sec, point_age_sec, last_message_age_sec, recovery_age_sec).
Neither default worker imported any calibrated helper. Python 3.8 syntax checks
passed. The complete verification runner took 0.943 seconds, exit 0, and all
recorded core/helper/worker/test/config/snapshot hashes were unchanged.
See `calibrated_transport_receipt.json` for full stdout/stderr and hashes.
The actual outer recorded-reference full fusion recomputation is deliberately
not included in this receipt: the root agent owns that separate one-time probe.
