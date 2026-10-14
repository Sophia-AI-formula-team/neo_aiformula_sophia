# Critical checkpoint analysis — 2026-10-15, before production candidate fix

## Genuine failure evidence

The original neo 0.3.0 code in `runs/outer_closure_02` completed a genuine Unity
lap: 1028 recorded samples, 285.62462532386445 m accumulated recording, reference
length 286.99153447035962 m, closure `complete`, and all closure checks true.
Distance to the starting sample was 1.38930173397832 m, within the unchanged
1.5 m radius; heading error was 1.1658106875102796 degrees, within 25 degrees.

The closure boundary checkpoint succeeded with `progress=0.0`. The final orderly
worker exit returned code 0 without a forced kill, but its shutdown acknowledgement
reported `checkpoint_ok=false`, `persistence.state=save_failed`, and
`error=negative_progress`. The run correctly remains `accepted=false` and retains
its cleanup error. An orderly process exit is not a successful final checkpoint.

The last persisted file is the successful closure-boundary snapshot; the final
negative progress value was not captured in the terminal trace. This report does
not invent its magnitude or rewrite the original failure.

Evidence SHA256:

- result.json: 64e4a4b4b9dc83d43fb69a3fe272fa38c253d99fe381b5b17a7993e172359935
- terminal_sample.json: 3996a44b2ecd13f017ae4e484a30b2dc53bd356708c900d503f8d7a9b77159d1
- checkpoints/outer_closure_02/topo.json: 86ce80a242a9601b1198dc628551c6feda49d850ebdc57c2e261c07e68353e78

Official map fusion is a separate failed quality gate: RMSE 3.453969157614023 m
exceeds the unchanged 3 m limit. The odometric lap is closed but official map
fusion is unavailable. This fix neither alters nor relaxes that result.

## Root cause in production source

`RouteReference.match` explicitly returns **unwrapped progress across the lap
seam**. A local search around zero includes negative along-route coordinates;
`point_at` intentionally wraps signed coordinates modulo route length. Therefore
negative progress immediately before the reference origin is a valid state.

MapAssist closes a lap as soon as its unchanged radius/heading checks pass,
initializes `self.progress=0.0`, then assigns the first subsequent local match
directly to progress. Closure can occur before physically crossing the first
sample, so a signed negative match near that seam is expected behavior. No
geometry or navigation defect is demonstrated by the negative sign alone.

The new persistence validator applies the same nonnegative rule to
`recorded_distance` and `progress`. This confuses accumulated travel distance
with a signed local route coordinate. `TopoState._checkpoint_state` faithfully
serializes the legitimate match, then `validate_state` rejects it. The same
production validator is used by the Windows simulation store, so this issue is
not caused by the Windows file durability adaptation.

## Minimum proposed production correction

Change only the negative-number check in `topo_checkpoint.validate_state`:

- recorded_distance remains finite and nonnegative in every state;
- progress remains finite and numeric (booleans/NaN/infinity rejected);
- a finite negative progress is accepted only when `closed_at` is present;
- an unclosed map still rejects negative progress, exactly as before;
- existing closed-map minimum sample count, pose validation, checksum, schema,
  configuration fingerprint, file bounds and restore continuity remain intact.

Do not clamp progress to zero, take modulo, change the recorded route, change
the lap closure conditions, relax map fusion, or change LYA/control. The schema
stays version 1 because the same numeric field now accepts the signed state
already emitted by the production algorithm. A runtime patch release should be
recorded by root (suggested MapAssist 0.3.1), preserving 0.3.0 failure evidence.

## Planned regressions and remaining acceptance

1. Original source reproduction: construct a closed reference and use its real
   match algorithm to produce a negative seam coordinate; original validator
   must reproduce `negative_progress`. This is a synthetic logic reproduction,
   not a substitute for the genuine lap above.
2. Candidate validator/disk roundtrip preserves that signed value exactly; still
   rejects negative travel distance, negative unclosed progress, malformed poses,
   too-short closed maps, boolean/nonfinite progress and checksum corruption.
3. Production MapAssist callbacks create a closed synthetic route, match before
   its seam without assigning a negative value manually, save, restart under
   continuous odom, and restore the same signed progress/reference geometry.
4. Run the existing checkpoint/MapAssist regressions and report any host shim
   needed for Windows. Such tests are not native DDS or POSIX fsync acceptance.
5. Root must deploy the pinned fix through the guarded installer and rerun the
   actual inner/outer closure + semantic-only restart cases. Preserve failed
   videos and clearly separate closure, fusion and restart verdicts.

No live, repository or semantic source has been edited at this report's creation.

## Subsequent reproduction and candidate verification

The original validator was reproduced in a separate Python 3.13 process using
the real RouteReference matcher: the synthetic closed reference returned
`-0.30000000000000193`, and unchanged 0.3.0 validation rejected it with
`negative_progress`. This measured synthetic value is not the unrecorded final
value from the Unity failure. Receipt: checkpoint_fix_01/original_receipt.json.

The staged one-condition validator fix then passed **118 named checkpoint and
MapAssist tests**, including all existing tests in those two modules plus three
new methods, with zero skipped/failure/error. The new callback test obtains its
negative progress through actual _on_odom and reference.match, saves, restarts,
and preserves signed progress, reference points and trusted flags. The new
format tests roundtrip the signed value and preserve the prior rejection rules.
Receipt: checkpoint_fix_01/candidate_receipt.json; elapsed 6.495437 seconds.

On Windows these native tests require an explicit test-only platform boundary:
real one-byte Windows locking and file fsync, while POSIX directory open/fsync
is a documented no-op. Neither this nor the synthetic callbacks is a claim of
POSIX durability, native DDS, physical vehicle or new Unity recovery acceptance.
The running simulation storage adapter remains separately identified.

All pinned source hashes were unchanged during both processes. Parent 24256 and
children 22884/10408 exited and were independently absent; no forced kill. CPU
was returned to root. No repo/live source was changed by this subtask. Candidate
source is frozen at topo_checkpoint SHA256
`42fc792b615f80418782b6826a5693edac210f81de0a7b6f7d8acdb907353a8e`.

The JSON schema number remains 1. An older unpatched 0.3.0 reader will still
reject a saved negative-seam value; preserve checkpoint backups when rolling
back. Do not silently clamp a checkpoint to make an older reader accept it.
