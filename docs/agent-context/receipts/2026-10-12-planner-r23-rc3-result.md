# R23 RC3 results: native integration passes, runtime still fails

Planner, input neo-004; evidence acceptance only, no driving acceptance.
Exact code: 764d860e64c5fe726fa03346f2142571fe1b2a99.
Tag: lane-mapping-20261012-r23-rc3, annotated object
89e63946ede9c629849168e174e2d7e99d632665.
Repository ID 1303517209. All listed CI runs are original automatic push runs,
attempt 1; no retry/cancel or lowered time/geometry gate.

## CI

6/8 workflows, 14/16 jobs succeeded. Exact branch/tag run pairs:

- Native: 38157404988 / 38157405144, Windows and Ubuntu pass.
- Handoff: 38157404996 / 38157404963, all four jobs each pass.
- Lane Foxy: 38157404966 / 38157404965, pass.
- Endpoint Foxy: 38157404995 / 38157405194, DDS failure after build/unit success.

Native jobs: 646 tests and 954 subtests pass, all nine actual API checks pass.
Lane Foxy: 3136 unit tests, 4 Header checks, 63 synthetic DDS checks and 26
real-upstream checks pass. Endpoint: five packages build; installed native9
verification and 4068 unit tests pass with zero skips; Header 4 pass.

Endpoint default_guard and startup_gap pass. The other seven scenarios time out
at runtime_smoke.py:300 waiting for five actual stationary image commits.
There were 145/133 teach_mask results, zero accepted; this is not zero input.
Branch: 100 processing deadlines and 45 bootstrap-not-established. Tag: 93
deadlines, 3 commit-guard rejections and 37 bootstrap-not-established. Worker
compute spans 226.660–450.276 ms. Journal wall time minus image source time is
261.901–550.679 ms, not a separately measured transport or receipt age.
Later scenario-specific assertions were not reached. No physical stop tested.
Original DDS JSON SHA256:
branch 4a0b1e855e26bf04e1d23b171447ab4555b92255053ce6b6b9ef735c52e8ae96;
tag 264c88336f734d3fc93557cab890133975d12f4cbac32a7790608a68b0ea7f3f.

## Windows actual native installation and regression

RC2 branch native run 38157245092, Windows job 114522650026, artifact 11709011134
provided the identical production native9 source used by RC3. The downloaded
binary is 43008 bytes, SHA256
f2f9712754f8cecaae379af5355c8aa82b3a28855fa2da2d0ce949ddf339ef45.
C SHA256 9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f.
Downloaded and installed actual calls and installed prefix were checked.
The previous binary was copied and verified before replacing that exact generated
file; its backup remains local. No dependency installation or compiler-flag change.

One complete three-package regression on RC3: 4067 passed, one Linux-only process
group test skipped, zero errors/failures. Pytest process 127.7186541 s; JUnit suite
125.875 s. All 190 source/binary pins unchanged. The regression independently
verified all nine actual calls and hashes; its verifier did not request a prefix
check, so prefix verification belongs to the preceding install receipt only.
Only unrelated unintegrated curve-fit and separately tested R24 helper tests were
excluded; the unintegrated helpers were not imported into this R23 run.
Summary SHA256 75549b0f8ad2f323517bbe65ec44572a882cb7fe03a46c289febb3cf73e23732.
JUnit SHA256 b029a4920be48669f45236512c56ac0bd30ce28918a2d0d1a613588e8dcaae93.

## Original recorded prefix: still failed

One replay of the same original trial28 111-frame fixture, original receipt/ACK
motion timeline, plane 0.007 m, matcher 40 ms and engine 80 ms. No live Unity
operation. Source/input/native/config integrity checks all pass. Stop at first
failure: 11 accepted of 12 attempted; seq162 processing_deadline at
106.2752000 ms, matcher 34.6718 ms. Failed frame did not commit; 121 confirmed
cells and 11 trace points remain. Candidate overlap 0.8079710 and candidate
residual RMSE 7.3379 to 4.4555 mm are not ground-truth error or a committed pose.
The remaining 99 frames were not processed. Process exit 0 means the diagnostic
completed, not that the algorithm passed. No rerun was used to select a better
timing sample. R22 timing from a separate run is not a controlled speed benchmark.
Report SHA256 1778bc9b3197779878689701a6f09f84139d205a09f92e3d743d61c2a694694e.
Reviewed manifest SHA256
72624b99033cc95dae710839e2928120c28693a344e207e071bbf08dac897c17.

Local originals remain in r23_native9_install_01, r23_full_regression_01,
r23_observed_01 and r23_native9_ci_01/rc3. No raw images/video/private paths
are published here. No first lap, route freeze, second lap or new video exists.
Vehicle, physics, simulator, camera and LYA parameters are unchanged.
Continue geometry-model repair and measured hotspot work, not threshold changes.
