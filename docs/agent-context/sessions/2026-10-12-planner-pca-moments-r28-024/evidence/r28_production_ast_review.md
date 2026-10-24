# Native moments production integration — static receipt

This is a read-only source/AST/hash review. No pytest, algorithm import,
extraction, engine, ROS or Unity call was executed by this review. R/T have no
writes from this reviewer. The excluded curve-fit prototype pair was not read.

## Findings

No static blocking discrepancy was found in the requested integration.

1. `local_moments.py::_moments` has exactly the same argument AST and ordered
   statement AST as the tested D `patch_moments.py::moments`. Only the function
   name/module wrapper differ. Eligibility, positive matching counts, shape and
   canonical-byte limits, finite bounds, FP-policy guard, native call, original
   three-expression fallback and error flow have not changed.
2. Production `test_paint_local_moments.py` preserves the entire `products`,
   `original_np`, `Spy`, `outcome` and `MomentsContract` AST after renaming only
   `HELPER.moments` to `HELPER._moments`. All 29 methods and assertions are
   unchanged. The new test-module import binding is explicitly separate from
   that body equality: `LANE_REQUIRE_NATIVE=1` makes missing native an error;
   otherwise the module intentionally uses `pytest.importorskip`.
3. The complete normalized `paint_geometry.py` text equals the sealed original
   plus exactly one import and the unique three-line replacement. Import is at
   line 17; the sole call is at lines 1776–1777. Counts remain `counts[enough]`;
   covariance is still passed directly to the unchanged `np.linalg.eigh` and
   original point-count/PCA/thickness/sign/geometry gates. No other source text
   differs in this file.
4. The new module's lines 10–16 catch only native `ImportError`, and bind only
   `LOCAL_MOMENTS_API_VERSION == 1`. Missing/mismatched optional native selects
   the original NumPy path. Exceptions from an admitted native call still
   propagate. This is optional acceleration, not a new geometry model.

The trusted return-shape assumption is unchanged: reshaping native bytes with
`-1` is not a general validation of malicious native return row counts. The
verified C/binary identity is therefore material. This review does not call a
missing/mismatched native simulation or prove the fresh production binding at
runtime; the proposed focused plan contains that small assertion separately.

## Prior execution independently read back (not rerun)

The immutable `r28_native_moments_01/run_01/result.json` records 29 tests passed,
0 failures/errors/skips; all 22 original-mask comparisons exact; 124 real
`local_moments` calls during candidate extraction. Every case compared the
current 29 dataclass fields (28 arrays plus source stamp) and five derived
arrays. All 44 NPZ file hashes were reread and matched the saved result. The
archived 11 original source hashes match that run's before-bindings; source
before/after equality and failure=null are recorded.

- Result SHA: `35756a1c43d1edd609880b7570c6e3dc24d59890d2cc99a7a0dd00bfc7f5a5a8`
- Actual test elapsed in that record: 0.027586499985773116 s (diagnostic only).
- Root separately reported whole-process exit 0 / 2.8385 s. This review did not
  remeasure the process. Neither time is an 80 ms engine/runtime acceptance.

## Reviewed production identities

- `local_moments.py`: `bf667e3a42e49afb14c00e26447c697403221c42ac3b41b60ec2da22febfa32f`
- `paint_geometry.py`: `0d41317e52643cfe9606976d801ec949ecd8d3d9ca7fbe456258c8b7ad4e5180`
- `test_paint_local_moments.py`: `9dbc3751f1a320b732dcd1b6451b174fdb03ab03988aab934705ce9dc4b85203`
- Native C: `9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f`
- Windows native9 binary: `f2f9712754f8cecaae379af5355c8aa82b3a28855fa2da2d0ce949ddf339ef45`

`REVIEW_01.json` contains the individual AST checks, all 44 output hash checks,
source/test hashes and explicit proposed test allowlist. Its SHA at creation is
`f576ff410c7914181461af8649ecea5778a189cea67bf9f90bda32949550e9c5`.

## Mock smoke boundary and remaining work

The exact eight-test draft is `r26_ci_cause_01/test_smoke_diagnostics_draft.py`,
SHA `8706eba1c492a2d3fcc69596e8d01d90565e03b90b0d49847a85b07bcfc43329`.
It imports the reviewed smoke file, whose top-level imports are standard library;
it never invokes `run_case`. Its fake clock, SimpleNamespace node/engine fields,
and temporary JSONL files test only failure-diagnostic semantics. This is not a
ROS/DDS execution, five committed camera frames, engine performance or safety
acceptance. A later published G test must be newly hashed after root adaptation.

Custom replay/source loaders must include the new reachable `local_moments.py`
before being reused. No loader was changed here. Full regression, recorded
40/80 ms budgets, Foxy/DDS, Unity laps and vehicle behavior remain separate
acceptance steps. The rejected closure candidate is not used or rehabilitated.
