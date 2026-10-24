# Paired primary-curve fit — independent static review

Prepared only. The helper and58 independent test methods were read in full, not executed. No native/Python/AST/test/profile/Unity operation, no main-helper edit, no extractor activation, no threshold/vehicle/simulator change. This reviewer created only the new package test and this report; root owns the helper and execution.

## Exact current source

- paired_curve_fit.py:15107 bytes, SHA d367d48e4b0e9afbd5c6fe7e18f349211ff7906dcc74b0a8d4706c2ea05a8679.
- footprint_geometry.py: SHA eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17; all four imported interval primitives and their arithmetic assumptions were read directly.
- test/test_paired_curve_fit.py:34581 bytes, SHA15333c87ba50ace1f2f0ecd130bfdd75e26e266b252d600af3b84261179092be; class PairedCurveProposalTests,58 distinct test_* methods, zero execution result.
- Checkout HEAD9b8636fc3c1aa9e9712142b3924a722211df19bd; helper/test remain uncommitted preparation, not part of that published tested R29 SHA.

Earlier helper65e930…/c7f427…/c315dc… revisions were unexecuted preparation. Current d367 includes per-side counters/failure reasons, append-before-side-call partial logging and NumPy binary64 adjacency for Foxy/Python3.8. They are not interchangeable tested identities. The new paired_curve_joint module and excluded prototype pair were not read/imported/hashed/tested; joint units are a separate root task, not this gate.

## Static soundness findings

No nominal-all-WB false-positive path was identified under the documented ordinary IEEE binary64 NumPy assumptions and the public propose_pair validation. This is not a formal global proof or a claim the new tests passed.

Public ingestion copies finite shape(2) origins/normals/bounds, positive scalar scale, nonzero unchanged normal, exactly two groups and6..512 matching centre/white/black occurrences. Complex/bool/object/too-wide floating geometry is rejected; integer-to-binary64 rounding is checked rather than accepted. Manually constructed malformed PrimaryPatch records are reconstructed/validated at public ingress. Inputs are copied; the returned report is nevertheless a mutable dictionary, not an immutable authenticated map artifact.

OriginalP is the closed half-face parallelogram j +u*a+v*b with u/v in[-1/2,1/2]. Plane orientation uses determinant sign, including negative axis orientation. Both original finite chart slabs are intersected exactly. The bounded nonempty polygon/segment/point supplies a vertex average and four centre/face averages; exactly five declared slots are retained, even when duplicate in a degenerate closed domain. Zero determinant is refused, not repaired; no initializer returns UNKNOWN, not a physical/global no. Rounded latent coordinates are not substituted into that exact-domain proof.

At fixed q, the two free coefficients use float nominal-world centre LS, NOT the original floating Gram/RHS computation. White=-1 and black=+1 produce the correct constraints sign*(B*free) <=sign*(y-yq). Every supplied occurrence enters centres/labels, including duplicates and off-chart support; no finite-extent label clipping, deduplication, radius/normal/scale/bounds change is present.

The float2D active-set routine is only a bounded proposal. Feasibility/dual comparisons are rounded and can yield UNKNOWN due rank, cycling, an infeasible restricted pool or budget; they are not global infeasibility/optimality certificates. A Gram/constraint overflow or NaN scan must never be interpreted as a proof. Any returned beta packet must pass the separate label check, so a favourable float solver outcome alone cannot admit it.

The label checker independently evaluates outward intervals of the ORIGINAL supplied nominal chart/polynomial, not the solver-expanded rows. Positive scale preserves division endpoint order; white negation reverses endpoints. Any definitely-negative interval is refused; nonfinite or uncertain lower endpoints use exact Fraction substitution of the actual rounded coefficient values and supplied binary64 coordinates. Closed zero signs are permitted without introducing an epsilon. Off-extent occurrences are intentionally checked as polynomial-extension labels; this is not a claim that those points lie on the finite exported curve or that the original rounded fit program has those signs. Overflow intervals force exact checking/UNKNOWN, not favourable clamping.

Coefficient adjacency explores27 binary64 packets using np.nextafter, not the unavailable Python3.8 math API. It checks all labels and finite float cost, but the shared anchor relation can separate after rounding. joint_certified remains False and shared_q_after_rounding explicitly unverified. WB signs, complete search or an LS score NEVER implies a rounded finite joint, correct side ownership, angle/J validity, a ready map or driving acceptance.

## Concrete integration blockers and limits

1. Actual primary owner/role/support provenance is absent from this numeric API. The caller must authenticate model fields and every original occurrence against the same actual final primary producer, and original cut/junction/axes. Ribbon12-label versus centre6 mismatch is refused, but an equal-count6 numeric ribbon-like payload is not distinguishable by counts or PrimaryPatch class alone. Do not fall back to an old primary, relabel a ribbon or infer authentication from successful beta signs.
2. Original rounded-curve joint still requires a separate independent certificate inside the actual originalP and both finite charts, plus all existing root/angle/J/ownership/geometry/motion gates. This review/test does not read or endorse the separate joint certifier and cannot authorize integration.
3. Pure nominal-world label signs differ from the original rounded fit-coordinate program and physical/camera calibration. The explicit objective/label_contract strings are accurate; these tests do not promote those contracts into physical proof.
4. public status completed_proposal_diagnostic means the five searches completed, even if ALL beta values are None. Inspect per-side reasons and both_beta64_labels_certified; unsuccessful search remains UNKNOWN/no-veto. No support deletion or widened gates may follow.
5. Exceptions may leave prior complete diagnostic packets plus a partial current q. Append-before-side-call retains coefficients[None,None] and counters; the current pair's both flag staysFalse until both side calls finish. Overall statusUNKNOWN and any partial packet cannot be treated as a complete adopted pair. Cancellation/memory/OS effects are not swallowed into success.
6. Interval soundness inherits footprint's ordinary binary64/no-flush-to-zero/no-unsafe-fast-math assumption. Resource counters and MAX_BITS checkpoints are not platform WCET, strict peak-memory bounds or proof that every temporary Fraction operation is below the checkpoint before its result is checked. Generic objects/properties and direct private-helper malformed calls are outside the validated public numeric contract.
7. No extractor call, map mutation or geometry acceptance is enabled by adding the file. Real40-cut/80-model captured packets, complete actual producer provenance, unchanged label/owner coverage and later observed runtime/full-chain effects still need new source-bound evidence. Prior65 synthetic/certificate tests and R29 CI/capture cannot be counted as this helper passing.

## Actual finite work budget and counters

After each float pivot, active stores at most2 positive-multiplier row IDs; adding the next violated ID makes a pool of at most3. Bases of sizes0,1,2 therefore require at most1+3+3=7 solve attempts per pivot. Every attempted singular basis system is counted before solve.

For public max_pivots<=24, per-side work is bounded by169 small systems (initial1 +7*24),24 pivots,25 full constraint scans,27 rounding packets, and at most27*2*N exact label substitutions. There are5 slots×2 sides: at most1690 small-system attempts,250 scans,270 packets and276480 exact substitutions at N512. Eigenvalue computation, Python/Fraction arithmetic, NumPy allocation/BLAS scheduling and other overhead remain outside a wall-time guarantee. These are operation-count maxima, not a measured speed promise or engine80ms fix.

Counters preserve rank/pivot/pool/nonfinite/no-certified27 reasons and partial attempted systems. Root added the append-before-call logging before execution; tests inject initial solve and later-side exceptions to check prefix retention. Counter truth matters even when a case fails; do not report a discarded/singular attempted system as no work.

## Independent test design and loader contract

The test normal-imports lane_mapping_lya_reference.paired_curve_fit. It has no absolute diagnostic path, helper SHA hardpin, production-oracle fallback, version skip or joint-module import. Fraction chart/label evaluation and originalP two-axis inversion are written independently of subject._chart_exact/_inside/_locations/_labels; handwritten SPD/KKT equations independently validate selected dyadic fixtures. Returned certified packets are checked against all original raw WB occurrences and the originalP/chart inequalities.

Coverage includes five exact hand locations; clipped original charts; sheared/negative axes; closed zero-width duplicate slots; empty/degenerate domains; unconstrained/single/two/three-member-pool SPD cases and exact counter totals; zero/one-pivot exhaustion; rank/negative-eigenvalue/pool failures; singular attempts counted; WB polarity and zero/one-ULP signs; overflow intervals and tiny curvature via exact fallback;27-packet failure; complete/permuted duplicates/off-extent labels; wrong normal sign; unnormalized numeric chart; readonly/sliced input preservation; mutable no-adoption results; partial exception logging;5/513 rejection and512 preservation; double-band ribbon-count mismatch; absent owner-authentication claim; shape/type/nonfinite/integer precision/pivot/MAX_BITS boundaries.

Python3.8 compatibility guard AST-inspects actual module/test source only in the future approved unit process and asserts no math neighbor-API attribute. All actual adjacent-value calls in helper and tests use NumPy. No AST was evaluated during this static preparation. Synthetic failure mocks are explicitly failure/counter tests, not another fitter or a physical oracle.

Suggested registered modules for root's four-binding raw-source loader:

- synthetic package lane_mapping_lya_reference;
- lane_mapping_lya_reference.footprint_geometry;
- lane_mapping_lya_reference.paired_curve_fit, registered BEFORE dataclass execution;
- test module r30_independent_paired_curve_fit_test_01 (or another unique name), registered with actual __file__ and marker __independent_test_source_sha256__ equal to the test raw-byte hash.

The test's normal import must resolve to exactly that verified module object. The compatibility inspect.getsource guard requires meaningful module __file__/loader/registry metadata and access to the same bound source. Hash and compile/exec the SAME raw bytes, not a separately reopened SourceFileLoader or pyc; bind footprint/helper/test/driver before/after. Root must fully read the source, issue a fresh bounded gate, preserve stdout/stderr/unique method IDs and any failure. This static report is not a CPU grant, test run or a successful58-test receipt.

## Conclusion

The current validated numeric proposal/label-certification separation is appropriately narrow; no nominal false-positive counterexample demanding a main-helper fix was found by static inspection. Owner authentication, actual rounded joint/geometry acceptance and measured runtime remain hard integration blockers.58 prepared independent tests and this review advance only to root's single bounded unit attempt. Do not call the candidate a repaired model, immutable map, complete SLAM, engine80ms solution or a successful two-lap demonstration.
