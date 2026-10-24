# Signed closed-route checkpoint progress — MapAssist 0.3.1

The real Unity `outer_closure_02` run closed its odometric lap but failed the final
checkpoint with `negative_progress`. The route matcher intentionally uses signed,
unwrapped progress around its seam; the new persistence validator incorrectly
treated it as a nonnegative travel distance. The [problem analysis](CRITICAL_CHECKPOINT_ANALYSIS.md)
was written before changing production code. The original run remains failed.

The fix changes one predicate in `topo_checkpoint.validate_state`: allow finite
negative progress only when `closed_at` exists. Recorded distance remains finite
and nonnegative, and an unclosed map still rejects negative progress. Other
validation, checksum, schema, restore continuity, geometry and control code remain
unchanged. Package metadata changes only `semantic_local_planner_ros` 0.3.0 → 0.3.1.

## Evidence

- [Original Unity result](evidence/outer_closure_02_result.json): 1028 samples,
  285.624625 m recorded, reference 286.991534 m; closure checks passed. Final
  process exit was orderly but checkpoint save failed. Official map fusion
  separately failed RMSE 3.453969 m > 3 m. This fix does not loosen that gate.
- [Original-code reproduction](evidence/original_receipt.json): real
  RouteReference.match produced synthetic progress −0.30000000000000193 and
  unchanged 0.3.0 validation rejected it. This is not the unrecorded final
  negative value from the Unity case.
- [Candidate regression ledger](evidence/candidate_receipt.json): **118 methods
  passed**, zero skipped/failure/error, Python 3.13, 6.495437 s. Includes all
  existing checkpoint and MapAssist methods in the two test modules plus three
  new methods. The new callback regression obtains the negative value through
  actual odom/matching callbacks, then saves/restarts and preserves reference
  points, trusted flags and signed progress.
- [Source delta manifest](evidence/manifest.json) records exact baseline and
  candidate hashes. Only the validator and two test modules differ; 52 copied
  files are byte-identical. Process receipts preserve exact commands, PID births,
  zero exit codes and absence checks. Tests were run on the staged candidates;
  hash-guarded copying confirms the committed runtime/test bytes are identical.

Windows testing uses real file fsync and one-byte msvcrt locks with an explicitly
declared no-op for POSIX directory fsync. These are synthetic callback/format
regressions, not POSIX durability, native ROS/DDS, physical vehicle or new Unity
restart acceptance. The actual simulation storage adapter remains separate.

## Deployment and next verification

The package's production `use_sim_time=true` checkpoint guard remains intact.
Native deployment must use the established deployment procedure with backups
and its own authorization. The Unity task uses a separate explicit simulation
adapter and root-controlled, hash-guarded installation.

After installation, rerun genuine inner/outer closure and semantic-only restart
through MCP and report closure, official fusion and restoration separately.
Keep failed videos/receipts. No camera, signal position, LYA, vehicle physics,
model, bbox-distance trial or unrelated feature is changed by this patch.

The checkpoint JSON schema stays 1. An unpatched 0.3.0 reader may reject a newly
saved negative seam value; preserve backups during rollback. Never silently
rewrite progress or mark an older run as passing.
