# Independent B02 OFF-only archive review

Scoped result: all 111 original PNG extractions, sequence 151 through 261, have identical ON/OFF observation content. This is an extraction-equivalence checkpoint, not geometry repair, runtime acceptance, a speedup estimate, a new-scene lap or two-lap driving success.

## Fixed actual evidence

- B02 APPROVED SHA: `2471e38429011e2230219889311a68bd36ed61371179d3d05ff982e2dd00cd54`.
- Original B02 PENDING SHA: `21799e6b35ea45b13efbe1c72733ac5b0741238ef9776f4aae74339d78e095e8`. APPROVED differs only in the four allowed control fields; remaining content is equal.
- B02 parent result SHA: `108cfa2ee39cf8414c0a90411387707f8fe0ef842bb9795f162149bcd0071d48`.
- Actual OFF child result SHA: `3476fb330f6f76ae7971fafebf40d8d7cdc48cacce41f95b2ea6a3b8ba68e909`.
- Frozen B02 driver SHA: `28734112f6a7be73ce510e57bd27a923a49357ff18db8a94105f8a53e1ad9d7b`.
- Independent read-only inspection source SHA: `468a0fdd0edda59f696c24b890d7fcf6cd4a65baf7e2d8abf1a6467f650a2ac0`.

## Independent checks actually performed

Ran `inspect_once.ps1` exactly once: PowerShell 7.6.5, exit 0, tool wall 5.0810402 s. It reads existing JSON/ZIP bytes only. No Python, NumPy loader, AST, package import, model/solver, native verifier, replay, Unity, source modification or retry was run. `inspection.json` retains the resulting ledger and all 111 per-case checks. Controlled paths reject excluded filenames/resolved leaf aliases before reading or hashing; neither excluded prototype's content was read.

All 374 declared source/reference file hashes match current physical bytes. The two parent and two child before/after maps each equal the same 374-pin manifest. This includes the frozen sources, original ON receipts and all original ON metadata/NPZ/case files; it does not assert that every unrelated worktree file is unchanged. All OFF files declared in the parent output-hash map match; the parent result itself separately matches its fixed SHA above.

For every case, original ON `_case.json` equals its original child case, and actual OFF `_case.json` equals its new child case. All 111 OFF `all_fields_byte_exact` flags are true, but the independent inspection does not rely solely on those flags: it opens each pair of NPZ archives read-only and compares every same-named `.npy` entry using literal `SequenceEqual<byte>` on the complete header and payload. There are exactly 33 numeric entries per case, with no extra or duplicate archive keys: 3,663 entry pairs and 89,266,628 payload bytes per arm. Each entry's NPY version/header, dtype, rank/dimensions, C-order, payload length and SHA separately match its producer descriptor.

The 29 dataclass fields plus five derived fields are complete in every case: 33 numeric arrays and one scalar source timestamp. Full producer descriptors and scalar values match, not just hashes; payload comparison preserves signed-zero and NaN bits. The recorded producer flags are 3,330 readonly arrays and 333 writable derived arrays across 111 cases, equal in both arms. These flags are authenticated captured producer metadata, not claims that ZIP files or freshly loaded arrays preserve Python memory writeability. No NumPy `np.load` flags were substituted.

The actual OFF source ledger has exactly 17 entries and matches the independently captured HEAD `9b8636fc3c1aa9e9712142b3924a722211df19bd` Git raw-byte SHA map. Recorded registry validation is true; the frozen reviewed driver checks the exact 17 module names and object identities in `finally`. No modules were re-imported by this review. The unchanged native binary/C identities remain within the 374 pins; no new binary or native test was introduced. OFF reports one original loader call, 261 exact raw-motion and attitude frames, 2,868 motion samples, zero odometry resets, no engine and no ON reexecution.

Owned identity links match start/started/owned/parent/child receipts and the actual command. Parent PID 63080 has birth `134362476871120923`; child PID 39484 has birth `134362476888608496`. Both PIDs are absent at review. Actual child exit is 0, stopped true; stop-attempt false and retries zero. The approved budget remains one 60 s primary and at most one 5 s stop, not a widened control deadline. OFF child elapsed is 26.315529899962712 s and parent elapsed is 28.956018100026995 s; these single archive-heavy offline observations are not runtime latency or statistical gain.

## Original failure retained and limits

Original B01 remains FAIL, parent SHA `562bfaa4dcffec426d357fc6ae78be6adb4cd0a8010daedb165a3ffb96932cb2`. Its completed ON child SHA remains `d7f9d87a59228304b67bc767ed4d87706854d4b9c9245ebc99a0b000022363a5`; ON was not rerun. Its original 338 ON files remain unchanged. B01 OFF produced zero images because four old BASE worktree CRLF hashes did not equal the immutable Git LF hashes. The line-ending comparison is retained only as the diagnostic cause; B02 execution accepts strict Git raw hashes only. No loader normalization or alternate-hash fallback was added, and old A/B01 source, approvals and outputs were not overwritten.

No model-content difference or source/ownership failure was found within this declared scope. Equal extraction does not prove that the map is geometrically adequate, that extraction plus transport/engine/controller fits 40/80/200 ms budgets, that consensus/localization is unbiased, or that the unchanged simulator can complete TEACH then fixed-route REPEAT. This review does not change any parameter, acceptance gate or production package.
