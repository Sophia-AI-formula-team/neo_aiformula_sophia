# R17 RC1 development result

- UTC readback: 2026-10-11 11:47:42--11:51:28; role: planner subagent.
  RESULT_READY for analysis, partial result; not vehicle release.
- Input: `2026-09-28-planner-neo-004`. Code checkpoint:
  [b8482aa3](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/b8482aa37935ff6ba268de6939ca6982681e0e0f),
  `lane-mapping-20261011-r17-rc1`, branch
  `repair/lane-keyframe-transaction-20261011`.
- Canonical repository ID `1303517209`, branch SHA, annotated tag object
  `027a7363d1de5d6057483e08a52309d70c18e032` and its commit target were
  independently read back and matched. Local HEAD also matched; subsequent dirty
  R18 work was not included or modified by this audit.
- Session: [014](../sessions/2026-10-11-planner-finite-domain-r17-014/SESSION.md).
  [ZIP](../bundles/2026-10-11-planner-finite-domain-r17-014.zip), 397707 bytes,
  independently verified SHA256
  `c63c9088c37f087ae3651cb66e3ac078901b5e4c53099740b86cc9483dab0441`.
  The sealed session and ZIP were not changed.

## Remote CI: 6/8 successful, not all green

All eight automatic push runs completed on the exact checkpoint, attempt 1.
There were 14 successful jobs and two failed jobs out of 16.

| Workflow | Branch | Tag |
| --- | --- | --- |
| Lane Foxy | [38136532480](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532480) success | [38136532659](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532659) success |
| GNSS Foxy | [38136532481](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532481) failure | [38136532623](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532623) failure |
| Native Windows/Linux | [38136532420](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532420) success | [38136532609](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532609) success |
| Handoff integrity | [38136532424](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532424) success | [38136532559](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38136532559) success |

Both GNSS jobs built five packages and completed **2995 Linux unit tests with
zero errors, failures or skips**, followed by four successful Header tests.
Their installed native module passed all seven real API checks. These are
actual stdout counts, not inferred from artifact membership; no JUnit XML was
extracted or counted.

Both jobs then failed the synthetic, real-ROS-message runtime smoke:

- Branch: 2/9 cases completed successfully; seven cases timed out.
- Tag: 6/9 cases completed successfully; `no_gnss_build`, `future_gps`
  and `uncertainty` timed out.
- Every timeout was `five actual stationary camera commits`, at
  `runtime_smoke.py:300`, waiting for the mapping engine's accepted count to
  reach five. The branch's 121 and tag's 145 already-executed checks were true,
  but timeout prevented later checks; these partial counts are **not DDS passes**.
- The runtime JSON establishes the failed wait, not the internal reason the
  worker failed to commit promptly. No profiling, retry, gate change or source
  modification was performed to reinterpret this failure.

Eight unexpired CI artifacts and their digests were recorded. Only the two
failed GNSS ZIPs were downloaded, with their API digests verified, and only
`evidence/runtime.json` was parsed. Raw reports remain local. Readback:
`r17_publication_01/ci_readback_01.json`, SHA256
`a91a2e31bde50908b78ef87c12725e9e93bbc9410495ce6155a46608455d3fb1`.
The readback used read-only `gh api` / `gh run view --log-failed`; there was no
dispatch, rerun, cancellation, Git write or test execution.

## Mandatory local failures and untested scope

Final Windows offline core evidence remains 2994 passed / one Linux-only skip,
166 source identities unchanged. The unrelated, unintegrated curve-fit prototype
was explicitly excluded. Linux CI's 2995/0-skip count is a different platform,
not a contradiction or a replacement for this local record.

The final fixed recorded-input candidate remains **12/13 accepted** of 111
available frames: sequence 163 was not committed because engine time
90.3375 ms exceeded the original 80 ms budget. Its visual match passed at
30.58 ms and overlap 0.6448275862; this does not make the whole frame pass.
The four development geometry cases remain **0/4**. Quality04 was not run;
the final coalescing optimization's 30 strict equivalence tests are not a new
quality execution. Earlier failures remain in sealed014.

No new Unity drive, complete TEACH lap, frozen driving route or REPEAT lap was
performed for this checkpoint. Last actual trial remains failed trial28.
No live Editor or vehicle state is asserted by this remote/offline audit.
Vehicle, simulation, camera, LYA speed, safety gates and GNSS mapping policy
were not changed by this task. CI success would not itself authorize driving;
here the overall CI is also failing.

Next owner: planner, session015. Investigate the stationary DDS commit timeout
and the finite-support/whole-frame runtime failures under the original gates.
Keep R17 as the immutable failed checkpoint; do not overwrite sealed014 or
present future R18 work as already tested by these runs.

