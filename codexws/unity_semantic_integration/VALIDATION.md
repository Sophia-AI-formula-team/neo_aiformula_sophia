# Archive validation

2026-10-12. The coordinating agent ran the following checks after the archive
was assembled. These checks did not start Unity, inference or video encoding,
and did not apply or roll back files in the original Simulator.

| Check | Result | Scope |
| --- | --- | --- |
| `pytest test_install.py` | 12 tests, 11 subtests passed; 0.39 s | Isolated temporary project; exact byte roundtrip, unrelated-file preservation, drift and collision refusal, explicit write flags, path scope, native recipe policy and interrupted-write handling |
| `install.py --action verify-after --simulator <original Simulator>` | `ok=true`, 116 managed files, 305 prerequisites, `writes=0`, exit 0 | Full before/after payload materialization from both pinned Git revisions plus exact currently installed hashes; no Simulator changes |
| Native integration policy checks, reported separately by the coordinating agent | Final rerun 78 passed, one ROS-dependent module skipped; 0.10 s | Native policy checks; the skipped ROS module does not establish native ROS/DDS operation |

These are archive/integration checks for the frozen source. The final63 Unity
four-run acceptance and media QA remain the historical receipts in `evidence/`;
they were not rerun for this archival change. Package core tests (212 tests and
203 subtests) are also distinct from this archive's 12 safety tests.

The coordinating agent also completed the full frozen 116-file roundtrip in an
isolated compatible project: verify-before, apply (116 writes), verify-after,
rollback (116 writes), and verify-before all passed. Every stage checked 305
prerequisites. The original Simulator received zero writes.

A Git index checkout verified all 224 nonempty payload sides against the frozen
manifest. Windows export used NUL-separated paths and a short target directory
with Git long-path support. An initial export failure from newline conversion
and a second from the Windows path limit were corrected before this check.
The manifest SHA remained
`7141f2721b58a8793e6bbad907ad0cd53be0db4f8f1760d9f56d886ccaa69f91`.

Frozen before/overlay/evidence files intentionally retain their original CRLF
and Unity metadata whitespace. Their attributes disable whitespace reformatting
checks; exact hashes are the acceptance criterion. New bridge, tooling and docs
retain normal whitespace checks. This changes no frozen source bytes.
