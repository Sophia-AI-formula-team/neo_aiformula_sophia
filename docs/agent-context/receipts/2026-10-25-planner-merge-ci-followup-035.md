# Integration CI follow-up

The sealed session `2026-10-25-planner-merge-all-035` remains unchanged. Its eight
readable files now match the sealed ZIP byte for byte in Git as well as on disk;
the scoped attributes preserve those original reviewed bytes on Windows/Linux.

At candidate `823991d`, Linux integration run 40124838895 and native graph run
40124838615 passed. The handoff matrix run 40124838523 passed its archive checks
but rejected two new README links because its sparse checkout omitted codexws.
The workflow now includes the experiment archive directory, preserving the link
checks and PR #18's exact archive instead of weakening either.

The original 404-test Linux result, local 33-test packager result and historical
R34 runtime limitations remain separate evidence. PR #29 records the final
candidate's checks and the user-authorized main merge.
