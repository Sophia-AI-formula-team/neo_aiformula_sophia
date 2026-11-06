# Header repair — 2026-11-06

## Objective and scope

The user requested repairing the image Header, merging into main with version
management, and organizing local source already retained by main. YOLOP and
fixed-trajectory code are excluded. No deployment or hardware action is requested.

## Input handoff

Read AGENTS.md, AGENT_CONTEXT.md, STATUS.md, PROTOCOL.md, the historical
2026-09-28-planner-neo-004 handoff and the latest experimental-import receipt.
The current request supersedes the historical source baseline for this repair.
Preserve the imported experiment and its known issues except this Header defect.

## Source and working tree

Base: 94e1797faaf511ed4cb9593c9646508dae17491b.
Branch: fix/road-detector-header-20261106, in a new isolated worktree, initially clean.
The original checkout and its untracked biased_mppi directory were not modified.
Existing local Python, pytest, NumPy and OpenCV were used; no installation occurred.

## Work performed

Restore full source Header propagation through image_callback and independently
deep-copy it into both published images. Bump package.xml and setup.py together
from 0.0.0 to 0.0.1; add the package changelog and dedicated Header CI workflow.
The existing four regression tests remain unchanged. No model or vendor change.

## Evidence and result

See [checks.json](evidence/checks.json).
Command before and after: `python -B -m pytest -q workspace/src/aiformula/perception/road_detector/test/test_output_contract.py`.
Before: exit 1, 1 passed / 3 failed. After: exit 0, 4 passed.
The tests execute actual callback/publish bodies and validate full Headers,
independent Header/Time objects, empty frames, encodings and image pixels.
Python 3.8 AST and package-version consistency checks pass. git diff --check passes.
CI and merge were pending when this source record was prepared; their actual
outcomes are recorded by the linked PR and GitHub checks, not inferred here.

## Risks and blockers

No ROS, DDS, model inference, Jetson build or physical vehicle validation was run.
Other imported experimental defects remain outside this fix. Local cleanup must
compare every removed source byte-for-byte with a retained Git object, preserve
unmatched files and bags, and exclude the entire local road_detector subtree to
leave YOLOP untouched. Cleanup manifest remains local and is not published.

## Next agent actions

Review and publish this scoped branch, wait for relevant CI, and merge the exact
reviewed head into main under the user's explicit authorization. Verify the merged
source before cleanup. Preserve historical records and the pre-import backup ref.
