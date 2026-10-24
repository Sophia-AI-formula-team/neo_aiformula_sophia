# Curve observer candidate (static, not installed)

Read [DESIGN.md](DESIGN.md) for the failure analysis, approach and boundaries.

API: `observe_lane_geometry(mask, camera, *, enabled=False, parameters=None)`.
`ObserverParameters` is an explicit immutable parameter set. False returns
without inspecting inputs or importing NumPy, OpenCV or pixel_projection.
Only the exact boolean True enables image geometry. This is a standalone staging
directory; run its test file in its own Python process to avoid the earlier
prototype's same-named `observer` module being reused from an import cache.

Result fields: `observed_points`, ordered `curves`, local `fragments`,
`fragment_gap_relations`, and `side_boundary_candidates`. Each fragment refers to
original frame-local positive pixel IDs. Curve partitioning does not alter the
individual ray-projected ground points. Every gap is explicitly unobserved;
extremes are not physical lane ends. Budgets bound positive pixels, contour
pixels, split work, fragments and pair enumeration; no arbitrary top-k sample is
silently substituted. Candidate-budget overflow preserves raw geometry only.

No callback reaches LLP/LYA, semantic core, map matching or commands. There is no
RGB/model invocation, capture simulation, data association, world anchor,
official ID, CAD, reference-route input, hand-selected junction or history.
`junction_recognized` is always false and `junction_status` always unknown.
Even a paired-side candidate cannot be passed as a verified junction observation
to the separate official-association prototype.

All 18 synthetic test methods are **prepared, not executed** at handoff. No Python
import, AST/test, saved-mask computation, fitting, model, encoder or Unity was
executed in this implementation step. Root will collect actual results during a
separately bounded verification window. Earlier 84-frame zero-candidate evidence
belongs to the earlier observer and does not establish acceptance of this one.
