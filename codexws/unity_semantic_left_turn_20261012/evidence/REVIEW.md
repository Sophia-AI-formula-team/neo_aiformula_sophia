# Disabled calibrated topology adapter — staged review

This is a new simulator-only helper. It does not modify or import into the running worker, original core, Unity Assets, OSM or recorded reference. The one authorized verification completed: 18 tests passed in 0.050 s; the full hash/real-reference check took 0.524 s. CPU was returned immediately afterwards.

## Interface

`load_calibrated_topology(configuration, reference, osm_path, *, enabled=False)` accepts a JSON path/object and an actual original `RouteReference`. The disabled path does not inspect arguments, read files, import core, or mutate inputs. Other truthy values do not enable it.

Enabled success returns `CalibrationResult(available, status, graph, node_positions, node_progress, prior_nodes, projections, direction)`. Invalid configuration/reference returns unavailable with a reason and no graph. Success validates portal association; it does **not** certify any requested bypass or road clearance.

The JSON schema is `semantic_calibrated_topology/v1`. Required keys:

- `calibration_id`: explicit version/identity string.
- `frame: "odom"`, `axes: "x=unity_z,y=-unity_x"`. Metres in the unchanged measured-reference frame.
- `osm_sha256`: exact lowercase SHA-256 of the official OSM being used.
- `outer_node_ids`: explicit simple closed official cycle, repeating its first ID.
- `direction`: integer +1 or -1, **checked against direction independently inferred from portal projections**.
- `logical_nodes`: exactly the OSM node ID set, each mapped to a declared two-coordinate control portal. These are calibrated priors, not visual observations or intersection-center claims.
- `edges`: entries `{node_ids:[from,to], points:[[x,y],...], bidirectional:false}`. Each directed edge follows official adjacency and starts/ends exactly at its declared prior portals. Consecutive geometry points must be nonzero and at most 1 m apart. Reverse travel is available only if explicitly configured or `bidirectional:true`.

Optional `max_projection_distance_m` defaults to 1.0 with a hard ceiling of 2.0. Optional `trusted_neighborhood_m` is a symmetric 0.5 m radius by default, constrained to 0.2–2.0 m. Optional `description` is metadata.

## Association and geometry boundaries

All outer portals are projected onto every segment of the original measured reference. All nearby geometric branches are counted **before** checking trust, so a nearby excluded branch cannot conceal ambiguous association. A unique connected projection neighborhood, finite original geometry/progress, and a fully trusted symmetric interval are required. Projected nodes must form exactly one distinct cycle and infer the configured direction; no similarity or scale is manufactured.

`node_positions` and the graph's outer `.nodes` are the reference projections used for entry localization. `prior_nodes` and `projections[*].prior_point` preserve the declared portals separately. Interior graph nodes keep their declared prior coordinates. Edges are **never translated, snapped or rebuilt** to meet projected points. Every selected outer endpoint must additionally lie within 1 m of its reference projection, even if the association limit was configured above 1 m.

The graph calls the unchanged original `TopologyGraph.alternative_route` to choose the logical path with original official chord costs, blocked-edge semantics, internal-node requirement, and no repeated-node spur. It then expands those chosen edges into configured dense points. Missing geometry or missing direction rejects that route (`None` plus `graph.last_route_status`); it does not select another logical path, invent a chord, or use a straight fallback. Unused edges need not be calibrated. Returned `ExpandedRoute.node_ids` and `.points` have intentionally different sizes; it is not an `OpenTopoRoute`.

## Integration requirements and remaining limits

The parent wrapper must leave original automatic fusion/alignment failure diagnostics intact and expose calibration as a separate mode. This helper neither switches MapAssist into map mode nor changes its gates. Caller must verify this JSON outer cycle equals the actual MapAssist official cycle, retain fresh odometry/commands, real signal votes, OA/branch/E-stop ownership, and guard reset behavior.

Configured segment density alone does not prove smoothness, curvature feasibility, correct lane, intersection permissions or vehicle clearance. Independent geometry audit and actual controlled runs are still required. The helper does not read CAD or silently derive calibration coordinates from it. Loading a saved reference requires the parent's explicit captured-reference/source-hash validation and warm-start label.

## Prepared verification

`test_calibrated_topology.py` uses synthetic fixtures with actual original `RouteReference` and `TopologyGraph`, without ROS, Unity, images or model inference. Cases cover default no-read behavior, original Dijkstra selection, separate dense expansion, preserved input state, inferred reverse direction, inconsistent/collapsed cyclic portals, trusted-neighborhood failure, ambiguous nearby branches including excluded alternatives, hard range cap, prior/projection separation, missing directed geometry, reverse permission, malformed finite/schema/ID/hash/adjacency data, sparse edge geometry, duplicate JSON fields, and invalid reference internals.

For a future explicitly authorized rerun:

```powershell
& 'E:\Mess\Projects\Programming\aiformula\YOLOP\YOLOP_A2\.venv\Scripts\python.exe' -B -m unittest -v test_calibrated_topology
```

Use this staging directory as cwd. `CALIBRATED_TEST_CORE` optionally overrides the semantic_planner_core package root. A passing unit suite is not simulator or hardware acceptance.

## Captured outer-reference result

`topology_verification.json` records the single executed test command/stdout/stderr and before/after source hashes. Candidate module/test/configuration, exact snapshot, and both installed/worktree core source files were unchanged. Its SHA-256 is `7f1c5034b7bfed90964f64cd1ed2cb03706d4cc928e314c5685d9bcf197e6346`.

Original `RouteReference` was reconstructed from all exact captured candidate samples with the snapshot's unchanged spacing, smoothing and gap parameters. All 4302 numerical reference values, full trust mask and other structural fields equal the captured fallback reference exactly; no full fusion fit was rerun. Reference length is 286.488546583 m.

For `unity_east_detour_outer_v1.json`, portals 00–04 project with errors 0.007296, 0.000101, 0.073784, 0.005388 and 0.002589 m respectively. Each has a unique nearby branch and fully trusted ±0.5 m neighborhood. Projection order independently yields direction +1, matching the declaration. Original Dijkstra selects 02 → 06 → 03; its two configured directed edges expand to 162 points without moving prior endpoints. This is explicit calibration validation only: the original automatic fusion remains `rmse_exceeds_limit`, and neither road clearance nor closed-loop turning has been validated by this check.
