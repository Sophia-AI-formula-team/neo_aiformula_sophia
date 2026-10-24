# Unity semantic topo integration

Read README.md, DEPLOYMENT.md and AGENT_HANDOFF.md before running this package.

- This package adapts neo MapAssist 0.3.1 and motor traffic gate 0.4.1 to Unity.
  Upstream baseline is ed8e397ae41732877dfef334f0b89f98c0b20ae8; export the fixed
  production commit 3c6fbac2ff9f8dd23b247a187e966648876400b1 verbatim. The necessary
  checkpoint predicate fix permits signed closed-route progress. Do not claim
  all 0.3.0 production code is unchanged. Core geometry, closure/map thresholds,
  original LYA/LLP algorithms and plant remain intact.
- Work on semantic topo here. SLAM/lane-map work in another chat is independent;
  coordinate only actual shared Unity/CPU ownership conflicts. Never consume its
  unpublished candidate code or modify its checkout.
- Use Unity MCP only. Before source installation, Refresh, scene changes or Play,
  establish Editor ownership and capture clean Edit state plus existing parameters.
- stop_wait and bbox-distance are mutually separate experiments. Never add range
  payloads, signal approach or detour to this package's active transport.
- Write concrete problem analysis before any necessary repair. Keep failed runs
  and source manifests; do not overwrite evidence to make a retry look original.
- A reference closure is distinct from official topology fusion. A changed PID
  is distinct from checkpoint recovery. Test production state and continuity.
- Windows checkpoint support is an explicit simulation adapter, not proof of
  native ROS/DDS, vehicle deployment or power-loss durability.
- Use a feature branch stacked on the pinned topo PR, scoped commits and draft
  PR. No force push, merge, release tag, weights, raw RGB or secrets in Git.

