# R23 local source check; native9 unbuilt

Windows Python3.13.13, NumPy2.4.4 and pytest8.4.2 already installed.
One run, no retry: optional process exit0; 357 passed,122 skipped,0 errors/failures,
479 total, JUnit1.494s, runner3.3521843s. All skips require actual native9.
Spies/pure Python and unchanged installed native8 are not native9 execution.

Exact selection in lane_mapping_lya_reference/test:

- test_native_current_union.py
- test_native_current_union_dispatch.py
- test_finite_support.py
- test_finite_nextafter.py
- test_finite_connected_projection.py
- test_finite_scalar_validation.py
- test_finite_coalescing_equivalence.py
- test_finite_batch_validation.py
- test_finite_union_hotloop.py
- test_native_finite_projection.py
- test_boundary_domain_consensus.py

Command: python -B -m pytest -p no:cacheprovider [above files] -q -ra
--junitxml=[new local output]. LANE_REQUIRE_NATIVE unset; bytecode/cache disabled.
Separate second command: only new two files, --collect-only, LANE_REQUIRE_NATIVE=1.
Exit2, explicit RuntimeError requiring native9. Expected refusal, not numerical pass.

Old union body (name normalized) and _connected_spans AST match exactly.
Union AST SHA be1b25c1c4609423e135e81b65a6348217d3f2780035264aa9068d8773605793.
35 pinned files unchanged; native8 binary unchanged:
690e7b900844f44f0d21cf1ba41dd903c437a36ac8053a41ea0f9f44a87a7f83.
Embedded old C SHA 0c54086e66554c8e9c96b48893e4b19c076c8a747b8f2cf08d6ad216fdeca12d.
Native9 capability absent and not silently claimed.

Local artifacts retained; raw machine paths removed from this summary:

- summary_01.json: 1f65effd4528f9cde34dcb2d6d08e096a3064e62abc11a30f19b2598a45c2f66
- optional_01.xml: 13aa8a5ce68174bc31d7e3cd1c987ecfe77aaa7eea4e58df7608da1a11280cec
- required_collection_01.xml: cd82e00c96c29b25fe04e8ddd22709b31868e5d5d90dac508c02f4576c93c470
- run_once.py: fe4e2a0de82139ba401d006756e88d2fa0592f5a551deb0dc8750df3bb73c7f5

No engine/benchmark/Unity/full-package regression/native9 numerical call.
