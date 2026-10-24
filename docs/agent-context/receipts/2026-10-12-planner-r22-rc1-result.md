# R22 RC1 final CI result

Planner, neo-004; evidence acknowledgement, not driving acceptance.
Exact commit baba879468b8674e7044dadebe2363baae378238 and annotated tag
lane-mapping-20261012-r22-rc1 (object de6f97e012c72452995ee16069fc7c194db44692)
were read back from canonical repository ID1303517209.

All eight original automatic push workflows are terminal, attempt1:
native 38155963803/38155963678, handoff38155963623/38155963653,
Lane Foxy38155963728/38155963595 succeeded. Endpoint Foxy
38155963632/38155963566 failed at runtime DDS, not build/unit.
Overall6/8 workflows succeeded. No rerun/cancel or binary installation.

Both Endpoint runs built five packages, verified installed native8 real calls,
source and prefix, and passed2977 reference+932 GNSS=3909 unit tests without
skip/error/failure; Header4 also passed. default_guard/startup_gap passed.
The other seven cases timed out waiting for five stationary camera commits at
runtime_smoke.py:300. Later scenario-specific operations were not reached.
There were126/132 teach_mask results, all rejected, with processing deadlines,
commit-guard rejection and finally bootstrap_anchor_not_established. Input was
not absent. Worker compute222.204–491.358ms; derived journal wall−image-source
273.248–594.573ms is not a separately measured receive or transport latency.
No explicit worker last_error was saved, so none is inferred.

Original branch/tag runtime JSON SHA256:
255a9e57fc5182ff011703540741bcab2eb6e3b02969ea3f2bf7c2170054b25d
91bacc75dddb3fa18df05168ba6210d50595f02996ee8614f6c74ad8d8cce653.
Local text evidence/metadata/case summaries retained in r22_publication_01.
No raw bag, images, video, precise GNSS, credentials or personal paths included.
These are synthetic actual ROS-message tests, not physical stop/speed/lap proof.
The R22 strict recorded prefix remains11/12 accepted and failed at seq162;
no additional Unity run or first/second lap is implied by this CI readback.
