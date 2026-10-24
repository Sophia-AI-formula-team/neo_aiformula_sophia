# Curve capacity dimension correction — static source evidence

The actual diagnostic remains unchanged: sequence153 stopped before both
commits with232983 scalar/header units versus its declared200000 gate. It
retained all source occurrences and signatures. No rerun or model success is
asserted here.

The previous status incorrectly called that gate the original mapper capacity.
The protected continuation_support.py storage_units definition is1+len(quads);
paint_geometry.py additionally counts one record per continuation signature.
It does not count the eight quad coordinates, two normal coordinates and flag
as eleven independent supports. Domains, segments and permissions also count
semantic records. The new curve41*N+headers+affine accounting is a distinct,
stricter raw-scalar/header metric. Adding the two dimensions and comparing that
sum with original max_support_segments is not valid original-capacity evidence.

This is a planner implementation/accounting error, not an upstream repository
or simulator defect. Fixing it must retain the unchanged200000 original support
limit, all occurrences, UNKNOWNs, signatures, incident copies and causal chart
witnesses. A new representation must explicitly count all retained semantic
records, shared table lifetimes and derived/exported records, while separately
reporting complete raw scalar/header inventory and payload bytes. Payload is
not total heap usage. No existing angle, sensitivity, voting or80/40/200ms
deadline gate is relaxed. The failed old representation remains unintegrated.

Static independent source check was reported by indexed_bounds_review; root
has read the current paint continuation/prepare/export definitions. This note
is a correction, not a numerical rerun, capacity pass or runtime acceptance.
