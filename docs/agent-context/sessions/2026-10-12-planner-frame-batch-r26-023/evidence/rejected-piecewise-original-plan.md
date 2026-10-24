# R26 piecewise short-closure model experiment — not integrated

The rejected unique-ID prototype fails two ordinary-rectangle positive controls.
This new estimator explicitly selects a low-complexity piecewise explanation;
it does not claim that raster face-to-side assignments are physical truth.

At an original contour, enumerate contiguous candidate closure chains of one
through k-1 faces (k=original minimum direction support). Observe complete
original-radius arms on either side without jumping invalid/opposite-polarity
faces. Both arms must satisfy original k/PCA/thickness/spread and have opposing
signed normals within the original30degree bound. The candidate closure also
needs a finite quantization-scale line fit; it never receives a map direction
from fewer than k observations. Original raw faces and links never change.

Select candidate partitions by a declared normalized residual plus complexity
objective, not uniqueness of discrete IDs. For each piece, fit a TLS line and
(only for original k-supported arms) a quadratic graph. Quantization variance
for a face is ((normal dot raw-face-vector)^2 + (normal dot WB-vector)^2)/12.
Score is sum(residual^2/variance)+p*log(max(N,2)), p=2 for line and3 for
quadratic. This is a specified model-selection prior; it is not a new sensor
measurement, a calibrated likelihood or a geometric acceptance threshold.
The candidate score is the sum over two arms and closure. Minimum score wins
among overlapping closures; exactly tied partitions are left unchanged.
Overlapping long-side support alone does NOT make opposite endpoints conflict.
Only closure/interior ownership conflicts are suppressed in score order.

Use the first original radius at which each candidate has valid side evidence.
Do not scan radii for lowest error. Chosen computational cuts are immediately
before/after each selected short closure. Existing _directions independently
refits each original target with unchanged radius/count/PCA/WB/J rules on its
new domain; short closure stays raw-only. No root/normal/graph point is invented
at a cut; no cap normal is borrowed from a long side. Original existing corner
certificates stay in force. This changes the geometric estimator, not its gates.

First evaluate all original22 development masks once, with all raw/topology/
projection arrays protected and complete coverage/error/lost/gained tables.
Do not feed primitive truth to extraction or candidate scoring. The independent
24-case holdout is generated/scored only after this algorithm is frozen. Normal
rectangle side coverage must remain>=90%; all-unknown is failure. Keep adverse
gap/U/FOV/J results and all failures. No runtime or driving claims from this
offline experiment; computation cost is separately unresolved.

Risks: the line/quadratic prior may mis-segment highly curved or irregular paint;
short two-point closure fits can absorb one side face. This experiment must
report those outcomes, not add posthoc exclusions or tune to face637.
