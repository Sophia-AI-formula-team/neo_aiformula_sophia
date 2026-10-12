# Anonymous current-mask curve/fragment observer: static prototype

## Problem and evidence boundary

The parent reports zero side-boundary candidates over the earlier 84-frame
diagnostic. This does not establish that the camera contains no junctions. The
old observer maps each connected component to one PCA line, then asks that its
entire support fit a straight line. A connected bent marking or a joined set of
road markings can therefore lose useful local boundary support. Separate dashes
remain separate components; component ends cannot prove a road opening.

The production lane publisher also cannot supply the lost evidence afterward:
its polygon and per-side/tracking ROI crop first, best-component/fragment-group
selection retains one result per side, `project()` fits y(x) and resamples at
0.5 m, and diagnostic output removes the actual pixels/line. This prototype
takes the original current mask before those operations. It neither calls nor
changes the publisher, native point selector, LYA or semantic core.

## Small representation change

1. Project each original positive pixel independently through the original
   K/R/t ray onto the existing base z=0 plane. Preserve pixel ID, uv, component
   ID and quantization-only uncertainty. No fitted/resampled ground cloud.
2. Extract ordered OpenCV contours with CHAIN_APPROX_NONE. Contour vertices are
   actual positive pixels, not a polygon approximation or synthesized skeleton.
   Preserve complete contour pixel identities. Domain/horizon gaps split an
   ordered contour into separate observed runs; never connect across that gap.
3. Divide a closed contour into two measured arcs at its furthest observed
   vertex. Recursively split an arc at its largest point-to-chord residual.
   The split point remains an original pixel. This creates local descriptors
   of a bent or branched component while keeping its complete raw curve.
4. Local PCA describes each final fragment's measured support only. No point
   is moved. Short, thick, unresolved or quantization-sensitive pieces remain
   unknown. Fragment ends are support/cut extremes, never physical road ends.
5. Report nearby aligned fragment pairs as **unconfirmed gap relations** with
   actual endpoint IDs and measured gap. They do not fill a gap, turn endpoints
   into junctions, or make the gap count as observed span.
6. A paired-side candidate needs two local same-side positive-pixel boundaries,
   their measured overlapping axis support, plausible separation/parallelism,
   and current longitudinal positive support on both sides before and beyond
   them. Repetitive transverse markings and bars crossing the vehicle corridor
   are excluded. Store the actual before/after and boundary inlier pixel IDs.

All outputs retain `junction_recognized=false` and `junction_status=unknown`.
Paired-side candidates are not independently verified junction stations or
directed branch bearings; they cannot satisfy official_association's positive
observation contract. A simple curve, missing side, single endpoint or zebra
pattern must not become a recognized junction. The finite 2D mask can still
have ambiguous paint-like patterns, so even the positive fixture remains
unconfirmed and has no official ID or estimated world anchor.

## Limits and generic parameters

The default domain remains 10 m forward / 4 m lateral to isolate representation
changes. A caller may explicitly compare a larger shadow-only domain, but that
does not expand any control boundary. Thresholds are generic experimental
priors (pixel counts, local residual/span, parallelism, gap, branch width and
main-continuation support), not tuned against a named site or frame.

Contours trace the edges of painted regions, so the two edges of one thick
marking are not independent physical lane boundaries. The side separation
gate prevents treating its narrow thickness as branch width; repeated nearly
coincident fragment descriptions are not independent temporal evidence. Complex
paint, holes, severe curvature, antialiasing, occlusion and clipped branches can
still yield unknown. A budget overflow refuses classification rather than
choosing an arbitrary subset. No morphology adds pixels and no history is used.

The plane, camera calibration and segmentation errors remain unknown. The
reported half-pixel bound is not total metric uncertainty. The caller must bind
the raw image/mask, actual capture sequence/epoch/time, calibration and capture
odometry. Old recordings without atomic capture pose cannot produce trusted
world observations by joining them to later receipt-time odometry.

## Verification status

Only static implementation and synthetic fixtures are authorized in this step.
No Python import, test, image calculation, fit, model or Unity invocation has
been run by this agent. Root owns the next bounded test and actual metadata-bound
mask review. There is no new driving, junction-detection or matching acceptance.
