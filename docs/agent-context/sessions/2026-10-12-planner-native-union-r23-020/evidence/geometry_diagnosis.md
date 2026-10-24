# Fixed original-window diagnosis, not a geometry repair

One archived R20 directions-only reconstruction, arc_8.0_phase_0.35 faces0/376/637.
Original first-selected windows, radii, PCA, coefficients, paths and projected
inputs unchanged. No new mask, alternative window, full extractor, engine or LP.
All original direction fields and three model outputs matched stored bytes.
Face0 also matched the earlier exact-label capture.

| Face | Support / radius | Violating labels | Max chart-height violation |
| --- | --- | --- | --- |
| 0 | 14 / 0.9m | 2 white + 2 black | 0.13480649433800962m |
| 376 | 8 / 0.45m | 1 white + 1 black | 0.0307801905175627m |
| 637 | 8 / 0.45m | none | 0m |

Exact rational residuals on the same binary64 input; NOT Euclidean truth error.
All-window consistency would reject0/376, not637. Earlier posthoc normal error
for637 remains32.779937 degrees. This audit proves no competing feasible normal;
label consistency alone does not establish physical direction accuracy/uniqueness.

Single execution exit0, original outputs exact, inputs/source unchanged.
Result SHA d5f411678d87ce166f114903cf32efa7cf4c9666af620d493d36a9df6ba7603d.
Capture SHA f4e6c5609749459d84f6d3285fcf2d2b89a54480393113d573e3c19846275dff.
