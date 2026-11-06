# Changelog

## 0.0.1 — 2026-11-06

- Restore the complete source image Header on the lane mask and annotated image
  after the 1027/1102 snapshot import. Both outputs preserve the acquisition stamp
  and camera frame, with independent deep copies of Header and Time.
- Keep the existing pixel conversion, inference and model assets unchanged.
- Retain the offline callback/output regression tests and run them in dedicated CI.
