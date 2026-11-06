# Supplementary full-lap check

Register this additional single paired lap before either learner unblinding or any lap outcome. This pair is descriptive and separate from the eight short-segment primary evaluation and its prespecified improvement rule.

- Run `full01_base` then `full01_learned` once on the same new seed, natural cruise (approach 0), A4, frozen weights/model, solver, start and exact four props. One cone is static. A close pair has one static and one moving cone. A final cone moves in a bend. The jury provides no layout or seed to the controller.
- Use the entire course (`limit=1`), cold-started control and no preloaded course map. The existing Unity stop logic decides completion, contact, track departure, timeout or other termination. No restarts after a driving failure. Retain failed evidence.
- Report lap completion (`status=lap`, gate 32) and elapsed simulation time, or first terminal reason, timestamp, progress in metres and fraction of the full lap. Include actual minimum cone and road margins, per-encounter exit state and first control difference. Complete laps satisfying ≥0.15 m cone clearance, ≥0.08 m road margin and every prop exit are separately identified.
- Compare lap times only if both complete safely. For incomplete runs, any matched-prefix free-road pace and previous obstacle exits must be explicitly labelled diagnostic and cannot become a lap speed victory. One new lap cannot establish a whole-track success rate.
