# Learning when to keep the reference controller

In four newly registered paired Unity segments, the frozen second model and the reference both passed all four safely. Their complete physics and command traces were identical in each pair. The model selected the reference on every encounter decision. Seven of eight choices changed from a learned alternative to the reference when the empirical risk penalty was applied; one already favored the reference. These trials demonstrated conservative rejection, with no measured increase in obstacle-zone efficiency.

## Data and method

The model kept V1's six action options, frozen lane detector and runtime solver 788eb7af5abf9f. The new return table incorporates the original 72 training rollouts and 48 additional Unity rollouts. These additional examples are seeded variations sampled from the original training families only. All outcomes from the original blind segments and full lap were excluded. Both the original and its derived variants stayed together in each leave-one-family-out development fold.

For every observed context and action, the training program records newly failed passes or new low-clearance passes compared with the reference in that same scene. It estimates excess risk from neighboring training contexts, balances each parent layout's contribution and accounts for the effective number of distinct families. The uncertainty score is a model heuristic. Its one-standard-error Wilson-style expression does not provide a certified confidence bound for repeated, related simulator layouts. The reference action has zero excess-risk debit by definition. At runtime the same C# kernel scorer reads ordinary return minus the learned penalty times this local risk estimate. It can still choose any of the six actions.

The selected risk weight was 6.0, risk-kernel radius 0.85 and action-scorer width 0.35. The 12-family tuning search prioritized avoiding new failures and newly inadequate outcomes, then successful clearance and obstacle-zone efficiency. Leave-one-family-out development rates are family-weighted. Across these folds the selected model introduced no new failures versus the reference; the best zero-penalty ablation introduced a family-weighted 6.25%. These development figures informed selection and are separate from the independent evaluation.

## Public-layout diagnostics

These three layouts had already been measured in the first learning experiment. They remained excluded from training.

| Previously revealed layout | Reference | Learned V1 | Learned V2 |
|---|---|---|---|
| H05 moving cone | contact | inner fast, cleared by 1.268 m | inner fast, cleared by 1.268 m |
| H07 bend and moving cone | cleared by 0.350 m | contact | inner slow, cleared by 2.274 m |
| Full mixed lap | 32 gates, 88.02 s | contact at 218.03 m, 24 gates | 32 gates, 88.02 s |

V2's pre-penalty ordinary scores already selected the slower inner line on H07 after the extra practice. The risk term preserved that choice. On the full lap, the risk term changed all eight ordinary proposals to the reference. The full replay then matched the reference physics row for row, including minimum cone clearance of 0.179 m.

## Records and reproduction

- `model.json` has file SHA-256 `22d6f47258e079bd74a93574010c02f78afb605e38c7fe8191d23a93cc4ea729`. It stores ordinary scores, local harm rates, uncertainty debits and adjusted returns. The independent scorer verified the frozen file before revealing the four layouts.
- `practice_layouts.json`, `training.json` and `runs/r01_a0.json` through `runs/r08_a5.json` contain the additional practice, outcomes, folds and 120 hashes. Parent-family holdout preprocessing uses training-fold samples.
- `runs/s01…s04_{base,learned}.json` hold all eight new stress-test arms. The three `diag_*_learned.json` files are fresh replays on the publicly revealed layouts, using the earlier recorded baseline for comparison. `risk_action_audit.json` independently reconstructs ordinary scores, individual risk debits and deployed choices, and records stream counts and digests. `events.jsonl` logs all 59 new runs.
- All evaluation and diagnostic arms have 20 Hz chase/clean-RGB and 5 Hz A4 imagery on the recording workstation in `.work/frames/<run>` and `.work/camera/<run>/a4_samples`. The four-stress-layout registration lives in scorer commit 7b3239b. The original whole-lap registration lives in 2b647e0. Ends of laps are judged at gate 32.

Activate the separate private Unity batch project, set AIFORMULA_LAP_COPY and work from the unity_laps directory. For a fresh replay point every generation and auditing command to the same new directory with `--output`. Keep the archived V1 training records accessible under learned_revision.

```powershell
python -B risk_encounters.py train
python -B risk_encounters.py fit
python -B risk_encounters.py external --matrix learned_holdout/v2_stress_layouts.json --film
python -B risk_encounters.py replay full01 --matrix learned_holdout/full_lap_layout.json --film
python -B risk_encounters.py replay h05 h07 --matrix learned_holdout/layouts.json --film
python -B risk_action_audit.py
```
