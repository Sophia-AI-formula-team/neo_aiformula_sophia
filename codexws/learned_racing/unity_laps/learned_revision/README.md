# Learned local encounter policy

From matched A4 images and Unity body motion, a frozen conditional-return model selects one of six driving actions for a short encounter. Training took 72 simulated segments, six alternatives from the same observed approach on each of 12 layouts. The action table was estimated from 426 near-obstacle observations in archived driving logs. It contains the prior passing controller and combinations of line offset and speed drawn from those logs. Steering continues to follow the current A4 road line, with the chosen offset and a curvature-limited longitudinal target. Vehicle controls still use the existing steering loop, command ramp and motor model.

The planner starts its local choice within the farthest observed lane point. A recent tracked object's lateral displacement/velocity, projection at encounter, direction of the road, vehicle speed and current lateral command are among the features. The selected feature set and kernel width were chosen with grouped leave-one-layout-out validation. When a newly appearing track interrupts an encounter the model can make another selection. The tracker's ego-motion correction uses Unity body pose. Track maps, phases, placements, future obstacles, judge data and random seeds are excluded from the feature vector. Tracking and outcome prediction depend on the fidelity of this simulated motion and perception.

## Training and records

- `actions.json` records the empirical six-action table. `runs/t01_a0.json` through `runs/t12_a5.json` are the actual Unity counterfactual rollouts, including contacts. Each group has exactly the same observations up to selection.
- `training.json` stores the contexts' measured outcomes, SHA-256 per trial, fitted feature scaling, grouped hyperparameter trials and return table. Returns distinguish failures and low-clearance passes before ranking obstacle-zone speed. The clearance quartile is empirical. Each development fold recomputes normalization and its utility threshold from other layouts.
- `model.json` freezes all numeric commands and fitted scores (file SHA-256 `6a142b122b0b7d3e56dd5e233779ea4f7b31ae7950c836f51fb4bfe198df79b0`). It was frozen before the independent scorer revealed any of the eight segment conditions or the additional whole lap. Runtime solver `788eb7af5abf9f`. In `parity.json` the no-model arm reproduces the preceding `ca509cc0c9c239` physics and command sequence exactly.
- `action_audit.json` checks all selected C# actions and scores against an independent Python reconstruction, each matched request/prefix and the counts and digests of chase, clean RGB and detector streams. `events.jsonl` records every run including failed ones. The training replay videos are marked `dev_t02` and `dev_t03`.
- `h01` to `h08` are independently registered segment pairs, `full01` is the separate complete-lap attempt. Results and full synchronized real frames are held under `runs/<id>.json`, `.work/frames/<id>/{NNNN}c.jpg`, `.work/frames/<id>/{NNNN}f.jpg`, and `.work/camera/<id>/a4_samples`. Chase and unannotated RGB are 20 Hz, A4 decisions/detections 5 Hz.

## Evaluation

Segment completions increased from 6/8 to 7/8, and prespecified safe completions increased from 5/8 to 6/8. The learned model rescued two previous contacts, introduced one new contact in a bend, and kept the 0.061 m near-miss in another scene. It missed the registered acceptance condition. Two phases on the same approach yielded different choices. The retained reference control passed the first narrowly; the learned inner, quicker line cleared the second with 1.268 m while the reference contacted the cone. In the new bend failure the model chose the quicker normal line with the moving cone estimated 1.77 m to its left. The car contacted the cone 2.8 s later.

The complete-lap reference passed all 32 gates in 88.02 s and had a minimum cone clearance of 0.179 m. The learned model passed its first three obstacles, then contacted the last one after 218.03 m and 24 gates. The 63.39 s to contact is an incomplete attempt.

## Reproduction

Use the isolated Unity editor and set AIFORMULA_LAP_COPY to the private project. Run the commands below from unity_laps. For a new replay, give every command the same fresh directory with `--output`. The scorer's original segment and whole-lap configurations are available from commits 4bfe2d7 and 2b647e0 in learned_holdout.

```powershell
python -B learn_encounters.py train
python -B learn_encounters.py fit
python -B learn_encounters.py parity
python -B learn_encounters.py eval t03 t02 --film
python -B learn_encounters.py external --matrix learned_holdout/layouts.json --film
python -B learn_encounters.py external --matrix learned_holdout/full_lap_layout.json --film
python -B learned_audit.py
```
