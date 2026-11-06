# Independent closed-loop evaluation

Run the scorer from `codexws/learned_racing/unity_laps`. Every run is from the same simulator and A4 pipeline as its matched reference. Evaluation geometry is used after driving.

| Registered v1 evidence | Frozen timing baseline | Learned selector |
|---|---:|---:|
| Eight short segments completed | 6/8 | 7/8 |
| Segments completed, road margin ≥0.08 m and cone margin ≥0.15 m | 5/8 | 6/8 |
| Free pace over identical non-obstacle distance in the four mutually safe pairs | 3.156 m/s | 3.139 m/s |
| Single new complete-lap attempt | 88.02 s, 32 gates | Contact after 218.03 m, gate 24 |

The three mutually safe obstacle segments were traversed 33.1%, 18.0% and 24.7% faster by duration (h02, h03 and h08 respectively). They cover a single static cone, staggered static cones and a static-moving pair mid-corner. The median paired duration change was −24.7%. The independent safety requirement failed when the learned selector caused a new contact with a moving cone in a bend (h07). The two repairs of earlier baseline contacts (h05 and h06) and one new failure are recorded individually.

At h05 both arms received the same observation at 6.8 s. The learned selector chose a faster inward line. Its measured speed rose from 2.72 to 4.31 m/s before crossing the cone's fixed station, with 1.27 m minimum cone clearance. The rear cleared the expanded gate at 9.51 s. The cone entered the laterally traversed footprint 1.21 s later, or 1.01 s later using the wider union of the body's projected widths throughout gate occupation. The previous controller contacted the cone at 12.21 s. A different phase in h04 produced the same underlying commands in both arms and only 0.061 m clearance.

With natural cruise, h01, h03 and h08 were all safe in both arms. With positive ordinary cruise caps, h04 was below the clearance criterion for both, h05/h06 improved and h07 regressed. The untouched whole-lap baseline had a minimum clearance of 0.179 m. The learned controller increased clearance at the first three cones, then contacted the final moving cone at 63.39 s.

## Inspect or reproduce

`protocol.md`, `full_lap_protocol.md` and the corresponding layout files record the original comparison rules. `training_audit.json` independently reconstructs all 72 v1 training returns and the first-encounter context for all twelve training layouts. `results.json` and `full_lap.json` contain the raw file digests, measured margins, matched-pair effects, action IDs, command changes and a separate reconstruction of the model's scores at every selection.

```powershell
python learned_training_audit.py learned_revision --out learned_holdout/training_audit.json
python learned_holdout_score.py learned_revision/runs --model learned_revision/model.json --out learned_holdout/results.json
python learned_holdout_score.py learned_revision/runs --model learned_revision/model.json --full-lap --out learned_holdout/full_lap.json
```
