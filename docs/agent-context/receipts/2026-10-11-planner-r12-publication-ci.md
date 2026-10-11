# R12 publication and CI readback

R12 is a development failure checkpoint, not a lap or vehicle release.
Canonical repository: Sophia-AI-formula-team/neo_aiformula_sophia,
immutable repository ID 1303517209. The existing repair branch
`repair/lane-keyframe-transaction-20261011` and annotated tag
`lane-mapping-20261011-r12-rc1` resolve to commit
`37119203ff84f1fcda71222b292d1edd3756008b`.
Unsigned annotated tag object: `950d755bb57803ee7c5a4c6dd324fb6ed01854fb`.
No main promotion, force push, legacy repository change or PR was performed.

All eight push-triggered workflows and their sixteen jobs completed success:

| Workflow | Branch run | Tag run |
| --- | --- | --- |
| Native | 38120338208 | 38120338239 |
| Lane Foxy | 38120338144 | 38120338227 |
| GNSS Foxy | 38120338175 | 38120338221 |
| Handoff | 38120338148 | 38120338230 |

Run URL prefix: https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/
This readback checks workflow/job conclusions; it does not recount remote test
cases from downloaded XML. Full local receipt `r12_publication_readback_01.json`
has SHA256 `25d24b416729c975088021acfc8258dc5b629a6d1634768c611e7e8a873c321b`.
The sealed session010 and its bundle are unchanged.

Subsequent uncommitted R13 exact index/local-packaging changes reached 105
accepted recorded-simulation masks (seq151..255). Seq256 was rejected for
insufficient overlap, engine58.83ms under the unchanged80ms gate. Fresh-past
overlap0.5008319468 remains below0.55; active-past overlap0.4209650582.
Current foreground mass50.0833333333 includes14.1 uncertified and1.0 missing.
No rejected frame commits. This is not an actual new Unity drive or a completed
lap. R13 report SHA256:
`6a50ee7a6811d6d166f51d3eb407771b30b5de16cbbd36f272dfc9f29d1af4bb`.

Geometry repair and two actual consecutive laps with unchanged plant settings
remain required. CI success does not override the documented geometry failures.
