# Jetson YOLOP selection experiment · 2026-11-19 JST

## 当日全部实验资料

[下载整日报告 ZIP](2026-11-19-field-report.zip) · [ZIP 校验值](records/report-zip.sha256)

压缩包收录发布快照中的 1,778 个文件，含完整时间线、两段对话、图文报告、ROS 日志和每场录包的下载入口。解压后打开顶层 README.md。

[完整时间线](TIMELINE.md) · [现场问题与修复](ISSUES.md) · [两段对话全文](conversations/) · [原始 ROS 日志](logs/) · [照片与图表](images/) · [车上活动代码和 Neo 差异](active-code/) · [所有文件与 SHA-256](records/)

17 份录包共有 1,993,511 条消息。逐场可下载原始数据库、元数据及 SQLite 辅助文件。[Semantic / MapAssist（3 份）](bag/semantic/) · [Concept（12 份）](bag/concept/) · [其他（2 份）](bag/other/) · [Release 完整附件](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/releases/tag/field-archive-2026-11-19)

以下为实验进行中留下的阶段记录。

今日全部测试按 YOLOP、Concept 与 Semantic 分类。见 [验收索引](TEST_INDEX.md) 和 [当天实验 ZIP](2026-11-19-experiment-report.zip)。

This is a reversible local experiment for A3 V14, A4 and A4Light in the Neo vehicle workspace. Neo repository ID: 1303517209. Repository main inspected at `d8277bd2b4ce95bcab2883e9b68b011749c0d9ad`. The experiment follows the A4 package AGENTS.md, DEPLOYMENT.md, JETSON.md and the A4 identity handoff in docs/agent-context.

## Run

Type `yolop` in a terminal, or open **YOLOP Perception** from the desktop or Applications menu. The terminal shortcut links `~/.local/bin/yolop` to the existing `yolop-launch` selector. Choose 1 (A3 V14), 2 (A4), or 3 (A4Light). For explicit selection, pass `a3`, `a4`, or `a4l`. Use Ctrl-C to stop a selection before starting another. The selector subscribes to the existing camera stream and holds a per-ROS-domain process lock. A3 V14, A4 and A4Light publish on the vehicle perception topics. A4 and A4Light use the upstream `--live` routing and duplicate publisher check with single-frame camera input. A4Light retains its lean runtime. The desktop and application menu entries open the selector in a terminal. The selector starts only the selected detector.

`allnodes.launch.py` and `allnodes_2.launch.py` now default `launch_yolop_a3` to false on the local machine. Consequently an unqualified allnodes start supplies no YOLOP lane mask until a YOLOP profile is selected. The existing explicit opt-in remains available.

## Sources and local changes

- A3 V14 stays in its existing runtime with online entrypoint and weights intact. Checkpoint SHA-256 `a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c`.
- A4 release tag `a4` resolves to `9369bbabf097c405d27e15f738d70e72f917a3a0`. Both selectable bundles were built from the vehicle packaging branch `perf/a4-lean-eager` at `af9d7386112f1cec3546eacffe72fa0968767836`. A4Light uses its `A4_lite` profile and lean eager mode. Both use the same A4 checkpoint, SHA-256 `b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3`, aligned step 1024, FP32 min 640 / max 1152, raw lane probability threshold 0.5.
- Local artifacts are under `~/aiformula_runtime/2026-11-19-yolop-selection`. The original pinned package is in `source/`, reference ZIPs in `archives/`, and runnable copies in `bundles/`. Foxy uses Python 3.8.10 on L4T R35.4.1. The experiment venv reuses the machine's existing NVIDIA Torch 2.0.0+nv23.05, TorchVision 0.15.1, NumPy 1.24.4 and system ROS/OpenCV, with a private copy of typing_extensions. The runnable copies postpone annotation evaluation in `outputs.py`, and `run_vehicle.py` permits Python 3.8 only through the explicit local experiment opt-in. Model and inference logic are unchanged.
- Two workspace source files changed by one literal each. Their installed paths already symlink to source. Original SHA-256 values are `1f7ce11d4d1b8e5e08d3401c4f768b4ec056f101d4ab48af8783b27b3c1422da` and `70c305fafd3a62e2a61c6ac1096279af66e70babb3ab1454c63cc64bff7b72e6`. Updated hashes are `33ba94b9bd20e759138eb5038ac1121bb5fd0f4daf361170da8029b4ad09b9b3` and `d4fbf4bbf6754c8441830e2ffbae2ad0923b91bd11fc01abc2bb6fc2e0f8142a`, respectively.

## Timeline and verification

| Time JST | Event |
| --- | --- |
| 09:34–09:36 | Audited Neo and the installed launchers, pinned A4 and lean sources, created the local experiment directory and drafted the selector. |
| 09:36–09:38 | Fetched and verified package files by Git blob SHA and the checkpoint by SHA-256. Built both upstream profile ZIPs. Created a separate Python venv. Missing dependency imports were resolved with existing libraries and the local annotation compatibility edit. Sandbox CUDA initialization failed with error 801. The host invocation passed offline CUDA inference for both profiles on one 640×360 synthetic image, with matching lane and obstacle mask bytes. |
| 09:38 | Installed the selector symlink. GitHub refused creation of `experiment/yolop-selection-20261119` with HTTP 403. The available GitHub CLI has no login. Cancelled the redundant full Git download after the verified package was available. |
| 09:38:47 | Saved byte-exact backups and set both local A3 defaults to false. |
| 09:38–09:39 | Both A4 selections reached single-frame ROS node startup in an isolated localhost domain and received bounded shutdown. Launch logs record the selected profile, checkpoint hash and shadow routes. Both upstream weight checks passed for identity, schema, tensor finiteness, and source contract. |
| 09:39:20 | Rolled back both defaults, compared both files byte for byte against backup, then reapplied. ROS `--show-args` confirmed both installed defaults are false. |
| 09:39–09:40 | First test attempt collected the pristine source and hit four Python 3.8 annotation errors. Repeated against a separate test copy containing the deployed compatibility edits: 47 passed in 9.64 s. |
| 09:46:25 | Switched A4 to the seven vehicle perception topics, added a selector lock per ROS domain, and installed terminal desktop and Applications menu shortcuts. Saved the prior selector in `before-live/`. |
| 09:47 | Isolated DDS test injected synthetic 640×360 BGR camera frames and received lane mask, lane overlay, cone mask, object overlay, detections JSON, traffic light state, and red pixel count on their existing vehicle topics. Confirmed one publisher on each route, matching image stamps and frame IDs, expected image encodings, no shadow routes in this run, and second selector rejection. Desktop entry validation passed. GitHub branch creation retry returned HTTP 403. |

The camera and vehicle consumers were represented by an isolated DDS probe. Real-camera performance, temporal calibration, representative bag timing, and vehicle motion remain untested.

## Rollback

1. Stop the selected launcher with Ctrl-C. To undo only the A4Light routing change, restore `yolop.launch` from `before-a4light-live/yolop.launch` with `cp -p`.
2. To undo only the A4 routing and shortcut follow-up, restore `yolop.launch` from `before-live/yolop.launch` with `cp -p`. Remove the two installed desktop entry copies if they still match the experiment's `YOLOP.desktop`. Their paths are `$(xdg-user-dir DESKTOP)/YOLOP.desktop` and `~/.local/share/applications/aiformula-yolop.desktop`.
3. To undo the local launcher defaults as well, from this machine run `python3 ~/aiformula_runtime/2026-11-19-yolop-selection/vehicle_defaults.py rollback`. The script restores the exact originals from `before/` and refuses to overwrite later unrelated edits.
4. Verify `~/.local/bin/yolop` links to `yolop-launch` before removing the shortcut with `unlink ~/.local/bin/yolop`. Verify `~/.local/bin/yolop-launch` points to this experiment, then remove that symlink with `unlink ~/.local/bin/yolop-launch`. Archive the dated local folder to remove the isolated profiles from use while keeping the record.

## Terminal shortcut follow-up

The selector shortcut was installed and verified at 09:55:54 JST. To recreate the link in the same layout, run `ln -s yolop-launch ~/.local/bin/yolop`. The verification opened the three-choice prompt with stdin closed and confirmed exit before detector selection. See the [timestamped change log](evidence/terminal-shortcut.log) for the initial interpretation, correction, and checks.

## A4Light vehicle topics follow-up

At 09:59:06 JST the Jetson selector was updated to pass `--live` for both A4 profiles. A4Light now uses the same camera input and seven vehicle outputs as A4. The running A4 process remained active. To switch, stop the running A4 from its terminal, then use `yolop` and select 3.

The bundle identity and selector argument checks passed on the Jetson. At 09:59:58 JST a separate localhost ROS Jazzy DDS probe passed with the deployed A4Light ROS node and stubbed inference. The probe received all seven expected outputs from a synthetic image and checked publisher counts, image headers, dimensions, encodings, and absence of shadow topics. The updated routing has yet to be exercised with A4Light inference on the Jetson.

See the [step-by-step timestamps](evidence/a4light-wiring.log) and [DDS topic output](evidence/a4light-dds-topics.log).

## Archived files

- [Selector](yolop.launch), [desktop entry](YOLOP.desktop), and [pre-follow-up selector](before-live/yolop.launch).
- [Launcher default diff](local-defaults.diff) and [apply/rollback helper](vehicle_defaults.py). These describe the deployed workspace files on the Jetson.
- [Foxy compatibility patch](foxy-compatibility.patch) against the pinned A4 package. The two edited files are identical across the deployed A4 and A4Light bundles.
- [CUDA smoke](smoke.sh), [ROS startup smoke](smoke_ros.sh), [isolated DDS smoke](smoke_live.sh), and [topic probe](verify_live_routes.py). The scripts expect the installed runtime layout described above.
- [Timeline](evidence/timeline.log), [A4 startup](evidence/a4.ros.log), [A4Light startup](evidence/a4l.ros.log), [A4 vehicle-topic startup](evidence/a4.vehicle-topics.log), and [duplicate selection rejection](evidence/duplicate-selection.log). The earlier startup logs show the shadow routes from before the 09:46 follow-up. Home paths in the published logs are shortened to `~/`.

The archive is on branch `experiment/yolop-selection-20261119` in Neo. The vehicle workspace launch changes are recorded as an experiment diff. The original weights, bundles, and local environment remain on the Jetson.

## 10-second Jetson benchmark and daily report

The 2026-11-19 [illustrated report](gpu-load-10s/README.md) compares A3 V14, A4 and A4Light using the live ZED camera. It includes GPU and CPU load, memory and thermals, object outputs and frame timing. The [ZIP package](2026-11-19-yolop-report.zip) contains the report, charts, timestamped samples and scripts.
