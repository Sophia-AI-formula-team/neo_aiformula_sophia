# YOLOP A2 vehicle integration

- Read `handoff.md`, `context.md`, and the repository-root `docs/agent_handoff_20261005.md` before continuing.
- Preserve the original runtime/model Python files and both checkpoint bytes from commit `06fa28bdc09c731249428b7d81850d23e4e216a3`. Original hashes are recorded in the repository-root `docs/yolop_a2_upstream_snapshot_20261005.json`.
- The upstream package metadata describes the Python 3.10+ development profile. The Jetson integration is separate: Python 3.8 (for ROS2 Foxy), NVIDIA PyTorch 2.0.0+nv23.05, TorchVision 0.15.1 built for CUDA 11.4, and NumPy 1.24.4, all in `/home/control/aiformula_runtime/yolop_a2_20261005/venv`.
- Do not run `pip install -e .` for that Jetson profile: its upstream requirements would select a different stack. The separate launch file adds this runtime and ROS wrapper to `PYTHONPATH`.
- Keep `torch.load(..., weights_only=True)` and the original checkpoint validation: object strict loading and the lane loader's missing-key rejection. Do not silently fall back to unrestricted checkpoint loading.
- Do not change system Python, system CUDA, the old torch installation, original lane algorithms, or semantic package contents for deployment compatibility.
- Use `ros2 launch launchers yolop_a2.launch.py` for the independent detector. Avoid duplicate lane-mask or `/cone_*` publishers. Traffic-light outputs are isolated by default.
- This task requires live-camera acceptance and manually started obstacle avoidance. Do not run bag smoke tests. Record actual results and any unverified stage in the dated root handoff.
- On 2026-10-05, live camera-to-mask output and object-image publication passed on this Jetson. Physical obstacle-triggered B-spline acceptance is still pending; the indoor bench scene had no obstacle inside the avoidance ROI. See the root handoff and `docs/validation_20261005` evidence rather than assuming full acceptance.
- The independent OA launch/stop wrapper was tested on Foxy after fixing its explicit executor context. OA is stopped at handoff; allnodes and A2 remain running. Read the root handoff before starting duplicate processes.
