# YOLOP A2 isolated Jetson environment — 2026-10-05

This records the environment built on the vehicle and its dependency/model
preflight. Live-camera and obstacle-avoidance acceptance are recorded separately
in [the integration handoff](agent_handoff_20261005.md). A successful dependency
preflight does not establish live acceptance.

## Platform and preserved software

| Component | Vehicle profile |
| --- | --- |
| Hardware | NVIDIA Jetson AGX Orin, aarch64 |
| OS / Jetson Linux | Ubuntu 20.04 / L4T 35.4.1 |
| CUDA | 11.4, existing `/usr/local/cuda` |
| ROS | ROS 2 Foxy |
| Python | 3.8.10, matching the existing ROS Python ABI |
| Private root | `/home/control/aiformula_runtime/yolop_a2_20261005` |
| Interpreter | Private root + `/venv/bin/python3` |
| Model/runtime source | `/home/workspace/src/aiformula/perception/road_detector/road_detector/YOLOP_A2_Deploy` |

The original system Python, CUDA libraries, and prior user-site PyTorch were
retained. The environment was created with
`python3 -m venv --system-site-packages --without-pip`. Its Python uses the
existing pip module to install packages into the venv. Packages outside the
venv were not uninstalled. Because system site packages are visible, this is an
isolated dependency override, not a hermetic copy of every ROS dependency.

The upstream deployment publication is commit
`06fa28bdc09c731249428b7d81850d23e4e216a3`. Its original runtime/model Python
files and both checkpoint files were retained byte-for-byte. Their source
snapshot is [yolop_a2_upstream_snapshot_20261005.json](yolop_a2_upstream_snapshot_20261005.json).
Integration launch files and handoff documentation are separate additions.

| Checkpoint | Bytes | SHA-256 |
| --- | ---: | --- |
| `weights/lane_epoch_282.pth` | 20921608 | `ac2336709034f89b8a7d69d0d7af158e677964699d41cc3520c08d16dc691bfe` |
| `weights/object_detector_usable.pth` | 77867961 | `5ef0574078014c0ac9d97c21539a31435e1c02499d57a944c02e07589a231fe0` |

## Python compatibility exception

The upstream package metadata describes its Python 3.10+ development profile.
This vehicle uses a separately verified Python 3.8 profile to remain compatible
with ROS 2 Foxy. Do not run `pip install -e .` for this profile: the upstream
requirements select a different Python/PyTorch/NumPy stack. The independent
`yolop_a2.launch.py` invokes the venv interpreter and puts the preserved runtime
and ROS wrapper source directories on `PYTHONPATH`.

The previous vehicle PyTorch 1.13 development build did not provide the required
`torch.load(..., weights_only=True)` interface. The new environment preserves
that restricted loader; it does not fall back to unrestricted pickle loading.
The object loader uses `load_state_dict(..., strict=True)`. The unchanged lane
loader remaps/filters compatible checkpoint keys, loads with `strict=False`,
and explicitly rejects any missing model keys. Do not describe the latter as
an upstream call to `strict=True`.

## Installed private dependency versions

| Dependency | Version / source |
| --- | --- |
| PyTorch | NVIDIA `2.0.0+nv23.05`, CPython 3.8 aarch64 wheel |
| TorchVision | `0.15.1`, built locally with CUDA support |
| NumPy | `1.24.4` |
| Pillow | `9.5.0` |
| SymPy | `1.13.3` |
| NetworkX | `3.1` |
| Jinja2 | `3.1.6` |
| filelock | `3.16.1` |
| Ninja | `1.13.2` |
| setuptools | `69.5.1` |

The PyTorch wheel was downloaded from NVIDIA's official archive:
[torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl](https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl).
The saved wheel is under the private root's `downloads` directory. It was
installed using `--no-deps`; the dependency versions above were selected
separately. On this vehicle it reports CUDA 11.4 and an available CUDA device.
The downloaded wheel SHA-256 is
`39eeb9894ef8c7b84249ab917f212a91703f30255d591b956ab12cc10e836532`.

TorchVision was built from the official
[pytorch/vision v0.15.1 source](https://github.com/pytorch/vision/tree/v0.15.1),
commit `42759b1cc82bed60481c2802811595833e2ddd9b`. The resulting local wheel
SHA-256 is
`0482583c88b5d3a9e45e4a6e58cc94b038ab428828e4cc0b44644e4421a43959`.
The source checkout is under the private root at `torchvision-v0.15.1`.

## CUDA header workaround without system installation

The first TorchVision build stopped because CUDA development headers, including
`cusparse.h`, were absent. Matching NVIDIA development-package archives were
downloaded and their headers extracted into the private root's `cuda_headers`
directory. This extracted 13 header files; it did not install OS packages,
replace `/usr/local/cuda`, or replace CUDA runtime libraries.

| Archive package | Version | Archive SHA-256 |
| --- | --- | --- |
| `libcusparse-dev-11-4` | `11.6.0.299-1` | `115a00856b786de1fe533ad537598327de08d9a98358f3c01461db9c2eca5725` |
| `libcublas-dev-11-4` | `11.6.6.84-1` | `8db159ef73ccaa926f327d324704dcea157d734e0a70c3df0e12d6367ca2d48a` |
| `libcusolver-dev-11-4` | `11.2.0.297-1` | `110bb052dc2f4e7d1170a8094488679dccdc25bb140aa63fcc9b63922410308a` |

Exact archive URLs and metadata are retained on the vehicle in
`/home/control/aiformula_runtime/yolop_a2_20261005/cuda_header_sources.json`.
They come from NVIDIA's `https://repo.download.nvidia.com/jetson/common/`
archive. The linked libraries remain those already supplied by the vehicle.

The successful build used these settings:

```sh
export A2_ROOT=/home/control/aiformula_runtime/yolop_a2_20261005
export MAX_JOBS=8
export FORCE_CUDA=1
export TORCH_CUDA_ARCH_LIST=8.7
export BUILD_VERSION=0.15.1
export CUDA_HOME=/usr/local/cuda
export PATH="$A2_ROOT/venv/bin:/usr/local/cuda/bin:$PATH"
export CPATH="$A2_ROOT/cuda_headers"
export NVCC_FLAGS="-I$A2_ROOT/cuda_headers"
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
cd "$A2_ROOT/torchvision-v0.15.1"
"$A2_ROOT/venv/bin/python3" -m pip install --no-build-isolation --no-deps .
```

This is a record of the completed build, not a request to rerun it over the
deployed environment. For a later reproduction, use a fresh private root and
verify the source revision, archives, and resulting CUDA operations before
selecting that environment. The initial failed build log is
`torchvision_build.log`; the successful build log is
`torchvision_build_with_headers.log`, both under the private root.

## Completed device preflight and its limits

The following succeeded on the vehicle in the private environment:

- TorchVision CUDA NMS and ROI Align execution.
- Restricted `weights_only=True` loading of both preserved checkpoints, with
  the model compatibility/completeness checks enforced.
- Inference through the preserved dual-model runtime on one all-black
  `640 × 360` image.
- A separate ROS wrapper build in the private root's `ros_build`, `ros_install`,
  and `ros_log` directories, without replacing the existing workspace install.

The all-black image is a dependency and inference preflight. It does not verify
camera delivery, meaningful lane segmentation, object recognition in the live
scene, latency under allnodes, avoidance selection, or B-spline output. No bag
smoke or bag playback was used for this deployment validation. Consult the
dated integration handoff for the actual live observations.

## Selection and rollback

Use the independent launch command documented in the integration handoff to
select this interpreter. Do not globally activate this venv for the legacy
detector or overwrite the existing ROS executables. The new detector and legacy
detector must not publish the same lane-mask topic simultaneously.

Rollback selects the preserved legacy detector and its prior Python environment
after stopping the new detector; it does not require deleting this private
root, uninstalling packages, or changing CUDA/system Python. Keep the venv,
downloaded archives, source checkout, build products, and logs for diagnosis
and repeatability. Source/launch backup paths and reversible selection steps
are recorded in [the integration handoff](agent_handoff_20261005.md).
