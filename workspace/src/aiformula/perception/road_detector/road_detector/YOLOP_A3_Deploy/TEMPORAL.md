# A4 Calibrated Temporal Input

This interface is packaged, but the bundled image-only ROS node does not call it.
Unity comparisons supplied accurate camera geometry; real-camera integration
remains unfinished. A4 renames a3-v76 without enabling temporal inference.
See [MODE_GUIDE_ZH.md](MODE_GUIDE_ZH.md) for the mode and release explanation.

## Python API

```python
from yolop_a3_runtime import PerceptionModel

model = PerceptionModel("/absolute/path/weights/shared_best.pth", device="cuda")
output = model.predict_frame(
    bgr_image,                        # uint8 HxWx3, same undistorted camera
    sequence="left-camera/calib-01",   # change after stream/calibration/plane reset
    timestamp=capture_seconds,        # image capture time, not arrival time
    plane_to_image=projection,        # finite, invertible metric 3x3 matrix
)
print(output.temporal_status)
# Start a new recording or seek:
model.reset()
```

One instance per camera stream, sequential calls only. Missing/invalid geometry
is an error in predict_frame. For intentionally independent images call
predict(image), which reports single_frame and clears history.

Status values: warmup, aligned, sequence_reset, timestamp_reset, shape_reset,
single_frame. aligned means the calibrated warp was used; pixels outside its
valid ground projection retain their current probability. It is not a score
for calibration quality. Consecutive timestamps must satisfy 0 < dt <= 0.25 s;
otherwise the current image is returned without history fusion.

## Meaning Of The Matrix

Define one fixed ground-plane coordinate system in meters for the whole sequence.
For plane coordinates (X,Y), the matrix G must satisfy

```text
G @ [X, Y, 1] = [u * Z_camera, v * Z_camera, Z_camera]
```

u,v are native undistorted pixel-center coordinates: top-left pixel=(0,0).
Z_camera is positive optical-axis depth in meters. With calibrated intrinsics K
and plane-to-optical-camera rigid transform [R|t], G=K@[R[:,0],R[:,1],t].
The pose must describe the camera at the image's capture time, including its
extrinsic offset from the odometry origin. Do not pass a ROS quaternion, a
4x4 pose or a world-space Unity position directly.

G is **not an arbitrary-scale homography**. Do not normalize G by G[2,2]:
the third row is used to reject invalid depths. The native frame dimensions,
image undistortion, intrinsics and calibration epoch must agree.
Supported projected depths use the audited 0.05 to 5000 meter clip interval.

The current-to-previous sampling map is G_previous @ inverse(G_current).
It samples only the preceding raw probability and excludes out-of-image,
behind-camera and clipped points. The learned gate blends current/warped
probabilities pixel by pixel. It cannot recover lanes neither frame predicted.
The object head continues to run only on the current image.

## Recorded Sequence Demo

```sh
python "$A3/tools/demo_sequence.py" --manifest /absolute/path/clip.json \
  --output /absolute/path/new-v76-sequence --device cuda
```

Manifest format (paths relative to the manifest):
```json
{"frames": [
  {"image": "frame000.png", "sequence": "left/calib-01", "timestamp": 0.0,
   "plane_to_image": [[514.0,636.0,843.0],[0.0,304.0,1023.0],[0.0,0.995,1.319]]}
]}
```
The numbers illustrate the schema, not your camera's calibration. Use measured
projections for actual data. The command writes separate lane/object masks,
combined overlays and metadata for every frame without overwriting an old run.

## ROS Integration

The bundled ROS wrapper intentionally accepts only images and logs single-frame
mode. No synchronized calibrated projection producer exists in this package.
Integrate the Python API in the node that already owns the calibrated camera pose,
or provide a separately reviewed synchronized adapter. Do not assume that merely
subscribing to odometry makes its axes, timing or ground plane correct.
