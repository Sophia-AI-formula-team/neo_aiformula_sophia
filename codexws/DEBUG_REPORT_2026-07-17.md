# Debug and Deployment Report — 2026-07-17 JST

## Work completed

1. Read `/home/nvidia/Desktop/context.md`, audited the local ROS source, and connected to `control@192.168.0.197`.
2. Replayed `/home/nvidia/Downloads/000750.jpg` on `/aiformula_sensing/zed_node/left_image/undistorted` while the physical camera was unplugged.
3. Confirmed the road detector produced a valid 640 x 360 mask, but the lane publisher produced empty left, center, and right `PointCloud2` messages.
4. Checked the active calibration and TF:
   - intrinsic file was for 640 x 360 nHD output;
   - vehicle frame was `base_footprint`;
   - camera frame was `zed_left_camera_optical_frame`;
   - the transform was valid and projected thousands of finite mask points into the vehicle ROI.
5. Found the actual lane failure: `min_component_pixels = 180` was a 1920-pixel-width threshold applied unchanged to a 640-pixel image. Valid 108-pixel and 74-pixel lane components were rejected.
6. Changed the threshold to scale with image width. At 640 pixels the requirement is 60 pixels. Rebuilt and deployed the remote package.
7. Verified the fix with the same image: left, center, and right clouds each contained 9 finite, non-zero points.
8. Deleted five duplicate `lane_line_publisher` source packages on the remote machine. No local package was deleted.
9. Compared the consolidated remote launcher with the local launch files. Added the missing `lane_points/lane_0215` and `kalman_filter/withoutkalman_0312` nodes. LYA was not added to allnodes and was not edited.
10. Corrected gyro odometry to subscribe to `/aiformula_sensing/vectornav/imu` through a configurable `imu_topic` launch argument.
11. Played only the image topic from `/media/control/R/OWEN/rosbag2_2026_06_29-17_48_44`. Started the unchanged LYA script separately. It received `/filtered_lane_pose`, published non-zero speed and steering commands, and CAN odometry increased by about 97 m. Playback was stopped on user request.
12. Diagnosed RViz failure. The profile contained three Ogre Image displays and stale 1600 x 900 geometry on a 1024 x 768 Jetson display, producing `GLXBadDrawable`.
13. Replaced the remote RViz profile with PointCloud2 and Grid displays only. Moved `vehicle_fit_image` to a separate `rqt_image_view`. Set `DISPLAY=:0` and `XAUTHORITY=/home/control/.Xauthority` explicitly for both GUI processes.
14. Validated RViz and `rqt_image_view` on the remote desktop without starting vehicle control. Both opened correctly; RViz reported OpenGL 4.6 and no GLX/Ogre/heap error.
15. Investigated the remote reboot at about 13:44 JST. The new boot reported `PMC reset source: SYS_RESET_N`. There was no retained previous-boot journal or pstore record and no evidence of OOM, panic, NVIDIA Xid, thermal trip, or watchdog reset. The exact cause is unproven; available evidence indicates a hard reset, power interruption, or reset-line event.

## Current remote state

- ROS nodes: stopped
- rosbag playback: stopped
- LYA: stopped
- RViz and `rqt_image_view`: stopped after validation
- motor and SocketCAN ROS processes: stopped
- physical camera: unplugged

## Current remote hashes

- corrected `lane_line_publisher.cpp`: `87a7748b3c363ec144f644dccfabd0f95a7befc599a45efeb3a2a0eaa574b93b`
- deployed lane executable: `7e3cfd2adaf8a8339b347d5948136a8db033df01e722c2b7e8745afc401bfd31`
- `allnodes.launch.py`: `f97159b6d6d0cb7a9dea731bced6f9c6495aa53a3e9e937da9521cfbdb33a6b4`
- data-only `2x2.rviz`: `ec0e1880b8870e33231bc848076f8a91854ae4dfcb0c85efb52209bfb207a39c`
- gyro odometry launch: `276d35667c18d0b7fc50287d51004e17bf9e42042491318a7c1475129bfe7f88`

## Local scope

The local ROS workspace was not modified. Local patches, evidence, and rollback files are under `/home/nvidia/Desktop/codexws`.
