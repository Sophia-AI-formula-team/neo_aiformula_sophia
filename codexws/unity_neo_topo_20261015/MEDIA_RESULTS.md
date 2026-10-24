# Media execution results — 2026-10-15

The earlier README and driver document describe the frozen preparation stage. This additional record supersedes their untested status without changing scripts referenced by completed render plans. Root granted the exclusive media CPU window after Unity returned to clean Edit mode and model workers exited.

Five recordings were encoded once each with the reviewed v03 builder and the fixed Windows Job supervisor. Every attempt completed inside its explicit 240-second primary bound, passed a complete H.264 decode, checked every frame index and rational PTS, matched all before/after input hashes, and ended with Job ActiveProcesses=0 and independently checked known process identities. No encoder retry or additional source-code change was needed. The longest primary operation was 30.22 seconds; each cleanup took less than 1 millisecond.

| Recording | Recorded acceptance | Video duration | Frames |
| --- | --- | ---: | ---: |
| inner_traffic_03 | true | 13.667 s | 205 |
| outer_traffic_03 | true | 12.800 s | 192 |
| inner_recovery_03 | true | 49.667 s | 745 |
| outer_recovery_03 | true | 51.000 s | 765 |
| outer_closure_02 | false | 38.800 s | 582 |

The final four recordings pin MapAssist 0.3.1, neo commit `3c6fbac2ff9f8dd23b247a187e966648876400b1`. The historical outer closure failure retains its original MapSHA `16e2034b124a` and literal integration receipts; the parent run identifies it as the old 0.3.0 case. Its footer explicitly shows accepted=false, topo closed=true, the original closure-complete reason, and checkpoint `save_failed / negative_progress`. No result or reason was rewritten.

Video layout is 1280×960 at 15 fps. The top 1280×720 observer view remains unobscured. The bottom panels show genuine recorded A3 overlay with the existing RGB signal helper, original LLP/LYA points, semantic stop gating, and semantic closure/persistence. Longer learning intervals are 4x; event windows and the red/green traffic recordings are 1x. Whole camera extents are covered continuously, including the final sample.

The media agent inspected all event contact sheets and selected full-resolution stop, green movement, restoration and failure frames. Root independently inspected key full-resolution frames. The inner recovery 4x sample at source wall 29.05 seconds correctly retains “LLP UNSUPPORTED + LYA / ABC=3 fallback”; it does not convert a fallback into an observed triplet. Final frozen/off states show actual post-test cleanup, with recorded acceptance separately labeled. Official OSM fusion remains visibly unavailable (`rmse_exceeds_limit`) after semantic closure; this is not represented as SLAM success.

[final_media_receipts.json](evidence/media/final_media_receipts.json) is the compact review index: five absolute video paths, SHA256, duration/frame count, machine proof, source equality, cleanup and sampled visual QA. [stable_media_package_files.json](evidence/media/stable_media_package_files.json) lists stable files and their hashes for packaging. Video/image evidence should be kept as artifact or release assets if ordinary Git history excludes large media. Original source recordings and complete machine outputs are unchanged.

Evidence limits remain: synthetic Unity odometry, semantic worker restarts while Unity/detector/follower/clock stay alive, inner component-only LYA recovery versus outer baseline, and held-stop/green-release traffic acceptance. These are not Editor/OS reboot, native ROS/DDS, physical-vehicle or moving-red braking-distance validation. No bbox-distance experiment was mixed into these videos.
