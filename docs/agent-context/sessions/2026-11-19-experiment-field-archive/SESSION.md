# Experiment archive handoff, 2026-11-19

## Objective and scope

Publish the field day already captured on SSD, grouped by recording system, with the original ROS logs, source and deployment records, screenshots, code differences, and both experiment conversations. The immutable repository ID is 1303517209.

## Input handoff

The operator requested the field archive in both the dated experiment section and the project conversation context. The handoff ID is 2026-11-19-user-field-archive. The source archive includes vehicle and workstation evidence, transfer manifests and the final effective-code audit.

## Source and working tree

The experiment checkout used commit 28078a63aea69c7873d1cf7fe7e8ab1c9365da5b. Its parent dbb95d976038b0538d4e6b8704154dc43b783d02 was the published head. Work proceeded on a separate checkout. An existing working-copy ZIP containing the afternoon Concept fix was reviewed and included. Runtime packages were left in their recorded state.

## Work performed

The operator's archive request arrived at 19:47 JST. The source inventory and repository identity were checked at 19:49. The two visible conversations were read through their complete histories. At 19:55–19:56, SQLite message windows, topic counts and checksums identified 3 Semantic, 12 Concept and 2 other recordings. At 19:58, vehicle, workstation, deployment and image attachments were assembled. The bag retrieval tool restored all 17 recordings; the compressed and member bytes were checked independently against the inventory at 20:05 JST. The full record and release-verification receipt are linked from the dated experiment timeline.

## Evidence and result

The inventory covers 7,802 archived files, including 7,222 from the vehicle and 926 original ROS log files. All 17 main SQLite databases passed quick_check and matched their original metadata, totaling 1,993,511 messages. One recording contains zero messages. The external artifact list includes all 23 downloads and their SHA-256. All archive members were checked against the SSD manifest or the previously verified active-code tarball. The original datasets retain recorded GNSS and field imagery as requested. Credential values were removed from the published conversations.

## Risks and blockers

The 17:03 auxiliary recording overlaps a Concept launch and contains only the vehicle and sensor topics. Five late red-wait records reached the Semantic legacy status in one recording; the same bag contains 15 corresponding motor gate reports. Absolute obstacle distance and full-lap closure remain subject to the evidence limits documented in the dated report. Some historical SSH sessions lost connection and some launch logs are empty. No new hardware observations were made while publishing.

## Next agent actions

To reproduce the evidence, use the release asset list and the category downloader in the dated section. Recompute the listed SHA-256 values and inspect the SQLite topic counts or the complete ROS launch logs. The branch receipt records GitHub's published commit, tag and downloaded byte verification.
