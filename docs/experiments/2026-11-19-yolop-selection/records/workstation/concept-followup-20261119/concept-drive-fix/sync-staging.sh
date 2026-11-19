#!/usr/bin/env bash
set -eu -o pipefail
repo=/home/nanato/aiformulaws/integration/yolop-selection-20261119.0N90OJ/concept-latency-source
dest=aiformula_runtime/concept-drive-fix-20261119-1603
cd "$repo"
git ls-files -co --exclude-standard -z -- vehicle_concept/src | tar --null -T - -czf - | sshpass -e ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=yes control@192.168.10.175 "mkdir -p ~/$dest/srcstage && tar xzf - -C ~/$dest/srcstage"
