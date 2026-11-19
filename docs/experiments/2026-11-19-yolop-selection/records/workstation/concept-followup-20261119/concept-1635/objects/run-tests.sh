#!/bin/bash
set -eu -o pipefail
w=integration/yolop-selection-20261119.0N90OJ/concept-latency-source/vehicle_concept/src
tar -C "$w" -czf - concept_obstacles | sshpass -e ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=yes control@192.168.10.175 'set -e; t=$(mktemp -d /tmp/concept-obs-XXXXXXXX); tar -xzf - -C "$t"; source ~/concept_v6275_ws/activate_concept.bash; export PYTHONPATH="$t/concept_obstacles:$PYTHONPATH"; python3 -m pytest -q "$t/concept_obstacles/test"; date -Is'
