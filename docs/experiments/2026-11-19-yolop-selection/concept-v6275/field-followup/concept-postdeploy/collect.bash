#!/usr/bin/env bash
set -e
source ~/concept_v6275_ws/activate_concept.bash
base=~/aiformula_runtime/concept-field-20261119/postdeploy-audit
cd "$base"
for bag in "$HOME"/rosbags/concept/concept_20261119_15*; do
    test -s "$bag/metadata.yaml" || continue
    label=${bag##*/}
    for db in "$bag"/*.db3; do
        if [ -r "$db" ]; then
            python3 summarize_run.py "$db" > "$label-${db##*/}.json"
        fi
    done
done
for log in "$HOME"/.ros/log/2026-11-19-15*/launch.log; do
    [ -r "$log" ] || continue
    if grep -qF '[drive-' "$log"; then
        name=${log%/*}
        cp "$log" "${name##*/}-launch.log"
    fi
done
date -Is > queried.txt
