#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p evidence/launch evidence/nodes evidence/recordings
now=$(date -Is)
printf '%s\n' "$now" > evidence/read-at.txt
scan='map_assist|semantic_045|semantic_vehicle|flag_stop|motor_gate_session|First lap|Topology alignment|oa_2|allnodes|trajectory_follower|yolop_a3|socket_can_sender|motor_controller|lane_line_publisher'
python3 - "$PWD/evidence" <<'PY'
from datetime import datetime
from pathlib import Path
import re, shutil, sys

dest = Path(sys.argv[1])
ros = Path.home()/'.ros/log'
related = re.compile(r'map_assist|semantic_045|semantic_vehicle|flag_stop|motor_gate_session|First lap|Topology alignment|oa_2|allnodes|trajectory_follower|yolop_a3|socket_can_sender|motor_controller|lane_line_publisher', re.I)
pids = set()
for p in sorted(ros.glob('2026-11-19-16*/launch.log')):
    text = p.read_text(errors='replace')
    if not related.search(text):
        continue
    (dest/'launch'/(p.parent.name+'-launch.log')).write_text(text)
    for line in text.splitlines():
        if related.search(line):
            pids.update(re.findall(r'pid \[(\d+)\]',line))
log_index=[]
for p in sorted(ros.glob('*.log')):
    if datetime.fromtimestamp(p.stat().st_mtime).astimezone().strftime('%Y-%m-%d %H') != '2026-11-19 16':
        continue
    try:
        text = p.read_text(errors='replace')
    except OSError:
        continue
    actual_node = re.search(r'Map assist:|First lap complete:|Topology alignment|No fresh pixel-stop output|motor_gate_session', text, re.I)
    if not actual_node and not any('_'+pid+'_' in p.name for pid in pids):
        continue
    log_index.append(f'{p.name}\t{p.stat().st_size}')
    if 0 < p.stat().st_size < 300000:
        shutil.copyfile(p,dest/'nodes'/p.name)
    elif actual_node:
        lines = [l for l in text.splitlines() if related.search(l)]
        (dest/'nodes'/p.name).write_text('\n'.join(lines[:400])+'\n')
(dest/'node-log-index.txt').write_text('\n'.join(log_index)+'\n')
PY
ps -eo pid,ppid,pgid,tty,stat,comm,args | grep -E 'ros2 (launch|run|bag)|map_assist|motor_gate_session|guarded_stop_wait|pixel_stop_flag|run_a3.py|oa_2' | grep -v '[g]rep' > evidence/live-processes.txt || true
if command -v tmux >/dev/null; then
    tmux list-panes -a -F '#{pane_id} #{pane_pid} #{pane_current_command}' > evidence/tmux-index.txt 2>/dev/null || true
    while read -r pane _; do
        if [[ -n "$pane" ]]; then
            log=$(tmux capture-pane -p -S -1800 -t "$pane" 2>/dev/null || true)
            if grep -Eqi "$scan" <<< "$log"; then
                printf '%s\n' "$log" > "evidence/tmux-${pane#%}.txt"
            fi
        fi
    done < evidence/tmux-index.txt
fi
source ~/semantic_045_ws/activate_semantic.bash
# Match the launch DDS environment when relevant processes still exist.
for proc in /proc/[0-9]*/cmdline; do
    if [[ -r "$proc" ]] && tr '\0' '\n' < "$proc" 2>/dev/null | grep -Eq '(/map_assist$|/motor_gate_session$|/motor_controller$|/pixel_stop_flag\.py$)'; then
        if [[ -r "${proc%/*}/environ" ]]; then
            while IFS= read -r item; do
                case "$item" in
                    ROS_DOMAIN_ID=*|ROS_LOCALHOST_ONLY=*|FASTRTPS_DEFAULT_PROFILES_FILE=*|RMW_IMPLEMENTATION=*) export "$item" ;;
                esac
            done < <(tr '\0' '\n' < "${proc%/*}/environ" 2>/dev/null)
        fi
        break
    fi
done
timeout 8 python3 capture_semantic.py --seconds 4 > evidence/live-topics.json 2> evidence/live-topics.err || true
find "$HOME/rosbags" -maxdepth 4 -type f -name metadata.yaml -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -15 > evidence/bags-index.txt || true
while IFS= read -r -d '' meta; do
    if grep -Eq '/semantic_drive/(map_assist_045|map_status|motor_session)|/flag_stop|/aiformula_control/motor_controller/gate_status' "$meta"; then
        bag="${meta%/*}"
        cp "$meta" "evidence/recordings/${bag##*/}-metadata.yaml"
        for db in "$bag"/*.db3; do
            if [[ -f "$db" ]]; then
                timeout 15 python3 capture_semantic.py --bag "$db" > "evidence/recordings/${db##*/}.json" 2>"evidence/recordings/${db##*/}.err" || true
            fi
        done
    fi
done < <(find "$HOME/rosbags" -maxdepth 4 -type f -name metadata.yaml -newermt '2026-11-19 15:55:00 +0900' -print0 2>/dev/null)
date -Is >> evidence/read-at.txt
