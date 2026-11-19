#!/bin/bash
cd "$HOME/aiformula_runtime/concept-field-1635" || exit 2
export TEST_DIRECT=true TEST_PANEL=true TEST_RVIZ=true INSTALLED_ZED=1 LOSS_TEST=1
export DISPLAY=:0 XAUTHORITY="$HOME/.Xauthority" WINDOW=24
date -Is
bash run.bash full-static 60 24 0 2
status=$?
if [[ $status == 0 ]]; then python3 summarize.py full-static > full-static-summary.json; status=$?; fi
date -Is
echo "$status" > full-static-done
exit "$status"
