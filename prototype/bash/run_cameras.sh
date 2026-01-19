#!/bin/bash

xterm -title "cam_d457" -e "bash -c 'pixi run ./scripts/d457_high.py; exec bash'" &
sleep 5s

xterm -title "cam_wrist" -e "bash -c 'bash bash/run_uvc_cameras.sh; exec bash'" &
sleep 5s

xterm  -title "rviz2" -e "bash -c 'pixi run rviz2; exec bash'" &
wait
