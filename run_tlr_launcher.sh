#!/bin/bash
source /home/sws/yolo_ws/install/setup.bash
ros2 launch tlr_detector tlr.launcher.py "$@"