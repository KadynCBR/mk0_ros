#!/bin/bash
if [ -n "$1" ]; then
    catmux_create_session $(ros2 pkg prefix mk0_bringup)/share/mk0_bringup/config/simu_bringup.yaml --overwrite $1
else
    catmux_create_session $(ros2 pkg prefix mk0_bringup)/share/mk0_bringup/config/simu_bringup.yaml
fi