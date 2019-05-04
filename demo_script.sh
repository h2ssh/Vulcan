#!/bin/bash

cd build/bin
pkill local_metric_hs
pkill state_estimator
pkill object_tracker
pkill motion_controll
pkill metric_planner
pkill debug_ui

xterm -e ./global_metric_hssh --emulate-lpm &
xterm -e ./state_estimator &
xterm -e ./object_tracker &
xterm -e ./motion_controller &
xterm -e ./metric_planner &
xterm -e ./debug_ui
