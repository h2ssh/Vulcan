#!/bin/bash

echo "Usage: generate_stability_logs.sh log_dir output_dir truth_map start_rect map_type"

LOG_DIR=$1
OUT_DIR=$2
TRUTH_MAP=$3
START_RECT=$4
MAP_TYPE=$5

# Need to run: state_estimator local_metric_hssh global_metric_hssh local_topo_hssh
LOGS=`ls $LOG_DIR`
for f in $LOGS; do
    LOG=$LOG_DIR$f
    LOG_BASE=`basename -s .log $LOG`
    echo "Processing "$LOG

    ./state_estimator > /dev/null &
    STATE_PID=$!

    ./local_metric_hssh > /dev/null &
    LM_PID=$!

    EVENTS_OUT=$OUT_DIR"/"$LOG_BASE"_raw_events.evt"
    ./local_topo_hssh --mcmc-max-iterations 25 --mcmc-samples-per-iteration 25 --constraint-log-prob -3 --repeat-log-prob -3 --save-events $EVENTS_OUT --map $MAP_TYPE > lth_stability.log &
    LT_PID=$!

    ./global_metric_hssh --initial-map $TRUTH_MAP --initial-rect $START_RECT > /dev/null &
    GM_PID=$!

    EVENT_LOG=$OUT_DIR"/"$LOG_BASE"_topo_events.log"

    lcm-logger -c "HSSH_LOCAL_TOPO_EVENTS|HSSH_LOCAL_POSE|HSSH_GLOBAL_POSE" $EVENT_LOG > /dev/null &
    LOGGER_PID=$!

    lcm-logplayer $LOG > /dev/null

    # Once logplayer finishes, then bring down the rest of the programs
    # Wait for everything to close before getting on to the next log
    kill $STATE_PID
    wait $STATE_PID

    kill $LM_PID
    wait $LM_PID

    kill $LT_PID
    wait $LT_PID

    kill $GM_PID
    wait $GM_PID

    kill $LOGGER_PID
    wait $LOGGER_PID

    echo "Finished "$LOG
done
