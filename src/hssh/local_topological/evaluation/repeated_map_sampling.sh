#!/bin/bash

CONFIG=$PWD"/local_topo_hssh.cfg"
LT_CMD=$PWD"/local_topo_hssh"
EVAL_CMD=$PWD"/evaluate_labels"
cd ../../data/maps

DIR=../../data/maps/
MAPS=`ls *.lpm`
LPM_EXT=.lpm
LTM_EXT=.ltm
RES_EXT=_repeat
BEST_EXT=_best.ltm

for map in $MAPS
do
    MAP_NAME=${map%$LPM_EXT}
    MAP_TYPE=`echo $MAP_NAME | tr -d '[:digit:]'`
    echo $MAP_TYPE

    echo "Labeling $MAP_NAME..."

    BEST_MAP=$MAP_NAME$BEST_EXT

    TRUTH_MAP=$MAP_NAME"_truth"$LTM_EXT

    RESULTS_FILE=$MAP_NAME"_results.txt"

    best=-1000000

    for n in `seq $4`
    do
        MAP_RESULT=$MAP_NAME$RES_EXT"_"$n$LTM_EXT
        echo $MAP_RESULT

        $LT_CMD --config-file $CONFIG --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map $MAP_TYPE --save-map $MAP_RESULT --one-shot $map > $MAP_RESULT.log 2>&1
        $EVAL_CMD $TRUTH_MAP $MAP_RESULT 10000 $MAP_NAME"_results.txt"

        logprob=`cat ltm_result.txt`
        if [ "$logprob" -gt "$best" ]
        then
            cp $MAP_RESULT $BEST_MAP
            let best=logprob
        fi
    done

    REPEAT_FILES=`ls | grep $MAP_NAME$RES_EXT"*"`
    tar -czf $MAP_NAME".tar.gz" $RESULTS_FILE $REPEAT_FILES
    rm $REPEAT_FILES
done
