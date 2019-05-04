#!/bin/bash

echo "Usage: compare_against_human_maps.sh 'best LTM' 'human map dir' 'results file'"

EVAL_CMD=$PWD"/evaluate_labels"

MAPS=`find $2 -name '*_truth.ltm'`

for map in $MAPS
do
    MAP_NAME=$2$map
    echo $MAP_NAME

    $EVAL_CMD $1 $MAP_NAME 1000 $3
done
