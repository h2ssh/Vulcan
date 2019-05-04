#!/bin/bash

echo "Usage: compare_human_maps.sh 'human map dir' 'final results'"

EVAL_CMD=$PWD"/evaluate_labels"

MAPS=`ls $1 | grep '_truth.ltm'`

for map in $MAPS; do
    MAP_NAME=$1$map
    RESULT_NAME=$1$map"_compare_results.txt"
    rm $RESULT_NAME
    echo $MAP_NAME

    for other in $MAPS; do
        if [ $other != $map ]; then
            OTHER_NAME=$1$other
            $EVAL_CMD $MAP_NAME $OTHER_NAME 1000 $RESULT_NAME
        fi
    done

    cat $RESULT_NAME >> $2
done
