#!/bin/bash

echo "Usage: generate_ppm.sh 'directory'"

cd ../../data/maps/
MAPS=`find $1 -name '*_result.ltm' -or -name '*_truth.ltm'`

for m in $MAPS; do
    echo $m
    $VULCAN_BIN/ltm_to_ppm $m
    base=`basename -s .ltm $m`
    echo $base
    convert $base".ppm" $base".png"
    convert $base"_map.pgm" $base"_map.png"
done

cp *.png ~/writing/thesis/img/
#rm *.png *.ppm *.pgm
