#!/bin/bash

echo "Usage: compare_against_all.sh  Assumes in build/bin directory and sampled maps are map_result.ltm"

rm *_human.txt

./compare_against_human_maps.sh bbb3_result.ltm ../../data/local_topo_eval/bbb/ bbb_human.txt
./compare_against_human_maps.sh bbbdow1_result.ltm ../../data/local_topo_eval/bbbdow/ bbbdow_human.txt
./compare_against_human_maps.sh eecs3_result.ltm ../../data/local_topo_eval/eecs/ eecs_human.txt
./compare_against_human_maps.sh pierpont1_result.ltm ../../data/local_topo_eval/pierpont/ pierpont_human.txt
./compare_against_human_maps.sh intel_result.ltm ../../data/local_topo_eval/intel/ intel_human.txt
./compare_against_human_maps.sh csail3_result.ltm ../../data/local_topo_eval/csail/ csail_human.txt
