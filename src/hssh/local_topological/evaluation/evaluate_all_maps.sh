#!/bin/bash

echo "Usage: evaluate_all_maps.sh samples-per-iter constraint-log-prob repeat-log-prob results-files"

rm $4

echo "Labeling bbb3 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map bbb --save-map bbb3_result.ltm --one-shot ../../data/maps/bbb3.lpm > bbb3.log 2>&1
echo "Generating bbb3 results..."
./evaluate_labels ../../data/maps/bbb3_truth.ltm bbb3_result.ltm 1000 $4

echo "Labeling bbbdow1 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map bbbdow --save-map bbbdow1_result.ltm --one-shot ../../data/maps/bbbdow1.lpm > bbbdow1.log 2>&1
echo "Generating bbbdow1 results..."
./evaluate_labels ../../data/maps/bbbdow1_truth.ltm bbbdow1_result.ltm 1000 $4

echo "Labeling eecs3 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map eecs --save-map eecs3_result.ltm --one-shot ../../data/maps/eecs3.lpm > eecs3.log 2>&1
echo "Generating eecs3 results..."
./evaluate_labels ../../data/maps/eecs3_truth.ltm eecs3_result.ltm 1000 $4

echo "Labeling pierpont1 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map pierpont --save-map pierpont1_result.ltm --one-shot ../../data/maps/pierpont1.lpm > pierpont1.log 2>&1
echo "Generating pierpont1 results..."
./evaluate_labels ../../data/maps/pierpont1_truth.ltm pierpont1_result.ltm 1000 $4

echo "Labeling ggb1 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map ggb --save-map ggb1_result.ltm --one-shot ../../data/maps/ggb1.lpm > ggb1.log 2>&1
echo "Generating ggb1 results..."
./evaluate_labels ../../data/maps/ggb1_truth.ltm ggb1_result.ltm 1000 $4

echo "Labeling ggb2 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map ggb --save-map ggb2_result.ltm --one-shot ../../data/maps/ggb2.lpm > ggb2.log 2>&1
echo "Generating ggb2 results..."
./evaluate_labels ../../data/maps/ggb2_truth.ltm ggb2_result.ltm 1000 $4

echo "Labeling ggb3 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map ggb --save-map ggb3_result.ltm --one-shot ../../data/maps/ggb3.lpm > ggb3.log 2>&1
echo "Generating ggb3 results..."
./evaluate_labels ../../data/maps/ggb3_truth.ltm ggb3_result.ltm 1000 $4

echo "Labeling tufts3 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map ggb --save-map tufts3_result.ltm --one-shot ../../data/maps/tufts3.lpm > tufts3.log 2>&1
echo "Generating tufts3 results..."
./evaluate_labels ../../data/maps/tufts3_truth.ltm tufts3_result.ltm 1000 $4

echo "Labeling abuilding map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map abuilding --save-map abuilding_result.ltm --one-shot ../../data/maps/abuilding.lpm > abuilding.log 2>&1
echo "Generating abuilding results..."
./evaluate_labels ../../data/maps/abuilding_truth.ltm abuilding_result.ltm 1000 $4

echo "Labeling aces3 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map aces --save-map aces3_result.ltm --one-shot ../../data/maps/aces3.lpm > aces3.log 2>&1
echo "Generating aces3 results..."
./evaluate_labels ../../data/maps/aces3_truth.ltm aces3_result.ltm 1000 $4

echo "Labeling infinite_corridor map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map infinite_corridor --save-map infinite_corridor_result.ltm --one-shot ../../data/maps/infinite_corridor.lpm > infinite_corridor.log 2>&1
echo "Generating infinite_corridor results..."
./evaluate_labels ../../data/maps/infinite_corridor_truth.ltm infinite_corridor_result.ltm 1000 $4

echo "Labeling seattle map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map seattle --save-map seattle_result.ltm --one-shot ../../data/maps/seattle.lpm > seattle.log 2>&1
echo "Generating seattle results..."
./evaluate_labels ../../data/maps/seattle_truth.ltm seattle_result.ltm 1000 $4

echo "Labeling sdr map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map sdr --save-map sdr_result.ltm --one-shot ../../data/maps/sdr.lpm > sdr.log 2>&1
echo "Generating sdr results..."
./evaluate_labels ../../data/maps/sdr_truth.ltm sdr_result.ltm 1000 $4

echo "Labeling intel_oregon map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map intel_oregon --save-map intel_oregon_result.ltm --one-shot ../../data/maps/intel_oregon.lpm > intel_oregon.log 2>&1
echo "Generating intel_oregon results..."
./evaluate_labels ../../data/maps/intel_oregon_truth.ltm intel_oregon_result.ltm 1000 $4

echo "Labeling intel map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map intel --save-map intel_result.ltm --one-shot ../../data/maps/intel.lpm > intel.log 2>&1
echo "Generating intel results..."
./evaluate_labels ../../data/maps/intel_truth.ltm intel_result.ltm 1000 $4

echo "Labeling csail3 map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map csail --save-map csail3_result.ltm --one-shot ../../data/maps/csail3.lpm > csail3.log 2>&1
echo "Generating csail3 results..."
./evaluate_labels ../../data/maps/csail3_truth.ltm csail3_result.ltm 1000 $4

echo "Labeling freiburg map..."
./local_topo_hssh --mcmc-max-iterations 500 --mcmc-samples-per-iteration $1 --constraint-log-prob $2 --repeat-log-prob $3 --map freiburg --save-map freiburg_result.ltm --one-shot ../../data/maps/freiburg.lpm > freiburg.log 2>&1
echo "Generating freiburg results..."
./evaluate_labels ../../data/maps/freiburg_truth.ltm freiburg_result.ltm 1000 $4
