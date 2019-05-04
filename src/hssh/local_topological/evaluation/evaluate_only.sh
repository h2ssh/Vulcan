#!/bin/bash

echo "Usage: evaluate_only.sh results-file   NOTE: Assumes bbb3_result.ltm, etc. already exist in the current directory"

rm $1

echo "Generating bbb3 results..."
./evaluate_labels ../../data/maps/bbb3_truth.ltm bbb3_result.ltm 1000 $1

echo "Generating bbbdow1 results..."
./evaluate_labels ../../data/maps/bbbdow1_truth.ltm bbbdow1_result.ltm 1000 $1

echo "Generating eecs3 results..."
./evaluate_labels ../../data/maps/eecs3_truth.ltm eecs3_result.ltm 1000 $1

echo "Generating pierpont1 results..."
./evaluate_labels ../../data/maps/pierpont1_truth.ltm pierpont1_result.ltm 1000 $1

echo "Generating ggb1 results..."
./evaluate_labels ../../data/maps/ggb1_truth.ltm ggb1_result.ltm 1000 $1

echo "Generating ggb2 results..."
./evaluate_labels ../../data/maps/ggb2_truth.ltm ggb2_result.ltm 1000 $1

echo "Generating ggb3 results..."
./evaluate_labels ../../data/maps/ggb3_truth.ltm ggb3_result.ltm 1000 $1

echo "Generating tufts3 results..."
./evaluate_labels ../../data/maps/tufts3_truth.ltm tufts3_result.ltm 1000 $1

echo "Generating abuilding results..."
./evaluate_labels ../../data/maps/abuilding_truth.ltm abuilding_result.ltm 1000 $1

echo "Generating aces3 results..."
./evaluate_labels ../../data/maps/aces3_truth.ltm aces3_result.ltm 1000 $1

echo "Generating infinite_corridor results..."
./evaluate_labels ../../data/maps/infinite_corridor_truth.ltm infinite_corridor_result.ltm 1000 $1

echo "Generating seattle results..."
./evaluate_labels ../../data/maps/seattle_truth.ltm seattle_result.ltm 1000 $1

echo "Generating sdr results..."
./evaluate_labels ../../data/maps/sdr_truth.ltm sdr_result.ltm 1000 $1

echo "Generating intel_oregon results..."
./evaluate_labels ../../data/maps/intel_oregon_truth.ltm intel_oregon_result.ltm 1000 $1

echo "Generating intel results..."
./evaluate_labels ../../data/maps/intel_truth.ltm intel_result.ltm 1000 $1

echo "Generating csail3 results..."
./evaluate_labels ../../data/maps/csail3_truth.ltm csail3_result.ltm 1000 $1

echo "Generating freiburg results..."
./evaluate_labels ../../data/maps/freiburg_truth.ltm freiburg_result.ltm 1000 $1
