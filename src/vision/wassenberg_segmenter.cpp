/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/wassenberg_segmenter.h"
#include <cmath>
#include <iostream>


// #define DEBUG_SEGMENTER


using namespace vulcan::vision;


WassenbergSegmenter::WassenbergSegmenter(const wassenberg_params_t& params)
: GraphBasedSegmenter(params.graphParams)
, params(params)
{
}


int WassenbergSegmenter::segmentGraph(void)
{
    /*
     * Steps for the Wassenberg segmentation algorithm:
     *
     * 0) Merge all regions with weight < minEdgeWeight.
     * 1) Calculate the credit limit for the created regions.
     * 2) Continue merging segments using the credit limit approach.
     */

    int finalMergedEdgeIndex = mergeLowWeightEdges();
    calculateComponentsCredit();
    mergeWithEdgeHeuristic(finalMergedEdgeIndex);

    return finalMergedEdgeIndex;
}


int WassenbergSegmenter::mergeLowWeightEdges(void)
{
    /*
     * The pixel edges are already sorted in non-decreasing order, so keep merging segments
     * until the first edge with value > minEdgeWeight is found.
     */

    for (size_t n = 0; n < pixelEdges.size(); ++n) {
        if (pixelEdges[n].value > params.minEdgeWeight) {
#ifdef DEBUG_SEGMENTER
            std::cout << "INFO:Wassenberg: Merged " << n << " initial segments less than " << params.minEdgeWeight
                      << '\n';
#endif

            return n;
        }

        unsigned int componentA = componentForest->findSet(pixelEdges[n].pixelA);
        unsigned int componentB = componentForest->findSet(pixelEdges[n].pixelB);

        if (componentA != componentB) {
            mergeComponents(componentA, componentB);
        }
    }

    // Curious -- all edges were merged -- display a warning as undersegmentation likely
    std::cerr << "WARNING:WassenbergSegmenter: No edges above minEdgeWeight found!\n";

    return pixelEdges.size();
}


void WassenbergSegmenter::calculateComponentsCredit(void)
{
#ifdef DEBUG_SEGMENTER
    float creditTotal = 0.0f;
    int validComponents = 0;
#endif

    for (auto compIt = components.begin(), endIt = components.end(); compIt != endIt; ++compIt) {
        compIt->credit = ceil(sqrt(4.0 * M_PI * compIt->size) * params.creditMultiplier);

#ifdef DEBUG_SEGMENTER
        if (compIt->size > 1) {
            creditTotal += compIt->credit;
            ++validComponents;
        }
#endif
    }

#ifdef DEBUG_SEGMENTER
    std::cout << "INFO:Wassenberg: Valid components:" << validComponents
              << " Average credit:" << (creditTotal / validComponents) << '\n';
#endif
}


void WassenbergSegmenter::mergeWithEdgeHeuristic(int startingEdgeIndex)
{
    for (auto edgeIt = pixelEdges.begin() + startingEdgeIndex, endIt = pixelEdges.end(); edgeIt != endIt; ++edgeIt) {
        unsigned int componentA = componentForest->findSet(edgeIt->pixelA);
        unsigned int componentB = componentForest->findSet(edgeIt->pixelB);

        if (componentA != componentB) {
            float credit = findPotentialMergeCredit(componentA, componentB);

            if (credit > edgeIt->value && (edgeIt->value <= params.maxEdgeWeight)) {
                unsigned int mergedComponent = mergeComponents(componentA, componentB);

                components[mergedComponent].credit = credit - edgeIt->value;
            }
        }
    }
}


float WassenbergSegmenter::findPotentialMergeCredit(int componentA, int componentB)
{
    float credit = 0.0f;

    if ((components[componentA].size > 1) && (components[componentB].size > 1)) {
        credit = fmin(components[componentA].credit, components[componentB].credit);
    } else if (components[componentA].size > 1) {
        credit = components[componentA].credit;
    } else {
        credit = components[componentB].credit;
    }

    return credit;
}
