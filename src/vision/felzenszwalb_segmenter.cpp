/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/felzenszwalb_segmenter.h"

namespace vulcan
{
namespace vision
{

FelzenszwalbSegmenter::FelzenszwalbSegmenter(const felzenszwalb_params_t& params)
                                        : GraphBasedSegmenter(params.graphParams)
                                        , params(params)
{
}


int FelzenszwalbSegmenter::segmentGraph(void)
{
    /*
    * Main segmentation phase:
    * 
    * 1) Go through the pixels and merge any components whose edge is less than the threshold.
    * 
    * Straight forward
    */
    
    for(auto pixelIt = pixelEdges.begin(), endIt = pixelEdges.end(); pixelIt != endIt; ++pixelIt)
    {
        unsigned int componentA = componentForest->findSet(pixelIt->pixelA);
        unsigned int componentB = componentForest->findSet(pixelIt->pixelB);
        
        if((componentA != componentB)                           && 
           (pixelIt->value < components[componentA].threshold) &&
           (pixelIt->value < components[componentB].threshold))
        {
            unsigned int mergedComponent = mergeComponents(componentA, componentB);
            
            // The new edge must be the minimum edge in the component because searching in non-decreasing order
            components[mergedComponent].threshold = pixelIt->value + params.k/components[mergedComponent].size;
        }
    }
    
    return 0;
}

}
}
