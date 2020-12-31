/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_WASSENBERG_SEGMENTER_H
#define SENSORS_VISION_WASSENBERG_SEGMENTER_H

#include <string>
#include "vision/vision_params.h"
#include "vision/graph_based_segmenter.h"

namespace vulcan
{
namespace vision
{
    
const std::string WASSENBERG_SEGMENTER_TYPE("wassenberg");

/**
* WassenbergSegmenter is an image segmentation algorithm described in
* "An Efficient Parallel Algorithm for Graph-Based Image Segmentation" by
* Wassenberg, et. al 2009.
*/
class WassenbergSegmenter : public GraphBasedSegmenter
{
public:
    
    /**
    * Constructor for WassenbergSegmenter.
    */
    WassenbergSegmenter(const wassenberg_params_t& params);
    
private:
    
    virtual int segmentGraph(void);
    
    int  mergeLowWeightEdges(void);
    void calculateComponentsCredit(void);
    void mergeWithEdgeHeuristic(int startingEdgeIndex);
    
    float findPotentialMergeCredit(int componentA, int componentB);
    
    wassenberg_params_t params;
};

}
}

#endif // SENSORS_VISION_WASSENBERG_SEGMENTER_H
