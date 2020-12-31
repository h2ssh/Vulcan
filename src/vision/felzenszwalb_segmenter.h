/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_FELZENSZWALB_SEGMENTER_H
#define SENSORS_VISION_FELZENSZWALB_SEGMENTER_H

#include "vision/graph_based_segmenter.h"
#include "vision/vision_params.h"
#include <string>

namespace vulcan
{
namespace vision
{

const std::string FELZENSZWALB_SEGMENTER_TYPE("felzenszwalb");

/**
 * FelzenszwalbSegmenter is an implementation (depends on the piece) of the
 * graph-based image segmentation algorithm presented in Felzenszwalb '04.
 *
 * See that paper for more details.
 *
 * The code provided at http://people.cs.uchicago.edu/~pff/segment/ is used
 * as inspiration for this implementation. I don't like the code at all.
 */
class FelzenszwalbSegmenter : public GraphBasedSegmenter
{
public:
    /**
     * Constructor for FelzenszwalbSegmenter.
     */
    FelzenszwalbSegmenter(const felzenszwalb_params_t& params);

private:
    virtual int segmentGraph(void);

    felzenszwalb_params_t params;
};

}   // namespace vision
}   // namespace vulcan

#endif   // SENSORS_VISION_FELZENSZWALB_SEGMENTER_H
