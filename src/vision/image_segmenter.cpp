/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cassert>
#include "math/geometry/convex_hull.h"
#include "vision/image_segment.h"
#include "vision/felzenszwalb_segmenter.h"
#include "vision/wassenberg_segmenter.h"
#include "vision/image_segmenter.h"

namespace vulcan
{
namespace vision
{

// Factory function
boost::shared_ptr<ImageSegmenter> create_image_segmenter(const std::string& type, const image_segmenter_params_t& params)
{
    if(type == FELZENSZWALB_SEGMENTER_TYPE)
    {
        return boost::shared_ptr<ImageSegmenter>(new FelzenszwalbSegmenter(params.felzParams));
    }
    else if(type == WASSENBERG_SEGMENTER_TYPE)
    {
        return boost::shared_ptr<ImageSegmenter>(new WassenbergSegmenter(params.wassenParams));
    }
    
    assert(false);
    return boost::shared_ptr<ImageSegmenter>();
}


/////////////// Implementation of the ImageSegmenter class.    ////////////////
void find_bounding_polygon_for_segments(std::vector<image_segment_t>& segments);

void ImageSegmenter::segmentImage(const Image& image, std::vector<image_segment_t>& segments)
{
    findImageSegments(image, segments);
    
    find_bounding_polygon_for_segments(segments);
}


void find_bounding_polygon_for_segments(std::vector<image_segment_t>& segments)
{
    for(auto segmentIt = segments.begin(), endIt = segments.end(); segmentIt != endIt; ++segmentIt)
    {
        segmentIt->boundingPolygon = math::convex_hull<int16_t>(segmentIt->boundaryPixels.begin(),
                                                                segmentIt->boundaryPixels.end());
    }
}

}
}
