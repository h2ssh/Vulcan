/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_IMAGE_SEGMENT_CLUSTERER_H
#define SENSORS_VISION_NAVTEXTURE_IMAGE_SEGMENT_CLUSTERER_H

#include <vector>

namespace vulcan
{

namespace utils
{
    class DisjointSetForest;
}

namespace sensors
{
    class Image;
}

namespace vision
{
    
struct image_segment_t;

/**
* ImageSegmentClusterer uses k-means to join adjacent image segments that were separated
* due to lighting conditions, excessive pixel noise, etc. k-means is used to redefine
* the colors in the image. Each segment is assigned one of the colors. Adjacent segments
* belonging to the same cluster are merged into the same segment.
*/
class ImageSegmentClusterer
{
public:
    
    /**
    * Constructor for ImageSegmentClusterer.
    */
    ImageSegmentClusterer(uint16_t numClusters, uint16_t maxIterations);
    
    /**
    * clusterSegments reduces the number of segments by using k-means clustering
    * and then merging adjacent segments belonging to the same cluster.
    */
    void clusterSegments(std::vector<image_segment_t>& segments,
                         utils::DisjointSetForest&     components,
                         const Image&         image,
                         std::vector<image_segment_t>& finalSegments);
    
private:
    
    int numClusters;
    int maxIterations;
};

}
}

#endif // SENSORS_VISION_NAVTEXTURE_IMAGE_SEGMENT_CLUSTERER_H
