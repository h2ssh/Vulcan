/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_NAVTEXURE_PARAMS_H
#define SENSORS_VISION_NAVTEXTURE_NAVTEXURE_PARAMS_H

#include <string>
#include "vision/vision_params.h"

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace vision
{

struct image_object_identifier_params_t
{
    std::string              segmenterType;
    image_segmenter_params_t segmenterParams;

    std::string        histogramType;
    histogram_params_t histogramParams;
    
    int numSegmentClusters;         // Number of clusters to use for the k-means clustering of image segments
    int maxClusteringIterations;    // Maximum number of iterations to run k-means

    float maxLaserDistance;         // Maximum range to consider from laser points. Should be correlated with homography performance (7 meters for current Vulcan measurement)
    int   minDynamicMatches;        // Minimum number of matches between segment and dynamic point for a match to be made
    
    std::string homographyFile;     // Location of the homography matrix parameters
    std::string calibrationFile;    // Location of the camera calibration parameters
};

struct navtexture_params_t
{
    std::string trainingDataFile;
    std::string rawFeaturesFile;
    std::string predictModelFile;

    image_object_identifier_params_t identifierParams;
};

/**
* load_navtexture_params loads the parameters for navtexture module.
*/
navtexture_params_t load_navtexture_params(const utils::ConfigFile& config);

}
}

#endif // SENSORS_VISION_NAVTEXTURE_NAVTEXURE_PARAMS_H
