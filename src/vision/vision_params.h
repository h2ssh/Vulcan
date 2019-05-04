/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_VISION_PARAMS_H
#define SENSORS_VISION_VISION_PARAMS_H

namespace vulcan
{
namespace utils { class ConfigFile; }
    
namespace vision
{

/*
* This file contains the definition of the parameter structs for all of the generic
* image processing algorithms. Each generic struct is provided with the appropriate
* load function so the structs can be easily integrated into any vision processing
* module.
*/

struct graph_based_segmenter_params_t
{
    float        sigma;             // Gaussian blurring sigma
    unsigned int minSegmentSize;    // Minimum allowable size for a segment
    float        initialThreshold;  // Initial threshold to use for components
};

struct felzenszwalb_params_t
{
    graph_based_segmenter_params_t graphParams;
    
    float k;                 // Parameter from the paper, affects the size of components found
};

struct wassenberg_params_t
{
    graph_based_segmenter_params_t graphParams;
    
    unsigned int minEdgeWeight;     // Weight below which segments are always merged
    unsigned int maxEdgeWeight;     // Weight above which segments are always split
    float        pixelSigma;        // Assumed noise on edge weights
    float        creditMultiplier;  // If credit is too conservative, then oversegmentation happens, can be less conservative setting this > 1.0f
};

struct image_segmenter_params_t
{
    felzenszwalb_params_t felzParams;
    wassenberg_params_t   wassenParams;
};

struct opponent_color_histogram_params_t
{
    int16_t rgBins;
    int16_t byBins;
    int16_t wbBins;
};

struct rgb_histogram_params_t
{
    int16_t rBins;
    int16_t gBins;
    int16_t bBins;
};

struct simple_color_constancy_histogram_params_t
{
    int16_t rPrimeBins;
    int16_t gPrimeBins;
};

struct intensity_histogram_params_t
{
    int16_t bins;
};

struct histogram_params_t
{
    opponent_color_histogram_params_t         opponentParams;
    rgb_histogram_params_t                    rgbParams;
    simple_color_constancy_histogram_params_t constancyParams;
    intensity_histogram_params_t              intensityParams;
};


// Functions for loading the various parameters classes
felzenszwalb_params_t    load_felzenszwalb_params   (const utils::ConfigFile& config);
wassenberg_params_t      load_wassenberg_params     (const utils::ConfigFile& config);
image_segmenter_params_t load_image_segmenter_params(const utils::ConfigFile& config);

histogram_params_t load_historam_params(const utils::ConfigFile& config);

}
}

#endif // SENSORS_VISION_VISION_PARAMS_H
