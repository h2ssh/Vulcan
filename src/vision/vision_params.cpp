/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <string>
#include <utils/config_file.h>
#include <vision/vision_params.h>


using namespace vulcan;
using namespace vulcan::vision;

const std::string GRAPH_BASED_HEADING("GraphBasedSegmenterParameters");
const std::string SIGMA_KEY("sigma");
const std::string MIN_SIZE_KEY("min_segment_size");

const std::string FELZENSZWALB_HEADING("FelzenszwalbSegmenterParameters");
const std::string K_KEY("k");

const std::string WASSENBERG_HEADING("WassenbergSegmenterParameters");
const std::string MIN_EDGE_WEIGHT_KEY("min_edge_weight");
const std::string MAX_EDGE_WEIGHT_KEY("max_edge_weight");
const std::string PIXEL_SIGMA_KEY("pixel_sigma");
const std::string CREDIT_MULTIPLIER_KEY("credit_multiplier");

const std::string OPPONENT_COLOR_HEADING("OpponentColorHistogramParameters");
const std::string RG_BINS_KEY("rg_bins");
const std::string BY_BINS_KEY("by_bins");
const std::string WB_BINS_KEY("wb_bins");

const std::string RGB_HISTOGRAM_HEADING("RGBColorHistogramParameters");
const std::string R_BINS_KEY("r_bins");
const std::string G_BINS_KEY("g_bins");
const std::string B_BINS_KEY("b_bins");

const std::string SIMPLE_CONSTANCY_HEADING("SimpleColorConstancyParameters");
const std::string R_PRIME_BINS_KEY("r_prime_bins");
const std::string G_PRIME_BINS_KEY("g_prime_bins");

const std::string INTENSITY_HISTOGRAM_HEADING("IntensityHistogramParameters");
const std::string INTENSITY_BINS_KEY("intensity_bins");


graph_based_segmenter_params_t load_graph_based_params(const vulcan::utils::ConfigFile& config, const std::string& heading);

opponent_color_histogram_params_t         load_opponent_color_histogram_params        (const utils::ConfigFile& config);
rgb_histogram_params_t                    load_rgb_histogram_params                   (const utils::ConfigFile& config);
simple_color_constancy_histogram_params_t load_simple_color_constancy_histogram_params(const utils::ConfigFile& config);
intensity_histogram_params_t              load_intensity_histogram_params             (const utils::ConfigFile& config);


felzenszwalb_params_t vulcan::vision::load_felzenszwalb_params(const utils::ConfigFile& config)
{
    felzenszwalb_params_t params;
    
    params.graphParams = load_graph_based_params(config, FELZENSZWALB_HEADING);
    
    params.k = config.getValueAsFloat(FELZENSZWALB_HEADING, K_KEY);
    
    // Initial threshold is just k as all segments start with size 1
    params.graphParams.initialThreshold = params.k;
    
    return params;
}


wassenberg_params_t vulcan::vision::load_wassenberg_params(const utils::ConfigFile& config)
{
    wassenberg_params_t params;
    
    params.graphParams = load_graph_based_params(config, WASSENBERG_HEADING);
    
    params.minEdgeWeight    = config.getValueAsUInt32(WASSENBERG_HEADING, MIN_EDGE_WEIGHT_KEY);
    params.maxEdgeWeight    = config.getValueAsUInt32(WASSENBERG_HEADING, MAX_EDGE_WEIGHT_KEY);
    params.pixelSigma       = config.getValueAsFloat(WASSENBERG_HEADING, PIXEL_SIGMA_KEY);
    params.creditMultiplier = config.getValueAsFloat(WASSENBERG_HEADING, CREDIT_MULTIPLIER_KEY);
    
    // Just an arbitrary number larger than the maximum component distance so the credit calculation can work
    params.graphParams.initialThreshold = 1000.0;
    
    return params;
}


image_segmenter_params_t vulcan::vision::load_image_segmenter_params(const utils::ConfigFile& config)
{
    image_segmenter_params_t params;
    
    params.felzParams   = load_felzenszwalb_params(config);
    params.wassenParams = load_wassenberg_params(config);
    
    return params;
}


histogram_params_t vulcan::vision::load_historam_params(const utils::ConfigFile& config)
{
    histogram_params_t params;
    
    params.constancyParams = load_simple_color_constancy_histogram_params(config);
    params.intensityParams = load_intensity_histogram_params(config);
    params.opponentParams  = load_opponent_color_histogram_params(config);
    params.rgbParams       = load_rgb_histogram_params(config);
    
    return params;
}


graph_based_segmenter_params_t load_graph_based_params(const utils::ConfigFile& config, const std::string& heading)
{
    graph_based_segmenter_params_t params;
    
    params.sigma          = config.getValueAsFloat(heading, SIGMA_KEY);
    params.minSegmentSize = config.getValueAsUInt32(heading, MIN_SIZE_KEY);
    
    return params;
}


opponent_color_histogram_params_t load_opponent_color_histogram_params(const utils::ConfigFile& config)
{
    opponent_color_histogram_params_t params;
    
    params.rgBins = config.getValueAsInt16(OPPONENT_COLOR_HEADING, RG_BINS_KEY);
    params.byBins = config.getValueAsInt16(OPPONENT_COLOR_HEADING, BY_BINS_KEY);
    params.wbBins = config.getValueAsInt16(OPPONENT_COLOR_HEADING, WB_BINS_KEY);
    
    return params;
}


rgb_histogram_params_t load_rgb_histogram_params(const utils::ConfigFile& config)
{
    rgb_histogram_params_t params;
    
    params.rBins = config.getValueAsInt16(RGB_HISTOGRAM_HEADING, R_BINS_KEY);
    params.gBins = config.getValueAsInt16(RGB_HISTOGRAM_HEADING, G_BINS_KEY);
    params.bBins = config.getValueAsInt16(RGB_HISTOGRAM_HEADING, B_BINS_KEY);
    
    return params;
}


simple_color_constancy_histogram_params_t load_simple_color_constancy_histogram_params(const utils::ConfigFile& config)
{
    simple_color_constancy_histogram_params_t params;
    
    params.rPrimeBins = config.getValueAsInt16(SIMPLE_CONSTANCY_HEADING, R_PRIME_BINS_KEY);
    params.gPrimeBins = config.getValueAsInt16(SIMPLE_CONSTANCY_HEADING, G_PRIME_BINS_KEY);
    
    return params;
}


intensity_histogram_params_t load_intensity_histogram_params(const utils::ConfigFile& config)
{
    intensity_histogram_params_t params;
    
    params.bins = config.getValueAsInt16(INTENSITY_HISTOGRAM_HEADING, INTENSITY_BINS_KEY);
    
    return params;
}
