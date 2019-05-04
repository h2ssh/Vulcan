/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <vision/navtexture/navtexture_params.h>
#include <utils/config_file.h>

namespace vulcan
{
namespace vision
{

// String headings for the config file descriptions
const std::string NAV_TEXTURE_HEADING("NavTextureParameters");
const std::string TRAINING_FILE_KEY("training_data_file");
const std::string RAW_FILE_KEY("raw_data_file");
const std::string MODEL_FILE_KEY("predict_model_file");

const std::string OBJECT_IDENTIFIER_HEADING("ObjectIdentiferParameters");
const std::string SEGMENTER_TYPE_KEY("segmenter_type");
const std::string HISTOGRAM_TYPE_KEY("histogram_type");
const std::string NUM_CLUSTERS_KEY("num_segment_clusters");
const std::string MAX_ITERATIONS_KEY("max_clustering_iterations");
const std::string LASER_DIST_KEY("max_laser_distance");
const std::string DYNAMIC_MATCH_KEY("min_dynamic_matches");
const std::string HOMOGRAPHY_FILE_KEY("homography_file");
const std::string CALIBRATION_FILE_KEY("calibration_file");

// Helpers for loading the various class params files
image_object_identifier_params_t load_identifier_params(const utils::ConfigFile& config);


navtexture_params_t load_navtexture_params(const utils::ConfigFile& config)
{
    navtexture_params_t params;

    params.trainingDataFile = config.getValueAsString(NAV_TEXTURE_HEADING, TRAINING_FILE_KEY);
    params.rawFeaturesFile  = config.getValueAsString(NAV_TEXTURE_HEADING, RAW_FILE_KEY);
    params.predictModelFile = config.getValueAsString(NAV_TEXTURE_HEADING, MODEL_FILE_KEY);
    
    params.identifierParams = load_identifier_params(config);
    
    return params;
}


image_object_identifier_params_t load_identifier_params(const utils::ConfigFile& config)
{
    image_object_identifier_params_t params;
    
    params.segmenterType   = config.getValueAsString(OBJECT_IDENTIFIER_HEADING, SEGMENTER_TYPE_KEY);
    params.segmenterParams = load_image_segmenter_params(config);

    params.histogramType    = config.getValueAsString(OBJECT_IDENTIFIER_HEADING, HISTOGRAM_TYPE_KEY);
    params.histogramParams  = load_historam_params(config);
    
    params.numSegmentClusters      = config.getValueAsInt16(OBJECT_IDENTIFIER_HEADING, NUM_CLUSTERS_KEY);
    params.maxClusteringIterations = config.getValueAsInt16(OBJECT_IDENTIFIER_HEADING, MAX_ITERATIONS_KEY);

    params.maxLaserDistance  = config.getValueAsFloat(OBJECT_IDENTIFIER_HEADING, LASER_DIST_KEY);
    params.minDynamicMatches = config.getValueAsInt16(OBJECT_IDENTIFIER_HEADING, DYNAMIC_MATCH_KEY);
    
    params.homographyFile  = config.getValueAsString(OBJECT_IDENTIFIER_HEADING, HOMOGRAPHY_FILE_KEY);
    params.calibrationFile = config.getValueAsString(OBJECT_IDENTIFIER_HEADING, CALIBRATION_FILE_KEY);
    
    return params;
}

}
}
