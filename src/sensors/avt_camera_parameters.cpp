/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "sensors/avt_camera_parameters.h"
#include "utils/config_file.h"
#include <iostream>


using namespace vulcan::sensors;


// Keys for the different parameters
const std::string AVT_CAMERA_HEADING("AVTCameraParameters");
const std::string IP_KEY("ip_address");
const std::string UNIQUE_ID_KEY("unique_id");
const std::string CONFIG_KEY("config_file_index");
const std::string FORMAT_KEY("pixel_format");
const std::string QUEUE_KEY("queue_size");
const std::string PACKET_SIZE_KEY("max_packet_size");
const std::string SUBSAMPLE_KEY("subsample");

// Specific strings for the string->enum conversions
const std::string FILE_INDEX_FACTORY("factory");
const std::string FILE_INDEX_1("1");
const std::string FILE_INDEX_2("2");
const std::string FILE_INDEX_3("3");
const std::string FILE_INDEX_4("4");

const std::string FORMAT_MONO8("mono8");
const std::string FORMAT_BAYER8("bayer8");
const std::string FORMAT_BAYER16("bayer16");
const std::string FORMAT_RGB24("rgb24");
const std::string FORMAT_DEFAULT("default");


// Helper functions for dealing with loading the enumerations
avt_config_file_index_t config_index_from_string(const std::string& configString);
avt_pixel_format_t pixel_format_from_string(const std::string& formatString);


avt_camera_parameters_t vulcan::sensors::load_avt_camera_parameters_from_config_file(const utils::ConfigFile& config)
{
    avt_camera_parameters_t params;

    params.ipAddress = config.getValueAsString(AVT_CAMERA_HEADING, IP_KEY);
    params.uniqueId = config.getValueAsUInt32(AVT_CAMERA_HEADING, UNIQUE_ID_KEY);

    params.config = config_index_from_string(config.getValueAsString(AVT_CAMERA_HEADING, CONFIG_KEY));
    params.format = pixel_format_from_string(config.getValueAsString(AVT_CAMERA_HEADING, FORMAT_KEY));

    params.queueSize = config.getValueAsUInt32(AVT_CAMERA_HEADING, QUEUE_KEY);
    params.maxPacketSize = config.getValueAsUInt32(AVT_CAMERA_HEADING, PACKET_SIZE_KEY);

    params.subsample = config.getValueAsBool(AVT_CAMERA_HEADING, SUBSAMPLE_KEY);

    return params;
}


avt_config_file_index_t config_index_from_string(const std::string& configString)
{
    avt_config_file_index_t config = AVT_CONFIG_FACTORY;

    if (configString == FILE_INDEX_FACTORY) {
        config = AVT_CONFIG_FACTORY;
    } else if (configString == FILE_INDEX_1) {
        config = AVT_CONFIG_1;
    } else if (configString == FILE_INDEX_2) {
        config = AVT_CONFIG_2;
    } else if (configString == FILE_INDEX_3) {
        config = AVT_CONFIG_3;
    } else if (configString == FILE_INDEX_4) {
        config = AVT_CONFIG_4;
    } else {
        std::cerr << "WARNING: load_avt_camera_parameters: Unknown config file index, " << configString
                  << ". Defaulting to 'factory'\n";
    }

    return config;
}


avt_pixel_format_t pixel_format_from_string(const std::string& formatString)
{
    avt_pixel_format_t format = AVT_PIXEL_DEFAULT;

    if (formatString == FORMAT_MONO8) {
        format = AVT_PIXEL_MONO_8;
    } else if (formatString == FORMAT_BAYER8) {
        format = AVT_PIXEL_BAYER_8;
    } else if (formatString == FORMAT_BAYER16) {
        format = AVT_PIXEL_BAYER_16;
    } else if (formatString == FORMAT_RGB24) {
        format = AVT_PIXEL_RGB_24;
    } else if (formatString == FORMAT_DEFAULT) {
        format = AVT_PIXEL_DEFAULT;
    } else {
        std::cerr << "WARNING: load_avt_camera_parameters: Unknown pixel format, " << formatString
                  << ". Defaulting to 'default'\n";
    }

    return format;
}
