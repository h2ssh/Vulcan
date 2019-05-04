/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_AVT_CAMERA_PARAMETERS_H
#define SENSORS_AVT_CAMERA_PARAMETERS_H

#include <string>

namespace vulcan
{

namespace utils
{
    class ConfigFile;
}
    
namespace sensors
{

/**
* avt_config_file_index_t specifies the possible parameter configurations stored in the
* non-volatile memory on the camera. These stored configurations allow for camera settings
* to be determined using the AVT viewer software, saved in some index, and then
* easily retrieved
*/
enum avt_config_file_index_t
{
    AVT_CONFIG_FACTORY,
    AVT_CONFIG_1,
    AVT_CONFIG_2,
    AVT_CONFIG_3,
    AVT_CONFIG_4
};

/**
* avt_pixel_format_t defines the pixel format for the image returned from the camera.
*/
enum avt_pixel_format_t
{
    AVT_PIXEL_DEFAULT,
    AVT_PIXEL_MONO_8,
    AVT_PIXEL_BAYER_8,
    AVT_PIXEL_BAYER_16,
    AVT_PIXEL_RGB_24        // 24-bits per pixel, 8-bits per channel, R G and B
};

/**
* avt_camera_parameters_t holds the adjustable parameters for a specific instantiation of the
* AVT camera using the PvAPI.
*/
struct avt_camera_parameters_t
{
    unsigned long           uniqueId;
    std::string             ipAddress;
    
    avt_config_file_index_t config;
    avt_pixel_format_t      format;

    unsigned int            queueSize;          ///< Number of frames to use for the image queue
    unsigned long           maxPacketSize;      ///< Maximum packet size supported by computer's network device
    
    bool                    subsample;          ///< Flag indicating if the image should be subsampled by a factor of 2
                                                ///< Output image will be (width/2, height/2)
};


/**
* load_avt_camera_parameters_from_config_file loads parameters for using an AVT camera from the
* provided configuration file.
* 
* The configuration file parameters and their corresponding ranges are:
* 
*   [AVTCameraParameters]
*   ip_address        = xxx.xxx.xxx.xxx
*   unique_id         = (0, sizeof(unsigned long)]
*   config_file_index = {factory, 1, 2, 3, 4}
*   pixel_format      = {mono8, bayer8, bayer16, rgb24, default}
*   queue_size        = [1,100]
*   max_packet_size   = [1500, 9000]
*   subsample         = {1, 0}
*/
avt_camera_parameters_t load_avt_camera_parameters_from_config_file(const utils::ConfigFile& config);

}
}

#endif // SENSORS_AVT_CAMERA_PARAMETERS_H
