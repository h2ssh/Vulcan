/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_AVT_CAMERA_H
#define SENSORS_AVT_CAMERA_H

#include <PvApi.h>
#include <string>
#include <vector>
#include "sensors/camera.h"
#include "sensors/avt_camera_parameters.h"
#include "utils/sensor_time.h"

namespace vulcan
{
namespace sensors
{
    
/**
* initialize_pvapi_library handles the generic initialization procedures that
* need to be performed before creating an instance of AVTCamera.
* 
* \param    verbose         Print information about all connected cameras
*/
void initialize_pvapi_library(bool verbose);
    

/**
* AVTCamera is a wrapper around the AVT PvAPI library for GigE Vision cameras.
*/
class AVTCamera : public Camera
{
public:

    /**
    * Constructor for AVTCamera.
    *
    * If an IP address is specified in the parameters, then it will be used for opening
    * the camera. Otherwise, the uniqueId will be used.
    */
    AVTCamera(const avt_camera_parameters_t& params);

    // Camera interface implementation
    
    /**
    * startCapture tells the camera to begin capturing images. This method
    * needs to be called before getNextImage(), otherwise getNextImage() will
    * block forever.
    */
    virtual void startCapture(void);
    
    /**
    * stopCapture halts capture of images.
    */
    virtual void stopCapture(void);
    
    /**
    * getNextImage grabs the next available image from the camera. The method
    * blocks until a new image arrives. The next available image is returned
    * to ensure that a fresh image is provided after each call.
    * 
    * For efficiency, pass the same Image to getNextImage() for each call.
    * 
    * If getNextImage() is not called fast enough, the image queue will overflow.
    * Fast enough depends on the camera, but something in the 20fps range should
    * be expected.
    */
    virtual bool getNextImage(Image& image);

    // AVTCamera specific methods for controlling the capture parameters

    /**
    * loadConfiguration loads a pre-saved configuration of parameters saved on
    * the camera.
    *
    * \param    config          Configuration to be loaded
    */
    void loadConfiguration(avt_config_file_index_t config);

    /**
    * setPixelFormat changes the pixel format of the image being captured.
    */
    void setPixelFormat(avt_pixel_format_t format);

private:
    
    void setCameraParameters(void);
    void setPacketSize      (void);
    void allocateImageFrames(void);
    void setPostAcquisitionStartParams(void);
    
    // Specifications for the camera that was loaded
    unsigned long           uniqueId;
    std::string             ipAddress;
    avt_camera_parameters_t params;
    utils::SensorTime time_;
    
    // PvAPI library specifics
    tPvHandle             cameraHandle;
    std::vector<tPvFrame> queuedFrames;
    int                   currentFrameIndex;
};

}
}

#endif // SENSORS_AVT_CAMERA_H
