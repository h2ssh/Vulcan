/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_CAMERA_H
#define SENSORS_CAMERA_H

#include <core/image.h>

namespace vulcan
{
namespace sensors
{

/**
* Camera is an interface for capturing images from any of a variety of cameras.
* To use the interface, create whatever specialized Camera implementation is
* needed, call startCapture(), and then keep grabbing the available images
* by 
*
* See the various camera drivers defined in drivers/ to determine exactly what
* is needed to get a particular camera capturing images properly.
*/
class Camera
{
public:
    
    virtual ~Camera(void) { }
    
    /**
    * startCapture tells the camera to begin capturing images. This method
    * needs to be called before getNextImage(), otherwise getNextImage() will
    * block forever.
    */
    virtual void startCapture(void) = 0;
    
    /**
    * stopCapture halts capture of images.
    */
    virtual void stopCapture(void) = 0;
    
    /**
    * getNextImage grabs the next available image from the camera. The method
    * blocks until a new image arrives. The next available image is returned
    * to ensure that a fresh image is provided after each call.
    */
    virtual bool getNextImage(Image& image) = 0;
};

}
}

#endif // SENSORS_CAMERA_H
