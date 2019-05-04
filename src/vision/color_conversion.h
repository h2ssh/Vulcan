/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_COLOR_CONVERSION_H
#define SENSORS_VISION_COLOR_CONVERSION_H

#include <core/image.h>

namespace vulcan
{
namespace vision
{

/** convertPixelColorspace converts a pixel between various colorspaces. */
void convertPixelColorspace(unsigned char fromA, unsigned char fromB, unsigned char fromC, unsigned char& toA, unsigned char& toB, unsigned char& toC, Colorspace from, Colorspace to);

/** convertImageColorspace converts a pixel between various colorspaces. */
void convertImageColorspace(unsigned char* from, unsigned char* to, unsigned int width, unsigned int height, Colorspace csFrom, Colorspace csTo);

}
}

#endif // SENSORS_VISION_COLOR_CONVERSION_H
