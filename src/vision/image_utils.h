/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_IMAGE_UTILS_H
#define SENSORS_VISION_IMAGE_UTILS_H

#include <string>
#include "core/image.h"

namespace vulcan
{
namespace vision
{

/**
* save_image_to_file saves an image to the specified file. The extension of the filename determines
* the format to use for saving. No extension, or an unknown extension, and the default of pgm/ppm will
* be used.
*
* \param    image           Image to save
* \param    filename        Filename to use
* \return   True if saved successfully.
*/
bool save_image_to_file(const Image& image, const std::string& filename);

/**
* load_image_from_file loads an image from the specified file. Currently, the format of the saved image
* is determined by the extension on the filename. If there is no extension, .ppm is the assumed format.
* Reading the header of the file would obviously be a superior method for loading the image.
*
* \param    filename        Filename of the image to be loaded
* \param    image           Image in which to store the data (output)
* \return   True if the image was loaded successfully. False otherwise.
*/
bool load_image_from_file(const std::string& filename, Image& image);

}
}

#endif // SENSORS_VISION_IMAGE_UTILS_H
