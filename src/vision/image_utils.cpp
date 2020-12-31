/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/image_utils.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace vulcan
{
namespace vision
{

// Helpers for saving files
bool save_ppm_file(const std::string& filename, const Image& image);
bool save_pgm_file(const std::string& filename, const Image& image);

bool load_ppm_pgm_file(const std::string& filename, Image& image);

// Read through the input until a non-comment is found
std::string next_header_value(std::istream& in);

bool save_image_to_file(const Image& image, const std::string& filename)
{
    /*
     * The .ppm format is ridiculously simple. The header is the following:
     *
     * P6 <whitespace> <width> <whitespace> <height> <whitespace> <max color val> "\n"
     * rows of data, R,G,B for each pixel.
     */

    Colorspace cspace = image.getColorspace();

    if (cspace == RGB)   // default to .ppm
    {
        return save_ppm_file(filename, image);
    } else   // default to .pgm
    {
        return save_pgm_file(filename, image);
    }
}


bool load_image_from_file(const std::string& filename, Image& image)
{
    /*
     * ppm format specifies the width, height, and max value of pixel byte in the header.
     * For Image, the max value cannot be > 255. Also, right now, the format assumes
     * data in certain places, those exactly created by Image, so that is where the
     * restriction comes from.
     */

    // If the file is a .png, then load a .png instead
    if ((filename.find(".ppm") != std::string::npos) || ((filename.find(".pgm") != std::string::npos))) {
        return load_ppm_pgm_file(filename, image);
    }

    return false;
}


bool save_ppm_file(const std::string& filename, const Image& image)
{
    std::string fn = (filename.find(".ppm") == std::string::npos) ? (filename + ".ppm") : filename;
    std::ofstream imageOut(fn.c_str());

    uint16_t width = image.getWidth();
    uint16_t height = image.getHeight();

    if (imageOut.is_open()) {
        // Write the header information: using max val for pixel as 255 for now -- 8-bit RGB
        imageOut << "P6 " << width << " " << height << " " << 255 << '\n';

        unsigned char* pixels = image.getPixelBuffer();

        imageOut.write((char*)pixels, width * height * 3);

        return true;
    } else   // unable to open the file
    {
        return false;
    }
}


bool save_pgm_file(const std::string& filename, const Image& image)
{
    std::string fn = (filename.find(".pgm") == std::string::npos) ? (filename + ".pgm") : filename;
    std::ofstream imageOut(fn.c_str());

    uint16_t width = image.getWidth();
    uint16_t height = image.getHeight();

    if (imageOut.is_open()) {
        // Write the header information: using max val for pixel as 255 for now -- 8-bit RGB
        imageOut << "P5 " << width << " " << height << " " << 255 << '\n';

        unsigned char* pixels = image.getPixelBuffer();

        imageOut.write((char*)pixels, width * height);

        return true;
    } else   // unable to open the file
    {
        return false;
    }
}


bool load_ppm_pgm_file(const std::string& filename, Image& image)
{
    std::ifstream imageIn(filename.c_str());

    std::string data;
    unsigned char* pixels = 0;
    unsigned int width = 0;
    unsigned int height = 0;
    int max = 0;
    std::string type;

    if (!imageIn.is_open()) {
        std::cout << "Error: Unable to load image file: " << filename << std::endl;
        return false;
    }

    // read the header -- next_header_value skips the comments in the header
    std::istringstream valueIn(next_header_value(imageIn));
    valueIn >> data;
    if (valueIn.fail()) {
        return false;
    }

    valueIn.str(next_header_value(imageIn));
    valueIn.clear();
    valueIn >> width;
    if (valueIn.fail()) {
        return false;
    }

    valueIn.str(next_header_value(imageIn));
    valueIn.clear();
    valueIn >> height;
    if (valueIn.fail()) {
        return false;
    }

    valueIn.str(next_header_value(imageIn));
    valueIn.clear();
    valueIn >> max;
    if (valueIn.fail()) {
        return false;
    }

    // ensure that data is readable for us
    if (max > 255) {
        std::cout << "Error: Unable to load image. Too many bytes per pixel." << std::endl;
        return false;
    }

    // ppm or pgm?
    unsigned int pixelWidth = (data.find("P5") == std::string::npos) ? 3 : 1;
    unsigned int imageSize = width * height * pixelWidth;

    pixels = new unsigned char[imageSize];

    imageIn.ignore(1);   // flush the whitespace that is in the file before hitting the pixels
    imageIn.read((char*)(pixels), height * width * pixelWidth);

    Colorspace cspace = (pixelWidth == 3) ? RGB : MONO;

    image.setPixelBuffer(pixels, cspace, width, height);

    return true;
}


std::string next_header_value(std::istream& in)
{
    // Comments in the ppm/pgm start with #. Read the file until a string not starting with # is found
    std::string value;
    while (in.good()) {
        in >> value;
        if (value.find('#') == std::string::npos) {
            return value;
        } else   // read the rest of the current line and throw it away
        {
            // TODO
        }
    }

    return std::string();
}

}   // namespace vision
}   // namespace vulcan
