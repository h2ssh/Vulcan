/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file
* \author   Collin Johnson
*
* Definition of Image.
*/

#include "core/image.h"
#include <cstring>

namespace vulcan
{

typedef unsigned char uchar;
typedef unsigned int uint;


Image::Image(void) :
        width(0),
        height(0),
        cspace(RGB),
        pixels(0)
{
}


Image::Image(unsigned int w, unsigned int h, Colorspace cs) :
        width(w),
        height(h),
        cspace(cs)
{
    if(cspace != MONO)
    {
        pixels = new unsigned char[width * height * 3];
    }
    else
    {
        pixels = new unsigned char[width * height];
    }
}


Image::Image(unsigned char* pix, unsigned int w, unsigned int h, Colorspace cs, bool copy) :
        width(w),
        height(h),
        cspace(cs)
{
    if(copy)
    {
        if(cspace != MONO)
        {
            pixels = new uchar[width * height * 3];
            pixels = static_cast<uchar*>(memcpy(pixels, pix, width * height * 3 * sizeof(uchar)));
        }
        else
        {
            pixels = new uchar[width * height];
            pixels = static_cast<uchar*>(memcpy(pixels, pix, width * height * sizeof(uchar)));
        }
    }
    else
    {
        pixels = pix;   // NOTE: This is bad and dangerous because deleting pix causes a dangling reference in the image and instant segfault on next access
    }
}


Image::Image(const Image& i) :
        width(i.width),
        height(i.height),
        cspace(i.cspace)
{
    // just allocate and copy the buffers
    if(cspace != MONO)
    {
        pixels = new unsigned char[width * height * 3];
        pixels = static_cast<unsigned char*>(memcpy(pixels, i.pixels, height * width * sizeof(unsigned char) * 3));
    }
    else
    {
        pixels = new unsigned char[width * height];
        pixels = static_cast<unsigned char*>(memcpy(pixels, i.pixels, height * width * sizeof(unsigned char)));
    }
}


Image& Image::operator=(const Image& rhs)
{
    // If the width * height of rhs > width * heigh of lhs, then allocate new memory
    // Otherwise, just copy over width and height and leave the current buffer

    if((rhs.width * rhs.height) > (width * height))
    {
        // delete the old image to keep the memory leaks away
        delete [] pixels;

        if(rhs.cspace != MONO)
        {
            pixels = new unsigned char[rhs.width * rhs.height * 3];
        }
        else
        {
            pixels = new unsigned char[rhs.width * rhs.height];
        }
    }

    // just copy all the members, no reference counts or the like to worry about
    width  = rhs.width;
    height = rhs.height;
    cspace = rhs.cspace;

    if(rhs.cspace != MONO)
    {
        pixels = static_cast<unsigned char*>(memcpy(pixels, rhs.pixels, height * width * sizeof(unsigned char) * 3));
    }
    else
    {
        pixels = static_cast<unsigned char*>(memcpy(pixels, rhs.pixels, height * width * sizeof(unsigned char)));
    }

    return *this;
}


Image::~Image()
{
    // If image buffer isn't empty, then delete it
    if(pixels != 0)
    {
        delete pixels;
    }
}


bool Image::getPixel(uint x, uint y, uchar& a, uchar& b, uchar& c) const
{
    // Image uses a lazy evaluation scheme, so RGB values may not have been calculated, if not, do that and then get the value

    // Valid position?
    if((x >= width) || (y >= height) || (pixels == 0))
    {
        return false;
    }

    int offset = (cspace == MONO) ? (x + y * width) : (x + y * width) * 3;

    if(cspace == MONO)
    {
        a = pixels[offset];
        b = pixels[offset];
        c = pixels[offset];
    }
    else
    {
        a = pixels[offset];
        b = pixels[offset + 1];
        c = pixels[offset + 2];
    }

    return true;
}


bool Image::setPixel(uint x, uint y, uchar a, uchar b, uchar c)
{
    // if the pixels are of the same colorspace, just assign directly, otherwise convert first before assigning

    if((x >= width) || (y >= height) || (pixels == 0))
    {
        return false;
    }

    int offset = (cspace == MONO) ? (x + y * width) : (x + y * width) * 3;

    if(cspace == MONO)
    {
        pixels[offset] = a;
    }
    else
    {
        pixels[offset]     = a;
        pixels[offset + 1] = b;
        pixels[offset + 2] = c;
    }

    return true;
}


void Image::setPixelBuffer(unsigned char* pixels, Colorspace color, unsigned int width, unsigned int height)
{
    if(this->pixels)
    {
        delete [] this->pixels;
    }

    this->pixels = pixels;
    this->cspace = color;
    this->width  = width;
    this->height = height;
}

} // namespace vulcan
