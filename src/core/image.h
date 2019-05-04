/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file
* \author  Collin Johnson
*
* Declaration of a basic Image type.
*/

#ifndef CORE_IMAGE_H
#define CORE_IMAGE_H

#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cstdint>
#include <cstddef>

namespace vulcan
{

/** Colorspace is an enum defining the colorspaces supported by Image. */
enum Colorspace
{
    DEFAULT,    ///< Specifies to use whatever namespace is currently being used
    RGB,        ///< Standard 8-bit RGB format
    HSV,        ///< Hue, Saturation, Value -- a nice colorspace to work with
    YUV,        ///< Another brightness invariant colorspace like HSV with the advantage that the format is supported in hardware
    MONO        ///< 8-bit B&W color. Conversion to this cannot be undone.
};

/**
* Image is an abstract data type that encapsulates information pertaining to an
* image frame pulled from the camera. An Image contains the dimensions of the image
* as well as a buffer of all the pixels, using one of the colorspaces defined in the Colorspace enum.
*
* All pixels, regardless of colorspace, are represented as 3 unsigned bytes. One byte for each component
* of the color, e.g. R, G, B or Y,U,V. If the colorspace is not typically defined on this scale, the
* value will be normalized to fit within this scale. For example, HSV is defined with H as 0-360, and
* S and V 0-1. But each will be between 0-255 using the Image class.
*
* Conversion between the various colorspaces is provided via functions defined in color_conversions.h.
*/
class Image
{
public:

    /**
    * Default constructor for Image. Creates a null image with 0 height and width.
    */
    Image(void);

    /**
    * Constructor for Image. Creates an empty image of the specified width and height.
    *
    * \param    w              Width of the image
    * \param    h              Height of the image
    * \param    cs             Colorspace of the image
    */
    Image(unsigned int w, unsigned int h, Colorspace cs);

    /** Constructor for Image. */
    Image(unsigned char* pix, unsigned int w, unsigned int h, Colorspace cs, bool copy = true);

    /**
    * Copy constructor for Image.
    *
    * \param    i               Image to be copied
    */
    Image(const Image& i);

    /**
    * Assignment operator for Image.
    *
    * \param    rhs             Image on right-hand side of the '=' construction
    * \return   Reference to lhs image that is produced.
    */
    Image& operator=(const Image& rhs);

    /**
    * Destructor for Image. Cleans up the buffer.
    */
    ~Image();

    void setTimestamp(int64_t timestamp) { this->timestamp = timestamp; }

    size_t     getWidth(void)  const { return width; }
    size_t     getHeight(void) const { return height; }
    Colorspace getColorspace(void) const { return cspace; }
    int64_t    getTimestamp(void) const { return timestamp; }

    /**
    * getPixel retrieves the values of the pixel at the specified location.
    *
    * \param    x           x-position in the image   x < image.width
    * \param    y           y-position in the image   y < image.height
    * \param    a           First pixel byte
    * \param    b           Second pixel byte
    * \param    c           Third pixel byte
    * \param    cs          Colorspace of the pixel to output
    * \return   True if the values are valid, i.e. there actually was a buffer to read from.
    */
    bool getPixel(unsigned int x, unsigned int y, unsigned char& a, unsigned char& b, unsigned char& c) const;

    /**
    * setPixel changes a pixel in the Image. If the colorspace of the pixel does not match the current
    * colorspace of the image, then the pixel values will be converted appropriately.
    *
    * \param    x           x-position of the pixel  x < image.width
    * \param    y           y-position of the pixel  y < image.height
    * \param    a           First byte of the pixel value
    * \param    b           Second byte of the pixel value
    * \param    c           Third byte of the pixel value
    * \param    cs          Colorspace of the provided pixel values
    * \return   True if pixel was changed. False indicates there is no image data, or
    *           the requested position to write is outside the bounds of the image.
    */
    bool setPixel(unsigned int x, unsigned int y, unsigned char a, unsigned char b, unsigned char c);

    /**
    * getPixelBuffer retrieves the internal buffer of image pixels. Pixels are stored in row-major
    * order. To access pixel (x,y) do: (y*width + x)*bytesPerPixel NOTE: Using this is
    * dangerous. Deletion will mostly likely cause a program crash. However, it is
    * probably necessary for speed considerations to have access to the raw buffer.
    *
    * \return   Buffer of pixels in the image.
    */
    unsigned char* getPixelBuffer() const { return pixels; }

    /**
    * setPixelBuffer is a method which changes the internal pixel buffer to the provided
    * buffer. The method is intended for changing an already existing image to a new buffer
    * without making a copy of the image.
    *
    * Ownership of the memory for the pixel buffer is acquired by Image instance.
    *
    * \param    pixels          New pixel buffer for the image
    * \param    color           Colorspace of the pixel buffer
    * \param    width           Width of the image in pixels
    * \param    height          Height of the image in pixels
    */
    void setPixelBuffer(unsigned char* pixels, Colorspace color, unsigned int width, unsigned int height);

private:

    unsigned int width;             ///< Width of the image
    unsigned int height;            ///< Height of the image

    Colorspace cspace;               ///< Current colorspace of the pixels buffer

    unsigned char* pixels;          ///< Pixels that make up the image

    int64_t timestamp;      ///< Time at which the image was created

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar (width,
            height,
            cspace);

        auto size = width*height;

        if(cspace != Colorspace::MONO)
        {
            size *= 3;
        }

        if(!pixels)
        {
            pixels = new unsigned char[size];
        }

        ar (cereal::binary_data(pixels, size),
            timestamp);
    }
};

} // namespace vulcan

DEFINE_SYSTEM_MESSAGE(Image, ("SENSOR_IMAGE"))

#endif // CORE_IMAGE_H
