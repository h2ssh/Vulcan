/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_SPIN_IMAGE_H
#define SENSORS_VISION_SPIN_IMAGE_H

#include <vector>
#include <core/point.h>

namespace vulcan
{

namespace sensors
{
    class Image;
}

namespace vision
{

/**
* SpinImage is an implementation of the intensity domain spin image, which is a feature descriptor
* for the texture of a surface. A spin image is a two dimensional histogram where one dimension
* represents the distance from the descriptor center and the other dimension is an intensity histogram.
* The feature gives some idea of the distribution of intensity around a central point. The intensity
* distribution is a good description of the texture.
*/
class SpinImage
{
public:

    /**
    * Default constructor for SpinImage.
    */
    SpinImage(void) {}
    
    /**
    * Constructor for SpinImage.
    */
    SpinImage(const Image& image, const Point<int16_t>& center, int radius, int intensityBins);

    /**
    * calculate calculates a spin image based on the input parameters.
    */
    void calculate(const Image& image, const Point<int16_t>& center, int radius, int intensityBins);

    /**
    * compare determines the similarity between two spin images. The similarity measure is in the range [0,1], where
    * 1.0 is an exact match and 0.0 makes you wonder why you bothered comparing in the first place.
    */
    float compare(const SpinImage& toCompare);
    
    // Methods for looking at the details of the histogram values.
    int getRadius(void)        const { return radius; }
    int getIntensityBins(void) const { return intensityBins; }
    
    const std::vector<float>& getValues(void) const { return values; }

private:

    void buildSpinImage(const Image& image, const Point<int16_t>& center);
    int  calculatePixelBin(int x, int y, const Point<int16_t>& center, const Image& image);
    void normalizeValues(int totalValues);

    std::vector<float> values;

    int radius;
    int intensityBins;
};

}
}

#endif // SENSORS_VISION_SPIN_IMAGE_H
