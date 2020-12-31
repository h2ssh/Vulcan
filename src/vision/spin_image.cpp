/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/spin_image.h"
#include "core/image.h"
#include <algorithm>
#include <iostream>


using namespace vulcan;
using namespace vulcan::vision;


SpinImage::SpinImage(const Image& image, const Point<int16_t>& center, int radius, int intensityBins)
: values((radius + 1) * intensityBins, 0)
, radius(radius)
, intensityBins(intensityBins)
{
    buildSpinImage(image, center);
}


void SpinImage::calculate(const Image& image, const Point<int16_t>& center, int radius, int intensityBins)
{
    this->radius = radius;
    this->intensityBins = intensityBins;

    values.resize((radius + 1) * intensityBins, 0);
    std::fill(values.begin(), values.end(), 0);

    buildSpinImage(image, center);
}


float SpinImage::compare(const SpinImage& toCompare)
{
    return 1.0f;
}


void SpinImage::buildSpinImage(const Image& image, const Point<int16_t>& center)
{
    int totalValues = 0;

    int width = image.getWidth();
    int height = image.getHeight();

    for (int y = -radius; y <= radius; ++y) {
        if ((center.y + y < 0) || (center.y + y >= height)) {
            continue;
        }

        for (int x = -radius; x <= radius; ++x) {
            if ((center.x + x < 0) || (center.x + x >= width)) {
                continue;
            }

            totalValues += calculatePixelBin(x, y, center, image);
        }
    }

    normalizeValues(totalValues);
}


int SpinImage::calculatePixelBin(int x, int y, const Point<int16_t>& center, const Image& image)
{
    int addValue = 0;

    float binDivider = 256.0f / intensityBins;

    uint8_t pixel[3];
    int dist = ceil(sqrt(y * y + x * x));
    float value = 0.0f;

    if (dist <= radius) {
        image.getPixel(center.x + x, center.y + y, pixel[0], pixel[1], pixel[2]);
        value = (static_cast<float>(pixel[0]) + pixel[1] + pixel[2]) / 3.0f;

        int bin = value / binDivider;

        values[dist * intensityBins + bin] += 1;

        addValue = 1;
    }

    return addValue;
}


void SpinImage::normalizeValues(int totalValues)
{
    if (!totalValues) {
        return;
    }

    for (size_t n = 0; n < values.size(); ++n) {
        values[n] /= totalValues;
    }
}
