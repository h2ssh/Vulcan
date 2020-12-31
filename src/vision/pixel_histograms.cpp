/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/pixel_histograms.h"
#include "core/image.h"
#include "vision/vision_params.h"
#include <cassert>
#include <iostream>


using namespace vulcan;
using namespace vulcan::vision;


pixel_histogram_method_t convert_string_to_method(const std::string& methodString);
uint16_t calculate_bin_size(uint16_t numBins, uint16_t maxBinValue);

// Histogram comparison metrics
float histogram_intersection(const std::vector<float>& histA, const std::vector<float>& histB);


boost::shared_ptr<PixelHistogram> PixelHistogram::createPixelHistogram(pixel_histogram_method_t method,
                                                                       const Image& image,
                                                                       const std::vector<Point<int16_t>>& pixels,
                                                                       const histogram_params_t& params)
{
    boost::shared_ptr<PixelHistogram> histogram;

    switch (method) {
    case HISTOGRAM_OPPONENT_COLOR:
        histogram.reset(new OpponentColorHistogram(params.opponentParams));
        break;

    case HISTOGRAM_RGB:
        histogram.reset(new RGBColorHistogram(params.rgbParams));
        break;

    case HISTOGRAM_SIMPLE_COLOR_CONSTANCY:
        histogram.reset(new SimpleColorConstancyHistogram(params.constancyParams));
        break;

    case HISTOGRAM_INTENSITY:
        std::cerr << "ERROR:Intensity histogram not currently implemented" << std::endl;
        assert(false);
        break;

    default:
        std::cerr << "ERROR:Unknown histogram type" << std::endl;
        assert(false);
        break;
    }

    histogram->method = method;
    histogram->calculateHistogram(image, pixels);

    return histogram;
}


boost::shared_ptr<PixelHistogram> PixelHistogram::createPixelHistogram(const std::string& method,
                                                                       const Image& image,
                                                                       const std::vector<Point<int16_t>>& pixels,
                                                                       const histogram_params_t& params)
{
    return createPixelHistogram(convert_string_to_method(method), image, pixels, params);
}


boost::shared_ptr<PixelHistogram> PixelHistogram::copyPixelHistogram(pixel_histogram_method_t method,
                                                                     const std::vector<float>& values,
                                                                     const std::vector<uint16_t>& dimensions)
{
    boost::shared_ptr<PixelHistogram> histogram;

    switch (method) {
    case HISTOGRAM_OPPONENT_COLOR:
        histogram.reset(new OpponentColorHistogram(values, dimensions));
        break;

    case HISTOGRAM_RGB:
        histogram.reset(new RGBColorHistogram(values, dimensions));
        break;

    case HISTOGRAM_SIMPLE_COLOR_CONSTANCY:
        histogram.reset(new SimpleColorConstancyHistogram(values, dimensions));
        break;

    case HISTOGRAM_INTENSITY:
        std::cerr << "ERROR:Intensity histogram not currently implemented" << std::endl;
        assert(false);
        break;

    default:
        std::cerr << "ERROR:Unknown histogram type" << std::endl;
        assert(false);
        break;
    }

    histogram->method = method;

    return histogram;
}


boost::shared_ptr<PixelHistogram> PixelHistogram::copyPixelHistogram(const std::string& method,
                                                                     const std::vector<float>& values,
                                                                     const std::vector<uint16_t>& dimensions)
{
    return copyPixelHistogram(convert_string_to_method(method), values, dimensions);
}


PixelHistogram::PixelHistogram(const std::vector<uint16_t>& dimensions, const std::vector<float>& values)
: dimensions(dimensions)
, values(values)
{
}


float PixelHistogram::compare(boost::shared_ptr<PixelHistogram> toCompare)
{
    assert(this->method == toCompare->method);

    return this->calculateHistogramSimilarity(toCompare);
}


// Implementation for RGBColorHistogram
RGBColorHistogram::RGBColorHistogram(const std::vector<float>& values, const std::vector<uint16_t>& dimensions)
: PixelHistogram(dimensions, values)
{
}


RGBColorHistogram::RGBColorHistogram(const rgb_histogram_params_t& params)
{
    dimensions.resize(3);
    dimensions[0] = params.rBins;
    dimensions[1] = params.gBins;
    dimensions[2] = params.bBins;

    values.resize(params.rBins * params.gBins * params.bBins, 0);
}


void RGBColorHistogram::calculateHistogram(const Image& image, const std::vector<Point<int16_t>>& pixels)
{
    int rowStride = dimensions[0];
    int sliceStride = dimensions[0] * dimensions[1];

    int rBinSize = calculate_bin_size(dimensions[0], 256);
    int gBinSize = calculate_bin_size(dimensions[1], 256);
    int bBinSize = calculate_bin_size(dimensions[2], 256);

    uint8_t rValue = 0;
    uint8_t gValue = 0;
    uint8_t bValue = 0;

    unsigned int row = 0;
    unsigned int column = 0;
    unsigned int slice = 0;

    assert(pixels.size() > 0);

    for (auto pixelIt = pixels.begin(), endIt = pixels.end(); pixelIt != endIt; ++pixelIt) {
        image.getPixel(pixelIt->x, pixelIt->y, rValue, gValue, bValue);

        row = rValue / rBinSize;
        column = (gValue / gBinSize) * rowStride;
        slice = (bValue / bBinSize) * sliceStride;

        if (row + column + slice > values.size()) {
            std::cerr << "ERROR:RGBHistogram:Pixel out of bounds:RGB(" << (int)rValue << ',' << (int)gValue << ','
                      << (int)bValue << ")->(" << row << ',' << column << ',' << slice << ")\n";
            assert(false);
        }

        ++values[row + column + slice];
    }

    for (auto valueIt = values.begin(), endIt = values.end(); valueIt != endIt; ++valueIt) {
        *valueIt /= pixels.size();
    }
}


float RGBColorHistogram::calculateHistogramSimilarity(boost::shared_ptr<PixelHistogram> histogram)
{
    return histogram_intersection(values, histogram->getValues());
}


// Implementation for OpponentColorHistogram
OpponentColorHistogram::OpponentColorHistogram(const std::vector<float>& values,
                                               const std::vector<uint16_t>& dimensions)
: PixelHistogram(dimensions, values)
{
}


OpponentColorHistogram::OpponentColorHistogram(const opponent_color_histogram_params_t& params)
{
    dimensions.resize(3);
    dimensions[0] = params.rgBins;
    dimensions[1] = params.byBins;
    dimensions[2] = params.wbBins;

    values.resize((params.rgBins * params.byBins * params.wbBins), 0);
}


void OpponentColorHistogram::calculateHistogram(const Image& image, const std::vector<Point<int16_t>>& pixels)
{
    int rowStride = dimensions[0];
    int sliceStride = dimensions[0] * dimensions[1];

    int rgBinSize = calculate_bin_size(dimensions[0], 256);
    int byBinSize = calculate_bin_size(dimensions[1], 256);
    int wbBinSize = calculate_bin_size(dimensions[2], 256);

    uint8_t rValue = 0;
    uint8_t gValue = 0;
    uint8_t bValue = 0;

    unsigned int row = 0;
    unsigned int column = 0;
    unsigned int slice = 0;

    for (auto pixelIt = pixels.begin(), endIt = pixels.end(); pixelIt != endIt; ++pixelIt) {
        image.getPixel(pixelIt->x, pixelIt->y, rValue, gValue, bValue);

        row = abs(rValue - gValue) / rgBinSize;
        column = abs(2 * static_cast<int>(bValue) - rValue - gValue) / byBinSize;
        slice = (static_cast<int>(rValue) + gValue + bValue) / wbBinSize;

        std::cout << "vals:" << row << ' ' << column << ' ' << slice << " stride:" << rowStride << ' ' << sliceStride
                  << '\n';

        column *= rowStride;
        slice *= sliceStride;

        if (row + column + slice > values.size()) {
            std::cerr << "ERROR:OpponentColorHistogram:Pixel out of bounds:RGB(" << (int)rValue << ',' << (int)gValue
                      << ',' << (int)bValue << ")->(" << row << ',' << column << ',' << slice << ")\n";
            assert(false);
        }

        ++values[row + column + slice];
    }

    for (auto valueIt = values.begin(), endIt = values.end(); valueIt != endIt; ++valueIt) {
        *valueIt /= pixels.size();
    }
}


float OpponentColorHistogram::calculateHistogramSimilarity(boost::shared_ptr<PixelHistogram> histogram)
{
    return histogram_intersection(values, histogram->getValues());
}


// Implementation for SimpleColorConstancyHistogram
SimpleColorConstancyHistogram::SimpleColorConstancyHistogram(const std::vector<float>& values,
                                                             const std::vector<uint16_t>& dimensions)
: PixelHistogram(dimensions, values)
{
}


SimpleColorConstancyHistogram::SimpleColorConstancyHistogram(const simple_color_constancy_histogram_params_t& params)
{
    dimensions.resize(2);
    dimensions[0] = params.rPrimeBins;
    dimensions[1] = params.gPrimeBins;

    values.resize((params.rPrimeBins * params.gPrimeBins), 0);
}


void SimpleColorConstancyHistogram::calculateHistogram(const Image& image, const std::vector<Point<int16_t>>& pixels)
{
    int rowStride = dimensions[0];

    int rBinSize = calculate_bin_size(dimensions[0], 256);
    int gBinSize = calculate_bin_size(dimensions[1], 256);

    uint8_t rValue = 0;
    uint8_t gValue = 0;
    uint8_t bValue = 0;

    int pixelSum = 0;

    int row = 0;
    int column = 0;

    for (auto pixelIt = pixels.begin(), endIt = pixels.end(); pixelIt != endIt; ++pixelIt) {
        image.getPixel(pixelIt->x, pixelIt->y, rValue, gValue, bValue);

        pixelSum = rValue + gValue + bValue;

        row = (rValue / pixelSum) / rBinSize;
        column = (gValue / pixelSum) / gBinSize;

        column *= rowStride;

        ++values[row + column];
    }

    for (auto valueIt = values.begin(), endIt = values.end(); valueIt != endIt; ++valueIt) {
        *valueIt /= pixels.size();
    }
}


float SimpleColorConstancyHistogram::calculateHistogramSimilarity(boost::shared_ptr<PixelHistogram> histogram)
{
    return histogram_intersection(values, histogram->getValues());
}


// Helpers
pixel_histogram_method_t convert_string_to_method(const std::string& methodString)
{
    if (methodString == HISTOGRAM_OPPONENT_COLOR_STRING) {
        return HISTOGRAM_OPPONENT_COLOR;
    } else if (methodString == HISTOGRAM_RGB_STRING) {
        return HISTOGRAM_RGB;
    } else if (methodString == HISTOGRAM_SIMPLE_COLOR_CONSTANCY_STRING) {
        return HISTOGRAM_SIMPLE_COLOR_CONSTANCY;
    } else if (methodString == HISTOGRAM_INTENSITY_STRING) {
        return HISTOGRAM_INTENSITY;
    } else {
        std::cerr << "ERROR: Unknown histogram method: " << methodString << std::endl;
        assert(false);
    }

    return HISTOGRAM_RGB;
}


uint16_t calculate_bin_size(uint16_t numBins, uint16_t maxBinValue)
{
    uint16_t binSize = maxBinValue / numBins;

    // If the max value and the bin size don't divide evenly, then increase the bin size by
    // to account for the extra pixels. NOTE: This is a junky solution, but it'll have to do
    // for now
    if (maxBinValue % numBins != 0) {
        ++binSize;
    }

    return binSize;
}


// Histogram comparison metrics
float histogram_intersection(const std::vector<float>& histA, const std::vector<float>& histB)
{
    /*
     * This metric comes from "Color Indexing" by Swain and Ballard, 1991
     *
     * Because the histograms are normalized, the histogram intersection is the
     * city-block metric between the histograms. Thus, the intersection becomes:
     *
     *   H(A, B) = 1 - 0.5*sum(abs(histA_i - histB_i))
     */

    float totalDiff = 0.0f;

    for (size_t n = 0; n < histA.size(); ++n) {
        totalDiff += fabs(histA[n] - histB[n]);
    }

    return 1.0f - totalDiff / 2.0f;
}
