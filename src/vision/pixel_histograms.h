/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_PIXEL_HISTOGRAMS_H
#define SENSORS_VISION_PIXEL_HISTOGRAMS_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "core/point.h"

namespace vulcan
{
namespace vision
{

class  Image;
struct histogram_params_t;
struct opponent_color_histogram_params_t;
struct rgb_histogram_params_t;
struct simple_color_constancy_histogram_params_t;
struct intensity_histogram_params_t;
    
/**
* pixel_histogram_method_t defines the different histogramming methods
* that can be performed on a set of pixels.
*/
enum pixel_histogram_method_t
{
    HISTOGRAM_OPPONENT_COLOR,
    HISTOGRAM_RGB,
    HISTOGRAM_SIMPLE_COLOR_CONSTANCY,
    HISTOGRAM_INTENSITY
};

// String descriptions of the histogram methods
const std::string HISTOGRAM_OPPONENT_COLOR_STRING("opponent_color");
const std::string HISTOGRAM_RGB_STRING("rgb");
const std::string HISTOGRAM_SIMPLE_COLOR_CONSTANCY_STRING("simple_color_constancy");
const std::string HISTOGRAM_INTENSITY_STRING("intensity");

/**
* PixelHistogram is the base class for a variety of pixel histogramming methods.
* The particular methods are defined in pixel_histogram_method_t. Each method
* performs its own series of operations on the pixels to form some description.
* 
* Each histogram has the following properties:
* 
*   - Number of dimensions (1, 2, 3)
*   - Size of each dimension
*   - Normalized bins with range [0, 1]
* 
* Histograms of the same type can be compared with the compare() method, which
* gives a similarity measure of the histograms between [0, 1] using some distance
* metric. A future extension might support multiple distance metrics.
* 
* The raw histogram buffer can be accessed via the getRawValues() method. The reason
* for doing so would be to transmit the histogram to another module, to visualize
* the histogram, or to save it to disk for analysis. Similarly, if loading a histogram
* from its raw values, the setRawValues() method can be used.
*/
class PixelHistogram
{
public:
    
    /**
    * createPixelHistogram creates a new pixel histogram using the specified method.
    * 
    * The method used for the histogramming controls the all parameters of the histogram:
    *   - Number of dimensions
    *   - Size of the dimensions
    * 
    * \param    method          Method to be used for histogramming
    * \param    image           Image containing the pixel values
    * \param    pixels          Location of the pixels to consider
    * \param    params          Parameters for the histogram to be created
    */
    static boost::shared_ptr<PixelHistogram> createPixelHistogram(pixel_histogram_method_t                  method,
                                                                  const Image&                     image,
                                                                  const std::vector<Point<int16_t>>& pixels,
                                                                  const histogram_params_t&                 params);
    
    /**
    * createPixelHistogram creates a new pixel histogram using the specified method.
    * 
    * The method used for the histogramming controls the all parameters of the histogram:
    *   - Number of dimensions
    *   - Size of the dimensions
    * 
    * \param    method          String description of the histogram method to be used.
    * \param    image           Image containing the pixel values
    * \param    pixels          Location of the pixels to consider
    * \param    params          Parameters for the histogram to be created
    */
    static boost::shared_ptr<PixelHistogram> createPixelHistogram(const std::string&                        method,
                                                                  const Image&                     image,
                                                                  const std::vector<Point<int16_t>>& pixels,
                                                                  const histogram_params_t&                 params);
    
    /**
    * copyPixelHistogram creates a copy of a histogram with the provided parameters and values.
    */
    static boost::shared_ptr<PixelHistogram> copyPixelHistogram(pixel_histogram_method_t     method,
                                                                const std::vector<float>&    values,
                                                                const std::vector<uint16_t>& dimensions);
    
    /**
    * copyPixelHistogram creates a copy of a histogram with the provided parameters and values.
    */
    static boost::shared_ptr<PixelHistogram> copyPixelHistogram(const std::string&           method,
                                                                const std::vector<float>&    values,
                                                                const std::vector<uint16_t>& dimensions);
    
    /**
    * Destructor for PixelHistogram.
    */
    virtual ~PixelHistogram(void) { }
    
    /**
    * compare determines the similarity between two histograms. The similarity measure is in the range [0,1], where
    * 1.0 is an exact match and 0.0 makes you wonder why you bothered comparing in the first place.
    * 
    * NOTE: If toCompare->getHistogramMethod() != this->getHistogramMethod(), then the program will fail fast.
    */
    float compare(boost::shared_ptr<PixelHistogram> toCompare);
    
    /**
    * getMethod retrieves the method used for creating the histogram values.
    */
    pixel_histogram_method_t getMethod(void) const { return method; }
    
    // Methods for looking at the details of the histogram values.
    const std::vector<uint16_t>& getDimensions(void) const { return dimensions; }
    const std::vector<float>&    getValues(void)     const { return values; }
    
protected:
    
    /**
    * Default constructor for PixelHistogram.
    */
    PixelHistogram(void) { }
    
    /**
    * Constructor for PixelHistogram when creating from existing histogram values.
    * 
    * \param    dimensions          Dimensions of the histogram
    * \param    values              Values of the histogram
    */
    PixelHistogram(const std::vector<uint16_t>& dimensions, const std::vector<float>& values);
    
    /**
    * calculateHistogram calculates the histogram using the provided image and the pixel coordinates.
    * 
    * \param    image               Image containing the pixel values
    * \param    pixels              Coordinates of the pixels to include in the histogram
    */
    virtual void calculateHistogram(const Image&                     image,
                                    const std::vector<Point<int16_t>>& pixels) = 0;
    
    /**
    * calculateHistogramSimilarity calculates the similarity of this histogram with the provided histogram.
    */
    virtual float calculateHistogramSimilarity(boost::shared_ptr<PixelHistogram> histogram) = 0;

    std::vector<uint16_t> dimensions;
    std::vector<float>    values;
    
private:
    
    pixel_histogram_method_t method;
};

/**
* RGBColorHistogram is the simplest color histogram. It bins the RGB values into separate bins
* using three dimensions.
*/
class RGBColorHistogram : public PixelHistogram
{
public:
    
    /**
    * Constructor for RGBColorHistogram.
    */
    RGBColorHistogram(const std::vector<float>& values, const std::vector<uint16_t>& dimensions);
    
    /**
    * Constructor for RGBColorHistogram.
    */
    RGBColorHistogram(const rgb_histogram_params_t& params);
    
private:
    
    /**
    * calculateHistogram calculates the histogram using the provided image and the pixel coordinates.
    */
    virtual void calculateHistogram(const Image&                     image,
                                    const std::vector<Point<int16_t>>& pixels);
    
    /**
    * calculateHistogramSimilarity calculates the similarity of this histogram with the provided histogram.
    */
    virtual float calculateHistogramSimilarity(boost::shared_ptr<PixelHistogram> histogram);
};

/**
* OpponentColorHistogram creates a three-dimensional histogram of an RGB image,
* where the three dimensions are:
* 
* rg = r - g
* by = 2*b - r - g
* wb = r + g + b
* 
* The above values come from "Color Indexing" by Swain and Ballard, 1991
*/
class OpponentColorHistogram : public PixelHistogram
{
public:
    
    /**
    * Constructor for OpponentColorHistogram.
    */
    OpponentColorHistogram(const std::vector<float>& values, const std::vector<uint16_t>& dimensions);
    
    /**
    * Constructor for OpponentColorHistogram.
    */
    OpponentColorHistogram(const opponent_color_histogram_params_t& params);
    
private:
    
    /**
    * calculateHistogram calculates the histogram using the provided image and the pixel coordinates.
    */
    virtual void calculateHistogram(const Image&                     image,
                                    const std::vector<Point<int16_t>>& pixels);
    
    /**
    * calculateHistogramSimilarity calculates the similarity of this histogram with the provided histogram.
    */
    virtual float calculateHistogramSimilarity(boost::shared_ptr<PixelHistogram> histogram);
};

/**
* SimpleColorConstancyHistogram creates a two-dimensional histogram, where the two dimensions are:
* 
* r' = r / (r+g+b)
* g' = g / (r+g+b)
* 
* The above values come from "Color Indexing" by Swain and Ballard, 1991
*/
class SimpleColorConstancyHistogram : public PixelHistogram
{
public:
    
    /**
    * Constructor for SimpleColorConstancyHistogram.
    */
    SimpleColorConstancyHistogram(const std::vector<float>& values, const std::vector<uint16_t>& dimensions);
    
    /**
    * Constructor for SimpleColorConstancyHistogram.
    */
    SimpleColorConstancyHistogram(const simple_color_constancy_histogram_params_t& params);
    
private:
    
    /**
    * calculateHistogram calculates the histogram using the provided image and the pixel coordinates.
    */
    virtual void calculateHistogram(const Image&                     image,
                                    const std::vector<Point<int16_t>>& pixels);
    
    /**
    * calculateHistogramSimilarity calculates the similarity of this histogram with the provided histogram.
    */
    virtual float calculateHistogramSimilarity(boost::shared_ptr<PixelHistogram> histogram);    
};

}
}

#endif // SENSORS_VISION_PIXEL_HISTOGRAM_H
