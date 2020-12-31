/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_FILTERS_H
#define SENSORS_VISION_FILTERS_H

#include <cmath>
#include <cstring>
#include "core/image.h"

namespace vulcan
{
namespace vision
{

/**
* Gaussian2DFilter implements a simple 2D Gaussian filter. The radius of the filter is fixed
* at compile time. The single configurable parameter is sigma, the std. dev. of the Gaussian.
*/
template <int radius>
class Gaussian2DFilter
{
public:
    
    /**
    * Constructor for Gaussian2DFilter.
    */
    Gaussian2DFilter(float sigma = 1.0f) :
                accum(0),
                accumWidth(0)
    {
        buildKernel(sigma);
    }
    
    /**
    * Destructor for Gaussian2DFilter.
    */
    ~Gaussian2DFilter(void)
    {
        delete [] accum;
    }
    
    /**
    * setSigma changes the sigma value for the filter.
    */
    void setSigma(float sigma)
    {
        buildKernel(sigma);
    }
    
    /**
    * apply applies the filter to the given source image. The results of the filtering
    * are stored in the destination image.
    * 
    * \pre  source.getWidth() == destination.getWidth() && source.getHeight() == destination.getHeight()
    */
    void apply(const Image& source, Image& destination)
    {
        // Need to change this to using the two separable filters, but that'll come later
        filterImage(source, destination);
    }
    
private:
    
    void buildKernel(float sigma)
    {
        float sigmaSq = sigma * sigma;
        double min = exp(-(radius*radius / (2 * sigmaSq)));
        
        normalizer = 0;
        
        for(int r = 0; r < 2*radius+1; ++r)
        {
            kernel[r]   = static_cast<int>(ceil(exp(-(pow(r, 2) / (2 * sigmaSq))) / min));
            normalizer += kernel[r];
        }
    }
    
    void filterImage(const Image& source, Image& destination)
    {
        /*
         * Separable filters have the nice property that they run in time linear
         * to the size of the filter as opposed to quadratic.
         *
         * The algorithm that is applied below comes from the book:
         *
         *  Fundamentals of Computer Graphics (Second Edition) by Peter Shirley
         *
         * NOTE: At the current time, this algorithm completely ignores the boundaries for simplicity.
         * I don't think it will be an issue.
         *
         */
        
        unsigned int width  = source.getWidth();
        unsigned int height = source.getHeight();
        
        unsigned char* pixels = source.getPixelBuffer();
        unsigned char* outPix = destination.getPixelBuffer();
        
        // Allocate accumulation buffer
        if(width != accumWidth)
        {
            delete [] accum;
            accumWidth = width*3;
            accum = new int[accumWidth];
        }
        
        int curPix[3];
        
        for(unsigned int y = radius; y < height - radius; ++y)
        {
            // Reset the value of the row convolution
            memset(accum, 0, accumWidth * sizeof(int));
            
            for(unsigned int x = 0; x < width; ++x)
            {
                curPix[0] = 0;
                curPix[1] = 0;
                curPix[2] = 0;
                
                for(int i = -radius; i <= radius; ++i)
                {
                    curPix[0] += kernel[i+radius] * pixels[((y - i)*width + x)*3];
                    curPix[1] += kernel[i+radius] * pixels[((y - i)*width + x)*3 + 1];
                    curPix[2] += kernel[i+radius] * pixels[((y - i)*width + x)*3 + 2];
                }
                
                accum[x*3]     = curPix[0] / normalizer;
                accum[x*3 + 1] = curPix[1] / normalizer;
                accum[x*3 + 2] = curPix[2] / normalizer;
            }
            
            for(unsigned int x = radius; x < width - radius; ++x)
            {
                curPix[0] = 0;
                curPix[1] = 0;
                curPix[2] = 0;
                
                // Convolve across the columns
                for(int i = -radius; i <= radius; ++i)
                {
                    curPix[0] +=  kernel[i+radius] * accum[3*(x - i)];
                    curPix[1] +=  kernel[i+radius] * accum[3*(x - i) + 1];
                    curPix[2] +=  kernel[i+radius] * accum[3*(x - i) + 2];
                }
                
                outPix[(y*width + x)*3]     = curPix[0] / normalizer;
                outPix[(y*width + x)*3 + 1] = curPix[1] / normalizer;
                outPix[(y*width + x)*3 + 2] = curPix[2] / normalizer;
            }
        }
    }
    
    // Pre-calculated kernel values
    int kernel[2*radius + 1];
    int normalizer;
    
    // Accumulation buffer for the separable filters
    int* accum;
    unsigned int accumWidth;
};

}
}

#endif // SENSORS_VISION_FILTERS_H
