/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     uniform_distribution.h
* \author   Collin Johnson
* 
* Definition of UniformDistribution.
*/

#ifndef MATH_UNIFORM_DISTRIBUTION_H
#define MATH_UNIFORM_DISTRIBUTION_H

#include <cassert>

namespace vulcan
{
namespace math
{
    
/**
* UniformDistribution represents a simple uniform distribution where,
* 
*   p(z) =  1/(z_max-z_min)  if  z_min <= z <= z_max
*           0                otherwise
*/
class UniformDistribution
{
public:
    
    /**
    * Constructor for UniformDistribution.
    * 
    * \pre      zMin < zMax
    * \param    zMin            Minimum value in the range (optional, default = 0.0)
    * \param    zMax            Maximum value in the range (optional, default = 1.0)
    */
    explicit UniformDistribution(double zMin = 0.0, double zMax = 1.0)
    : zMin(zMin)
    , zMax(zMax)
    , uniform(1.0 / (zMax - zMin))
    {
        assert(zMin < zMax);
    }
    
    /**
    * setRange sets the range for the distribution.
    * 
    * \pre  zMin < zMax
    */
    void setRange(double zMin, double zMax)
    {
        assert(zMin < zMax);
        
        this->zMin = zMin;
        this->zMax = zMax;
        uniform    = 1.0 / (zMax - zMin);
    }
    
    /**
    * likelihood calculates the likelihood given the distribution, as described above.
    * 
    * \param    value           Value to calculate likelihood of
    * \return   Likelihood of the value, given the distribution.
    */
    double likelihood(double value) const
    {
        return (value < zMin || value > zMax) ? 0.0 : uniform;
    }
    
private:
    
    double zMin;
    double zMax;
    double uniform;
};
    
}
}

#endif // MATH_UNIFORM_DISTRIBUTION_H
