/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     particle_sampling.cpp
* \author   Collin Johnson
* 
* Definition of:
*
*   - generate_uniform_samples_at_position
*   - generate_random_samples_at_position
*/

#include <hssh/metrical/relocalization/particle_sampling.h>
#include <hssh/metrical/localization/particle.h>

namespace vulcan
{
namespace hssh
{
    
inline float random_orientation(void)
{
    return (drand48() * M_PI * 2.0) - M_PI;
}


void generate_uniform_samples_at_position(Point<float> position, int numSamples, double weight, std::vector<particle_t>& samples)
{
    particle_t sample;
    float thetaIncrement = 2.0*M_PI / numSamples;
    
    while(--numSamples >= 0)
    {
        sample.id     = samples.size();
        sample.pose   = pose_t(position.x, position.y, wrap_to_pi(numSamples*thetaIncrement));
        sample.weight = weight;
        
        samples.push_back(sample);
    }
}


void generate_random_samples_at_position(Point<float> position, int numSamples, double weight, std::vector<particle_t>& samples)
{
    particle_t sample;
    
    while(--numSamples >= 0)
    {
        sample.id     = samples.size();
        sample.pose   = pose_t(position.x, position.y, random_orientation());
        sample.weight = weight;
        
        samples.push_back(sample);
    }
}

}
}
