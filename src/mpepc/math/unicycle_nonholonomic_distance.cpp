/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     unicycle_nonholonomic_distance.cpp
* \author   Jong Jin Park
*
* Definitions for nonholonomic distance functions
*/

#include <mpepc/math/unicycle_nonholonomic_distance.h>
#include <mpepc/math/egocentric_polar_coordinates.h>
#include <cmath>
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace mpepc
{

// distance to target in position space
double distance_on_manifold(double r, double phi, double kPhi)
{
    double weightedPhi = kPhi*phi;
    
    return sqrt(r*r + weightedPhi*weightedPhi);
};

// stabilizing vector fields
double gradient_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon)
{
    if(r > rangeEpsilon)
    {
        return atan(-kPhi*phi/r/r);
    }
    else
    {
        return 0; // avoid numerical error and align with target orientation
    }
};

double smooth_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon)
{
    if(r > rangeEpsilon)
    {
        return atan(-kPhi*phi);
    }
    else
    {
        return 0; // avoid numerical error and align with target orientation
    }
};

double stabilizing_delta_star(double r, double phi, double kPhi, stabilizing_vector_field_type_t vectorFieldType, double rangeEpsilon)
{
    double deltaStar;
    switch(vectorFieldType)
    {
        case GRADIENT_DESCENT:
            deltaStar = gradient_descent_on_manifold(r, phi, kPhi, rangeEpsilon);
            break;
        case SMOOTH_DESCENT:
            deltaStar = smooth_descent_on_manifold(r, phi, kPhi, rangeEpsilon);
            break;
        default:
            std::cout<<"ERROR: UnicycleLyapunovChart: Unknown vector field type.\n";
            assert(false);
            break;
    }
    
    return deltaStar;
};


// distance to a target pose over the manifold equipped with stabilizing vector field
double distance_to_manifold(double r, double phi, double delta, double kPhi, double kDelta, stabilizing_vector_field_type_t vectorFieldType, double rangeEpsilon)
{
    double deltaStar = stabilizing_delta_star(r, phi, kPhi, vectorFieldType, rangeEpsilon);
    
    return kDelta * fabs(delta - deltaStar);
};


// distance to target pose in egocentric polar coordinates
double unicycle_nonholonomic_distance(double r, double phi, double delta, double kPhi, double kDelta, stabilizing_vector_field_type_t vectorFieldType, double rangeEpsilon)
{
    double distanceOnManifold = distance_on_manifold(r, phi, kPhi);
    double distanceTOManifold = distance_to_manifold(r, phi, delta, kPhi, kDelta, vectorFieldType, rangeEpsilon);
    
    return distanceOnManifold + distanceTOManifold;
};

} // mpepc
} // vulcan
