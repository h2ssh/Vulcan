/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_nonholonomic_distance.h
 * \author   Jong Jin Park
 *
 * Declaration of distance functions relavent for unicycle-type vehicles with
 * non-holonomic constraints (that can rotate in place but cannot move sideways).
 * Supports two (analytically describable) stabilizing vector field types.
 * Uses egocentric polar coordinates.
 */

#ifndef UNICYCLE_NONHOLONOMIC_DISTANCE_H
#define UNICYCLE_NONHOLONOMIC_DISTANCE_H

namespace vulcan
{
namespace mpepc
{

enum stabilizing_vector_field_type_t
{
    GRADIENT_DESCENT,
    SMOOTH_DESCENT
};

// distance to target in position space
double distance_on_manifold(double r, double phi, double kPhi);

// stabilizing vector fields
double gradient_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon = 0.05);

double smooth_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon = 0.05);

double stabilizing_delta_star(double r,
                              double phi,
                              double kPhi,
                              stabilizing_vector_field_type_t vectorFieldType,
                              double rangeEpsilon = 0.05);

// distance to a target pose over the manifold equipped with stabilizing vector field
double distance_to_manifold(double r,
                            double phi,
                            double delta,
                            double kPhi,
                            double kDelta,
                            stabilizing_vector_field_type_t vectorFieldType,
                            double rangeEpsilon = 0.05);

// distance to target pose in egocentric polar coordinates
double unicycle_nonholonomic_distance(double r,
                                      double phi,
                                      double delta,
                                      double kPhi,
                                      double kDelta,
                                      stabilizing_vector_field_type_t vectorFieldType,
                                      double rangeEpsilon = 0.05);

}   // namespace mpepc
}   // namespace vulcan

#endif   // UNICYCLE_NONHOLONOMIC_DISTANCE_H
