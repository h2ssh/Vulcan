/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lambda.h
* \author   Collin Johnson
*
* Declaration of Lambda describing transform between frames of
* reference along a topological path. Includes functions for calculating
* and evaluating lambdas.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LAMBDA_H
#define HSSH_LOCAL_TOPOLOGICAL_LAMBDA_H

#include "core/multivariate_gaussian.h"
#include <cereal/access.hpp>
#include <ostream>

namespace vulcan
{
struct pose_t;
struct pose_distribution_t;
namespace hssh
{

/**
* Lambda represents the pose transform between the reference frame of two places. The transform is (x, y, theta),
* where x and y are
*/
struct Lambda
{
    float x;
    float y;
    float theta;

    float xVariance;
    float yVariance;
    float thetaVariance;

    MultivariateGaussian transformDistribution;       // Gaussian distribution representing the transform --
                                                            // allows likelihood calculation

    // Constructors
    explicit Lambda(float x = 0, float y = 0, float theta = 0, float xVariance = 0.5f, float yVariance = 0.5f, float
        thetaVariance = 0.1f);
    Lambda(const pose_t& end, const pose_t& reference);
    Lambda(const pose_distribution_t& end, const pose_distribution_t& reference);

    /**
    * apply applies the lambda to the given reference pose. apply applies the homogeneous transform to get the location
    * of the endpoint relative to the given reference.
    *
    * \param    origin          Origin pose that is the reference
    * \return   Other end of the lambda after apply homogeneous transform defined by lambda.
    */
    pose_distribution_t apply(const pose_distribution_t& origin) const;

    /**
    * invert flips the frame of reference for the lambda value. Instead of A->B, the transform is B->A
    * This isn't just making all the signs negative. It's a rotation of the coordinate frame in -theta direction.
    *
    * \return   Inverted lambda.
    */
    Lambda invert(void) const;

    /**
    * rotate rotates the lambda by the specified amount.
    *
    * \param    angle           Angle to rotate
    * \return   New Lambda with rotated appropriately.
    */
    Lambda rotate(float angle) const;

    /**
    * magnitude calculates the magnitude of the displacement vector between the two frames of reference.
    *
    * \return   Magnitude of the displacement. The length of the vector running between the two places.
    */
    double magnitude(void) const;

    /**
    * The heading of the labmda is the direction the vector from the origin to the end points: atan2(y, x).
    */
    double heading(void) const;

    /**
    * merge merges two Lambdas together using a Kalman filter to update the estimate for a given lambda value.
    *
    * \param    newLambda           Newly measured Lambda to be merged
    */
    void merge(const Lambda& newLambda);

private:

    void setupDistribution(void);

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void load(Archive& ar)
    {
        ar (x,
            y,
            theta,
            xVariance,
            yVariance,
            thetaVariance,
            transformDistribution);

        setupDistribution();
    }

    template <class Archive>
    void save(Archive& ar) const
    {
        ar (x,
            y,
            theta,
            xVariance,
            yVariance,
            thetaVariance,
            transformDistribution);
    }
};


inline std::ostream& operator<<(std::ostream& out, const Lambda& lambda)
{
    out << "mean: (" << lambda.x << ',' << lambda.y << ',' << lambda.theta
        << ") var: (" << lambda.xVariance << ',' << lambda.yVariance << ',' << lambda.thetaVariance << ')';
    return out;
}

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_LAMBDA_H
