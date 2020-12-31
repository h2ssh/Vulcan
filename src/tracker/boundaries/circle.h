/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     circle.h
 * \author   Collin Johnson
 *
 * Declaration of CircleBoundary.
 */

#ifndef TRACKER_BOUNDARIES_CIRCLE_H
#define TRACKER_BOUNDARIES_CIRCLE_H

#include "tracker/boundaries/shapes.h"

namespace vulcan
{
namespace tracker
{

/**
 * CircleBoundary is an estimator for an object with a circle boundary being tracked by a laser. The radius of the
 * circle is estimated. The current model at a desired position can be retrieved with the atPosition method which will
 * return the estimated boundary centered at that position.
 *
 * The radius estimation uses a simple weighted mean of all measurements of the circle. Each measurement is an Arc of
 * a portion of the complete circle boundary. The weight of an individual measurement is:
 *
 *       central angle / 2pi
 *
 * The motivation is the greater the observed central angle, the more constrained the circle fit was, and therefore, the
 * higher quality. A nearly straight line produces a very large radius circle, which means the weight is extremely low.
 * Seeing half a circle (the greatest possible measurement) gives a weight of 0.5.
 */
class CircleBoundary
{
public:
    /**
     * Default constructor for CircleBoundary.
     */
    CircleBoundary(void);

    /**
     * addEstimate adds another estimate of the boundary from laser data.
     *
     * \param    arc         Arc estimate of the boundary
     */
    void addEstimate(Arc arc);

    /**
     * atPosition retrieves the estimated boundary at the given position.
     *
     * \param    position            Position for the center of the estimated boundary
     * \return   Circle centered at position with the estimated radius.
     */
    Circle atPosition(Position position) const { return Circle(radius_, position); }

private:
    double radius_;
    double totalWeight_;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_BOUNDARIES_CIRCLE_H
