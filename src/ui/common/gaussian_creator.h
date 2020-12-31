/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gaussian_creator.h
 * \author   Collin Johnson
 *
 * Declaration of GaussianCreator.
 */

#ifndef UI_COMMON_GAUSSIAN_CREATOR_H
#define UI_COMMON_GAUSSIAN_CREATOR_H

#include "core/multivariate_gaussian.h"
#include "ui/common/gl_event.h"

namespace vulcan
{
namespace ui
{

/**
 * GaussianCreator is a mouse handler for creating a three-dimensional Gaussian representing a pose.
 * To create a Gaussian, the (x,y,theta) variances must all be specified. The Gaussian creator is
 * intended for setting the position and orientation.
 *
 * To create a Gaussian use the following steps:
 *
 *   1) Set the variances and reset the creator.
 *   2) Move the mouse to the desired (x,y) position of the Gaussian
 *   3) Left click down to start setting the orientation.
 *   4) Move the mouse until the line points in the desired direction and release the mouse button.
 */
class GaussianCreator : public GLMouseHandler
{
public:
    /**
     * Constructor for GaussianCreator.
     *
     * \param    xVariance           Variance along x-axis of the Gaussian being created
     * \param    yVariance           Variance along y-axis of the Gaussian being created
     * \param    thetaVariance       Variance of orientation
     */
    GaussianCreator(double xVariance = 1.0f, double yVariance = 1.0f, double thetaVariance = 1.0f);

    /**
     * setVariances sets the variances for the Gaussian being created.
     *
     * \param    xVariance           Variance along x-axis of the Gaussian being created
     * \param    yVariance           Variance along y-axis of the Gaussian being created
     * \param    thetaVariance       Variance of orientation
     */
    void setVariances(double xVariance, double yVariance, double thetaVariance);

    /**
     * reset resets the
     */
    void reset(void);

    /**
     * isFinished checks to see if the Gaussian is finished being created.
     */
    bool isFinished(void) const { return gaussianIsDone; }

    /**
     * getGaussian retrieves the Gaussian as it has currently been created.
     */
    MultivariateGaussian getGaussian(void) const { return MultivariateGaussian(mean, covariance); }

    // GLMouseHandler interface
    virtual GLEventStatus handleLeftMouseDown(const GLMouseEvent& event);
    virtual GLEventStatus handleLeftMouseUp(const GLMouseEvent& event);
    virtual GLEventStatus handleMouseMoved(const GLMouseEvent& event);

private:
    Vector mean;
    Matrix covariance;

    Point<float> thetaPosition;

    bool leftWasDown;
    bool gaussianIsDone;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_GAUSSIAN_CREATOR_H
