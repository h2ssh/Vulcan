/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gaussian_creator.cpp
* \author   Collin Johnson
* 
* Definition of GaussianCreator.
*/

#include <ui/common/gaussian_creator.h>

namespace vulcan
{
namespace ui
{
    
GaussianCreator::GaussianCreator(double xVariance, double yVariance, double thetaVariance)
    : mean(3)
    , covariance(3, 3)
{
    setVariances(xVariance, yVariance, thetaVariance);
    reset();
}


void GaussianCreator::setVariances(double xVariance, double yVariance, double thetaVariance)
{
    covariance.zeros();
    covariance(0,0) = xVariance;
    covariance(1,1) = yVariance;
    covariance(2,2) = thetaVariance;
}


void GaussianCreator::reset(void)
{
    gaussianIsDone = false;
    
    mean.zeros();
}


GLEventStatus GaussianCreator::handleLeftMouseDown(const GLMouseEvent& event)
{
    thetaPosition = event.glCoords;
    leftWasDown   = true;

    return GLEventStatus::capture;
}


GLEventStatus GaussianCreator::handleLeftMouseUp(const GLMouseEvent& event)
{
    gaussianIsDone = true;
    
    return GLEventStatus::passthrough;
}


GLEventStatus GaussianCreator::handleMouseMoved(const GLMouseEvent& event)
{
    if(!event.leftIsDown)
    {
        mean(0) = event.glCoords.x;
        mean(1) = event.glCoords.y;
    }
    else // if(event.leftIsDown)
    {
        mean(2) = angle_to_point(thetaPosition, event.glCoords);
        return GLEventStatus::capture;
    }
    
    return GLEventStatus::passthrough;
}
    
} // namespace ui
} // namespace vulcan
