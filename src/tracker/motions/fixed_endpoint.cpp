/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     fixed_endpoint.cpp 
* \author   Collin Johnson
* 
* Defintion of FixedEndpointMotion.
*/

#include <tracker/motions/fixed_endpoint.h>
#include <tracker/motions/visitor.h>
#include <tracker/laser_object.h>
#include <tracker/object_state.h>

namespace vulcan
{
namespace tracker
{
    
FixedEndpointMotion::FixedEndpointMotion(void)
{
}
    

bool FixedEndpointMotion::isPivoting(void) const
{
    return model_.isPivoting();
}


bool FixedEndpointMotion::isSliding(void) const
{
    return model_.isSliding();
}


void FixedEndpointMotion::accept(ObjectMotionVisitor& visitor) const
{
    return visitor.visitFixedEndpoint(*this);
}


std::unique_ptr<ObjectMotion> FixedEndpointMotion::clone(void) const
{
    return std::make_unique<FixedEndpointMotion>(*this);
}


ObjectMotionStatus FixedEndpointMotion::modelStatus(void) const
{
    if(model_.isUndetermined())
    {
        return ObjectMotionStatus::undetermined;
    }
    else if(model_.isValid())
    {
        return ObjectMotionStatus::valid;
    }
    else 
    {
        return ObjectMotionStatus::invalid;
    }
}


object_motion_state_t FixedEndpointMotion::updateMotionEstimate(const LaserObject& object)
{
    model_.updateModel(object.minErrorBoundary());
    return object_motion_state_t(model_.position().x, model_.position().y, 0.0f, 0.0f);
}


Position FixedEndpointMotion::estimateFuturePosition(int deltaTimeMs) const
{
    std::cout << "STUB! FixedEndpointMotion::estimateFuturePosition\n";
    return model_.position();
}

} // namespace tracker
} // namespace vulcan
