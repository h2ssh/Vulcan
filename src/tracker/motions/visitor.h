/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visitor.h
* \author   Collin Johnson
* 
* Declaration of ObjectMotionVisitor interface.
*/

#ifndef TRACKER_MOTIONS_OBJECT_MOTION_VISITOR_H
#define TRACKER_MOTIONS_OBJECT_MOTION_VISITOR_H

namespace vulcan
{
namespace tracker
{
    
class StridingMotion;
class SteadyMotion;
class FixedEndpointMotion;
class StationaryMotion;

/**
* ObjectMotionVisitor is an interface for a visitor of the ObjectMotion hierarchy.
*/
class ObjectMotionVisitor
{
public:

    virtual ~ObjectMotionVisitor(void) {}
    
    virtual void visitStriding     (const StridingMotion&      motion) = 0;
    virtual void visitSteady       (const SteadyMotion&        motion) = 0;
    virtual void visitFixedEndpoint(const FixedEndpointMotion& motion) = 0;
    virtual void visitStationary   (const StationaryMotion&    motion) = 0;
};

}
}

#endif // TRACKER_MOTIONS_OBJECT_MOTION_VISITOR_H
