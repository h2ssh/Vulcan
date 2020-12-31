/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     person.cpp
* \author   Collin Johnson
* 
* Definition of Person.
*/

#include "tracker/objects/person.h"
#include "tracker/dynamic_object_visitor.h"
#include "tracker/laser_object.h"

namespace vulcan
{
namespace tracker 
{

Person::Person(ObjectId id, int64_t timestamp, const StridingMotion& motion, const ObjectBoundary& boundary)
: BaseType(timestamp, motion, boundary)
, id_(id)
{
}


void Person::updateModel(const LaserObject& object)
{
    BaseType::updateModel(object);
    
    if(canSeeTwoLegs(object))
    {
        assert(object.estimatedTwoArcs());
        updateTwoLegs(object.estimatedTwoArcs()->shape);
    }
    else
    {
        updateSingleLeg(object.estimatedArc().shape);
    }
    
    createBoundaryForLegs();
}


std::unique_ptr<DynamicObject> Person::clone(void) const
{
    return std::unique_ptr<DynamicObject>(new Person(*this));
}


void Person::accept(DynamicObjectVisitor& visitor) const
{
    visitor.visitPerson(*this);
}


bool Person::canSeeTwoLegs(const LaserObject& object) const
{
    return object.estimatedTwoArcs()    // Two arcs must have been fit to see two legs
        // Two legs are seen if the TwoArcs error is less than the single Arc error
        && (object.estimatedTwoArcs()->error <= object.estimatedArc().error); 
}


void Person::updateSingleLeg(Arc measuredLeg)
{
    // If there are no legs, then a new model needs to be created
    if(legModels_.empty())
    {
        legModels_.resize(1);
        lastLegPositions_.resize(1);
    }
    
    // If there's a single leg, then always use that model until the second leg is seen
    if(legModels_.size() == 1)
    {
        legModels_.front().addEstimate(measuredLeg);
        lastLegPositions_.front() = measuredLeg.center();
    }
    else // legModels_.size() == 2
    {
        // Decide which leg is closer
        int closerIndex = distance_between_points(lastLegPositions_[0], measuredLeg.center())
            < distance_between_points(lastLegPositions_[1], measuredLeg.center()) ? 0 : 1;
        
        // And apply the update to it
        legModels_[closerIndex].addEstimate(measuredLeg);
        lastLegPositions_[closerIndex] = measuredLeg.center();
    }
}


void Person::updateTwoLegs(TwoArcs measuredLegs)
{
    // If there are less than two legs, create models for each leg
    if(legModels_.size() < 2)
    {
        legModels_.resize(2);
        lastLegPositions_.resize(2);
    }
    
    // Find the best match between the legs
    double matchedIndexError = distance_between_points(lastLegPositions_[0], measuredLegs[0].center())
        + distance_between_points(lastLegPositions_[1], measuredLegs[1].center());
    double oppositeIndexError = distance_between_points(lastLegPositions_[1], measuredLegs[0].center())
        + distance_between_points(lastLegPositions_[0], measuredLegs[1].center());
    
    // Apply the appropriate update
    if(matchedIndexError < oppositeIndexError)
    {
        for(int n = 0; n < 2; ++n)
        {
            legModels_[n].addEstimate(measuredLegs[n]);
            lastLegPositions_[n] = measuredLegs[n].center();
        }
    }
    else
    {
        legModels_[0].addEstimate(measuredLegs[1]);
        legModels_[1].addEstimate(measuredLegs[0]);
        lastLegPositions_[0] = measuredLegs[1].center();
        lastLegPositions_[1] = measuredLegs[0].center();
    }
}


void Person::createBoundaryForLegs(void)
{
    assert(lastLegPositions_.size() == legModels_.size());
    
    // If there's only a single known leg, the boundary is a single circle
    if(legModels_.size() == 1)
    {
        // The legs are assigned their last estimated position for use in the boundary
        boundary_.assign(legModels_.front().atPosition(lastLegPositions_.front()), 
                         BoundaryType::one_circle);
    }
    // If there are two known legs, the boundary is two circles
    else // legModels_.size() == 2
    {
        TwoCircles legs = { 
            legModels_.front().atPosition(lastLegPositions_.front()),
            legModels_.back().atPosition(lastLegPositions_.back())
        };
        
        boundary_.assign(legs, BoundaryType::two_circles);
    }
}

} // namespace tracker
} // namespace vulcan
