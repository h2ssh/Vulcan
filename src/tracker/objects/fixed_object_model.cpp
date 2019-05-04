/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     fixed_object_model.cpp
* \author   Collin Johnson
* 
* Definition of FixedObjectModel.
*/

#include <tracker/objects/fixed_object_model.h>
#include <tracker/utils/endpoints.h>
#include <iostream>

namespace vulcan
{
namespace tracker 
{
    
using Models = std::vector<EndpointModel>;


// A visitor for the ObjectBoundary to update the boundary
struct endpoint_update_visitor : public boost::static_visitor<bool>
{
    bool operator()(const Rectangle&  rect);
    bool operator()(const Circle&     circle);
    bool operator()(const TwoCircles& circles);
    bool operator()(const CircleRect& circleRect);
    bool operator()(const TwoRects&   rects);
    
    Models& endpoints;
    ObjectBoundary& prevBoundary;
    math::UncertainValue<>& width;
    
    endpoint_update_visitor(Models& endpoints, ObjectBoundary& prevBoundary, math::UncertainValue<>& width)
    : endpoints(endpoints)
    , prevBoundary(prevBoundary)
    , width(width)
    {
    }
};


void update_or_add_endpoint(const Circle& circle, Models& endpoints);
int  update_closest_endpoint(const Position& position, Models& endpoints);
void match_and_update_endpoints(const Endpoints& endpoints, Models& models);


bool is_object_undetermined(const Models& endpoints);
bool is_object_invalid     (const Models& endpoints);
bool is_object_stationary  (const Models& endpoints);
bool is_object_pivoting    (const Models& endpoints);
bool is_object_sliding     (const Models& endpoints);

std::size_t count_endpoints_of_type(const Models& endpoints, EndpointType type);
    

FixedObjectModel::FixedObjectModel(void)
: type_(Type::invalid)
, movingIndex_(-1)
{
}


FixedObjectModel::FixedObjectModel(const ObjectBoundary& boundary)
: FixedObjectModel()
{
    updateModel(boundary);
}


void FixedObjectModel::updateModel(const ObjectBoundary& boundary)
{
    if(updateEndpoints(boundary))
    {
        assert(!endpoints_.empty());
        
        determineType();

        if(isPivoting())
        {
            setPivotingEndpoints();
        }
        else if(isSliding())
        {
            setSlidingEndpoints();
        }
    }
}


ObjectBoundary FixedObjectModel::boundary(void) const
{
    // If the object has been determined to be sliding or pivoting, then the boundary should consist of a rectangle
    // that runs from the moving endpoint to the fixed endpoint
    if(isSliding() || isPivoting())
    {
        return createBoundaryFromEndpoints();
    }
    // Otherwise, the boundary is just the last observed boundary because no additional information is known
    else
    {
        return boundary_;
    }
}


Position FixedObjectModel::fixedPosition(void) const
{
    return fixedPosition_;
}


Position FixedObjectModel::movingPosition(void) const
{
    return (movingIndex_ >= 0) ? endpoints_[movingIndex_].position() : position();
}


Arc FixedObjectModel::estimatedArc(void) const
{
    return (movingIndex_ >= 0) ? endpoints_[movingIndex_].estimatedArc() : Arc();
}


math::angle_range_t FixedObjectModel::angleRange(void) const
{
    return (movingIndex_ >= 0) ? endpoints_[movingIndex_].angleRange(fixedPosition()) : math::angle_range_t();
}


Line<float> FixedObjectModel::estimatedSegment(void) const
{
    return (movingIndex_ >= 0) ? endpoints_[movingIndex_].estimatedSegment() : Line<float>();
}


bool FixedObjectModel::isValid(void) const
{
    return type_ != Type::invalid;
}


bool FixedObjectModel::isUndetermined(void) const
{
    return type_ == Type::undetermined;
}


bool FixedObjectModel::isStationary(void) const
{
    return type_ == Type::stationary;
}


bool FixedObjectModel::isPivoting(void) const
{
    return type_ == Type::pivoting;
}


bool FixedObjectModel::isSliding(void) const
{
    return type_ == Type::sliding;
}


bool FixedObjectModel::updateEndpoints(const ObjectBoundary& boundary)
{
    return boundary.visitShape(endpoint_update_visitor(endpoints_, boundary_, width_));
}


void FixedObjectModel::determineType(void)
{
    if(is_object_invalid(endpoints_))
    {
        type_ = Type::invalid;
    }
    else if(is_object_stationary(endpoints_))
    {
        type_ = Type::stationary;
    }
    else if(is_object_pivoting(endpoints_))
    {
        type_ = Type::pivoting;
    }
    else if(is_object_sliding(endpoints_))
    {
        type_ = Type::sliding;
    }
    else if(is_object_undetermined(endpoints_))
    {
        type_ = Type::undetermined;
    }
    else
    {
        type_ = Type::invalid;
    }
}


void FixedObjectModel::setPivotingEndpoints(void)
{
    if(endpoints_.size() == 1)
    {
        movingIndex_ = 0;
    }
    else
    {
        // If only one of the endpoints is rotating, the answer is simple
        if((endpoints_.front().type() == EndpointType::rotating) &&
            (endpoints_.back().type() != EndpointType::rotating))
        {
            movingIndex_ = 0;
        }
        else if((endpoints_.back().type() == EndpointType::rotating) &&
            (endpoints_.front().type() != EndpointType::rotating))
        {
            movingIndex_ = 1;
        }
        // Otherwise need to decide which of the rotating endpoints is the better model
        else
        {
            // The larger of the two radii is going to represent the full motion of the door the best
            if(endpoints_.front().estimatedArc().radius() > endpoints_.back().estimatedArc().radius())
            {
                movingIndex_ = 0;
            }
            else
            {
                movingIndex_ = 1;
            }
        }
        
        fixedPosition_ = endpoints_[movingIndex_].estimatedArc().center();
    }
}


void FixedObjectModel::setSlidingEndpoints(void)
{
    if(endpoints_.size() == 1)
    {
        movingIndex_ = 0;
    }
    else
    {
        if(endpoints_.front().type() == EndpointType::sliding)
        {
            movingIndex_ = 0;
        }
        else
        {
            assert(endpoints_.back().type() == EndpointType::sliding);
            movingIndex_ = 1;
        }
        
        auto fixedIndex = (movingIndex_ == 0) ? 1 : 0;
        fixedPosition_ = endpoints_[fixedIndex].position();
    }
}


ObjectBoundary FixedObjectModel::createBoundaryFromEndpoints(void) const
{
    // The length is the distance between endpoints if sliding or the radius of the arc if pivoting
    float length = isPivoting() ? estimatedArc().radius() : distance_between_points(movingPosition(),
                                                                                          fixedPosition());
    float width = width_.mean();
    
    // Create an axis-aligned rectangle boundary that has the correct dimensions centered around the origin
    Rectangle boundary(Position(-length/2, -width/2), Position(length/2, width/2));
    // Rotate around the center so the major axis points in the correct direction
    boundary.rotate(angle_to_point(fixedPosition(), movingPosition()));
    // Translate the rectangle so the pre-rotation rectangle bottom left would have sat at (0,0)
    boundary.translate(fixedPosition().x - boundary.bottomLeft.x, fixedPosition().y - boundary.bottomLeft.y);
    return ObjectBoundary(boundary);
}


bool endpoint_update_visitor::operator()(const Rectangle& rect)
{
    auto rectEnds = major_axis_endpoints(rect);
    
    width.addSample(std::min(rect.height(), rect.width()));
    
    // If there aren't any endpoints, create two new ones
    if(endpoints.empty())
    {
        endpoints.emplace_back(rectEnds[0]);
        endpoints.emplace_back(rectEnds[1]);
    }
    // If there's only one endpoint, add a new endpoint furthest from the existing endpoint
    // and update the other one
    else if(endpoints.size() == 1)
    {
        if(distance_between_points(rectEnds[0], endpoints.front().position()) <
            distance_between_points(rectEnds[1], endpoints.front().position()))
        {
            endpoints.front().addPositionMeasurement(rectEnds[0]);
            endpoints.emplace_back(rectEnds[1]);
        }
        else
        {
            endpoints.front().addPositionMeasurement(rectEnds[1]);
            endpoints.emplace_back(rectEnds[0]);
        }
    }
    // Just update both endpoints
    else
    {
        match_and_update_endpoints(rectEnds, endpoints);
    }

    return true;
}


bool endpoint_update_visitor::operator()(const Circle& circle)
{
    return false;
//     if(endpoints.empty())
//     {
//         endpoints.emplace_back(circle.center());
//     }
//     else if(endpoints.size() == 1)
//     {
//         update_or_add_endpoint(circle, endpoints);
//     }
//     else
//     {
//         update_closest_endpoint(circle.center(), endpoints);
//     }
//
//     return true;
}


bool endpoint_update_visitor::operator()(const TwoCircles& circles)
{
    // TODO: How to update TwoCircles?
    return false;
}


bool endpoint_update_visitor::operator()(const CircleRect& circleRect)
{
    // TODO: How to update CircleRect?
    return false;
}


bool endpoint_update_visitor::operator()(const TwoRects& rects)
{
    // TODO: How to update TwoRects?
    return false;
}


void update_or_add_endpoint(const Circle& circle, Models& endpoints)
{
    // When incorporating a circle, need to decide which endpoint it matches.
    // If the circle doesn't contain the current endpoint position, then a second endpoint needs to be added.
    // Otherwise, match whichever one is closer.
    if(circle.contains(endpoints.front().position()))
    {
        endpoints.front().addPositionMeasurement(circle.center());
    }
    else
    {
        endpoints.emplace_back(circle.center());
    }
}


int update_closest_endpoint(const Position& position, Models& endpoints)
{
    assert(endpoints.size() == 2);

    // front endpoint is closer
    if(distance_between_points(position, endpoints.front().position()) <
        distance_between_points(position, endpoints.back().position()))
    {
        endpoints.front().addPositionMeasurement(position);
        return 0;
    }
    else // back endpoint is closer
    {
        endpoints.back().addPositionMeasurement(position);
        return 1;
    }
}


void match_and_update_endpoints(const Endpoints& endpoints, Models& models)
{
    // Calculate the association score for each of the models. Use the association with the lower cumulative
    // score between the two options.
    
    assert(models.size() == 2);
    
    auto zeroScores = models[0].calculateAssociationScores(endpoints);
    auto oneScores  = models[1].calculateAssociationScores(endpoints);
    
    if(zeroScores[0] + oneScores[1] < zeroScores[1] + oneScores[0])
    {
        models[0].addPositionMeasurement(endpoints[0]);
        models[1].addPositionMeasurement(endpoints[1]);
    }
    else
    {
        models[0].addPositionMeasurement(endpoints[1]);
        models[1].addPositionMeasurement(endpoints[0]);
    }
}


bool is_object_undetermined(const Models& endpoints)
{
    // An object is undetermined if any of its endpoints are undetermined
    return count_endpoints_of_type(endpoints, EndpointType::undetermined) > 0;
}


bool is_object_invalid(const Models& endpoints)
{
    // An object is invalid if any of its endpoints are invalid
    return count_endpoints_of_type(endpoints, EndpointType::invalid) > 0;
}


bool is_object_stationary(const Models& endpoints)
{
    // An object is stationary if all of its endpoints are stationary
    return count_endpoints_of_type(endpoints, EndpointType::stationary) == endpoints.size();
}


bool is_object_pivoting(const Models& endpoints)
{
    // An object is pivoting if it has at least one rotating endpoint and only rotating or stationary endpoints
    auto numRotating   = count_endpoints_of_type(endpoints, EndpointType::rotating);
    auto numStationary = count_endpoints_of_type(endpoints, EndpointType::stationary);
    
    return (numRotating > 0) && (numRotating + numStationary == endpoints.size());
}


bool is_object_sliding(const Models& endpoints)
{
    // An object is sliding if it has one sliding endpoint and only stationary endpoints otherwise
    auto numSliding    = count_endpoints_of_type(endpoints, EndpointType::sliding);
    auto numStationary = count_endpoints_of_type(endpoints, EndpointType::stationary);
    
    return (numSliding == 1) && (numSliding + numStationary == endpoints.size());
}


std::size_t count_endpoints_of_type(const Models& endpoints, EndpointType type)
{
    return std::count_if(endpoints.begin(), endpoints.end(),
                        [type](const EndpointModel& model)
                        {
                            return model.type() == type;
                        });
}

} // namespace tracker
} // namespace vulcan
