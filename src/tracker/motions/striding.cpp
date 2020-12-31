/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     striding.cpp
* \author   Collin Johnson
*
* Definition of StridingMotion.
*/

#include "tracker/motions/striding.h"
#include "tracker/motions/visitor.h"
#include "tracker/utils/endpoints.h"
#include "tracker/laser_object.h"
#include "math/statistics.h"
#include "utils/timestamp.h"
#include <cassert>

// #define DEBUG_ENDPOINTS
// #define DEBUG_PHASE_END
// #define DEBUG_INFO

namespace vulcan
{
namespace tracker
{

// Parameters for a valid stride
const float kMaxStride = 1.5f;
const float kMinPeriod = 0.1f;
const float kMaxPeriod = 2.0f;
const float kMinAngle  = 0.25f;
const float kMaxWidth  = 1.0f;

// Size of the median filter used for smoothing the calculated stride angles
const std::size_t kFilterRadius = 3;
const std::size_t kFilterSize   = 1 + (kFilterRadius * 2);

struct endpoint_visitor : boost::static_visitor<std::pair<Endpoints, int>>
{
    float direction;
    float prevAngle;
    int   numPoints;
    
    std::pair<Endpoints, int> operator()(const Rectangle&  rect);
    std::pair<Endpoints, int> operator()(const Circle&     circle);
    std::pair<Endpoints, int> operator()(const TwoCircles& circles);
    std::pair<Endpoints, int> operator()(const CircleRect& circleRect);
    std::pair<Endpoints, int> operator()(const TwoRects&   rects);
    
    endpoint_visitor(float direction, float prevAngle, int numPoints)
    : direction(direction)
    , prevAngle(prevAngle)
    , numPoints(numPoints)
    {
    }
};


float stride_angle(const Position& first, const Position& second, float direction);
Endpoints order_endpoints_to_minimize_angle_diff(const Endpoints& endpoints, float prevAngle, float direction);


StridingMotion::StridingMotion(void)
{
}


StridingMotion::StridingMotion(int64_t timestamp, const Position& position, const tracking_filter_params_t& params)
: ObjectMotion(position, velocity_t(0.0f, 0.0f))
, swingStartPosition_(position)
, numEndpoints_(0)
, numDataPoints_(0)
, motionTracker_(timestamp, position, params)
{
}


StridingMotion::StridingMotion(const StridingMotion& lhs) = default;


StridingMotion::~StridingMotion(void)
{
    // For std::unique_ptr
}


float StridingMotion::direction(void) const
{
    return std::atan2(motionTracker_.slowState().y, motionTracker_.slowState().x);
}


float StridingMotion::period(void) const
{
    return period_.mean();
}


float StridingMotion::stride(void) const
{
    return strideLength_.mean();
}


float StridingMotion::width(void) const
{
    return width_.mean();
}


void StridingMotion::accept(ObjectMotionVisitor& visitor) const
{
    visitor.visitStriding(*this);
}


std::unique_ptr<ObjectMotion> StridingMotion::clone(void) const
{
    return std::make_unique<StridingMotion>(*this);
}


ObjectMotionStatus StridingMotion::modelStatus(void) const
{
    // Enough data needs to have arrived in order to start making estimates
    if(!haveEnoughData() && (distance_between_points(position(), swingStartPosition_) < kMaxStride))
    {
        return ObjectMotionStatus::undetermined;
    }
    // There's enough data so ensure the model being created is reasonable.
    else if(haveValidModel())
    {
        return ObjectMotionStatus::valid;
    }
    else
    {
        return ObjectMotionStatus::invalid;
    }
}


object_motion_state_t StridingMotion::updateMotionEstimate(const LaserObject& object)
{
    motionTracker_.update(object);

    auto newEndpoints = findCurrentEndpoints(object.minErrorBoundary());

    if(numEndpoints_ == 2)
    {
        width_.addSample(distance_between_points(newEndpoints[0], newEndpoints[1]));
        addStrideAngle(object.timestamp(), newEndpoints);

        if(strideAngles_.angles.hasEnoughDataForExtremum())
        {
            auto endIndex = findPhaseEndIndex();

            if(endIndex < strideAngles_.angles.size())
            {
                auto swingStart = strideAngles_.timestamps[0];
                auto swingEnd   = strideAngles_.timestamps[endIndex];
                double stridePeriod = utils::usec_to_sec(swingEnd - swingStart);

                period_.addSample(stridePeriod);
                
                swingStartPosition_ = Position(motionTracker_.fastState().x, motionTracker_.fastState().y);
                
                auto zeroDist = distance_between_points(strideAngles_.endpoints[0][0],
                                                              strideAngles_.endpoints[endIndex][0]);
                auto oneDist = distance_between_points(strideAngles_.endpoints[0][1],
                                                             strideAngles_.endpoints[endIndex][1]);
                
                strideLength_.addSample(std::max(zeroDist, oneDist));
                
//                 float stepAngle = wrap_to_pi((wrap_to_2pi(medianAngles_[endIndex]) +
//                     wrap_to_2pi(medianAngles_.front())) / 2.0f);
//                 float stepDir = angle_diff(medianAngles_[endIndex], medianAngles_.front());
//                 float direction = angle_sum(stepAngle, std::copysign(M_PI_2, stepDir));
//                 std::cout << "Step angle:" << stepAngle << " Dir:" << direction << '\n';
                
                maxAngle_.addSample(std::abs(angle_diff(strideAngles_.angles[endIndex],
                                                              strideAngles_.angles.front())));

                // Erase the previous stride so only changes in the current stride are detected
                // Can use endIndex because it is offset by the filter radius so this will leave enough data in the
                // stride angles to keep the median filter functioning correctly
                strideAngles_.timestamps.erase(strideAngles_.timestamps.begin(), strideAngles_.timestamps.begin() + endIndex + 1);
                strideAngles_.angles.erase(endIndex + 1);
                strideAngles_.endpoints.erase(strideAngles_.endpoints.begin(), strideAngles_.endpoints.begin() + endIndex + 1);
                
                printStrideInfo();
                ++numDataPoints_;
            }
        }
        
        lastEndpoints_ = newEndpoints;

        if(!strideAngles_.angles.empty())
        {
            lastEndpointAngle_ = strideAngles_.angles.back();
        }
        
        printStrideInfo();
    }

    return motionTracker_.slowState();
}


Position StridingMotion::estimateFuturePosition(int deltaTimeMs) const
{
    // TODO: Estimate motion based on the striding legs
    double deltaTime = deltaTimeMs / 1000.0;
    
    Position estimatedPos = position();
    estimatedPos.x += velocity().x * deltaTime;
    estimatedPos.y += velocity().y * deltaTime;
    
    return estimatedPos;
}


bool StridingMotion::haveEnoughData(void) const
{
    return numDataPoints_ > 2;
}


bool StridingMotion::haveValidModel(void) const
{
    // The stride needs to be short enough and the period not too long
    
    return (stride() < kMaxStride) &&
        (period() > kMinPeriod) &&
        (period() < kMaxPeriod) &&
        (width() < kMaxWidth) && 
        (maxAngle_.mean() > kMinAngle) && 
        (strideLength_.mean() < kMaxStride);
}


StridingMotion::Endpoints StridingMotion::findCurrentEndpoints(const ObjectBoundary& boundary)
{
    auto newEndpoints = boundary.visitShape(endpoint_visitor(direction(), lastEndpointAngle_, numEndpoints_));
    numEndpoints_ = newEndpoints.second;
    
#ifdef DEBUG_ENDPOINTS
    std::cout << "DEBUG: StridingMotion: Endpoints: Prev:(" << lastEndpoints_[0] << "->" << lastEndpoints_[1]
              << " New:" << newEndpoints.first[0] << "->" << newEndpoints.first[1] 
              << " Num:" << newEndpoints.second << '\n';
#endif

    return newEndpoints.first;
}


void StridingMotion::addStrideAngle(int64_t timestamp, const Endpoints& endpoints)
{
    assert(strideAngles_.timestamps.size() == strideAngles_.angles.size());
    
    // The stride angle is the angle between the legs relative to the direction of motion
    // A neutral stance is when the legs are side-by-side. In this orientation, the legs are orthogonal to the
    // direction of motion
    // The stride measures deviations from this neutral position
    // The deviations are in the range [-pi/2, pi/2] because going greater than pi/2 would just be flipping the
    // endpoints with one another and doesn't make physical sense
    
    strideAngles_.timestamps.push_back(timestamp);
    strideAngles_.angles.push_back(stride_angle(endpoints[0], endpoints[1], direction()));
    strideAngles_.endpoints.push_back(endpoints);
}


std::size_t StridingMotion::findPhaseEndIndex(void)
{
    auto angleExtremum = strideAngles_.angles.extremumAfter(0);

    if(angleExtremum)
    {
        std::cout << "DEBUG: Found phase end:" << angleExtremum->index << ":" << angleExtremum->value << '\n';

        return angleExtremum->index;
    }
    else
    {
        return strideAngles_.timestamps.size();
    }
}


void StridingMotion::printStrideInfo(void)
{
#ifdef DEBUG_INFO
    std::cout << "DEBUG:StridingMotion: Stride info:\n"
              << "Width:     " << width_ << '\n'
              << "Period:    " << period_ << '\n'
              << "Max angle: " << maxAngle_ << '\n'
              << "Length:    " << strideLength_ << '\n';
#endif
}


std::pair<Endpoints, int> endpoint_visitor::operator()(const Rectangle& rect)
{
    auto rectEnds = major_axis_endpoints(rect);
    return std::make_pair(order_endpoints_to_minimize_angle_diff(rectEnds, prevAngle, direction), 2);
}


std::pair<Endpoints, int> endpoint_visitor::operator()(const Circle& circle)
{
    std::cerr << "WARNING:StridingMotion: endpoint_visitor visited a circle. Operation not supported.\n";
    Endpoints circleEnds = {circle.center(), circle.center()};
    return std::make_pair(circleEnds, 1);
}


std::pair<Endpoints, int> endpoint_visitor::operator()(const TwoCircles& circles)
{
    auto circleEnds = two_circles_endpoints(circles);
    return std::make_pair(order_endpoints_to_minimize_angle_diff(circleEnds, prevAngle, direction), 2);
}


std::pair<Endpoints, int> endpoint_visitor::operator()(const CircleRect& circleRect)
{
    auto ends = circle_rect_endpoints(circleRect);
    return std::make_pair(order_endpoints_to_minimize_angle_diff(ends, prevAngle, direction), 2);
}


std::pair<Endpoints, int> endpoint_visitor::operator()(const TwoRects& rects)
{
    auto ends = two_rects_endpoints(rects);
    return std::make_pair(order_endpoints_to_minimize_angle_diff(ends, prevAngle, direction), 2);
}


float stride_angle(const Position& first, const Position& second, float direction)
{
    float neutralStanceAngle = 0;//direction;
    float endpointAngle      = angle_to_point(first, second);
    float strideAngle        = angle_diff(endpointAngle, neutralStanceAngle);

    return endpointAngle;
#ifdef DEBUG_ENDPOINTS
    std::cout << "DEBUG:StridingMotion: Neutral:" << neutralStanceAngle << " End:" << endpointAngle
              << " Stride:" << strideAngle << '\n';
#endif

    return strideAngle;
}


Endpoints order_endpoints_to_minimize_angle_diff(const Endpoints& endpoints, float prevAngle, float direction)
{
    float zeroOneAngle = stride_angle(endpoints[0], endpoints[1], direction);
    float oneZeroAngle = stride_angle(endpoints[1], endpoints[0], direction);
    
    if(std::abs(angle_diff(zeroOneAngle, prevAngle)) < std::abs(angle_diff(oneZeroAngle, prevAngle)))
    {
        return endpoints;
    }
    else
    {
        return Endpoints{endpoints[1], endpoints[0]};
    }
}

} // namespace tracker
} // namespace vulcan
