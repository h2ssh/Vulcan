/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_boundary.cpp
* \author   Collin Johnson
* 
* Definition of ObjectBounday.
*/

#include "tracker/object_boundary.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <iostream>

// #define DEBUG_SHAPE_DECISION

namespace vulcan
{
namespace tracker
{

using namespace boost::accumulators;
using DistAcc = accumulator_set<double, stats<tag::mean, tag::min>>;


std::ostream& operator<<(std::ostream& out, BoundaryType type)
{
    switch(type)
    {
    case BoundaryType::rectangle:
        out << "rectangle";
        break;
    case BoundaryType::one_circle:
        out << "one-circle";
        break;
    case BoundaryType::two_circles:
        out << "two-circles";
        break;
    case BoundaryType::two_rects:
        out << "two-rectangles";
        break;
    case BoundaryType::circle_rect:
        out << "circle-rect";
        break;
    case BoundaryType::best:
        out << "best";
        break;
    case BoundaryType::unknown:
        out << "unknown";
        break;
    }
    
    return out;
}


struct distance_visitor : public boost::static_visitor<std::tuple<double, double>>
{
    ConstPointIter begin;
    ConstPointIter end;
    
    std::tuple<double, double> operator()(const Rectangle&  rect);
    std::tuple<double, double> operator()(const Circle&     circle);
    std::tuple<double, double> operator()(const TwoCircles& circles);
    std::tuple<double, double> operator()(const CircleRect& circleRect);
    std::tuple<double, double> operator()(const TwoRects&   rects);
    
    distance_visitor(ConstPointIter begin, ConstPointIter end)
    : begin(begin)
    , end(end)
    {
    }
};

struct object_dist_visitor : public boost::static_visitor<std::tuple<double, double>>
{
    ConstPointIter begin;
    ConstPointIter end;
    
    std::tuple<double, double> operator()(const Rectangle&  rect);
    std::tuple<double, double> operator()(const Circle&     circle);
    std::tuple<double, double> operator()(const TwoCircles& circles);
    std::tuple<double, double> operator()(const CircleRect& circleRect);
    std::tuple<double, double> operator()(const TwoRects&   rects);
    
    object_dist_visitor(ConstPointIter begin, ConstPointIter end)
    : begin(begin)
    , end(end)
    {
    }
};

struct move_boundary_visitor : public boost::static_visitor<ObjectBoundary>
{
    Position deltaPosition;
    
    ObjectBoundary operator()(const Rectangle&  rect);
    ObjectBoundary operator()(const Circle&     circle);
    ObjectBoundary operator()(const TwoCircles& circles);
    ObjectBoundary operator()(const CircleRect& circleRect);
    ObjectBoundary operator()(const TwoRects&   rects);
    
    move_boundary_visitor(const Position& deltaPosition)
    : deltaPosition(deltaPosition)
    {
    }
};


ObjectBoundary::ObjectBoundary(void)
{
}


ObjectBoundary::ObjectBoundary(const Rectangle& rectangle)
{
    assign(rectangle, BoundaryType::rectangle);
}


ObjectBoundary::ObjectBoundary(const Circle& circle)
{
    assign(circle, BoundaryType::one_circle);
}


ObjectBoundary::ObjectBoundary(const TwoCircles& circles)
{
    assign(circles, BoundaryType::two_circles);
}


ObjectBoundary::ObjectBoundary(const TwoRects& rects)
{
    assign(rects, BoundaryType::two_rects);
}


ObjectBoundary::ObjectBoundary(const CircleRect& circleRect)
{
    assign(circleRect, BoundaryType::circle_rect);
}


void ObjectBoundary::updateBoundary(const ObjectBoundary& boundary)
{
    // TODO: Intelligent boundary update, rather than just copying over the existing model
    *this = boundary;
}


void ObjectBoundary::updateApproximation(Circle approx)
{
    const double kNewCircleWeight = 0.6;
    const double kOldCircleWeight = 1.0 - kNewCircleWeight;

    Circle newCircle = Circle((circleApproximation_.radius() * kOldCircleWeight) + (approx.radius() * kNewCircleWeight),
                              approx.center());
    circleApproximation_ = newCircle;
}


std::tuple<double, double> ObjectBoundary::distanceFromBoundary(ConstPointIter begin, ConstPointIter end) const
{
    if(begin == end)
    {
        return std::make_tuple(1000.0, 1000.0);
    }
    else
    {
        return visitShape(distance_visitor(begin, end));
    }
}


std::tuple<double, double> ObjectBoundary::distanceToObject(ConstPointIter begin, ConstPointIter end) const
{
    if(begin == end)
    {
        return std::make_tuple(1000.0, 1000.0);
    }
    else
    {
        return visitShape(object_dist_visitor(begin, end));
    }
}


ObjectBoundary ObjectBoundary::boundaryAtPosition(Position position) const
{
    auto centerDelta = position - center_;
    auto movedBoundary = visitShape(move_boundary_visitor(centerDelta));
    movedBoundary.assignApproximation(Circle(circleApproximation_.radius(), movedBoundary.position()));
    return movedBoundary;
}


std::tuple<double, double> distance_visitor::operator()(const Rectangle& rect)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(rect.distanceFromBoundary(p));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> distance_visitor::operator()(const Circle& circle)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(circle.distanceFromBoundary(p));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> distance_visitor::operator()(const TwoCircles& circles)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(std::min(circles[0].distanceFromBoundary(p), circles[1].distanceFromBoundary(p)));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> distance_visitor::operator()(const CircleRect& circleRect)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(std::min(circleRect.circle.distanceFromBoundary(p), circleRect.rect.distanceFromBoundary(p)));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> distance_visitor::operator()(const TwoRects& rects)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(std::min(rects[0].distanceFromBoundary(p), rects[1].distanceFromBoundary(p)));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> object_dist_visitor::operator()(const Rectangle& rect)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(rect.distanceToPoint(p));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> object_dist_visitor::operator()(const Circle& circle)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(circle.distanceToPoint(p));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> object_dist_visitor::operator()(const TwoCircles& circles)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(std::min(circles[0].distanceToPoint(p), circles[1].distanceToPoint(p)));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> object_dist_visitor::operator()(const CircleRect& circleRect)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(std::min(circleRect.circle.distanceToPoint(p), circleRect.rect.distanceToPoint(p)));
    });

    return std::make_tuple(min(acc), mean(acc));
}


std::tuple<double, double> object_dist_visitor::operator()(const TwoRects& rects)
{
    DistAcc acc;
    std::for_each(begin, end, [&](Position p) {
        acc(std::min(rects[0].distanceToPoint(p), rects[1].distanceToPoint(p)));
    });

    return std::make_tuple(min(acc), mean(acc));
}


ObjectBoundary move_boundary_visitor::operator()(const Rectangle& rect)
{
    auto movedRect = rect;
    movedRect.translate(deltaPosition.x, deltaPosition.y);
    return ObjectBoundary(movedRect);
}


ObjectBoundary move_boundary_visitor::operator()(const Circle& circle)
{
    auto movedCircle = circle;
    movedCircle.translate(deltaPosition.x, deltaPosition.y);
    return ObjectBoundary(movedCircle);
}


ObjectBoundary move_boundary_visitor::operator()(const TwoCircles& circles)
{
    auto movedCircles = circles;
    movedCircles[0].translate(deltaPosition.x, deltaPosition.y);
    movedCircles[1].translate(deltaPosition.x, deltaPosition.y);
    return ObjectBoundary(movedCircles);
}


ObjectBoundary move_boundary_visitor::operator()(const CircleRect& circleRect)
{
    auto movedCircleRect = circleRect;
    movedCircleRect.circle.translate(deltaPosition.x, deltaPosition.y);
    movedCircleRect.rect.translate(deltaPosition.x, deltaPosition.y);
    return ObjectBoundary(movedCircleRect);
}


ObjectBoundary move_boundary_visitor::operator()(const TwoRects& rects)
{
    auto movedRects = rects;
    movedRects[0].translate(deltaPosition.x, deltaPosition.y);
    movedRects[1].translate(deltaPosition.x, deltaPosition.y);
    return ObjectBoundary(movedRects);
}

} // namespace tracker
} // namespace vulcan
