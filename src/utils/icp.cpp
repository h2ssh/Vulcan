/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     icp.cpp
* \author   Collin Johnson
*
* Definition of functions for doing ICP:
*
*   - icp_2d
*/

#include "utils/icp.h"
#include "core/pose.h"
#include "core/vector.h"
#include "core/matrix.h"
#include "core/point.h"
#include "core/angle_functions.h"
#include <iostream>
#include <cassert>

// #define DEBUG_TRANSFORM

namespace vulcan
{
namespace utils
{

struct icp_pair_t
{
    int   index;
    float distance;
};

void transform_points(const std::vector<Point<float>>& points,
                      const pose_t&                   transform,
                      std::vector<Point<float>>&       transformed);
void match_points(const std::vector<Point<float>>& from,
                  const std::vector<Point<float>>& to,
                  float                                  maxMatchDistance,
                  bool                                   canMatchEndpoints,
                  std::vector<icp_pair_t>&               matches);
icp_pair_t closest_point_index(const Point<float>&              from,
                               const std::vector<Point<float>>& to,
                               float                                  maxMatchDistance);
pose_t find_transform(const std::vector<Point<float>>& from,
                             const std::vector<Point<float>>& to,
                             const std::vector<icp_pair_t>&         matches);
Point<float> mean_point(const std::vector<Point<float>>& points);
bool is_transform_converged(const pose_t& previous, const pose_t& current);


pose_t icp_2d(const std::vector<Point<float>>& from,
                     const std::vector<Point<float>>& to,
                     const pose_t&                   initial)
{
    const int   MAX_ITERATIONS     = 500;
    const float MAX_MATCH_DISTANCE = 0.5f;

    // If either is empty, then can't calculate a transform!
    if(from.empty() || to.empty())
    {
        return pose_t(0.0f, 0.0f, 0.0f);
    }

    std::vector<Point<float>> transformed(from.size());
    std::vector<icp_pair_t>         matchIndices(from.size());
    pose_t previousTransform;
    pose_t transformIncrement;
    pose_t currentTransform  = initial;

    int numIterations = 0;

    do
    {
        previousTransform = currentTransform;

        transform_points(from, previousTransform, transformed);
        match_points(transformed, to, MAX_MATCH_DISTANCE, true, matchIndices);

        transformIncrement = find_transform(transformed, to, matchIndices);

        currentTransform = pose_t(transformIncrement.x + currentTransform.x*std::cos(transformIncrement.theta) - currentTransform.y*std::sin(transformIncrement.theta),
                                         transformIncrement.y + currentTransform.x*std::sin(transformIncrement.theta) + currentTransform.y*std::cos(transformIncrement.theta),
                                         angle_sum(currentTransform.theta, transformIncrement.theta));

    } while(!is_transform_converged(previousTransform, currentTransform) && (numIterations++ < MAX_ITERATIONS));

    return currentTransform;
}


void transform_points(const std::vector<Point<float>>& points,
                      const pose_t&                   transform,
                      std::vector<Point<float>>&       transformed)
{
    Point<float> position = transform.toPoint();

    transformed.resize(points.size());

    for(std::size_t n = 0; n < points.size(); ++n)
    {
        transformed[n] = position + rotate(points[n], transform.theta);
    }
}


void match_points(const std::vector<Point<float>>& from,
                  const std::vector<Point<float>>& to,
                  float                                  maxMatchDistance,
                  bool                                   canMatchEndpoints,
                  std::vector<icp_pair_t>&               matches)
{
    std::size_t start = canMatchEndpoints ? 0 : 1;
    std::size_t end   = canMatchEndpoints ? from.size() : from.size()-1;

    matches.resize(from.size());

    if(!canMatchEndpoints)
    {
        matches.front().index = -1;
        matches.back().index  = -1;
    }

    for(std::size_t n = start; n < end; ++n)
    {
        icp_pair_t match = closest_point_index(from[n], to, maxMatchDistance);

        if(!canMatchEndpoints && (match.index == 0 || static_cast<std::size_t>(match.index+1) == to.size()))
        {
            match.index = -1;
        }

        matches[n] = match;
    }
}


icp_pair_t closest_point_index(const Point<float>&              from,
                               const std::vector<Point<float>>& to,
                               float                                  maxMatchDistance)
{
    float closestDistance = HUGE_VALF;
    int   closestIndex    = -1;

    for(std::size_t n = 0; n < to.size(); ++n)
    {
        float distance = distance_between_points(from, to[n]);

        if((distance < closestDistance) && (distance < maxMatchDistance))
        {
            closestDistance = distance;
            closestIndex    = n;
        }
    }

    if(closestIndex == -1)
    {
        closestDistance = -1.0;
    }

    return {closestIndex, closestDistance};
}


pose_t find_transform(const std::vector<Point<float>>& from,
                             const std::vector<Point<float>>& to,
                             const std::vector<icp_pair_t>&         matches)
{
    /*
    * Finding the transform via ICP involves solving a least-squares estimation problem that minimizes the
    * error between the from and to points. This problem can be solved in closed-form using the SVD. The
    * calculation goes as follows:
    *
    *   - f_bar, t_bar = mean values of points in from and to
    *   - f_rel, t_rel = a point with the mean subtracted out, i.e. from[n] - f_bar
    *   - H = sum(f_rel dot t_rel')  -- 2x2 matrix
    *   - USV' = SVD(H)
    *   - R = VU', rotation matrix of transform
    *   - T = t_bar - R*f_bar, position offset of transform
    */

    assert(from.size() == matches.size());

    auto comparePairsOp = [](const icp_pair_t& lhs, const icp_pair_t& rhs) { return lhs.distance < rhs.distance; };

    float  maxDist = std::max_element(matches.begin(), matches.end(), comparePairsOp)->distance;

    Point<float> fromBar;
    Point<float> toBar;

    int    numValid  = 0;
    double sumWeight = 0.0;

    for(std::size_t n = 0; n < matches.size(); ++n)
    {
        if(matches[n].index == -1)
        {
            continue;
        }

        double weight = (maxDist - matches[n].distance) / maxDist; //(matches[n].distance < 0.001) ? 1.0/0.001 : 1.0/matches[n].distance;
        sumWeight += weight;

        fromBar.x += from[n].x * weight;
        fromBar.y += from[n].y * weight;

        toBar.x += to[matches[n].index].x * weight;
        toBar.y += to[matches[n].index].y * weight;

        ++numValid;
    }

    assert(numValid);

    fromBar.x /= sumWeight;
    fromBar.y /= sumWeight;

    toBar.x /= sumWeight;
    toBar.y /= sumWeight;

    Point<float> fromRel;
    Point<float> toRel;

    Matrix h(2, 2);
    h.zeros();

    for(std::size_t n = 0; n < from.size(); ++n)
    {
        if(matches[n].index == -1)
        {
            continue;
        }

        double weight = 1.0;//(maxDist - matches[n].distance) / sumDist; //(matches[n].distance < 0.001) ? 1.0/0.001 : 1.0/matches[n].distance;

        fromRel = from[n]              - fromBar;
        toRel   = to[matches[n].index] - toBar;

        h(0, 0) += fromRel.x * toRel.x * weight;
        h(0, 1) += fromRel.x * toRel.y * weight;
        h(1, 0) += fromRel.y * toRel.x * weight;
        h(1, 1) += fromRel.y * toRel.y * weight;
    }

    Matrix u;
    Matrix v;
    Vector s;

    arma::svd(u, s, v, h);

    Matrix r = v * arma::trans(u);

    if(arma::det(r) < 0)
    {
        Matrix fix(2, 2);
        fix.zeros();
        fix(0,0) = 1.0;
        fix(1,1) = arma::det(u * arma::trans(v));
        r        = u * fix * arma::trans(v);
    }

    float              rotation = std::atan2(-r(0,1), r(0,0));
    Point<float> position = toBar - rotate(fromBar, rotation);

#ifdef DEBUG_TRANSFORM
    std::cout<<"From:"<<fromBar<<" To:"<<toBar<<" Transform:"<<pose_t(position.x, position.y, rotation)<<'\n'
             <<"Max:"<<maxDist<<' '<<" Sum:"<<sumDist<<' '<<" Rotated:"<<rotate(fromBar, rotation)<<" Other:"<<rotate(fromBar, -rotation)<<'\n';
#endif

    return pose_t(position.x, position.y, rotation);
}


Point<float> mean_point(const std::vector<Point<float>>& points)
{
    assert(!points.empty());

    Point<float> meanPoint;

    for(auto& point : points)
    {
        meanPoint += point;
    }

    return Point<float>(meanPoint.x/points.size(), meanPoint.y/points.size());
}


bool is_transform_converged(const pose_t& previous, const pose_t& current)
{
    const float POSITION_TOLERANCE    = 1e-5;
    const float ORIENTATION_TOLERANCE = 1e-5;

    return (std::abs(previous.x - current.x)         < POSITION_TOLERANCE) &&
           (std::abs(previous.y - current.y)         < POSITION_TOLERANCE) &&
           (std::abs(previous.theta - current.theta) < ORIENTATION_TOLERANCE);
}

} // namespace utils
} // namesapce vulcan
