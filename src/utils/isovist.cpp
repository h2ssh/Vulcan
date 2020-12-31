/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist.cpp
* \author   Collin Johnson
*
* Implementation of classes:
*
*   - Isovist
*   - IsovistField
*/

#include "utils/isovist.h"
#include "math/covariance.h"
#include "math/geometry/convex_hull.h"
#include "math/moments_features.h"
#include "math/trigonometry.h"
#include "math/zernike_moments.h"
#include "math/geometry/shape_fitting.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/range/iterator_range.hpp>
#include <cmath>

namespace vulcan
{
namespace utils
{

static math::ZernikeMoments<5> zernike;


double area     (Isovist::Iter begin, Isovist::Iter end);
double perimeter(Isovist::Iter begin, Isovist::Iter end);
double circularity(double area, double perimeter);

template <class BinaryOp> // BinaryOp = double func(Point<double>, Point<double>)
double accumulate_pair(Isovist::Iter begin, Isovist::Iter end, BinaryOp op);


std::string Isovist::scalarName(Scalar scalar)
{
    switch(scalar)
    {
        case kArea:
            return "area";
        case kPerimeter:
            return "perimeter";
        case kCircularity:
            return "circularity";
        case kOrientation:
            return "orientation";
        case kWeightedOrientation:
            return "weighted_orientation";
        case kShapeEccentricity:
            return "eccentricity";
        case kShapeCompactness:
            return "shape compactness";
        case kShapeWaviness:
            return "waviness";
        case kMinDistOrientation:
            return "min_dist_orientation";
        case kMaxThroughDist:
            return "max_through_dist";
        case kMaxThroughDistOrientation:
            return "max_through_dist_orientation";
        case kMinLineDist:
            return "min_line_dist";
        case kMinLineNormal:
            return "min_line_normal";
        case kMinNormalDiff:
            return "min_normal_diff";
        case kDmax:
            return "dist_max";
        case kDmin:
            return "dist_min";
        case kDavg:
            return "dist_avg";
        case kDstd:
            return "dist_std";
        case kDVariation:
            return "dist_variation";
        case kDskewness:
            return "dist_skewness";
        case kShapeDistAvg:
            return "shape_dist_avg";
        case kShapeDistStd:
            return "shape_dist_std";
        case kShapeDistVariation:
            return "shape_dist_variation";
        case kShapeDistCompactness:
            return "shape_dist_compactness";
        case kDeltaAvg:
            return "delta_avg";
        case kDeltaStd:
            return "delta_std";
        case kDeltaVariation:
            return "delta_variation";
        case kDistRelationAvg:
            return "dist_relation_avg";
        case kDistRelationStd:
            return "dist_relation_std";
        case kRayCompactness:
            return "ray compactness";
        case kAngleBetweenMinDists:
            return "angle between min dists";
        case kMinHalfArea:
            return "min half area";
        case kAreaBalance:
            return "area balance";
        default:
            return "unknown";
    }

    return "Unknown";
}


void Isovist::calculateScalars(PointIter begin, PointIter end, std::vector<double>& scalars)
{
    scalars_.resize(kNumScalars + zernike.numMoments());

    calculateDistanceScalars(begin, end, scalars);
    calculateDeltaScalars(begin, end, scalars);
    calculatePolygonScalars(begin, end, scalars);
    calculateMomentsScalars(begin, end, scalars);
}


void Isovist::calculateDistanceScalars(PointIter begin, PointIter end, std::vector<double>& scalars)
{
    using namespace boost::accumulators;

    accumulator_set<double, stats<tag::mean, tag::lazy_variance, tag::skewness, tag::max, tag::min>> distanceAcc;
    accumulator_set<double, stats<tag::mean, tag::max, tag::lazy_variance>> distFromCenterAcc;
    accumulator_set<double, stats<tag::mean, tag::lazy_variance, tag::count>> relationAcc;

    auto shapeCenter = std::accumulate(begin, end, Point<float>(0.0f, 0.0f));
    shapeCenter.x /= std::distance(begin, end);
    shapeCenter.y /= std::distance(begin, end);

    double lastDist = distance_between_points(position_, *(end - 1));
    double minDist = std::numeric_limits<double>::max();
    double minDistOrientation = 0.0;

    for(auto p : boost::make_iterator_range(begin, end))
    {
        double dist = distance_between_points(position_, p);
        distanceAcc(dist);
        distFromCenterAcc(distance_between_points(shapeCenter, p));
        relationAcc(std::abs(dist - lastDist));

        // Keeping track of min dist here for the min dist orientation calculation
        if(dist < minDist)
        {
            minDist = dist;
            minDistOrientation = angle_to_point(position_, p);
        }
        lastDist = dist;
    }

    double secondShortestDist = std::numeric_limits<double>::max();
    double secondShortestOrientation = 0.0;

    // Find the other local minimum. Need to see that it's actually a minimum when doing the computation
    // Only relative distances matter here, so we can use the squared_point_distance for efficiency
    double prevDist = squared_point_distance(position_, *(end-2)); // *begin == *end-1 b/c poly wraps around
    double curDist = squared_point_distance(position_, *begin);

    // Want the min diff to be far enough for the discretization error to not be the source of
    // the next min dist
    const double kMinAngleDiff = std::acos(minDist / (minDist + 0.1));

    for(auto pIt = begin, endIt = end - 1; pIt != endIt; ++pIt)
    {
        double nextDist = (pIt + 1 != endIt) ? squared_point_distance(position_, *(pIt + 1)) :
            squared_point_distance(position_, *begin);

        // If this is a local minima cell, check that it is a little far from the min dist
        // then see if its angle diff is greater than the existing, but it is still within 0.1m
        // of the min dist. If so, it's a discretization error that is causing the miss.
        if((curDist < secondShortestDist)
            && (curDist < nextDist) && (curDist < prevDist))
        {
            // Avoid the angle_to_point except when it matters -- atan2 is expensive!
            double orientation = angle_to_point(position_, *pIt);

            if(angle_diff_abs(orientation, minDistOrientation) > kMinAngleDiff)
            {
                secondShortestDist = curDist;
                secondShortestOrientation = orientation;
            }
        }

        prevDist = curDist;
        curDist = nextDist;
    }

    scalars[kMinDistOrientation] = minDistOrientation;
    scalars[kAngleBetweenMinDists] = angle_diff_abs(minDistOrientation, secondShortestOrientation);

    scalars[kDavg] = mean(distanceAcc);
    scalars[kDstd] = std::sqrt(variance(distanceAcc));
    scalars[kDVariation] = (scalars[kDavg] > 0.0) ? scalars[kDstd] / scalars[kDavg] : 0.0;
    scalars[kDskewness] = skewness(distanceAcc);
    scalars[kDmin] = min(distanceAcc);
    scalars[kDmax] = max(distanceAcc);
    scalars[kShapeDistAvg] = mean(distFromCenterAcc);
    scalars[kShapeDistStd] = std::sqrt(variance(distFromCenterAcc));
    scalars[kShapeDistVariation] = (scalars[kShapeDistAvg] > 0.0) ? scalars[kShapeDistStd] / scalars[kShapeDistAvg] : 0.0;
    scalars[kShapeDistCompactness] = (max(distFromCenterAcc) > 0.0) ? scalars[kShapeDistAvg] / max(distFromCenterAcc) : 1e5;

    scalars[kRayCompactness] = (max(distFromCenterAcc) > 0.0) ? (mean(distFromCenterAcc) / max(distFromCenterAcc)) : 1.0;

    if(count(relationAcc) > 1)
    {
        assert(!std::isnan(mean(relationAcc)));
        scalars[kDistRelationAvg] = mean(relationAcc);
        scalars[kDistRelationStd] = std::sqrt(variance(relationAcc));
    }

    double maxDist = 0.0;
    int maxIdx = 0;

    int otherSide = std::distance(begin, end) / 2;
    for(std::size_t n = 0, end = otherSide; n < end; ++n)
    {
        double dist = squared_point_distance(*(begin + n), *(begin + n + otherSide));
        if(dist > maxDist)
        {
            maxDist = dist;
            maxIdx = n;
        }
    }

    scalars[kMaxThroughDist] = std::sqrt(maxDist);
    scalars[kMaxThroughDistOrientation] = angle_to_point(position_, *(begin + maxIdx));
}


void Isovist::calculateDeltaScalars(PointIter begin, PointIter end, std::vector<double>& scalars)
{
    using namespace boost::accumulators;

    accumulator_set<double, stats<tag::mean, tag::lazy_variance>> deltaAcc;

    auto rayDeltaFunc = [begin, &deltaAcc](int n) {
        deltaAcc(distance_between_points(*(begin + n - 1), *(begin + n)));
    };
    auto rayBegin     = boost::make_counting_iterator<std::size_t>(1);
    auto rayEnd       = boost::make_counting_iterator<std::size_t>(std::distance(begin, end));

    std::for_each(rayBegin, rayEnd, rayDeltaFunc);

    scalars[kDeltaAvg]       = mean(deltaAcc);
    scalars[kDeltaStd]       = std::sqrt(variance(deltaAcc));
    scalars[kDeltaVariation] = (scalars[kDeltaAvg] > 0.0) ? scalars[kDeltaStd] / scalars[kDeltaAvg] : 0.0;
}


void Isovist::calculatePolygonScalars(PointIter begin, PointIter end, std::vector<double>& scalars)
{
    scalars[kArea]        = area(begin, end);
    scalars[kPerimeter]   = perimeter(begin, end);
    scalars[kCircularity] = circularity(scalars[kArea], scalars[kPerimeter]);

    if((scalars[kArea] > 0.0) && (scalars[kPerimeter] > 0.0))
    {
        auto shapeFeatures = math::shape_features(begin, end, scalars[kArea], position_);
        scalars[kWeightedOrientation] = shapeFeatures[math::kShapeWeightedOrientation];
        scalars[kShapeEccentricity] = shapeFeatures[math::kShapeEccentricity];
        scalars[kOrientation] = shapeFeatures[math::kShapeOrientation];
        scalars[kShapeCompactness] = shapeFeatures[math::kShapeCompactness];

        auto hull = math::convex_hull<float>(begin, end);
        scalars[kShapeWaviness] = hull.perimeter() / scalars[kPerimeter];
    }
    else
    {
        scalars[kWeightedOrientation] = 0.0;
        scalars[kShapeEccentricity] = 0.0;
        scalars[kOrientation] = 0.0;
        scalars[kShapeCompactness] = 0.0;
        scalars[kShapeWaviness] = 0.0;
    }
}


void Isovist::calculateMomentsScalars(PointIter begin, PointIter end, std::vector<double>& scalars)
{
    if(scalars[kArea] > 0.0)
    {
        auto zernikeFeatures = zernike.moments<float>(begin,
                                                      end,
                                                      position_,
                                                      scalars[kDmax] * 3);
        std::copy(zernikeFeatures.begin(), zernikeFeatures.end(), scalars.begin() + kNumScalars);
    }
    else
    {
        std::fill(scalars.begin() + kNumScalars, scalars.end(), 0.0);
    }
}


void Isovist::calculateDerivs(std::vector<Point<float>>& endpoints)
{
    // Find the two halves separated by the min dist line. Store those at the end of endpoints. Compute the
    // scalars for each side -- except Zernike. The derivative of each can be easily found with abs of difference.

    std::size_t numPoints = endpoints.size();
    double minDist = HUGE_VAL;
    int minIdx = 0;

    int otherSide = numPoints / 2;
    for(int n = 0, end = otherSide; n < end; ++n)
    {
        double dist = squared_point_distance(endpoints[n], endpoints[n + otherSide]);
        if(dist < minDist)
        {
            minDist = dist;
            minIdx = n;
        }
    }

    scalars_[kMinLineDist] = std::sqrt(minDist);
    scalars_[kMinLineNormal] = angle_sum(angle_to_point(endpoints[minIdx],
                                                                    endpoints[minIdx + otherSide]), M_PI_2);

    // Copy the derivative points on to the end of endpoints. Want to continuous regions, so do the [n,n+otherSide)
    // then [n+otherSide,end) [0,n)
    // add one to num points because the ranges need to wrap around
    endpoints.resize((numPoints + 1) * 2);    // resize to ensure the iterators don't change
    auto halfIt = std::copy(endpoints.begin() + minIdx, endpoints.begin() + minIdx + otherSide, endpoints.begin() + numPoints);
    *halfIt++ = *(endpoints.begin() + numPoints);
    auto endIt = std::copy(endpoints.begin() + minIdx + otherSide, endpoints.begin() + numPoints, halfIt);
    std::copy(endpoints.begin(), endpoints.begin() + minIdx, endIt);
    endpoints.back() = *halfIt;

    std::vector<double> halfScalars(kNumScalars);
    std::vector<double> otherHalfScalars(kNumScalars);
    scalarDerivs_.resize(kNumScalars);

    calculateDistanceScalars(endpoints.begin() + numPoints, halfIt, halfScalars);
    calculateDeltaScalars(endpoints.begin() + numPoints, halfIt, halfScalars);
    calculatePolygonScalars(endpoints.begin() + numPoints, halfIt, halfScalars);
    calculateDistanceScalars(halfIt, endpoints.end(), otherHalfScalars);
    calculateDeltaScalars(halfIt, endpoints.end(), otherHalfScalars);
    calculatePolygonScalars(halfIt, endpoints.end(), otherHalfScalars);

    for(int n = 0; n < kNumScalars; ++n)
    {
        scalarDerivs_[n] = std::abs(halfScalars[n] - otherHalfScalars[n]);
    }

    // Handle wraparound for the angles
    scalarDerivs_[kOrientation] = angle_diff_abs_pi_2(halfScalars[kOrientation], otherHalfScalars[kOrientation]);
    scalarDerivs_[kWeightedOrientation] = angle_diff_abs_pi_2(halfScalars[kWeightedOrientation],
                                                                    otherHalfScalars[kWeightedOrientation]);
    scalarDerivs_[kMinDistOrientation] = angle_diff_abs(halfScalars[kMinDistOrientation],
                                                              otherHalfScalars[kMinDistOrientation]);
    scalarDerivs_[kMaxThroughDistOrientation] = angle_diff_abs(halfScalars[kMaxThroughDistOrientation],
                                                                     otherHalfScalars[kMaxThroughDistOrientation]);

    // Assign features based on the half-isovists
    scalars_[kMinHalfArea] = std::min(halfScalars[kArea], otherHalfScalars[kArea]);
    if(scalars_[kMinHalfArea] > 0.0)
    {
        scalars_[kAreaBalance] = scalars_[kMinHalfArea] / std::max(halfScalars[kArea], otherHalfScalars[kArea]);
    }
    else
    {
        scalars_[kAreaBalance] = 1.0;
    }
}


double area(Isovist::Iter begin, Isovist::Iter end)
{
    double sum = accumulate_pair(begin, end, [](const Point<double>& lhs, const Point<double>& rhs) {
        return lhs.x*rhs.y - lhs.y*rhs.x;
    });
    return std::abs(sum / 2.0);
}


double perimeter(Isovist::Iter begin, Isovist::Iter end)
{
    return accumulate_pair(begin, end, [](const Point<double>& lhs, const Point<double>& rhs) {
        return distance_between_points(lhs, rhs);
    });
}


double circularity(double area, double perimeter)
{
    return (perimeter > 0.0) ? (4.0*M_PI*area) / (perimeter*perimeter) : 1.0;
}


template <class BinaryOp>
double accumulate_pair(Isovist::Iter begin, Isovist::Iter end, BinaryOp op)
{
    if(begin == end)
    {
        return 0.0;
    }

    double sum = 0.0;

    for(auto first = begin, second = begin+1; second < end; ++first, ++second)
    {
        sum += op(*first, *second);
    }

    return sum;
}

} // namespace utils
} // namespace vulcan
