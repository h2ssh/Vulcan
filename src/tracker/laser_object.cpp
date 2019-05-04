/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_object.cpp
* \author   Collin Johnson
*
* Definition of LaserObject.
*/

#include <tracker/laser_object.h>
#include <math/clustering.h>
#include <math/geometry/shape_fitting.h>
#include <math/geometry/shape_intersection_area.h>
#include <math/statistics.h>
#include <utils/algorithm_ext.h>
#include <boost/range/iterator_range.hpp>
#include <array>

// #define DEBUG_UNCERTAINTY

namespace vulcan
{
namespace tracker
{

const float kMaxCircleRadius = 0.25f;
const float kMaxLegRadius    = 0.15f;

using DistIndex = std::pair<double, int>;
using Outliers = std::vector<DistIndex>;

Circle fit_circle_to_points(ConstPointIter begin, ConstPointIter end, Position laserPosition, double maxRadius);
Outliers find_outliers(ConstPointIter begin, ConstPointIter end);
boost::optional<EstimatedShape<TwoArcs>> fit_two_arcs_to_points(ConstPointIter begin,
                                                                ConstPointIter end,
                                                                Position laserPosition,
                                                                double laserVariance,
                                                                const std::vector<int>& splitIndices);
EstimatedShape<TwoRects> fit_two_rects_to_points(ConstPointIter begin,
                                                 ConstPointIter end,
                                                 Position laserPosition,
                                                 double laserVariance,
                                               const std::vector<int>& splitIndices);
Arc circle_to_arc(Circle circle, ConstPointIter begin, ConstPointIter end);
double circle_fit_error(const Circle& circle, ConstPointIter begin, ConstPointIter end, float maxRadius);
Matrix arc_fit_uncertainty(const Arc& arc, ConstPointIter begin, ConstPointIter end, double laserVariance);
double rect_fit_error(const Rectangle& rect, Position laserPosition, ConstPointIter begin, ConstPointIter end);
Matrix rect_fit_uncertainty(const Rectangle& rect, ConstPointIter begin, ConstPointIter end, double laserVariance);

inline double two_circles_area(const TwoCircles& circles)
{
    return circles[0].area() + circles[1].area();
}


LaserObject::LaserObject(void)
{
}


LaserObject::LaserObject(int64_t         timestamp,
                         ConstPointIter  pointsBegin,
                         ConstPointIter  pointsEnd,
                         const Position& laserPosition,
                         double          variance)
: timestamp_(timestamp)
, points_(pointsBegin, pointsEnd)
, laserVariance_(variance)
, fitTwoArcs_(boost::none)
, fitTwoRects_(boost::none)
, laserPosition_(laserPosition)
{
    assert(points_.size() > 2);
    assert(variance > 0.0);
    assert(std::distance(pointsBegin, pointsEnd) > 2);

    fitShapesToPoints(true);   // always use outliers rather than clustering
    calculateUncertainty(BoundaryType::best);
    calculateCircleApprox();

#ifdef DEBUG_UNCERTAINTY
    std::cout << "DEBUG: LaserObject: Uncertainties for object at " << center() << ":\n"
        << centerDistribution_.getCovariance();
#endif
}


LaserObject::LaserObject(const std::vector<const LaserObject*>& objects)
: laserVariance_(objects.front()->laserVariance_)
{
    assert(!objects.empty());

    for(auto& o : objects)
    {
        points_.insert(points_.end(), o->points_.begin(), o->points_.end());
    }

    timestamp_ = objects.front()->timestamp_;
    laserPosition_ = objects.front()->laserPosition_;

    // If the lasers aren't the same, then need to perform a sort on the points
    bool diffLaser = utils::contains_if(objects, [this](const auto& obj) {
        return obj->laserPosition_ != laserPosition_;
    });

    fitShapesToPoints(!diffLaser);
    calculateUncertainty(BoundaryType::best);
    calculateCircleApprox();
}


LaserObject LaserObject::generateShadowedCircle(void) const
{
    // Copy all existing state
    LaserObject shadow = *this;

    // Replace the existing two-arc with a single arc plus a fabricated arc. The circle sits on the line running
    // between the detected circle and laser. Extend this line to extent two circle radii further and then it should
    // be good to go.
    auto distance = distance_between_points(laserPosition_, fitArc_.shape.center());
    if(distance > 0.0)
    {
        auto radiusToDist = fitArc_.shape.radius() / distance;
        auto newDistRatio = 1.0 + (2.0 * radiusToDist);
        auto newCenter = Point<float>(
            laserPosition_.x + ((fitArc_.shape.center().x - laserPosition_.x) * newDistRatio),
            laserPosition_.y + ((fitArc_.shape.center().y - laserPosition_.y) * newDistRatio));

        EstimatedShape<TwoArcs> twoArcs;
        twoArcs.shape[0] = fitArc_.shape;
        twoArcs.shape[1] = math::Arc<float>(fitArc_.shape.radius(), newCenter, math::angle_range_t());
        twoArcs.error = fitArc_.error / 2.0;
        twoArcs.uncertainty = fitArc_.uncertainty * 1.5;
        shadow.fitTwoArcs_ = twoArcs;

        std::cout << "Created a shadow for " << fitArc_.shape.center() << ',' << fitArc_.shape.radius()
            << " at " << twoArcs.shape[1].center() << ',' << twoArcs.shape[1].radius() << '\n';
    }

    return shadow;
}


void LaserObject::setBoundaryType(BoundaryType type)
{
    calculateUncertainty(type);
}


ObjectBoundary LaserObject::minErrorBoundary(void) const
{
    return minErrorBoundaryAndUncertainty().first;
}


ObjectBoundary LaserObject::boundaryWithType(BoundaryType type) const
{
    switch(type)
    {
    case BoundaryType::rectangle:
        return ObjectBoundary(fitRect_.shape);
    case BoundaryType::one_circle:
        return ObjectBoundary(fitArc_.shape.toCircle());
    case BoundaryType::two_circles:
        if(fitTwoArcs_)
        {
            TwoCircles circles = {{ fitTwoArcs_->shape[0].toCircle(), fitTwoArcs_->shape[1].toCircle() }};
            return ObjectBoundary(circles);
        }
        else
        {
            return minErrorBoundary();
        }
    case BoundaryType::two_rects:
        if(fitTwoRects_)
        {
            return ObjectBoundary(fitTwoRects_->shape);
        }
        else
        {
            return minErrorBoundary();
        }
    case BoundaryType::circle_rect:
        std::cerr << "ERROR: LaserObject: circle-rect not currently estimated.\n";
        return minErrorBoundary();
    case BoundaryType::best:
        return minErrorBoundary();
    case BoundaryType::unknown:
        return minErrorBoundary();
    default:
        return minErrorBoundary();
    }

    return minErrorBoundary();
}


void LaserObject::fitShapesToPoints(bool searchForOutliers)
{
    fitRect_.shape = math::minimum_geometric_error_bounding_rectangle(points_.begin(), points_.end());
    fitRect_.error = rect_fit_error(fitRect_.shape, laserPosition_, points_.begin(), points_.end());
    fitRect_.uncertainty = rect_fit_uncertainty(fitRect_.shape, points_.begin(), points_.end(), laserVariance_);

    auto fitCircle = fit_circle_to_points(points_.begin(), points_.end(), laserPosition_, kMaxCircleRadius);
    fitArc_.error = circle_fit_error(fitCircle, points_.begin(), points_.end(), kMaxCircleRadius);
    fitArc_.shape = circle_to_arc(fitCircle, points_.begin(), points_.end());
    fitArc_.uncertainty = arc_fit_uncertainty(fitArc_.shape, points_.begin(), points_.end(), laserVariance_);

    if(std::distance(points_.begin(), points_.end()) >= 6)
    {
        std::vector<int> splitIndices;
        if(searchForOutliers)
        {
            Outliers outliers = find_outliers(points_.begin(), points_.end());
            for(auto o : outliers)
            {
                splitIndices.push_back(o.second);
            }
        }

        // If there aren't any split indices from outliers, attempt to cluster the points
        if(splitIndices.empty())
        {
            auto clusters = math::kmeans_2d_fixed(points_.begin(), points_.end(), 2);
            assert(clusters.assignedCluster.size() == points_.size());
            // Convert the clusters into a new set of points to be used
            decltype(points_) clusteredPoints;
            for(int c = 0; c < clusters.numClusters; ++c)
            {
                for(std::size_t n = 0; n < points_.size(); ++n)
                {
                    if(clusters.assignedCluster[n] == c)
                    {
                        clusteredPoints.push_back(points_[n]);
                    }
                }
            }

            // Split only at the cluster boundaries
            int splitPoint = 0;
            for(int size : clusters.clusterSizes)
            {
                splitPoint += size;
                splitIndices.push_back(splitPoint + 1);
            }
            std::swap(points_, clusteredPoints);
        }

        fitTwoArcs_ = fit_two_arcs_to_points(points_.begin(),
                                             points_.end(),
                                             laserPosition_,
                                             laserVariance_,
                                             splitIndices);
//             fitTwoRects_ = fit_two_rects_to_points(points_.begin(), points_.end(), laserVariance_, splitIndices);
    }
}


void LaserObject::calculateUncertainty(BoundaryType type)
{
    std::pair<ObjectBoundary, Matrix> boundaryAndUncertainty;

    switch(type)
    {
    case BoundaryType::rectangle:
        boundaryAndUncertainty.first.assign(fitRect_.shape, BoundaryType::rectangle);
        boundaryAndUncertainty.second = fitRect_.uncertainty;
        break;
    case BoundaryType::one_circle:
        boundaryAndUncertainty.first.assign(fitArc_.shape.toCircle(), BoundaryType::one_circle);
        boundaryAndUncertainty.second = fitArc_.uncertainty;
        break;
    case BoundaryType::two_circles:
        if(fitTwoArcs_)
        {
            TwoCircles fitCircles = { fitTwoArcs_->shape[0].toCircle(), fitTwoArcs_->shape[1].toCircle() };
            boundaryAndUncertainty.first.assign(fitCircles, BoundaryType::two_circles);
            boundaryAndUncertainty.second = fitTwoArcs_->uncertainty;
        }
        else
        {
            boundaryAndUncertainty.first.assign(fitArc_.shape.toCircle(), BoundaryType::one_circle);
            boundaryAndUncertainty.second = fitArc_.uncertainty;
        }
        break;
    case BoundaryType::two_rects:
        std::cerr << "WARNING: LaserObject: two_rects not currently estimated. Using best.\n";
        boundaryAndUncertainty = minErrorBoundaryAndUncertainty();
        break;
    case BoundaryType::circle_rect:
        std::cerr << "WARNING: LaserObject: circle_rect not currently estimated. Using best.\n";
        boundaryAndUncertainty = minErrorBoundaryAndUncertainty();
        break;
    case BoundaryType::best:
    case BoundaryType::unknown: // intentional fall-through
    default:
        boundaryAndUncertainty = minErrorBoundaryAndUncertainty();
        break;
    }

    Vector centerMean(2);
    Matrix centerCov(2, 2);

    centerMean(0) = boundaryAndUncertainty.first.position().x;
    centerMean(1) = boundaryAndUncertainty.first.position().y;

    centerCov = boundaryAndUncertainty.second.submat(0, 0, 1, 1);
    centerDistribution_.setDistributionStatistics(centerMean, centerCov);
}


void LaserObject::calculateCircleApprox(void)
{
    float maxDistToCenter = 0.0f;
    for(auto& point : points_)
    {
        float dist = distance_between_points(center(), point);
        if(dist > maxDistToCenter)
        {
            maxDistToCenter = dist;
        }
    }
    circleApproximation_ = Circle(maxDistToCenter, center());
}


std::pair<ObjectBoundary, Matrix> LaserObject::minErrorBoundaryAndUncertainty(void) const
{
    ObjectBoundary bestBoundary;
    Matrix uncertainty;
    bestBoundary.assignApproximation(circleApproximation_);

    if((fitArc_.error < fitRect_.error)
        && (!fitTwoArcs_ || (fitArc_.error < fitTwoArcs_->error))
        && (!fitTwoRects_ || (fitArc_.error < fitTwoRects_->error)))
    {
        bestBoundary.assign(fitArc_.shape.toCircle(), BoundaryType::one_circle);
        uncertainty = fitArc_.uncertainty;
    }
    else if((!fitTwoArcs_ || (fitRect_.error < fitTwoArcs_->error))
        && (!fitTwoRects_ || (fitRect_.error < fitTwoRects_->error)))
    {
        bestBoundary.assign(fitRect_.shape, BoundaryType::rectangle);
        uncertainty = fitRect_.uncertainty;
    }
    else if(fitTwoArcs_ && (!fitTwoRects_ || (fitTwoArcs_->error < fitTwoRects_->error)))
    {
        TwoCircles fitCircles = { fitTwoArcs_->shape[0].toCircle(), fitTwoArcs_->shape[1].toCircle() };
        bestBoundary.assign(fitCircles, BoundaryType::two_circles);
        uncertainty = fitTwoArcs_->uncertainty;
    }
    else // two rects fit the best
    {
        bestBoundary.assign(fitTwoRects_->shape, BoundaryType::two_rects);
        uncertainty = fitTwoRects_->uncertainty;
    }

    return std::make_pair(bestBoundary, uncertainty);
}


Circle fit_circle_to_points(ConstPointIter begin,
                            ConstPointIter end,
                            Position laserPosition,
                            const double maxRadius)
{
    auto fitCircle = math::minimum_geometric_error_circle(begin, end, 0.01, maxRadius);

    auto minDistIt = std::min_element(begin, end, [laserPosition](const auto& lhs, const auto& rhs) {
        return distance_between_points(lhs, laserPosition) < distance_between_points(rhs, laserPosition);
    });

    double closestLaserDist = distance_between_points(laserPosition, *minDistIt);

    // A fit circle can't have a distance to center closer than the closest laser point. It doesn't make physical sense.
    // because the laser is hitting the outside of the circle. The center being closer than the closest laser means
    // the laser must be seeing through the object, which doesn't make sense.
    // Don't need to worry about the rectangle because it is a minimum error bounding rectangle and the size won't
    // grow arbitrarily. It can't be much larger than the convex hull of the points
    // If the min geometric error circle doesn't make physical sense, then just use the mean circle, which isn't
    // a good fit, but will be physically valid
    if(distance_between_points(fitCircle.center(), laserPosition) < closestLaserDist)
    {
        fitCircle = math::mean_position_and_radius_circle(begin, end);
    }

    return fitCircle;
}


Outliers find_outliers(ConstPointIter begin, ConstPointIter end)
{
    int numPoints  = std::distance(begin, end);
    int firstSplit = 3;

    int firstQuartile = std::max(numPoints / 4, firstSplit);
    int thirdQuartile = std::min(numPoints * 3 / 4, numPoints - firstSplit);

    Outliers distances;
    distances.reserve(numPoints-1);
    for(int n = 1; n < numPoints; ++n)
    {
        distances.push_back(std::make_pair(distance_between_points(*(begin+n), *(begin+n-1)), n));
    }

    std::sort(distances.begin(), distances.end(), [](const DistIndex& lhs, const DistIndex& rhs) {
        return lhs.first < rhs.first;
    });

    auto first  = distances[distances.size() / 4];
    auto third  = distances[distances.size() * 3 / 4];
    double iqr  = (third.first - first.first) * 3;

    std::vector<DistIndex> outliers;
    for(auto& d : distances)
    {
        if((d.second >= firstQuartile) && (d.second <= thirdQuartile) && (d.first > third.first + iqr))
        {
            outliers.push_back(d);
        }
    }

    outliers.push_back(std::make_pair(0.0, numPoints / 2));

    return outliers;
}


boost::optional<EstimatedShape<TwoArcs>> fit_two_arcs_to_points(ConstPointIter begin,
                                                                ConstPointIter end,
                                                                Position laserPosition,
                                                                double laserVariance,
                                                                const std::vector<int>& splitIndices)
{
    TwoCircles fitCircles;
    std::array<double, 2> circleErrors;
    boost::optional<EstimatedShape<TwoArcs>> estimatedArcs;

    for(int splitIndex : splitIndices)
    {
        // Ignore any splits that don't leave enough room for a proper circle fit
        if((std::distance(begin, begin + splitIndex) < 3) || (std::distance(begin + splitIndex, end) < 3))
        {
            continue;
        }

        fitCircles[0] = fit_circle_to_points(begin, begin+splitIndex, laserPosition, kMaxLegRadius);
        fitCircles[1] = fit_circle_to_points(begin+splitIndex, end, laserPosition, kMaxLegRadius);

        // The two fit circle must not be concentric. If they are, then they don't  make sense, as they should be two
        // unique circles. Some overlap of boundaries is allowed because sensor error is expected.
        // If they are concentric, just assign them to their mean circles, which won't be a good fit, but won't
        // be concentric either
        if(fitCircles[0].contains(fitCircles[1].center()) || fitCircles[1].contains(fitCircles[0].center()))
        {
            fitCircles[0] = math::mean_position_and_radius_circle(begin, begin + splitIndex);
            fitCircles[1] = math::mean_position_and_radius_circle(begin + splitIndex, end);
        }

        circleErrors[0] = circle_fit_error(fitCircles[0], begin, begin+splitIndex, kMaxLegRadius);
        circleErrors[1] = circle_fit_error(fitCircles[1], begin+splitIndex, end, kMaxLegRadius);

        if(!estimatedArcs || (circleErrors[0] + circleErrors[1] < estimatedArcs->error))
        {
            EstimatedShape<TwoArcs> arcs;
            arcs.error = circleErrors[0] + circleErrors[1];
            arcs.shape[0] = circle_to_arc(fitCircles[0], begin, begin + splitIndex);
            arcs.shape[1] = circle_to_arc(fitCircles[1], begin + splitIndex, end);

            arcs.uncertainty = arc_fit_uncertainty(arcs.shape[0], begin, begin+splitIndex, laserVariance) +
                arc_fit_uncertainty(arcs.shape[1], begin+splitIndex, end, laserVariance);
            estimatedArcs = arcs;
        }
    }

    return estimatedArcs;
}


EstimatedShape<TwoRects> fit_two_rects_to_points(ConstPointIter begin,
                                                 ConstPointIter end,
                                                 Position laserPosition,
                                                 double laserVariance,
                                                 const std::vector<int>& splitIndices)
{
    TwoRects fitRects;
    std::array<double, 2> rectErrors;
    EstimatedShape<TwoRects> estimatedRects;
    estimatedRects.error = HUGE_VAL;

    for(int splitIndex : splitIndices)
    {
        fitRects[0] = math::minimum_geometric_error_bounding_rectangle(begin, begin+splitIndex);
        fitRects[1] = math::minimum_geometric_error_bounding_rectangle(begin+splitIndex, end);

        rectErrors[0] = rect_fit_error(fitRects[0], laserPosition, begin, begin+splitIndex);
        rectErrors[1] = rect_fit_error(fitRects[1], laserPosition, begin+splitIndex, end);

        if(rectErrors[0] + rectErrors[1] < estimatedRects.error)
        {
            estimatedRects.error = rectErrors[0] + rectErrors[1];
            estimatedRects.shape = fitRects;
            estimatedRects.uncertainty = rect_fit_uncertainty(fitRects[0], begin, begin+splitIndex, laserVariance) +
                rect_fit_uncertainty(fitRects[1], begin+splitIndex, end, laserVariance);
        }
    }

    return estimatedRects;
}


Arc circle_to_arc(Circle circle, ConstPointIter begin, ConstPointIter end)
{
    // The range needs to contain both the first and last point fit to the circle. Create it containing the first
    // angle. Then expand it to contain the second angle. Ensures that the range invariants are correctly maintained.
    math::angle_range_t range(angle_to_point(circle.center(), *begin));
    range.expand(angle_to_point(circle.center(), *(end-1)));

    return Arc(circle.radius(), circle.center(), range);
}


double circle_fit_error(const Circle& circle, ConstPointIter begin, ConstPointIter end, float maxRadius)
{
    double totalError = 0.0;

    for(; begin != end; ++begin)
    {
        totalError += std::abs(distance_between_points(circle.center(), *begin) - circle.radius());
    }

    if(circle.radius() > maxRadius)
    {
        totalError += 5.0;
    }

    return totalError;
}


Matrix arc_fit_uncertainty(const Arc& arc, ConstPointIter begin, ConstPointIter end, double laserVariance)
{
    int numPoints = std::distance(begin, end);
    Matrix W(numPoints, 3);
    for(int n = 0; n < numPoints; ++n)
    {
        W(n, 0) = ((begin + n)->x - arc.center().x) / arc.radius();
        W(n, 1) = ((begin + n)->y - arc.center().y) / arc.radius();
        W(n, 2) = 1.0;
    }

    Matrix wPseudo = arma::trans(W) * W;
    wPseudo.diag() += 1e-8;

    Matrix cov = laserVariance * arma::inv(wPseudo);
    return cov;
}


double rect_fit_error(const Rectangle& rect, Position laserPosition, ConstPointIter begin, ConstPointIter end)
{
    // Find the visible edge(s) given the laser position. The visible edges will include the closest edge and possibly
    // one other edge. If the angle subtended by the closest edge is includes the angle subtended by an adjacent edge,
    // then that edge isn't visible (draw the figure and it is easy to see this)
    std::array<Line<float>, 4> edges;
    edges[0] = Line<float>(rect.bottomLeft, rect.bottomRight);
    edges[1] = Line<float>(rect.bottomRight, rect.topRight);
    edges[2] = Line<float>(rect.topRight, rect.topLeft);
    edges[3] = Line<float>(rect.topLeft, rect.bottomLeft);

    // Find the closest edge
    auto minIt = std::min_element(edges.begin(), edges.end(), [laserPosition](const auto& lhs, const auto& rhs) {
        return distance_to_line_segment(laserPosition, lhs) < distance_to_line_segment(laserPosition, rhs);
    });

    // Compute the angle subtend by this edge
    int closestIdx = std::distance(edges.begin(), minIt);
    math::angle_range_t edgeRange(angle_to_point(laserPosition, edges[closestIdx].a));
    edgeRange.expand(angle_to_point(laserPosition, edges[closestIdx].b));

    double initialExtent = edgeRange.extent;

    // Search the neighbors to see if they cause the extent to grow
    int visibleIdx = -1;
    std::array<int, 2> neighborIdx;
    neighborIdx[0] = (closestIdx == 0) ? 3 : closestIdx - 1;
    neighborIdx[1] = (closestIdx == 3) ? 0 : closestIdx + 1;
    for(auto idx : neighborIdx)
    {
        edgeRange.expand(angle_to_point(laserPosition, edges[idx].a));
        edgeRange.expand(angle_to_point(laserPosition, edges[idx].b));

        if(edgeRange.extent > initialExtent)
        {
            visibleIdx = idx;
            initialExtent = edgeRange.extent;
        }
    }

    // Set the visible edges for use in the distance computation
    std::array<Line<float>, 2> visibleEdges;
    visibleEdges[0] = edges[closestIdx];
    if(visibleIdx != -1)
    {
        visibleEdges[1] = edges[visibleIdx];
    }
    // Reuse the closest index for ease of implementation, though not efficiency
    else
    {
        visibleEdges[1] = edges[closestIdx];
    }

    // Compute the distance to these lines for each of the points
    return std::accumulate(begin, end, 0.0, [&visibleEdges](double total, const Position& p) {
        return total + std::min(distance_to_line(p, visibleEdges[0]), distance_to_line(p, visibleEdges[1]));
    });
}


Matrix rect_fit_uncertainty(const Rectangle& rect, ConstPointIter begin, ConstPointIter end, double laserVariance)
{
    int numPoints = std::distance(begin, end);
    Matrix W(numPoints, 3);
    for(int n = 0; n < numPoints; ++n)
    {
        auto boundaryPoint = rect.closestPointOnBoundary(*(begin + n));
        auto radius = distance_between_points(rect.center(), boundaryPoint.first);
        W(n, 0) = ((begin + n)->x - rect.center().x) / radius;
        W(n, 1) = ((begin + n)->y - rect.center().y) / radius;
        W(n, 2) = 1.0;
    }

    Matrix wPseudo = arma::trans(W) * W;
    wPseudo.diag() += 1e-8;

    Matrix cov = laserVariance * arma::inv(wPseudo);
    return cov;
}

} // namespace tracker
} // namespace vulcan
