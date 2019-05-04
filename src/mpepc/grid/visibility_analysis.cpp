/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_analysis.cpp
* \author   Collin Johnson
*
* Definition of VisibilityAnalysis.
*/

#include <mpepc/grid/visibility_analysis.h>
#include <math/geometry/shape_fitting.h>
#include <hssh/local_metric/lpm.h>
#include <tracker/dynamic_object_collection.h>

namespace vulcan
{
namespace mpepc
{

math::Polygon<double> static_view_for_laser(const pose_t& pose,
                                            const laser_configuration_t& laser,
                                            const hssh::LocalPerceptualMap& map);
math::Polygon<double> dynamic_view_for_laser(const pose_t& pose,
                                             const laser_configuration_t& laser,
                                             const math::Polygon<double>& staticView,
                                             const tracker::DynamicObjectCollection& objects);
math::Polygon<double> merge_laser_views(Point<double> origin,
                                        const std::vector<math::Polygon<double>>& views);

VisibilityAnalysis::VisibilityAnalysis(const pose_t& pose,
                                       const std::vector<laser_configuration_t>& lasers,
                                       const hssh::LocalPerceptualMap& map,
                                       const tracker::DynamicObjectCollection& objects)
: origin_(pose.toPoint())
{
    std::vector<math::Polygon<double>> staticViews;
    for(auto& l : lasers)
    {
        staticViews.push_back(static_view_for_laser(pose, l, map));
    }

    staticView_ = merge_laser_views(origin_, staticViews);

    std::vector<math::Polygon<double>> dynamicViews;
    for(std::size_t n = 0; n < lasers.size(); ++n)
    {
        dynamicViews.push_back(dynamic_view_for_laser(pose, lasers[n], staticViews[n], objects));
    }

    dynamicView_ = merge_laser_views(origin_, dynamicViews);
}


VisibilityAnalysis::VisibilityAnalysis(const pose_t& pose,
                                       const laser_configuration_t& laser,
                                       const hssh::LocalPerceptualMap& map,
                                       const tracker::DynamicObjectCollection& objects)
{
    auto laserPose = laser.offset.compound(pose);
    origin_ = laserPose.toPoint();

    staticView_ = static_view_for_laser(pose, laser, map);
    dynamicView_ = dynamic_view_for_laser(pose, laser, staticView_, objects);
}


math::Polygon<double> static_view_for_laser(const pose_t& pose,
                                            const laser_configuration_t& laser,
                                            const hssh::LocalPerceptualMap& map)
{
    // Compute the view of the laser in global coordinates
    auto laserPose = laser.offset.compound(pose);
    auto origin = laserPose.toPoint();

    auto rotatedRange = laser.range;
    rotatedRange.startAngle = angle_sum(rotatedRange.startAngle, laserPose.theta);

    // Create the ray trace scan of the static environment
    PointVec<double> rays(rotatedRange.numIncrements + 2);
    rays[0] = utils::global_point_to_grid_point(origin, map);
    utils::trace_range_until_condition(utils::global_point_to_grid_point(origin, map),
                                       rotatedRange,
                                       15,
                                       map,
                                       [](const auto& lpm, Point<int> cell) {
        return lpm.getCellType(cell) & (hssh::kObstacleOccGridCell | hssh::kUnobservedOccGridCell);
    },
                                       rays.begin() + 1);

    std::transform(rays.begin(), rays.end(), rays.begin(), [&map](const auto ray) {
        return utils::grid_point_to_global_point(ray, map);
    });

    rays.back() = origin;
    return math::Polygon<double>(rays);
}


math::Polygon<double> dynamic_view_for_laser(const pose_t& pose,
                                             const laser_configuration_t& laser,
                                             const math::Polygon<double>& staticView,
                                             const tracker::DynamicObjectCollection& objects)
{
    auto laserPose = laser.offset.compound(pose);
    auto origin = laserPose.toPoint();

    // Intersect the static rays with dynamic objects to get the dynamic rays
    PointVec<double> rays(staticView.begin(), staticView.end());
    std::vector<Point<float>> intersections;
    for(auto& r : rays)
    {
        Line<float> ray(origin, r);

        for(auto& obj : objects)
        {
            if(obj->boundary().circleApproximation().intersections(ray, intersections))
            {
                // If there's a collision, always shorten the ray to ensure that we find the closest
                // intersection amongst the objects
                ray.b = intersections.front();
                intersections.clear();
            }
        }

        // Adjust the stored length after considering all intersections
        r = ray.b;
    }

    return math::Polygon<double>(rays);
}


math::Polygon<double> merge_laser_views(Point<double> origin,
                                        const std::vector<math::Polygon<double>>& views)
{
    PointVec<double> uniquePoints;

    for(std::size_t n = 0; n < views.size(); ++n)
    {
        for(auto& p : views[n])
        {
            bool isUnique = true;
            for(std::size_t m = 0; (m < views.size()) && isUnique; ++m)
            {
                isUnique &= (m == n) || !views[m].contains(p);
            }

            if(isUnique)
            {
                uniquePoints.push_back(p);
            }
        }
    }

    std::sort(uniquePoints.begin(), uniquePoints.end(), [&origin](const auto lhs, const auto rhs) {
        return wrap_to_2pi(angle_to_point(origin, lhs))
            < wrap_to_2pi(angle_to_point(origin, rhs));
    });

    return math::Polygon<double>(uniquePoints);
}

} // namespace mpepc
} // namespace vulcan
