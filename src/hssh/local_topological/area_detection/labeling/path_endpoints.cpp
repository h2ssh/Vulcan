/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     path_endpoints.cpp
 * \author   Collin Johnson
 *
 * Definition of find_path_endpoints.
 */

#include "hssh/local_topological/area_detection/labeling/path_endpoints.h"
#include "core/angle_functions.h"
#include "core/line.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include <cassert>
#include <cmath>

// #define DEBUG_ENDPOINT_SEARCH

namespace vulcan
{
namespace hssh
{

double point_alignment(const DirectedPoint& p, double pathDirection);
double pair_alignment(const DirectedPoint& p1, const DirectedPoint& p2, double pathDirection);


PointPair find_path_endpoints(const std::vector<DirectedPoint>& points,
                              const Line<double>& pathAxis,
                              double pathDirection,
                              const AreaGraph& graph)
{
    /*
     * To determine the endpoints,
     *   - Generate all possible pairs of endpoints and select those that are most aligned to the axis.
     */

    assert(points.size() >= 2);

#ifdef DEBUG_ENDPOINT_SEARCH
    std::cout << "DEBUG: find_path_endpoints: Searching for endpoints with axis:" << pathAxis
              << " and dir: " << pathDirection << " with points:\n";
    for (auto& p : points) {
        std::cout << p.point << '\n';
    }
#endif

    // HACK: Require solution to be within 5m of the furthest solution.
    const double kMaxDistFromFurthest = 5.0;

    PointPair bestPair = std::make_pair(points[0], points[1]);
    //     double bestAlignment = -1.0;
    double bestDistance = 0.0;   // graph.distanceBetweenNodes(points[0].node, points[1].node);
    //     double bestWeight = (points[0].weight + points[1].weight) / 2.0;
    //     double bestWidth = (points[0].width + points[1].width) / 2.0;

    double bestScore = 0.0;   // bestWeight * bestDistance * bestWidth;

    for (std::size_t n = 0; n < points.size(); ++n) {
        double p1Alignment = point_alignment(points[n], pathDirection);
        //         auto p1Proj = closest_point_on_line_segment(points[n].point, pathAxis);

        for (std::size_t i = n + 1; i < points.size(); ++i) {
            double p2Alignment = point_alignment(points[i], pathDirection);
            //             auto p2Proj = closest_point_on_line_segment(points[i].point, pathAxis);

            double alignment = pair_alignment(points[n], points[i], pathDirection) * p1Alignment * p2Alignment;
            //             double alignment = p1Alignment + p2Alignment;
            double distance = graph.distanceBetweenNodes(points[n].node, points[i].node);

            if (points[n].isGateway) {
                distance += 1.0;
            }
            if (points[i].isGateway) {
                distance += 1.0;
            }

            //             double weight = (points[n].weight + points[i].weight) / 2.0; // * alignment;
            double width = (points[n].width + points[i].width) / 2.0;

            double score = width * distance * alignment;   // * weight;

            //             if(((distance > bestDistance - kMaxDistFromFurthest) && (alignment > bestAlignment))
            //                 || (distance > (bestDistance + kMaxDistFromFurthest)))
            if (((distance > bestDistance - kMaxDistFromFurthest) && (score > bestScore))
                || (distance > (bestDistance + kMaxDistFromFurthest)))
            //             if(score > bestScore)
            {
                //                 bestAlignment = alignment;
                bestDistance = distance;
                //                 bestWeight = weight;
                //                 bestWidth = width;
                bestScore = score;
                bestPair = std::make_pair(points[n], points[i]);
            }

#ifdef DEBUG_ENDPOINT_SEARCH
            std::cout << "Alignment: (p1,p2,p1Align,p2Align,pair,p1Proj,p2Proj,dist): " << points[n].point << ','
                      << points[i].point << ',' << p1Alignment << ',' << p2Alignment << ','
                      << pair_alignment(points[n], points[i], pathDirection) << ',' << p1Proj << ',' << p2Proj << ','
                      << distance_between_points(p1Proj, p2Proj) << ',' << alignment << '\n';
#endif
        }
    }

#ifdef DEBUG_ENDPOINT_SEARCH
    std::cout << "Selected: " << bestPair.first.point << " -> " << bestPair.second.point << " align:" << bestAlignment
              << " dist:" << bestDistance << "\n\n";
#endif

    return bestPair;
}


double point_alignment(const DirectedPoint& p, double pathDirection)
{
    return (M_PI_2 - angle_diff_abs_pi_2(p.direction, pathDirection)) / M_PI_2;
}


double pair_alignment(const DirectedPoint& p1, const DirectedPoint& p2, double pathDirection)
{
    return (M_PI_2 - angle_diff_abs_pi_2(angle_to_point(p1.point, p2.point), pathDirection)) / M_PI_2;
}

}   // namespace hssh
}   // namespace vulcan
