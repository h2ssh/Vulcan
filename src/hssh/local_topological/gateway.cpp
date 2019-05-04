/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway.cpp
* \author   Collin Johnson
*
* Implementation of Gateway.
*/

#include <hssh/local_topological/gateway.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/area_detection/gateways/gateway_utils.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <math/covariance.h>
#include <core/pose.h>
#include <utils/ray_tracing.h>
#include <cassert>

namespace vulcan
{
namespace hssh
{

static int32_t gNextId = 2 << 30;


// Helpers for handling the actual calculations of the normals
std::tuple<double, double> isovist_directions(Line<float> boundary, const utils::Isovist& isovist);
int is_cell_left_of_line(const Point<int>& cell, const Line<int>& line);

inline double gateway_length(const Point<double>& center, const Line<double>& boundary)
{
    return distance_between_points(center, boundary.a) + distance_between_points(center, boundary.b);
}


template <typename T>
Line<T> sort_boundary_endpoints(const Point<T>& a, const Point<T>& b)
{
    return (a.y < b.y) || (a.y == b.y && a.x < b.x) ? Line<T>(a, b) : Line<T>(b, a);
}


Gateway::Gateway(void)
: timestamp_(-1)
, id_(-1)
, length_(0.0)
{
}


Gateway::Gateway(int64_t                    timestamp,
                 int32_t                    id,
                 const Line<int>&     cellBoundary,
                 const Point<int>&    skeletonCell,
                 const VoronoiSkeletonGrid& grid)
: timestamp_(timestamp)
, id_((id > 0) ? id : gNextId++)
, boundary_(sort_boundary_endpoints(utils::grid_point_to_global_point(cellBoundary.a, grid), utils::grid_point_to_global_point(cellBoundary.b, grid)))
, center_(utils::grid_point_to_global_point(skeletonCell, grid))
, length_(gateway_length(center_, boundary_))
, skeletonCell_(skeletonCell)
, boundaryCells_(sort_boundary_endpoints(cellBoundary.a, cellBoundary.b))
{
    calculateDirections(grid, nullptr);
    gateway_boundary_cells(*this, grid, cellsAlongBoundary_);
}


Gateway::Gateway(int64_t                    timestamp,
                 int32_t                    id,
                 const Line<int>&     cellBoundary,
                 const Point<int>&    skeletonCell,
                 const VoronoiIsovistField& isovists,
                 const VoronoiSkeletonGrid& grid)
: Gateway(timestamp, id, cellBoundary, skeletonCell, grid)
{
    calculateDirections(grid, &isovists);
}


// Gateway::Gateway(int32_t                    id,
//                  const Line<double>&  boundary,
//                  const Point<double>& center,
//                  float                      direction)
// : timestamp_(0)
// , id_(id)
// , boundary_(sort_boundary_endpoints(boundary.a, boundary.b))
// , center_(center)
// , length_(gateway_length(center_, boundary_))
// , leftDirection_(direction)
// , rightDirection_(angle_sum(direction, M_PI))
// {
// }


Gateway Gateway::changeReferenceFrame(const pose_t& transform, const VoronoiSkeletonGrid& grid) const
{
    Line<double>  newBoundary;
    Point<double> newCenter;

    newBoundary.a = homogeneous_transform(boundary_.a, transform.x, transform.y, transform.theta);
    newBoundary.b = homogeneous_transform(boundary_.b, transform.x, transform.y, transform.theta);
    newCenter     = homogeneous_transform(center_,     transform.x, transform.y, transform.theta);

    Gateway newGateway;
    newGateway.timestamp_ = timestamp_;
    newGateway.id_ = id_;
    newGateway.boundary_ = newBoundary;
    newGateway.center_ = newCenter;
    newGateway.leftDirection_ = angle_sum(leftDirection_, transform.theta);
    newGateway.rightDirection_ = angle_sum(rightDirection_, transform.theta);
    newGateway.boundaryCells_ = Line<int>(utils::global_point_to_grid_cell(newBoundary.a, grid),
                                                utils::global_point_to_grid_cell(newBoundary.b, grid));
    newGateway.skeletonCell_ = utils::global_point_to_grid_cell(newCenter, grid);
    gateway_boundary_cells(newGateway, grid, newGateway.cellsAlongBoundary_);

    return newGateway;
}


void Gateway::reverseDirections(void)
{
    std::swap(leftDirection_, rightDirection_);
}


int Gateway::isCellToLeft(const Point<int>& cell) const
{
    if(cell == skeletonCell_)
    {
        return 0;
    }

    Line<int> boundaryA(boundaryCells_.a, skeletonCell_);
    Line<int> boundaryB(skeletonCell_, boundaryCells_.b);

    // Find which half of the gateway is closest to the line. Then see if it is left or right.
    Line<int> boundaryToCheck = distance_to_line_segment(cell, boundaryA) <
        distance_to_line_segment(cell, boundaryB) ? boundaryA : boundaryB;

    if(!cellsAlongBoundary_.empty())
    {
        if(utils::contains(cellsAlongBoundary_, cell))
        {
            return 0;
        }

        return left_of_line(boundaryToCheck, cell) ? 1 : -1;
    }
    else
    {
        std::cerr << "WARNING:Gateway: Using inaccurate isCellToLeft check due to no cellsAlongBoundary_. Create your"
            << " gateways using a VoronoiSkeletonGrid constructor for better accuracy.\n";
        return is_cell_left_of_line(cell, boundaryToCheck);
    }
}


int Gateway::isPointToLeft(const Point<float>& point) const
{
    Line<float> boundaryA(boundary_.a, center_);
    Line<float> boundaryB(center_, boundary_.b);

    // Find which half of the gateway is closest to the line. Then see if it is left or right.
    if(distance_to_line_segment(point, boundaryA) < distance_to_line_segment(point, boundaryB))
    {
        return left_of_line(boundaryA, point) ? 1 : -1;
    }
    else
    {
        return left_of_line(boundaryB, point) ? 1 : -1;
    }
}


bool Gateway::intersectsWithCellBoundary(const Line<int>& line, Point<int>& intersection) const
{
    return line_segment_intersection_point(line, Line<int>(boundaryCells_.a, skeletonCell_), intersection) ||
            line_segment_intersection_point(line, Line<int>(boundaryCells_.b, skeletonCell_), intersection);
}


bool Gateway::intersectsWithBoundary(const Line<double>& line, Point<double>& intersection) const
{
    return line_segment_intersection_point(line, Line<double>(boundary_.a, center_), intersection) ||
            line_segment_intersection_point(line, Line<double>(boundary_.b, center_), intersection);
}


bool Gateway::isSimilarTo(const Gateway& rhs) const
{
    double MAX_ENDPOINT_DIST = 0.5;
    double MAX_GATEWAY_ANGLE = M_PI_4;

    // If the new gateway is outside the radius of the existing gateway, they can't match
    if((distance_between_points(center_, rhs.center_) > vulcan::length(rhs.boundary_)) &&
        (distance_between_points(center_, rhs.center_) > vulcan::length(boundary_)))
    {
        return false;
    }

    bool hasCloseEndpoints =
        ((distance_between_points(boundary_.a, rhs.boundary_.a) < MAX_ENDPOINT_DIST)
            && (distance_between_points(boundary_.b, rhs.boundary_.b) < MAX_ENDPOINT_DIST))
        || ((distance_between_points(boundary_.a, rhs.boundary_.b) < MAX_ENDPOINT_DIST)
            && (distance_between_points(boundary_.b, rhs.boundary_.a) < MAX_ENDPOINT_DIST));

    if(hasCloseEndpoints)
    {
        return true;
    }

    return are_gateway_angles_close(*this, rhs, MAX_GATEWAY_ANGLE, MAX_ENDPOINT_DIST);
}


void Gateway::calculateDirections(const VoronoiSkeletonGrid& grid, const VoronoiIsovistField* isovists)
{
    // Use the skeleton to calculate the direction of the gateway
    float normalAngle = angle_sum(vulcan::direction(boundary_), M_PI/2.0f);

    leftDirection_ = normalAngle;
    rightDirection_ = angle_sum(normalAngle, M_PI);

    auto skeletonDirections = voronoi_direction(skeletonCell_, grid, normalAngle);

    // If isovists are available use them and create a comparison and match the closer of the two angles
    if(isovists && isovists->contains(skeletonCell_))
    {
        // Find the isovist corresponding to the skeleton cell. Split the isovist in two based on the boundary. Arbitrarily
        // decide left and right
        std::tie(leftDirection_, rightDirection_) = isovist_directions(boundary_, isovists->at(skeletonCell_));

        // Make sure comparing apples-to-apples with the directions, so associated them so the left directions are
        // closer to each other than left to right directions
        if(angle_diff_abs(leftDirection_, skeletonDirections.left)
            > angle_diff_abs(leftDirection_, skeletonDirections.right))
        {
            std::swap(leftDirection_, rightDirection_);
        }

        if(angle_diff_abs_pi_2(skeletonDirections.left, normalAngle)
            < angle_diff_abs_pi_2(leftDirection_, normalAngle))
        {
            leftDirection_ = skeletonDirections.left;
        }

        if(angle_diff_abs_pi_2(skeletonDirections.right, normalAngle)
            < angle_diff_abs_pi_2(rightDirection_, normalAngle))
        {
            rightDirection_ = skeletonDirections.right;
        }
    }
    else
    {
        leftDirection_ = skeletonDirections.left;
        rightDirection_ = skeletonDirections.right;
    }

    // Gateways closely track the normal in their construction, so don't let the direction change by too much
    if(angle_diff_abs_pi_2(leftDirection_, normalAngle) > M_PI / 36.0)
    {
        leftDirection_ = normalAngle;
        rightDirection_ = angle_sum(normalAngle, M_PI);
    }
}


// Operator overloads
bool operator<(const Gateway& lhs, const Gateway& rhs)
{
    return lhs.center() < rhs.center();
}


bool operator==(const Gateway& lhs, const Gateway& rhs)
{
    return lhs.boundary() == rhs.boundary();
}


bool operator!=(const Gateway& lhs, const Gateway& rhs)
{
    return !(lhs == rhs);
}


std::tuple<double, double> isovist_directions(Line<float> boundary, const utils::Isovist& isovist)
{
    return std::make_tuple(isovist.scalar(utils::Isovist::kOrientation),
                           angle_sum(isovist.scalar(utils::Isovist::kOrientation), M_PI));
}


int is_cell_left_of_line(const Point<int>& cell, const Line<int>& line)
{
    bool blIsLeft = left_of_line(line, cell);
    bool brIsLeft = left_of_line(line, Point<int>(cell.x + 1, cell.y));
    bool tlIsLeft = left_of_line(line, Point<int>(cell.x,     cell.y + 1));
    bool trIsLeft = left_of_line(line, Point<int>(cell.x + 1, cell.y + 1));

    if(blIsLeft && brIsLeft && tlIsLeft && trIsLeft)
    {
        return 1;
    }
    else if(blIsLeft || brIsLeft || tlIsLeft || trIsLeft)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

} // namespace hssh
} // namespace vulcan
