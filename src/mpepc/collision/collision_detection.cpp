/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     collision_detection.cpp
* \author   Collin Johnson
*
* Definition of various constructors for robot_collision_model_t and
* the detect_collisions function.
*/

#include <mpepc/collision/collision_detection.h>
#include <mpepc/grid/obstacle_distance_grid.h>
#include <core/float_comparison.h>
#include <math/geometry/rectangle.h>
#include <core/pose.h>
#include <hssh/local_metric/lpm.h>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <set>

// #define DEBUG_EVENTS

namespace vulcan
{
namespace mpepc
{

struct vertex_event_t
{
    int yStart;
    int yEnd;

    double xLeft;
    double xRight;

    double xLeftStep;
    double xRightStep;
};

// Helpers for creating the vertex_event_t
template <typename Grid>
std::vector<Point<double>> calculate_model_vertices(const math::Rectangle<float>& model, const pose_t& pose, const Grid& grid);
std::vector<vertex_event_t>      create_vertex_events(const std::vector<Point<double>>& vertices);
double                           calculate_x_step    (const Point<double>& start, const Point<double>& end);

// Helpers for doing the actual detection
bool event_has_collision(const vertex_event_t& event, const hssh::LocalPerceptualMap& grid);
float closest_event_obstacle(const vertex_event_t& event, const ObstacleDistanceGrid& obstacleMap);


unsigned int detect_collisions(const math::Rectangle<float>&   model,
                               const pose_t&            poseToCheck,
                               const hssh::LocalPerceptualMap& grid)
{
    std::vector<Point<double>> vertices(calculate_model_vertices(model, poseToCheck, grid));
    std::vector<vertex_event_t>      events(create_vertex_events(vertices));

    unsigned int numCollisions = 0;

    for(auto eventIt = events.begin(); (eventIt != events.end()) && (numCollisions == 0); ++eventIt)
    {
        if(event_has_collision(*eventIt, grid))
        {
            ++numCollisions;
        }
    }

#ifdef DEBUG_EVENTS
    if(numCollisions)
    {
        std::cout<<"DEBUG:detect_collision: Robot vertices:"<<vertices[0]<<' '<<vertices[1]<<' '<<vertices[2]<<' '<<vertices[3]<<'\n';
        std::cout<<"Events:\n";
        for(auto eventIt = events.begin(); eventIt != events.end(); ++eventIt)
        {
            std::cout<<"y:"<<eventIt->yStart<<"->"<<eventIt->yEnd
                     <<"x:"<<eventIt->xLeft<<"->"<<eventIt->xRight
                     <<" xLeftStep:"<<eventIt->xLeftStep<<" xRightStep:"<<eventIt->xRightStep<<'\n';
        }
    }
#endif

    return numCollisions;
}


float closest_obstacle(const math::Rectangle<float>& model,
                       const pose_t&          poseToCheck,
                       const ObstacleDistanceGrid&   obstacleMap)
{
    std::vector<Point<double>> vertices(calculate_model_vertices(model, poseToCheck, obstacleMap));
    std::vector<vertex_event_t>      events(create_vertex_events(vertices));

    float minDistance = HUGE_VAL;

    // The minDistance can't be less than 0, so the search can terminate early if that condition occurs
    for(auto eventIt = events.begin(); (eventIt != events.end()) && (minDistance > 0.0f); ++eventIt)
    {
        float distance = closest_event_obstacle(*eventIt, obstacleMap);

        if(distance < minDistance)
        {
            minDistance = distance;
        }
    }

#ifdef DEBUG_EVENTS
    if(minDistance == 0.0)
    {
        std::cout<<"DEBUG:detect_collision: Robot vertices:"<<vertices[0]<<' '<<vertices[1]<<' '<<vertices[2]<<' '<<vertices[3]<<'\n';
        std::cout<<"Events:\n";
        for(auto eventIt = events.begin(); eventIt != events.end(); ++eventIt)
        {
            std::cout<<"y:"<<eventIt->yStart<<"->"<<eventIt->yEnd
                     <<" x:"<<eventIt->xLeft<<"->"<<eventIt->xRight
                     <<" xLeftStep:"<<eventIt->xLeftStep<<" xRightStep:"<<eventIt->xRightStep<<'\n';
        }
    }
#endif

    return minDistance;
}


template <typename Grid>
std::vector<Point<double>> calculate_model_vertices(const math::Rectangle<float>& model, const pose_t& pose, const Grid& grid)
{
    std::vector<Point<double>> vertices(4);
    vertices[0] = rotate(model.bottomLeft,  pose.theta) + pose.toPoint();
    vertices[1] = rotate(model.bottomRight, pose.theta) + pose.toPoint();
    vertices[2] = rotate(model.topRight,    pose.theta) + pose.toPoint();
    vertices[3] = rotate(model.topLeft,     pose.theta) + pose.toPoint();

    for(int n = vertices.size(); --n >= 0;)
    {
        vertices[n].x = (vertices[n].x - grid.getBottomLeft().x) * grid.cellsPerMeter();
        vertices[n].y = (vertices[n].y - grid.getBottomLeft().y) * grid.cellsPerMeter();
    }

    std::sort(vertices.begin(), vertices.end(), [](const Point<double>& lhs, const Point<double>& rhs)
                                                    {
                                                        return (absolute_fuzzy_equal(lhs.y, rhs.y, 1e-5) && (lhs.x < rhs.x)) ||
                                                               (!absolute_fuzzy_equal(lhs.y, rhs.y, 1e-5) && (lhs.y < rhs.y));
                                                    });

    float angTo1 = angle_to_point(vertices[0], vertices[1]);

    if(angTo1 < 0 && vertices[0].x > vertices[1].x)
    {
        std::cout<<"Swapping 0 and 1\n";
        std::swap(vertices[0], vertices[1]);
        angTo1 = angle_to_point(vertices[0], vertices[1]);
    }

    float angTo2 = angle_to_point(vertices[0], vertices[2]);
    float angTo3 = angle_to_point(vertices[0], vertices[3]);

    float diff21 = angle_diff(angTo2, angTo1);
    float diff23 = angle_diff(angTo2, angTo3);

    // If the two differences have a different sign, then vertex 2 and 3 need to be switched
    // The situation can arise due to fucking floating points
    if(diff21 * diff23 < 0)
    {
        std::cout<<"Swapping 2 and 3."<<angTo1<<' '<<angTo2<<' '<<angTo3<<'\n';
        std::swap(vertices[2], vertices[3]);
    }

    return vertices;
}


std::vector<vertex_event_t> create_vertex_events(const std::vector<Point<double>>& vertices)
{
    assert(vertices.size() == 4);

    Point<double> leftEvent;
    Point<double> rightEvent;

    if(vertices[1].x <= vertices[2].x)
    {
        leftEvent  = vertices[1];
        rightEvent = vertices[2];
    }
    else
    {
        leftEvent  = vertices[2];
        rightEvent = vertices[1];
    }

    std::vector<vertex_event_t> events(3);

    events[0].yStart = vertices[0].y;
    events[0].yEnd   = vertices[1].y;

    events[1].yStart = vertices[1].y;
    events[1].yEnd   = vertices[2].y;

    events[2].yStart = vertices[2].y;
    events[2].yEnd   = vertices[3].y;

    events[0].xLeft      = vertices[0].x;
    events[0].xRight     = vertices[0].x;
    events[0].xLeftStep  = calculate_x_step(vertices[0], leftEvent);
    events[0].xRightStep = calculate_x_step(vertices[0], rightEvent);

    if(leftEvent.y < rightEvent.y)
    {
        events[1].xLeft      = leftEvent.x; //events[0].xLeft + events[0].xLeftStep * (events[0].yEnd-events[0].yStart);
        events[1].xRight     = events[0].xRight + events[0].xRightStep * (events[0].yEnd-events[0].yStart + 1);
        events[1].xLeftStep  = calculate_x_step(leftEvent, vertices[3]);
        events[1].xRightStep = events[0].xRightStep;

        events[2].xLeft      = events[1].xLeft + events[1].xLeftStep * (events[1].yEnd-events[1].yStart + 1);
        events[2].xRight     = rightEvent.x; //events[1].xRight + events[1].xRightStep * (events[1].yEnd-events[1].yStart);
        events[2].xLeftStep  = events[1].xLeftStep;
        events[2].xRightStep = calculate_x_step(rightEvent, vertices[3]);
    }
    else if(rightEvent.y < leftEvent.y)
    {
        events[1].xLeft      = events[0].xLeft + events[0].xLeftStep * (events[0].yEnd-events[0].yStart + 1);
        events[1].xRight     = rightEvent.x; //events[0].xRight + events[0].xRightStep * (events[0].yEnd-events[0].yStart);
        events[1].xLeftStep  = events[0].xLeftStep;
        events[1].xRightStep = calculate_x_step(rightEvent, vertices[3]);

        events[2].xLeft      = leftEvent.x; // events[1].xLeft + events[1].xLeftStep * (events[1].yEnd-events[1].yStart);
        events[2].xRight     = events[1].xRight + events[1].xRightStep * (events[1].yEnd-events[1].yStart + 1);
        events[2].xLeftStep  = calculate_x_step(leftEvent, vertices[3]);
        events[2].xRightStep = events[1].xRightStep;
    }

    if(events[0].xLeft > events[0].xRight || events[1].xLeft > events[1].xRight || events[2].xLeft > events[2].xRight)
    {
        std::cout<<"DEBUG:detect_collision: Robot vertices:"<<vertices[0]<<' '<<vertices[1]<<' '<<vertices[2]<<' '<<vertices[3]<<'\n';
        std::cout<<"Events:\n";
        for(auto eventIt = events.begin(); eventIt != events.end(); ++eventIt)
        {
            std::cout<<"y:"<<eventIt->yStart<<"->"<<eventIt->yEnd
                    <<" x:"<<eventIt->xLeft<<"->"<<eventIt->xRight
                    <<" xLeftStep:"<<eventIt->xLeftStep<<" xRightStep:"<<eventIt->xRightStep<<'\n';
        }
    }

    return events;
}


double calculate_x_step(const Point<double>& start, const Point<double>& end)
{
    // If start and end are the same, then degenerate condition of needing x step to infinity
    double dx = end.x - start.x;
    int    dy = static_cast<int>(end.y) - static_cast<int>(start.y) + 1;

    return dx / dy;
}


bool event_has_collision(const vertex_event_t& event, const hssh::LocalPerceptualMap& grid)
{
    double xLeft  = event.xLeft;
    double xRight = event.xRight;

    for(int y = event.yStart; y <= event.yEnd; ++y)
    {
        int incr   = (event.xLeft < event.xRight) ? 1 : -1;
        int xStart = (y-event.yStart) * event.xLeftStep + event.xLeft;
        int xEnd   = (y-event.yStart) * event.xRightStep + event.xRight + incr;

        if(!(xLeft <= xRight || fabs(xLeft - xRight) < 0.001))
        {
            std::cerr<<"ERROR: collision_detection:"<<xLeft<<' '<<xRight<<" Step:"<<event.xLeftStep<<','<<event.xRightStep
                     <<"y:"<<y<<" yEnd:"<<event.yEnd<<'\n';
            assert(xLeft <= xRight || fabs(xLeft - xRight) < 0.001);
            return true;
        }

        for(int x = xStart; x != xEnd; x += incr)
        {
            if(grid.getCellType(Point<int>(x, y)) & hssh::kUnsafeOccGridCell)
            {
                return true;
            }
        }

        xLeft  += event.xLeftStep;
        xRight += event.xRightStep;
    }

    return false;
}


float closest_event_obstacle(const vertex_event_t& event, const ObstacleDistanceGrid& obstacleMap)
{
    float minDistance = HUGE_VAL;

    double xLeft  = event.xLeft;
    double xRight = event.xRight;

    int xLeftSteps  = fabs(event.xLeftStep);
    int leftDir     = event.xLeftStep > 0 ? 1 : -1;
    int xRightSteps = fabs(event.xRightStep);
    int rightDir    = event.xRightStep > 0 ? 1 : -1;

    // Bailout early if obstacle distance goes to 0 because it can't possibly be lower!
    for(int y = event.yStart; (y <= event.yEnd) && (minDistance > 0.0f); ++y)
    {
        if(!(xLeft <= xRight || fabs(xLeft - xRight) < 0.001))
        {
            std::cerr<<"ERROR: collision_detection:"<<xLeft<<' '<<xRight<<" Step:"<<event.xLeftStep<<','<<event.xRightStep
                     <<"y:"<<y<<" yEnd:"<<event.yEnd<<'\n';
            assert(xLeft <= xRight || fabs(xLeft - xRight) < 0.001);
            return true;
        }

        for(int x = 0; x <= xLeftSteps && (minDistance > 0.0f); ++x)
        {
            float distance = obstacleMap.getObstacleDistance(Point<float>(xLeft + x*leftDir, y));
            if(distance < minDistance)
            {
                minDistance = distance;
            }
        }

        for(int x = 0; x <= xRightSteps && (minDistance > 0.0f); ++x)
        {
            float distance = obstacleMap.getObstacleDistance(Point<float>(xRight + x*rightDir, y));
            if(distance < minDistance)
            {
                minDistance = distance;
            }
        }

        xLeft  += event.xLeftStep;
        xRight += event.xRightStep;
    }

    return minDistance;
}

} // namespace mpepc
} // namespace vulcan
