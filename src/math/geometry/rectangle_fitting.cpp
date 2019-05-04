/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rectangle_fitting.cpp
* \author   Collin Johnson
* 
* Definition of minimum_area_bounding_rectangle.
*/

#include <math/geometry/shape_fitting.h>
#include <math/geometry/convex_hull.h>
#include <core/line.h>
#include <core/point.h>
#include <cassert>
#include <iostream>

// #define DEBUG_INPUT
// #define DEBUG_CALIPER_ROTATION
// #define DEBUG_BOUNDING_RECTANGLE
// #define DEBUG_FINAL_RECTANGLE

namespace vulcan
{
namespace math
{

const int NUM_CALIPERS = 4;
const int BOTTOM_INDEX = 0;
const int LEFT_INDEX   = 1;
const int TOP_INDEX    = 2;
const int RIGHT_INDEX  = 3;

using PointIter = std::vector<Point<float>>::const_iterator;

struct caliper_t
{
    int         vertexIndex;
    double      angle;
    Line<float> line;
};

struct caliper_rotation_t
{
    int    index;
    double angle;
};

// The calipers are arranged in clockwise order, starting with the bottom
struct rotating_calipers_state_t
{
    std::vector<Point<float>> hullVertices;
    caliper_t      calipers[NUM_CALIPERS];

    double totalRotation;
    double minError;

    Rectangle<float> minErrorRectangle;
};


// Helper functions for bounding area functions
// ErrorFunc = double(PointIter begin, PointIter end, const Rectangle<float>&);
template <class ErrorFunc>
Rectangle<float> rotating_calipers_bounding_rectangle(PointIter             begin,
                                                      PointIter             end,
                                                      const Polygon<float>& hull,
                                                      ErrorFunc             errorFunc);

void               initialize_calipers(rotating_calipers_state_t& state);
caliper_rotation_t find_minimum_caliper_rotation(rotating_calipers_state_t& state);
void               rotate_calipers(caliper_rotation_t rotation, rotating_calipers_state_t& state);
void               calculate_caliper_lines(rotating_calipers_state_t& state);
Rectangle<float>   create_bounding_rectangle(rotating_calipers_state_t& state);

inline int previous_index(int currentIndex, int totalIndices)
{
    return (currentIndex == 0) ? totalIndices - 1 : currentIndex - 1;
}


Rectangle<float> minimum_area_bounding_rectangle(PointIter begin,
                                                 PointIter end)
{
    return minimum_area_bounding_rectangle(begin, end, convex_hull<float>(begin, end));
}


Rectangle<float> minimum_area_bounding_rectangle(PointIter             begin,
                                                 PointIter             end,
                                                 const Polygon<float>& hull)
{
    return rotating_calipers_bounding_rectangle(begin, end, hull,
                                                [](PointIter begin, PointIter end, const Rectangle<float>& rect)
                                                {
                                                    return rect.area();
                                                });
}


Rectangle<float> minimum_geometric_error_bounding_rectangle(PointIter begin,
                                                            PointIter end)
{
    return minimum_geometric_error_bounding_rectangle(begin, end, convex_hull<float>(begin, end));
}


Rectangle<float> minimum_geometric_error_bounding_rectangle(PointIter begin,
                                                            PointIter end,
                                                            const Polygon<float>& hull)
{
    return rotating_calipers_bounding_rectangle(begin, end, hull,
                                                [](PointIter begin, PointIter end, const Rectangle<float>& rect) -> double
                                                {
                                                    double total = 0.0;
                                                    for(auto pIt = begin; pIt != end; ++pIt)
                                                    {
                                                        total += rect.distanceFromBoundary(*pIt);
                                                    }
                                                    return total;
                                                });
}


template <class ErrorFunc>
Rectangle<float> rotating_calipers_bounding_rectangle(PointIter begin,
                                                      PointIter end,
                                                      const Polygon<float>& hull,
                                                      ErrorFunc errorFunc)
{
    if(std::distance(begin, end) <= 2)
    {
        std::cerr<<"WARNING:Too few points for minimum area bounding rectangle.\n";
        return Rectangle<float>();
    }
    
    rotating_calipers_state_t state;
    state.totalRotation = 0.0f;
    
    // For Polygon, the first and last vertices are the same. That definition is a nuisance for this application though,
    // so erase the last vertex
    state.hullVertices = std::vector<Point<float>>(hull.begin(), hull.end()-1);
    
    if(state.hullVertices.size() <= 2)
    {
        std::cerr<<"WARNING:Too few points for proper minimum area bounding rectangle. Using axis-aligned bounding rectangle instead.\n";
        return axis_aligned_bounding_rectangle<float>(begin, end);
    }
    
#ifdef DEBUG_INPUT
    std::cout<<"Points:";
    for(auto pointIt = begin; pointIt != end; ++pointIt)
    {
        std::cout<<' '<<*pointIt;
    }
    std::cout<<'\n';
    
    std::cout<<"Hull:";
    for(auto pointIt = state.hullVertices.begin(), pointEnd = state.hullVertices.end(); pointIt != pointEnd; ++pointIt)
    {
        std::cout<<' '<<*pointIt;
    }
    std::cout<<'\n';
#endif
    
    initialize_calipers(state);
    
    state.minError = std::numeric_limits<double>::max();
    
    while(state.totalRotation < M_PI / 2.0)
    {
        caliper_rotation_t nextRotation = find_minimum_caliper_rotation(state);
        
        rotate_calipers(nextRotation, state);
        
        auto currentBounds = create_bounding_rectangle(state);
        auto error         = errorFunc(begin, end, currentBounds);
        
        if(error < state.minError)
        {
            state.minErrorRectangle = currentBounds;
            state.minError          = error;
        }
        
        state.totalRotation += std::abs(nextRotation.angle);
        
#ifdef DEBUG_CALIPER_ROTATION
        std::cout<<"DEBUG:total_rotation:"<<state.totalRotation<<'\n';
#endif
    }
    
#ifdef DEBUG_FINAL_RECTANGLE
    std::cout<<"INFO:bounding_rectangle:"<<state.minErrorRectangle.topRight<<','
             <<state.minErrorRectangle.bottomRight<<','
             <<state.minErrorRectangle.bottomLeft<<','
             <<state.minErrorRectangle.topLeft<<'\n';
#endif
    
    return state.minErrorRectangle;
}


void initialize_calipers(rotating_calipers_state_t& state)
{
    // Find the extrema and the vertex index associated with them
    std::vector<Point<float>>& points = state.hullVertices;
    
    float minX = points[0].x;
    float maxX = points[0].x;
    float minY = points[0].y;
    float maxY = points[0].y;
    
    int minXIndex = 0;
    int maxXIndex = 0;
    int minYIndex = 0;
    int maxYIndex = 0;
    
    for(int n = points.size(); --n >= 1;)
    {
        if(points[n].x < minX)
        {
            minX      = points[n].x;
            minXIndex = n;
        }
        else if(points[n].x > maxX)
        {
            maxX      = points[n].x;
            maxXIndex = n;
        }
        
        if(points[n].y < minY)
        {
            minY      = points[n].y;
            minYIndex = n;
        }
        else if(points[n].y > maxY)
        {
            maxY      = points[n].y;
            maxYIndex = n;
        }
    }
    
    state.calipers[BOTTOM_INDEX].vertexIndex = minYIndex;
    state.calipers[BOTTOM_INDEX].angle       = M_PI;
    
    state.calipers[LEFT_INDEX].vertexIndex = minXIndex;
    state.calipers[LEFT_INDEX].angle       = M_PI / 2.0f;
    
    state.calipers[TOP_INDEX].vertexIndex = maxYIndex;
    state.calipers[TOP_INDEX].angle       = 0.0;
    
    state.calipers[RIGHT_INDEX].vertexIndex = maxXIndex;
    state.calipers[RIGHT_INDEX].angle       = -M_PI / 2.0f;
    
    calculate_caliper_lines(state);
}


caliper_rotation_t find_minimum_caliper_rotation(rotating_calipers_state_t& state)
{
    // To find the rotation, take the vertex of the next index along hull and see the angle it forms
    // with the line of the current caliper
    float minRotation = 500.0f;
    int   minIndex    = 0;
    
    for(int n = 0; n < NUM_CALIPERS; ++n)
    {
        caliper_t& caliper = state.calipers[n];
        
        int previousVertexIndex = previous_index(caliper.vertexIndex, state.hullVertices.size());
        
        // The calipers rotate clockwise around the polygon, so the caliper rotation is always negative or zero.
        float hullAngle = angle_to_point(state.hullVertices[caliper.vertexIndex],
                                         state.hullVertices[previousVertexIndex]);
        float caliperRotation = angle_diff(hullAngle, caliper.angle);
        
#ifdef DEBUG_CALIPER_ROTATION
        std::cout<<"DEBUG:caliper_rotation:"<<n<<" Angle:"<<caliper.angle<<" Hull:"<<hullAngle<<" Rot:"<<caliperRotation
                 <<" caliper:"<<caliper.line
                 <<" hull:"<<Line<float>(state.hullVertices[caliper.vertexIndex], state.hullVertices[previousVertexIndex])<<'\n';
#endif
        
        if(std::abs(caliperRotation) < std::abs(minRotation))
        {
            minRotation = caliperRotation;
            minIndex    = n;
        }
    }
    
#ifdef DEBUG_CALIPER_ROTATION
    std::cout<<"DEBUG:caliper_rotation:Index:"<<minIndex<<" Angle:"<<minRotation<<'\n';
#endif
    
    return {minIndex, minRotation};
}


void rotate_calipers(caliper_rotation_t rotation, rotating_calipers_state_t& state)
{
    for(int n = 0; n < NUM_CALIPERS; ++n)
    {
        state.calipers[n].angle += rotation.angle;
    }
    
    // The minimum caliper needs its vertex index increased by 1, as the calipers are now
    // adjacent to the edge of the polygon running from this vertex to the next vertex around
    // the edge of the polygon
    state.calipers[rotation.index].vertexIndex = previous_index(state.calipers[rotation.index].vertexIndex,
                                                                state.hullVertices.size());
    
    calculate_caliper_lines(state);
}


void calculate_caliper_lines(rotating_calipers_state_t& state)
{
    // Need two points to define the line. The caliper maintains the angle of the vector as well as
    // one point in the vector. Treat the vector as unit length to find a second point on the line.
    for(int n = 0; n < NUM_CALIPERS; ++n)
    {
        caliper_t& caliper = state.calipers[n];
        
        caliper.line.a   = state.hullVertices[caliper.vertexIndex];
        caliper.line.b.x = caliper.line.a.x + std::cos(caliper.angle);
        caliper.line.b.y = caliper.line.a.y + std::sin(caliper.angle);
    }
}


Rectangle<float> create_bounding_rectangle(rotating_calipers_state_t& state)
{
    Rectangle<float> boundary;
    
    if(!line_intersection_point(state.calipers[BOTTOM_INDEX].line, state.calipers[LEFT_INDEX].line, boundary.bottomLeft))
    {
        std::cerr<<"ERROR:bounding_rectangle:Bottom and left caliper don't intersect\n";
        assert(false);
    }
    
    if(!line_intersection_point(state.calipers[BOTTOM_INDEX].line, state.calipers[RIGHT_INDEX].line, boundary.bottomRight))
    {
        std::cerr<<"ERROR:bounding_rectangle:Bottom and right caliper don't intersect\n";
        assert(false);
    }
    
    if(!line_intersection_point(state.calipers[TOP_INDEX].line, state.calipers[LEFT_INDEX].line, boundary.topLeft))
    {
        std::cerr<<"ERROR:bounding_rectangle:Top and left caliper don't intersect\n";
        assert(false);
    }
    
    if(!line_intersection_point(state.calipers[TOP_INDEX].line, state.calipers[RIGHT_INDEX].line, boundary.topRight))
    {
        std::cerr<<"ERROR:bounding_rectangle:Top and right caliper don't intersect\n";
        assert(false);
    }
    
#ifdef DEBUG_BOUNDING_RECTANGLE
    std::cout<<"DEBUG:bounding_rectangle:"<<boundary.topRight<<','<<boundary.bottomRight<<','<<boundary.bottomLeft<<','<<boundary.topLeft<<" area:"<<boundary.area()<<'\n';
#endif
    
    return boundary;
}

} // namespace math
} // namespace vulcan
