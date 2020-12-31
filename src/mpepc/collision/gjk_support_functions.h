/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gjk_support_functions.h
 * \author   Jong Jin Park
 *
 * Declaration of various support functions for Gilbert-Johnson-Keerti collision detection algorithm.
 */

#ifndef GJK_SUPPORT_FUNCTIONS_H
#define GJK_SUPPORT_FUNCTIONS_H

#include "core/line.h"
#include "core/point.h"
#include "core/pose.h"
#include <cmath>

namespace vulcan
{
namespace mpepc
{

using namespace math;

template <typename T>
T distance_to_point_from_line_segment(const Line<T>& lineSegment, const Point<T>& point, Point<T>* closestPoint)
{
    T distanceToPoint;

    Point<T> OA = lineSegment.a - point;   // pointA - pointO;
    Point<T> OB = lineSegment.b - point;   // pointB - pointO;
    Point<T> AB = OB - OA;                 //

    if (inner_product_between_points(AB, OA) > 0)   // A is the closest point to the point
    {
        closestPoint = lineSegment.a;
        distanceToPoint = point_norm(OA);
    } else if (inner_product_between_points(AB, OB) < 0)   // B is the closest point to the origin
    {
        closestPoint = lineSegment.b;
        distanceToPoint = point_norm(OB);
    } else   // closest point is on the line segment
    {
        T projectedLength = -(inner_product_between_points(OA, AB) / (AB.x * AB.x + AB.y * AB.y));
        closestPoint = lineSegment.a + projectedLength * AB;
        distanceToPoint = point_norm(closestPoint - point);
    }

    return distanceToPoint;
};


template <typename T>
Point<T> gjk_support_point_2D(const std::vector<Point<T>>& shape, const Point<T>& direction, Point<T>* supportVertex)
{
    // find support point (the farthest vetex on the direction)
    // initialize with the first vertice
    double maxInnerProduct = inner_product_between_points(shape.front(), direction);
    size_t indexAtMaximum = 0;

    // iterate through vertices
    size_t numVertex = shape.size();
    for (size_t currentIndex = 1; currentIndex < numVertex; currentIndex++) {
        // check inner product
        Point<T> currentVertex = shape[currentIndex];
        double currentInnderProduct = inner_product_between_points(currentVertex, direction);

        // determine maximum and store
        if (currentInnderProduct > maxInnerProduct) {
            maxInnerProduct = currentInnderProduct;
            indexAtMaximum = currentIndex;
        }
    }

    // Detect if the support is a line. If it is, return the other endpoint of the line.
    if (numVertex > 1 && supportVertex) {
        // retrieve previous and next vertices
        size_t prevIndex;
        size_t nextIndex;
        size_t currentIndex = indexAtMaximum;
        if (currentIndex == 0) {
            prevIndex = numVertex - 1;
            nextIndex = 1;
        } else if (currentIndex == numVertex - 1) {
            prevIndex = numVertex - 2;
            nextIndex = 0;
        } else {
            prevIndex = currentIndex - 1;
            nextIndex = currentIndex + 1;
        }

        Point<T> prevEdgeVector = shape[prevIndex] - shape[currentIndex];
        Point<T> nextEdgeVector = shape[nextIndex] - shape[currentIndex];

        // if the direction is normal to the line of interest, then the support
        // is the entire line rather than a single point, so return that as well.
        if (fabs(inner_product_between_points(prevEdgeVector, direction) < 0.01)) {
            *supportVertex = prevEdgeVector;
        } else if (fabs(inner_product_between_points(nextEdgeVector, direction) < 0.01)) {
            *supportVertex = prevEdgeVector;
        } else {
            supportVertex = nullptr;   // if not return empty pointer
        }
    }
}

template <typename T>
std::vector<Point<T>> gjk_support_vector_2D(const std::vector<Point<T>>& shape, const Point<T>& direction)
{
    Point<T> supportVertex;
    Point<T> supportPointOnA = gjk_support_point_2D(shape, direction, *supportVertex);
    if (supportVertex) {
        return std::vector<Point<T>>{supportPointOnA, supportVertex};
    } else {
        return std::vector<Point<T>>{supportPointOnA};
    }
}

template <typename T>
struct minkowskiSupport
{
    Point<T> pointA;
    Point<T> pointB;
    Point<T> minkowskiDifference;
};

template <typename T>
minkowskiSupport gjk_minkowski_support_2D(const std::vector<Point<T>>& shapeA,
                                          const std::vector<Point<T>>& shapeB,
                                          const Point<T>& directionAB)
{
    // directionAB is a direction from some point on A to B in minkowski space.
    // This function encodes a single iteration toward achieving better direction.
    minkowskiSupport support;
    support.pointA =
      gjk_support_point_2D(shapeA, directionAB);   // point on A that is farthest along the current directionAB
    support.pointB =
      gjk_support_point_2D(shapeB, -directionAB);   // point on B that is farthest along the current directionAB
    support.minkowskiDifference = support.pointB - support.pointA;

    return support;
};

template <typename T>
struct gjk_info_t
{
    std::vector<Point<T>> supportOnA;
    std::vector<Point<T>> supportOnB;
    Point<T> directionAB;
    double closestDistance;
    bool isInCollision;
};

// this implements an efficient gjk-distance algorithm. Works for convex 2D shapes.
// this is a simplex algorithm which uses inner products to iterate to extremum.
// For convex shapes with n number of vertices, the algorithm should converge
// before n iterations, unless the shapes are in collision which will put the
// algorithm into an infinite loop - so we detect if the algorithm is looping
// more than it should to detect collision, which is a hackish but efficient.
template <typename T>
double distance_between_convex_polygons(const std::vector<Point<T>>& shapeA,
                                        const std::vector<Point<T>>& shapeB,
                                        gjk_info_t* gjkInfo)
{
    // set initial direction
    Point<T> centerA = std::accumulate(shapeA.begin(), shapeA.end(), 0.0) / shapeA.size();
    Point<T> centerB = std::accumulate(shapeB.begin(), shapeB.end(), 0.0) / shapeB.size();
    Point<T> directionAB = centerB - centerA;

    // find support points in minkowski space
    minkowskiSupport simplex1a = gjk_minkowski_support_2D(shapeA, shapeB, directionAB);
    minkowskiSupport simplex1b = gjk_minkowski_support_2D(shapeA, shapeB, -directionAB);

    // form simplex from the support points
    std::vector<Point<T>> simplex;
    if (point_norm(simplex1a.minkowskiDifference) < point_norm(simplex1b.minkowskiDifference)) {
        simplex.push_back(simplex1b);
        simplex.push_back(simplex1a);
    } else {
        simplex.push_back(simplex1a);
        simplex.push_back(simplex1b);
    }

    // iterate to the closest points
    // initialize
    size_t maxIter = std::max(shapeA.size(), shapeB.size());
    size_t numIter = 0;
    Point<T> closestPoint;   // closest point to origin in Minkowski space
    bool isInCollision = false;
    T closestDistance = 10000.0;   // some large number

    // actual iteration
    while (true) {
        // distance to origin from the current 2-simplex (line) in Minkowski space
        closestDistance = distance_to_point_from_line_segment(simplex, Point<T>(0, 0), closestPoint);
        numIter++;   // update iteration number

        // We are done if already touching origin, or the number of iterations
        // is above the max possible number, which means we are in collision.
        if (numIter > maxIter || closestDistance < 0.001) {
            isInCollision = true;
            closestDistance = 0.0;
            break;
        }

        // define search direction, update minkowski support, and find a new candidate point to evaluate
        directionAB = closestPoint / closestDistance;   // normalized search direction
        minkowskiSupport simplex1c = gjk_minkowski_support_2D(shapeA, shapeB, directionAB);   // updated support
        Point<T> testPoint = simplex1c.minkowskiDifference;                                   // new simplex candidate

        // check if we are making progress toward origin using the candidate
        T projectedProgress =
          -inner_product_between_points(directionAB, testPoint - simplex.back());   // positive means progress
        if (projectedProgress < 0.001) {
            break;   // terminate if not making progress
        } else {
            // if making progress update the simplex for the next search.
            T mag1 = point_norm(simplex[0]);
            T mag2 = point_norm(simplex[1]);

            if (mag1 < mag2)   // compare size of two elements
            {
                // keep the first element and replace the second one.
                // simplex[0] = closestPoint; <- this occaisionlly makes convergence MUCH slower. DONT DO THIS!
                simplex[1] = testPoint;
            } else {
                // keep the second element, move it to the first container, and replace the second one.
                simplex[0] = simplex[1];
                simplex[1] = testPoint;
            }
        }
    }   // end while

    // update info
    gjkInfo->isInCollision = isInCollision;
    gjkInfo->closestDistance = closestDistance;   // this is already 0.0 if in collision
    if (!isInCollision)                           // update other info when relevant (when not in collision)
    {
        gjkInfo->supportOnA = gjk_support_vector_2D(shapeA, directionAB);
        gjkInfo->supportOnB = gjk_support_vector_2D(shapeB, -directionAB);
        gjkInfo->directionAB = directionAB;
    }

    return closestDistance;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // GJK_SUPPORT_FUNCTIONS_H