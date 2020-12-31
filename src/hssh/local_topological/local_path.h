/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_path.h
 * \author   Collin Johnson
 *
 * Declaration of LocalPath.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_PATH_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_PATH_H

#include "core/point.h"
#include "hssh/local_topological/lambda.h"

namespace vulcan
{
namespace hssh
{

/**
 * LocalPath maintains observed statistics about the local environment of the robot
 * as it drives along the path between two places. Currently, the local path only maintains
 * the Lambda value between the two place centers, along with the place ids, though these
 * ids probably aren't all that useful in the end.
 */
class LocalPath
{
public:
    /**
     * Default constructor for LocalPath.
     */
    LocalPath(void) : startId(0), endId(0) { }

    /**
     * Constructor for LocalPath.
     */
    LocalPath(uint32_t start, uint32_t end, const Lambda& lambda) : startId(start), endId(end), lambda(lambda) { }

    /**
     * Constructor for LocalPath.
     */
    LocalPath(const Point<float>& start, const Point<float>& end, const Lambda& lambda)
    : startPoint(start)
    , endPoint(end)
    , lambda(lambda)
    {
    }

    /**
     * Constructor for LocalPath.
     */
    LocalPath(uint32_t startId,
              uint32_t endId,
              const Point<float>& start,
              const Point<float>& end,
              const Lambda& lambda)
    : startId(startId)
    , endId(endId)
    , startPoint(start)
    , endPoint(end)
    , lambda(lambda)
    {
    }

    // Accessors
    /**
     * getStartId retrieves the id of the starting place for the path.
     *
     * \return   Id assigned to the start place.
     */
    uint32_t getStartId(void) const { return startId; }

    /**
     * getEndId retrieves the id of the ending place for the path.
     *
     * \return   Id assigned to the end place.
     */
    uint32_t getEndId(void) const { return endId; }

    /**
     * getStartPoint retrieves the starting boundary for the path.
     */
    Point<float> getStartPoint(void) const { return startPoint; }

    /**
     * getEndPoint retrieves the ending boundary for the place.
     */
    Point<float> getEndPoint(void) const { return endPoint; }

    /**
     * getLambda retrieves the measured lambda values for the path between start and end.
     *
     * \return   Estimated offset between the places.
     */
    Lambda getLambda(void) const { return lambda; }

private:
    uint32_t startId;
    uint32_t endId;

    Point<float> startPoint;
    Point<float> endPoint;

    Lambda lambda;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_LOCAL_PATH_H
