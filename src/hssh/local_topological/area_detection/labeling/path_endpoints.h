/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path_endpoints.h
* \author   Collin Johnson
*
* Declaration of find_path_endpoints function and DirectedPoint and PointPair types.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABEL_PATH_ENDPOINTS_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABEL_PATH_ENDPOINTS_H

#include "core/line.h"
#include "core/point.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

class AreaGraph;
class AreaNode;

/**
* DirectedPoint represents a point with an associated direction. The direction of the point should be associated with
* the axis direction of the visible region at the point.
*/
struct DirectedPoint
{
    Point<double> point;
    const AreaNode* node;
    double direction;
    double weight;
    double width;
    bool isGateway;
};

using PointPair = std::pair<DirectedPoint, DirectedPoint>;

/**
* find_path_endpoints selects the endpoints for a path that are most aligned to the path direction.
* The alignment of a pair of endpoints is calculated using the following:
*
*   - difference between path direction and each endpoint's direction
*   - difference between angle from one endpoint to another and the path direction
*
* The selected endpoints are those that minimize the above alignment values.
*
* When selecting endpoints aligned to the axis, the direction needs to be considered for alignment. The determination
* of how the direction is calculate is left to the caller. The best choice is to use the gateway direction for gateways
* and the isovist orientation for non-gateways.
*
* Preference is given to gateway endpoints.
*
* \param    points              Possible endpoints for the path
* \param    pathAxis            Line running down the axis of the path
* \param    pathDirection       Direction of the path, as determined by isovists
* \param    graph               Graph of the environment
* \pre  points.size() >= 2
* \return   A PointPair containing the endpoints found for the path.
*/
PointPair find_path_endpoints(const std::vector<DirectedPoint>& points,
                              const Line<double>& pathAxis,
                              double pathDirection,
                              const AreaGraph& graph);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABEL_PATH_ENDPOINTS_H
