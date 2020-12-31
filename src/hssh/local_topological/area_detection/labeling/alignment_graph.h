/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     alignment_graph.h
* \author   Collin Johnson
*
* Declaration of AlignmentGraph interface.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_GRAPH_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_GRAPH_H

#include "hssh/local_topological/area_detection/labeling/hypothesis_type.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

class AlignmentConstraint;
class AreaHypothesis;

/**
* AlignmentGraph is an mix-in interface with necessary methods for determining the type of areas involved
* in constraints.
*/
class AlignmentGraph
{
public:

    using Id = int;
    using IdVec = std::vector<Id>;

    /**
    * addArea adds a new area to the network. The initial type of the area is assigned to be the most appropriate type.
    *
    * \param    area            Area to be added
    * \param    type            Type of area (= HypothesisType::kArea means it can be anything)
    * \return   Id of the added area.
    */
    virtual Id addArea(AreaHypothesis* area, HypothesisType type) = 0;

    /**
    * addFixedArea adds a new area to the network that has a fixed, immutable type.
    *
    * \param    area            Area to be added
    * \param    type            Type to be permanently assigned to the area
    * \param    isConnected     Flag indicating if this particular area must be part of the connected graph of
    *                           path segments and decision points. In general, this flag should only be true for
    *                           the exited area
    * \return   Id of the added area in the network.
    */
    virtual Id addFixedArea(AreaHypothesis* area, HypothesisType type, bool isConnected) = 0;

    /**
    * addConstraint adds a new constraint to the network.
    *
    * \param    constraint      Constraint to be added
    */
    virtual void addConstraint(const AlignmentConstraint& constraint) = 0;

    /**
    * assignedType retrieves the type assigned to the specified area id.
    *
    * \pre area is a valid id returned from a call to addArea
    * \param    area            Area to query
    * \return   Type assigned to the specified area.
    */
    virtual HypothesisType assignedType(Id area) const = 0;

    /**
    * assignedArea retrieves the active area associated specified area id.
    *
    * \pre area is a valid id returned from a call to addArea
    * \param    area            Area to query
    * \return   Id of the active area associated with the provided id.
    */
    virtual Id assignedArea(Id area) const = 0;

    /**
    * isPathEndGateway checks if a particular gateway is at the end of a path. It checks the gateway associated
    * with the area associated with areaId, not the area on the other side of the gateway, which is otherAreaId.
    *
    * The otherAreaId is necessary to detect cases where two areas are adjacent via more than one gateway and one
    * of those gateways is a path endpoint of areaId. In this event, this gateway is still a path end gateway,
    * even if it isn't marked as such in the active hypothesis.
    *
    * \param    areaId          Id of the original area
    * \param    gatewayId       Id of the gateway associated with the area
    * \param    otherAreaId     Id of original area on the other side of the gateway
    * \return   True if area is a path and the gateway is one of the path's endpoints.
    */
    virtual bool isPathEndGateway(Id areaId, int32_t gatewayId, int otherAreaId) const = 0;

protected:

    ~AlignmentGraph(void) {}
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_GRAPH_H
