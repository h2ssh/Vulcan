/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_errors.h
* \author   Collin Johnson
* 
* Declaration of types for computing gateway errors:
* 
*   - gateway_dist_t : holds a generated gateway and distance to nearest ground-truth
*   - GatewayErrors : mapping of g.t. gateway cell -> vec<gateway_dist_t> with TP, FP, FN counts provided
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_TRAINING_GATEWAY_ERRORS_H
#define HSSH_LOCAL_TOPOLOGICAL_TRAINING_GATEWAY_ERRORS_H

#include <hssh/types.h>
#include <vector>

namespace vulcan
{
namespace hssh
{

class LabeledGatewayData;
class VoronoiEdges;

/**
* gateway_dist_t maintains the distance from a ground-truth gateway along its VoronoiEdge.
*/
struct gateway_dist_t
{
    cell_t gateway;
    int distance;
    
    gateway_dist_t(cell_t gw, int dist)
    : gateway(gw)
    , distance(dist)
    {
    }
};

/**
* GatewayErrors maintains error information for all generated gateways relative to a set of ground-truth gateways.
* Gateway errors are computed as follows:
*   
*   - for each generated gateway:
*       - find the nearest ground-truth gateway along its voronoi edge
*       - if no gateway exists on the edge, it is added to unmatched false positives and distance is the edge length
*       - if it matches, then the distance is the distance along the edge in cells
*       - associate the gen gateway with g.t. gateway
* 
*   - after association, compute TP, FP, FN for each g.t. gateway:
*       - if num assoc == 0, then FN
*       - if num assoc > 0, then TP
*       - if num assoc > 1, then FP += num assoc - 1
*/
class GatewayErrors
{
public:
    
    /**
    * Constructor for GatewayErrors.
    * 
    * \param    generated           Generated gateways
    * \param    groundTruth         Ground-truth gateways
    * \param    edges               Edges in the VoronoiSkeleton
    */
    GatewayErrors(//const std::vector<Gateway>& generated, 
                  const CellVector& generated,
                  const LabeledGatewayData& groundTruth, 
                  const VoronoiEdges& edges);
    
    /**
    * truePosDist is the cumulative distance of true positives from their associated ground-truth gateways.
    */
    int truePosDist(void) const;
    
    /**
    * falsePosError is the cumulative distance of false positives from their associated ground-truth gateways.
    */
    int falsePosDist(void) const;
    
    // Counts of the different types of errors
    int numTruePos(void) const { return truePos_.size(); }
    int numFalsePos(void) const { return falsePos_.size(); }
    int numFalseNeg(void) const { return falseNegs_.size(); }
    
private:
    
    CellVector falseNegs_;
    std::vector<gateway_dist_t> truePos_;
    std::vector<gateway_dist_t> falsePos_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_TRAINING_GATEWAY_ERRORS_H
