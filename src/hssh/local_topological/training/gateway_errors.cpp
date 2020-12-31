/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_errors.cpp
* \author   Collin Johnson
* 
* Definitions of types for computing gateway errors:
* 
*   - gateway_dist_t
*   - GatewayErrors
*/

#include "hssh/local_topological/training/gateway_errors.h"
#include "hssh/local_topological/training/labeled_gateway_data.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_edges.h"

namespace vulcan 
{
namespace hssh 
{
    
using DistVec = std::vector<gateway_dist_t>;
using GatewayAssoc = CellToTypeMap<DistVec>;
    
    
bool associate_with_ground_truth(cell_t gwy, const VoronoiEdges& edges, GatewayAssoc& associations);
int accumulate_dists(const DistVec& dists);

bool operator<(const gateway_dist_t& lhs, const gateway_dist_t& rhs)
{
    return lhs.distance < rhs.distance;
}
    

GatewayErrors::GatewayErrors(const CellVector& generated, 
                             const LabeledGatewayData& groundTruth, 
                             const VoronoiEdges& edges)
{
    // Create an association vector for each ground-truth gateway
    GatewayAssoc assoc;
    
    for(auto& gt : groundTruth)
    {
        if(gt.isGateway)
        {
            assoc[gt.cell] = DistVec();
        }
    }
    
    // Associate each generated with the corresponding ground-truth
    for(auto& gen : generated)
    {
        // If it isn't associated, then get the edge length and that's its error distance
        if(!associate_with_ground_truth(gen, edges, assoc))
        {
            auto edgeIt = edges.findEdgeForCell(gen);
            if(edgeIt != edges.end())
            {
                falsePos_.emplace_back(gen, edgeIt->size());
            }
        }
    }
    
    // Extract the false positives, false negatives, and true positives from the associations
    for(auto& gt : groundTruth)
    {
        if(gt.isGateway)
        {
            auto& gen = assoc[gt.cell];
            
            // If there aren't any associations, then it's a false negative
            if(gen.empty())
            {
                falseNegs_.push_back(gt.cell);
            }
            // Find the true positives and false positives
            else
            {
                // Sort in order of increasing distance
                std::sort(gen.begin(), gen.end());
                // The first will be the true positive and the rest will be the false positives
                truePos_.emplace_back(gen.front());
                falsePos_.insert(falsePos_.end(), gen.begin() + 1, gen.end());
            }
        }
    }
}


int GatewayErrors::truePosDist(void) const
{
    return accumulate_dists(truePos_);
}
    

int GatewayErrors::falsePosDist(void) const
{
    return accumulate_dists(falsePos_);
}


bool associate_with_ground_truth(cell_t gwy, const VoronoiEdges& edges, GatewayAssoc& associations)
{
    // Find the edge this gateway is along
    auto edgeIt = edges.findEdgeForCell(gwy);
    
    // If it isn't on an edge, then can't associated with any gateways
    if(edgeIt == edges.end())
    {
        return false;
    }
    
    const auto& edge = *edgeIt;
    auto gwyIt = std::find(edge.begin(), edge.end(), gwy);
    assert(gwyIt != edge.end());
    
    int minDist = edge.size();
    auto minIt = edge.end();
    
    // Find the closest associated gateway to the gateway
    for(auto cellIt = edge.begin(), endIt = edge.end(); cellIt != endIt; ++cellIt)
    {
        // See if this cell is associated with a gateway and is closer than any other gateways we've found
        auto assocIt = associations.find(*cellIt);
        int dist = std::abs(std::distance(cellIt, gwyIt));
        if((assocIt != associations.end()) && (dist < minDist))
        {
            minDist = dist;
            minIt = cellIt;
        }
    }
    
    // If we found a gateway, then add an association for it
    if(minIt != edge.end())
    {
        associations[*minIt].emplace_back(gwy, minDist);
    }
    
    return minIt != edge.end();
}


int accumulate_dists(const DistVec& dists)
{
    return std::accumulate(dists.begin(), dists.end(), 0, [](int total, const auto& dist) {
        return total + dist.distance;
    });
}

} // namespace hssh 
} // namespace vulcan
