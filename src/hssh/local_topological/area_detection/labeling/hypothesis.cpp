/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis.cpp
* \author   Collin Johnson
*
* Implementation of AreaHypothesis.
*/

#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/area_proposal.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/loops_and_trees.h"
#include "hssh/local_topological/area_detection/labeling/small_scale_star_builder.h"
#include "hssh/local_topological/area_detection/labeling/path_endpoints.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_detection/labeling/debug.h"
#include "hssh/local_topological/area_extent.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "math/covariance.h"
#include "math/geometry/convex_hull.h"
#include "math/geometry/shape_fitting.h"
#include "utils/algorithm_ext.h"
#include "utils/func_ptr.h"
#include <boost/core/null_deleter.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <algorithm>
#include <array>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <queue>

// #define DEBUG_EIG_STATS
// #define DEBUG_CONTAINMENT
// #define DEBUG_ENDPOINTS

namespace vulcan
{
namespace hssh
{

std::vector<AreaHypothesis*> extract_hypotheses(const std::vector<const AreaHypothesisBoundary*>& boundaries);


AreaHypothesis::AreaHypothesis(int id,
                               AreaGraph* graph,
                               const std::set<AreaNode*>&   nodes,
                               const std::set<AreaEdge*>&   edges,
                               const VoronoiSkeletonGrid*   grid,
                               const VoronoiIsovistField*   isovistField,
                               const utils::VisibilityGraph* visGraph,
                               const utils::VisibilityGraph* skeletonGraph,
                               const SmallScaleStarBuilder* starBuilder)
: areaGraph_(graph)
, nodes_(nodes.begin(), nodes.end())
, edges_(edges.begin(), edges.end())
, grid_(grid)
, isovistField_(isovistField)
, visGraph_(visGraph)
, skelGraph_(skeletonGraph)
, starBuilder_(starBuilder)
, boundariesDirty_(true)
, type_(HypothesisType::kArea)
, frontier_(false)
, boundary_(false)
, id_(id)
, length_(0.0)
, width_(0.0)
{
    assert(!nodes_.empty());
    assert(!edges_.empty());
    assert(std::find(nodes_.begin(), nodes_.end(), nullptr) == nodes_.end());
//     assert(std::find(edges_.begin(), edges_.end(), nullptr) == edges_.end());

    assert(edges_.find(nullptr) == edges_.end());

    initialize();

    // Must find path endpoints after initialize is called because that is where the axis direction is established.
    findPathEndpoints();

    for(auto cell : cells_)
    {
        if(visGraph->isVertex(cell))
        {
            visibilityVertices_.push_back(cell);
        }
    }

    for(auto node : nodes_)
    {
        graphVertices_.push_back(node->getCell());
    }

    // Sanity check the nodes for the hypothesis and their loop conditions. If no boundaries are on a loop, but
    // there's a loop node, then turn it to a non-loop because it is local to the hypothesis, which means that it
    // comes from a small loop in the skeleton, not a larger loop through the map
    bool haveBoundaryLoop = false;
    for(auto node : nodes)
    {
        if((node->getType() & AreaNode::kGateway) && node->isLoop())
        {
            haveBoundaryLoop = true;
            break;
        }
    }

    // Go through all the nodes and turn off any loops if no boundaries are loops
    if(!haveBoundaryLoop)
    {
        for(auto node : nodes)
        {
            if((~node->getType() & AreaNode::kGateway) && node->isLoop() && !haveBoundaryLoop)
            {
                node->setLoop(false);
            }
        }
    }
}


AreaHypothesis::AreaHypothesis(const std::vector<const AreaHypothesisBoundary*>& boundaries,
                               HypothesisType                                    classification,
                               bool                                              needExtent)
: AreaHypothesis(extract_hypotheses(boundaries), needExtent)
{
    assert(!boundaries.empty());
    type_ = classification;
}


AreaHypothesis::AreaHypothesis(const std::vector<AreaHypothesis*>& areas, bool needExtent)
: endNodes_{{nullptr, nullptr}}
, boundariesDirty_(true)
, frontier_(false)
, boundary_(false)
, length_(0.0f)
{
    assert(!areas.empty());

    id_ = areas.front()->getId();
    areaGraph_ = areas.front()->areaGraph_;

    std::set<AreaNode*> mergedNodes;
    //     std::set<AreaEdge*>       mergedEdges;
    std::set<const AreaHypothesisBoundary*> mergedBoundaries;

    std::size_t numSkeletonCells = 0;

    for(auto area : areas)
    {
        grid_         = area->grid_;
        isovistField_ = area->isovistField_;
        visGraph_     = area->visGraph_;
        skelGraph_    = area->skelGraph_;
        starBuilder_  = area->starBuilder_;

        mergedNodes.insert(area->nodes_.begin(), area->nodes_.end());
        edges_.insert(area->edges_.begin(), area->edges_.end());
        numSkeletonCells += area->cells_.size();
        mergedBoundaries.insert(area->boundaries_.begin(), area->boundaries_.end());

        boost::push_back(visibilityVertices_, boost::as_array(area->visibilityVertices_));
        boost::push_back(graphVertices_, boost::as_array(area->graphVertices_));

        boundary_ |= area->boundary_;
    }

    std::sort(visibilityVertices_.begin(), visibilityVertices_.end());
    utils::erase_unique(visibilityVertices_);

    std::sort(graphVertices_.begin(), graphVertices_.end());
    utils::erase_unique(graphVertices_);

    // One type of invalid area will have a boundary appear multiple times because the gateways doesn't actually enclose the space, and thus a single
    // hypothesis contains the edges on both sides of the boundary. This boundary will be removed, but this case needs to be handled properly.
    auto unequalBoundaryCondition = [&areas](const AreaHypothesisBoundary* toCopy) -> bool
    {
        std::array<bool, 2> similar = {{false, false}};
        for(auto a : areas)
        {
            if(!similar[0] && toCopy->getHypotheses()[0]->isSimilar(*a))
            {
                similar[0] = true;
            }
            else if(!similar[1] && toCopy->getHypotheses()[1]->isSimilar(*a))
            {
                similar[1] = true;
            }
        }
        return !similar[0] || !similar[1];
    };

    boundaries_.reserve(mergedBoundaries.size());
    std::copy_if(mergedBoundaries.begin(),
                 mergedBoundaries.end(),
                 back_inserter(boundaries_),
                 unequalBoundaryCondition);

    nodes_.reserve(mergedNodes.size());
    nodes_.insert(nodes_.end(), mergedNodes.begin(), mergedNodes.end());

    if(boundaries_.empty())
    {
        // If there are no boundaries, then there must be at least two termination points for the area
        assert(getPoints(AreaNode::kDeadEnd | AreaNode::kFrontier | AreaNode::kJunction).size() >= 2);
    }

    // For every boundary, copy over its max path dist -- yes this might run through the area, but it should be a
    // close enough approximation. The path most likely won't since that path would have to be internal to get
    // through the merged area though
    for(auto& area : areas)
    {
        for(auto& node : area->maxPathDist_)
        {
            bool isBoundary = utils::contains_if(boundaries_, [node](const auto& bnd) {
                return bnd->getNode() == node.first;
            });

            if(isBoundary)
            {
                maxPathDist_[node.first] = node.second;
            }
        }
    }

    assert(std::find(nodes_.begin(), nodes_.end(), nullptr) == nodes_.end());

    for(auto& e : edges_)
    {
        length_ += e->getLength();
        width_ += e->getAverageWidth() * e->getLength();
    }

    if(length_ > 0.0)
    {
        width_ /= length_;
    }

    // Copy over all skeleton cells and merge the extents into one

    if(needExtent)
    {
        bool haveExtents = true;

        std::vector<const AreaExtent*> extents;
        extents.reserve(areas.size());
        cells_.reserve(numSkeletonCells);
        for(auto& hyp : areas)
        {
            cells_.insert(cells_.end(), hyp->cells_.begin(), hyp->cells_.end());
            extents.push_back(hyp->extent_.get());
            haveExtents &= hyp->extent_ != nullptr;
        }

        if(haveExtents)
        {
            extent_.reset(new AreaExtent(extents));
            calculateAxis();
            polygonBoundary_ = extent_->polygonBoundary(math::ReferenceFrame::GLOBAL);
            rectangleBoundary_ = extent_->rectangleBoundary(math::ReferenceFrame::GLOBAL);
            center_ = extent_->center().toPoint();
        }
        else
        {
            initialize();
        }
    }
    else
    {
        // Create the polygon boundary via convex hull of the individual boundaries
        std::vector<Point<double>> hullPoints;
        for(auto& hyp : areas)
        {
            hullPoints.insert(hullPoints.end(), hyp->polygonBoundary_.begin(), hyp->polygonBoundary_.end());
        }
        polygonBoundary_ = math::convex_hull<double>(hullPoints.begin(), hullPoints.end());
        rectangleBoundary_ = math::axis_aligned_bounding_rectangle<double>(polygonBoundary_.begin(),
                                                                           polygonBoundary_.end());

        // Compute weighted mean of center
        center_ = Point<double>(0, 0);
        double totalArea = 0.0;
        for(auto& hyp : areas)
        {
            center_.x += hyp->center().x * hyp->extent_->area();
            center_.y += hyp->center().y * hyp->extent_->area();
            totalArea += hyp->extent_->area();
        }
        center_.x /= totalArea;
        center_.y /= totalArea;

        double sumCos = 0.0;
        double sumSin = 0.0;
        double sumEcc = 0.0;

        for(auto& hyp : areas)
        {
            sumCos += std::cos(hyp->axisDirection_) * hyp->cells_.size();
            sumSin += std::sin(hyp->axisDirection_) * hyp->cells_.size();
            sumEcc += hyp->axisRatio_ * hyp->cells_.size();
        }

        axisDirection_ = std::atan2(sumSin, sumCos);
        axisRatio_ = sumEcc / numSkeletonCells;
    }

    mergeEndpoints(areas);
}


AreaHypothesis::~AreaHypothesis(void)
{
    // For unique_ptr
}


Point<float> AreaHypothesis::center(void) const
{
    return center_;
}


math::Rectangle<float> AreaHypothesis::rectangleBoundary(void) const
{
//     assert(extent_);
//     return extent_->rectangleBoundary(math::ReferenceFrame::GLOBAL);
    return rectangleBoundary_;
}


Endpoints AreaHypothesis::endpoints(void) const
{
    return std::make_pair(endNodes_[0]->getPosition(), endNodes_[1]->getPosition());
}


bool AreaHypothesis::isSimilar(const AreaHypothesis& rhs) const
{
    // Always similar to self
    if(&rhs == this)
    {
        return true;
    }

    for(auto& edge : edges_)
    {
        if(rhs.edges_.find(edge) != rhs.edges_.end())
        {
            return true;
        }
    }

    return false;
}


HypothesisContainment AreaHypothesis::amountContained(const AreaHypothesis& rhs) const
{
//  amountContained determines how much this hypothesis contains the input hypothesis. The containment is determined
//  by comparing the boundaries of rhs with gateway nodes in this hypothesis. If rhs.boundaries is a subset of the
//  gateway nodes in this hypothesis, the containment is complete. If only some of rhs.boundaries are gateway nodes,
//  the containment is partial. If none of rhs.boundaries are gateway nodes, there is no containment.

    std::size_t numContained = 0;

    // Keep the most recent match around for disambiguating the one-boundary match areas
    const AreaNode* lhsMatch = nullptr;
    const AreaNode* rhsMatch = nullptr;

    // Search through the boundaries in rhs
    for(auto& b : rhs.boundaries_)
    {
        auto boundaryGateway = b->getGateway();

        // See if any of the internal or boundary gateway nodes are similar to the boundary's gateway
        for(auto& n : nodes_)
        {
            if((n->getType() & AreaNode::kGateway) && boundaryGateway.isSimilarTo(n->getGateway()))
            {
                lhsMatch = n;
                rhsMatch = b->getNode();
                ++numContained;
                break;
            }
        }
    }

#ifdef DEBUG_CONTAINMENT
    std::cout << "DEBUG:AreaHypothesis:Contained:" << numContained << " Total:" << rhs.numBoundaries() << '\n';
#endif

    if(numContained == 0)
    {
        return HypothesisContainment::none;
    }
    else if(numContained > 1)
    {
        if(numContained < rhs.numBoundaries())
        {
            return HypothesisContainment::partial;
        }
        else // numContained == rhs.numBoundaries()
        {
            return HypothesisContainment::complete;
        }
    }
    else // numContained == 1
    {
        assert(lhsMatch);
        assert(rhsMatch);
        return singleBoundaryContainment(lhsMatch, rhs, rhsMatch);
    }
}


bool AreaHypothesis::addBoundary(const AreaHypothesisBoundary* boundary)
{
    if(std::find(nodes_.begin(), nodes_.end(), boundary->getNode()) == nodes_.end())
    {
        std::cerr << "WARNING:AreaHypothesis::addBoundary: Attempted to add invalid boundary.\n";
        return false;
    }

    // Trying to add a duplicate boundary means the graph is being constructed incorrectly and should fail
    assert(std::find(boundaries_.begin(), boundaries_.end(), boundary) == boundaries_.end());
    boundaries_.push_back(boundary);
    boundariesDirty_ = true;

    return true;
}


bool AreaHypothesis::isValid(void) const
{
    // Need at least two cells to calculate valid isovist statistics
    if(cells_.size() < 5)
    {
        if(extent_)
        {
            std::cout << "Invalid area. " << rectangleBoundary()
                << " Not enough cells: " << cells_.size() << '\n';
        }
        return false;
    }

    if(edges_.size() > 1)
    {
        return true;
    }

    // The type of an edge is the OR of the endpoint types. If both endpoints are gateways, then the edge type will
    // be exactly GATEWAY_NODE, and thus the edge is valid.
    AreaEdge* edge = *edges_.begin();
    if((edge->getEndpoint(0)->getType() & AreaNode::kGateway) && (edge->getEndpoint(1)->getType() & AreaNode::kGateway))
    {
        return true;
    }

    assert(extent_);
    if(extent_->area() <= 0.5)
    {
        std::cout << "Invalid area. " << rectangleBoundary() << " Not big enough: " << extent_->area() << '\n';
    }
    return (extent_->area() > 0.5);

    // The estimated length is the measured length + the distance from the end of the edge to the wall
    // which is just half the width because the width comes from the voronoi skeleton
//     return (nodes_.size() > 1) && (length_ > 1.0);
}


SmallScaleStar AreaHypothesis::calculateStar(HypothesisType typeMask) const
{
    return calculateStar(typeMask, &AreaHypothesis::getGatewaysFullType);
}


std::vector<std::pair<const Gateway*, const Gateway*>> AreaHypothesis::pairwiseUnalignedGateways(void) const
{
    std::vector<std::pair<const Gateway*, const Gateway*>> unaligned;

    for(auto bIt = beginBoundary(); bIt < endBoundary(); ++bIt)
    {
        const Gateway& firstGateway = (*bIt)->getGateway();

        for(auto nextIt = bIt + 1; nextIt < endBoundary(); ++nextIt)
        {
            const Gateway& secondGateway = (*nextIt)->getGateway();

            if(!starBuilder_->areGatewaysAligned(firstGateway, secondGateway, center()))
            {
                unaligned.push_back(std::make_pair(&firstGateway, &secondGateway));
            }
        }
    }

    return unaligned;
}


std::vector<std::pair<const Gateway*, const Gateway*>> AreaHypothesis::pairwiseAlignedGateways(void) const
{
    std::vector<std::pair<const Gateway*, const Gateway*>> aligned;

    for(auto bIt = beginBoundary(); bIt < endBoundary(); ++bIt)
    {
        const Gateway& firstGateway = (*bIt)->getGateway();

        for(auto nextIt = bIt + 1; nextIt < endBoundary(); ++nextIt)
        {
            const Gateway& secondGateway = (*nextIt)->getGateway();

            if(starBuilder_->areGatewaysAligned(firstGateway, secondGateway, center()))
            {
                aligned.push_back(std::make_pair(&firstGateway, &secondGateway));
            }
        }
    }

    return aligned;
}


int AreaHypothesis::numGatewaysAlignedToAxis(HypothesisType typeMask) const
{
    auto gateways = getGatewaysFullType(typeMask);
    return starBuilder_->numGatewaysAlignedToAxis(gateways, axisDirection_);
}


int AreaHypothesis::numNonGatewayEnds(void) const
{
    const auto kEndType = AreaNode::kFrontier | AreaNode::kDeadEnd;

    int numNonGateway = 0;

    for(auto& node : endNodes_)
    {
        if(node && (node->getType() & kEndType))
        {
            ++numNonGateway;
        }
    }

    return numNonGateway;
}


bool AreaHypothesis::isEndGateway(int32_t gatewayId) const
{
    for(auto& node : endNodes_)
    {
        if(node && (node->getType() & AreaNode::kGateway) && (node->getGateway().id() == gatewayId))
        {
            return true;
        }
    }

    return false;
}


int AreaHypothesis::numEndpointsWithType(HypothesisType type) const
{
    int count = 0;

    for(auto& node : endNodes_)
    {
        for(auto& boundary : boundaries_)
        {
            if(boundary->getNode() == node)
            {
                auto otherGraph = boundary->getOtherHypothesis(*this);
                assert(otherGraph);
                if(is_hypothesis_type(otherGraph->getGatewayType(*(boundary->getNode())), type))
                {
                    ++count;
                }
                break;
            }

        }
    }

    return count;
}


std::size_t AreaHypothesis::countGatewaysWithMask(HypothesisType typeMask) const
{
    std::size_t count = 0;

    for(auto& boundary : boundaries_)
    {
        auto otherGraph = boundary->getOtherHypothesis(*this);
        assert(otherGraph);
        if(is_hypothesis_type(otherGraph->getGatewayType(*(boundary->getNode())), typeMask))
        {
            ++count;
        }
    }

    return count;
}


std::vector<AreaHypothesis*> AreaHypothesis::findAdjacentHypotheses(void) const
{
    std::vector<AreaHypothesis*> adjacent;

    for(auto& boundary : boundaries_)
    {
        auto otherGraph = boundary->getOtherHypothesis(*this);
        if(otherGraph)
        {
            adjacent.push_back(otherGraph);
        }
    }

    return adjacent;
}


AreaHypothesis* AreaHypothesis::adjacentArea(const Gateway& gateway)
{
    for(auto& boundary : boundaries_)
    {
        if(boundary->getGateway() == gateway)
        {
            return boundary->getOtherHypothesis(*this);
        }
    }

    return nullptr;
}


bool AreaHypothesis::hasMergeableBoundary(void) const
{
    for(auto boundary : boundaries_)
    {
        if(boundary->shouldMerge(BoundaryMergeCondition::kSameType))
        {
            return true;
        }
    }

    return false;
}


bool AreaHypothesis::isFrontier(void) const
{
    return extent_->frontierRatio() > 0.5;
}


bool AreaHypothesis::isBoundary(void) const
{
    return boundary_;
}


const HypothesisFeatures& AreaHypothesis::features(void) const
{
    calculateFeaturesIfNeeded();
    return *features_;
}


HypothesisType AreaHypothesis::getType(void) const
{
    return type_;
}


HypothesisType AreaHypothesis::getGatewayType(const AreaNode& gateway) const
{
    auto currentType = getType();

    if(currentType != HypothesisType::kPath)
    {
        return currentType;
    }

    return isEndpoint(gateway) ? HypothesisType::kPathEndpoint : HypothesisType::kPathDest;
}


AreaProposal AreaHypothesis::toAreaProposal(void) const
{
    AreaType proposalClassification;

    HypothesisType proposalType = getType();

    if(proposalType == HypothesisType::kArea)
    {
        proposalClassification = AreaType::area;
        std::cerr<<"WARNING::AreaSegmenter: Invalid final classification of hypothesis. Creating a generic area.\n";
    }
    else if(is_hypothesis_type(proposalType, HypothesisType::kPath))
    {
        proposalClassification = AreaType::path_segment;
    }
    else if(is_hypothesis_type(proposalType, HypothesisType::kDecision))
    {
        proposalClassification = AreaType::decision_point;
    }
    else if(is_hypothesis_type(proposalType, HypothesisType::kDest))
    {
        proposalClassification = AreaType::destination;
    }
    else
    {
        assert(is_hypothesis_type(proposalType, HypothesisType::kArea) && "Hypothesis type didn't match area!");
        proposalClassification = AreaType::area;
    }

    // A path is created with different data than a place
    if(proposalClassification == AreaType::path_segment)
    {
        AreaProposal::path_endpoint_t endpoints[2];
        for(int n = 0; n < 2; ++n)
        {
            assert(endNodes_[n]);

            // A gateway node will be on the boundary, see if this node is a boundary node or not
            auto bIt = std::find_if(boundaries_.begin(), boundaries_.end(), [n,this](const AreaHypothesisBoundary* b) {
                return b->getNode() == endNodes_[n];
            });

            endpoints[n].point = endNodes_[n]->getPosition();

            if(bIt != boundaries_.end())
            {
                endpoints[n].type = AreaType::area;
                endpoints[n].isGatewayEndpoint = true;
                endpoints[n].gateway = (*bIt)->getGateway();
            }
            else
            {
                endpoints[n].isGatewayEndpoint = false;
                if(endNodes_[n]->getType() & AreaNode::kFrontier)
                {
                    endpoints[n].type = AreaType::frontier;
                }
                else if(endNodes_[n]->getType() & AreaNode::kDeadEnd)
                {
                    endpoints[n].type = AreaType::dead_end;
                }
                else // don't know what type it is!
                {
                    std::cerr << "WARNING: AreaHypothesis: Unknown endpoint node type: " << endNodes_[n]->getType()
                        << ". Assigning as a dead end to be safe.\n";
                    endpoints[n].type = AreaType::dead_end;
                }
            }
        }

        return AreaProposal(id_,
                            endpoints[0],
                            endpoints[1],
                            frontier_,
                            boundary_,
                            *extent_,
                            getGatewaysBasicType(HypothesisType::kPath),
                            getGatewaysBasicType(HypothesisType::kDest),
                            getGatewaysBasicType(HypothesisType::kDecision),
                            getPoints(AreaNode::kFrontier),
                            getPoints(AreaNode::kDeadEnd | AreaNode::kJunction));
    }
    else
    {
        return AreaProposal(id_,
                            proposalClassification,
                            frontier_,
                            boundary_,
                            *extent_,
                            getGatewaysBasicType(HypothesisType::kPath),
                            getGatewaysBasicType(HypothesisType::kDest),
                            getGatewaysBasicType(HypothesisType::kDecision),
                            getPoints(AreaNode::kFrontier),
                            getPoints(AreaNode::kDeadEnd | AreaNode::kJunction));
    }
}


DebugHypothesis AreaHypothesis::toDebug(const HypothesisTypeDistribution& distribution) const
{
    HypothesisType current = getType();

    calculateFeaturesIfNeeded();
    assert(extent_);
    assert(features_);

    if(current == HypothesisType::kPath)
    {
        assert(endNodes_[0] && endNodes_[1]);
        DebugHypothesis::Endpoints debugEndpoints = {{endNodes_[0]->getPosition(), endNodes_[1]->getPosition()}};
        return DebugHypothesis(debugEndpoints,
                               *extent_,
                               distribution,
                               axisRatio_,
                               axisDirection_,
                               extent_->frontierRatio(),
                               std::vector<double>(features_->begin(), features_->end()));
    }
    else
    {
        return DebugHypothesis(current,
                               *extent_,
                               distribution,
                               axisRatio_,
                               axisDirection_,
                               extent_->frontierRatio(),
                               std::vector<double>(features_->begin(), features_->end()));
    }
}


void AreaHypothesis::initialize(void)
{
    extractSkeletonCells();
    calculateAxis();

    std::vector<Gateway> gateways;

    for(auto& n : nodes_)
    {
        if(boundaries_.empty() && (n->getType() & AreaNode::kGateway))
        {
            gateways.push_back(n->getGateway());
        }
    }

    for(auto& b : boundaries_)
    {
        gateways.push_back(b->getGateway());
    }

    extent_.reset(new AreaExtent(gateways, cells_, *grid_));
    polygonBoundary_ = extent_->polygonBoundary(math::ReferenceFrame::GLOBAL);
    rectangleBoundary_ = extent_->rectangleBoundary(math::ReferenceFrame::GLOBAL);
    center_ = extent_->center().toPoint();

    // Normalize the initial type distribution
    distribution_.normalize();

    // Setup the distances for paths leaving through each of the gateway nodes
    LoopGraph loopGraph;
    construct_loop_graph(*areaGraph_, loopGraph);
    NotExcludedEdge<LoopGraph> edgeFilter(&nodes_, &loopGraph);
    auto filtered = boost::make_filtered_graph(loopGraph, edgeFilter);

    std::vector<int> components(areaGraph_->sizeNodes(), -1);
    boost::connected_components(filtered, &components[0]);

    // Find the vertex index for the boundary so the correct component can be found
    for(auto node : nodes_)
    {
        if(node->getType() & AreaNode::kGateway)
        {
            int vertex = 0;
            for(std::size_t n = 0; n < areaGraph_->sizeNodes(); ++n)
            {
                if(loopGraph[n].node == node)
                {
                    vertex = n;
                    break;
                }
            }

            double maxDist = 0.0;
            for(std::size_t n = 0; n < areaGraph_->sizeNodes(); ++n)
            {
                if(components[n] == components[vertex])
                {
                    maxDist = std::max(areaGraph_->distanceBetweenNodes(node, loopGraph[n].node), maxDist);
                }
            }

            maxPathDist_[node] = maxDist;
        }
    }
}


SmallScaleStar AreaHypothesis::calculateStar(HypothesisType mask, GatewayFunc func) const
{
    auto gateways = CALL_MEMBER_FN(*this,func)(mask);
    return starBuilder_->buildStar(gateways,
                                   center(),
                                   rectangleBoundary());
}


std::vector<Gateway> AreaHypothesis::getGatewaysBasicType(HypothesisType type) const
{
    std::vector<Gateway> gateways;

    for(auto& boundary : boundaries_)
    {
        auto otherGraph = boundary->getOtherHypothesis(*this);
        if(otherGraph && (otherGraph->getType() == type))
        {
            gateways.push_back(boundary->getGateway());
        }
        assert(otherGraph);
    }

    return gateways;
}


std::vector<Gateway> AreaHypothesis::getGatewaysFullType(HypothesisType mask) const
{
    std::vector<Gateway> gateways;

    for(auto& boundary : boundaries_)
    {
        auto otherGraph = boundary->getOtherHypothesis(*this);
        if(otherGraph && (is_hypothesis_type(otherGraph->getGatewayType(*(boundary->getNode())), mask)))
        {
            gateways.push_back(boundary->getGateway());
        }
        assert(otherGraph);
    }

    return gateways;
}


std::vector<Point<double>> AreaHypothesis::getPoints(char mask) const
{
    std::vector<Point<double>> points;

    for(auto& node : nodes_)
    {
        if(node->getType() & mask)
        {
            points.push_back(node->getPosition());
        }
    }

    return points;
}


std::vector<const AreaNode*> AreaHypothesis::getNodes(char mask) const
{
    std::vector<const AreaNode*> nodes;

    for(auto& node : nodes_)
    {
        if(node->getType() & mask)
        {
            nodes.push_back(node);
        }
    }

    return nodes;
}


void AreaHypothesis::extractSkeletonCells(void)
{
    length_ = 0.0;

    for(auto& edge : edges_)
    {
        assert(edge);

        for(auto cellIt = edge->beginCell(), cellEnd = edge->endCell(); cellIt != cellEnd; ++cellIt)
        {
            cells_.push_back(cellIt->cell);
        }

        boundary_ |= edge->isBorderEdge();
        length_ += edge->getLength();
        width_ += edge->getAverageWidth() * edge->getLength();
    }

    if(length_ > 0.0)
    {
        width_ /= length_;
    }
}


void AreaHypothesis::calculateAxis(void)
{
    assert(!cells_.empty());

    double sumCos = 0.0;
    double sumSin = 0.0;
    double sumCosPlusPi = 0.0;
    double sumSinPlusPi = 0.0;
    double sumEccentricity = 0.0;

    for(auto& c : cells_)
    {
        // Try range [-pi/2,pi/2] and [0,pi] to see which gives a better average
        // Ignore any cells that haven't ended up in the isovist field
        if(isovistField_->contains(c))
        {
            double orientation = wrap_to_pi_2(isovistField_->at(c).scalar(utils::Isovist::kMaxThroughDistOrientation));
            double flippedOrientation = (orientation < 0) ? orientation + M_PI : orientation;

            sumCos += std::cos(orientation);
            sumSin += std::sin(orientation);
            sumCosPlusPi += std::cos(flippedOrientation);
            sumSinPlusPi += std::sin(flippedOrientation);
            sumEccentricity += isovistField_->at(c).scalar(utils::Isovist::kShapeEccentricity);
        }
    }

    double dir = wrap_to_pi_2(std::atan2(sumSin, sumCos));
    double plusPiDir = wrap_to_pi_2(std::atan2(sumSinPlusPi, sumCosPlusPi));

    double totalDiffDir = 0.0;
    double totalDiffPlusPi = 0.0;

    for(auto& c : cells_)
    {
        if(isovistField_->contains(c))
        {
            double orientation = wrap_to_pi_2(isovistField_->at(c).scalar(utils::Isovist::kMaxThroughDistOrientation));
            totalDiffDir += angle_diff_abs_pi_2(orientation, dir);
            totalDiffPlusPi += angle_diff_abs_pi_2(orientation, plusPiDir);
        }
    }

    axisDirection_ = (totalDiffDir < totalDiffPlusPi) ? dir : plusPiDir;
    axisRatio_ = sumEccentricity / cells_.size();
}


void AreaHypothesis::calculateFeaturesIfNeeded(void) const
{
    if(!features_)
    {
//         auto subgraph = visGraph_->createSubgraph(beginVisVertices(), endVisVertices());
        features_.reset(new HypothesisFeatures(*this, *extent_, *grid_, *isovistField_, *visGraph_, *skelGraph_));
    }
}


void AreaHypothesis::findPathEndpoints(void)
{
    std::vector<const AreaNode*> validBoundaryNodes;
    std::vector<DirectedPoint> possibleEndpoints;

    for(auto bnd : boundaryNodes())
    {
        // If there isn't space for the robot to fit on the other side, then skip this boundary
        if(maxPathDist_.empty() || (maxPathDist_[bnd] > 3.0))
        {
            validBoundaryNodes.push_back(bnd);
        }
    }

    // Use all nodes if there aren't enough valid boundaries found
    if(validBoundaryNodes.size() < 2)
    {
        validBoundaryNodes = boundaryNodes();
    }

    std::size_t numLoopNodes = 0;

    for(auto& bnd : validBoundaryNodes)
    {
        if(bnd->getType() & AreaNode::kGateway)
        {
            const auto& gwy = bnd->getGateway();

            DirectedPoint endpoint;
            endpoint.point = gwy.center();
            endpoint.node = bnd;
            endpoint.weight = gwy.probability(); //1.0 - other->getTypeDistribution().typeAppropriateness(HypothesisType::kDest);
            endpoint.width = gwy.length();
            endpoint.isGateway = true;

            double leftDiff = angle_diff_abs_pi_2(gwy.leftDirection(), axisDirection_);
            double rightDiff = angle_diff_abs_pi_2(gwy.rightDirection(), axisDirection_);
            endpoint.direction = leftDiff < rightDiff ? gwy.leftDirection() : gwy.rightDirection();

            if(bnd->isLoop())
            {
                ++numLoopNodes;
            }

            possibleEndpoints.push_back(endpoint);
        }
    }

    // Find the axis line for the path as the line intersecting the convex hull originating at the center of the
    // area and extending along the axis direction
    // The intersect is between line segment, so make the segment larger than any possible map to ensure an intersection
    // with the boundary occurs.
    Line<double> centerLine(Point<double>(center().x + 10000.0 * std::cos(axisDirection_),
                                                      center().y + 10000.0 * std::sin(axisDirection_)),
                                  Point<double>(center().x - 10000.0 * std::cos(axisDirection_),
                                                      center().y - 10000.0 * std::sin(axisDirection_)));
//
//     // If the center line intersects a gateway, then its an endpoint. If it intersects more than two gateways, consider
//     // the weird polygon approach.
//     for(auto& node : gatewayNodes)
//     {
//         if(node->isLoop() || line_segments_intersect(node->getGateway().boundary(), centerLine))
//         {
//             const auto& gwy = node->getGateway();
//
//             if((angle_diff_abs_pi_2(gwy.leftDirection(), axisDirection_) < M_PI / 3.0)
//                 || (angle_diff_abs_pi_2(gwy.rightDirection(), axisDirection_) < M_PI / 3.0))
//             {
//                 filteredGateways.push_back(node);
//             }
//         }
//     }
//
//     // obvious ends occur if at least two gateways are aligned to the axis of the area from the start
//     bool hasObviousEndGateways = filteredGateways.size() > 1;
//
//     // If there aren't end viable candidates based on loops, then consider the amount of graph that
//     // exists on the other side of a gateway
//     if(!hasObviousEndGateways && !maxPathDist_.empty())
//     {
//         filteredGateways.clear();
//         for(auto& bnd : maxPathDist_)
//         {
//             // Can the wheelchair fit back there?
//             if(bnd.second > 3.0)
//             {
//                 filteredGateways.push_back(bnd.first);
//             }
//         }
//
//         hasObviousEndGateways = filteredGateways.size() > 1;
//     }
//
//     if(!hasObviousEndGateways)
//     {
//         filteredGateways = gatewayNodes;
//         hasObviousEndGateways = filteredGateways.size() > 1;
//     }
//
//     for(auto& bnd : filteredGateways)
//     {
//         // For each gateway, select the direction that is most well-aligned with the axis of the area to get the best
//         // possible alignment for the endpoints
//         const auto& gwy = bnd->getGateway();
//         double leftDiff = angle_diff_abs_pi_2(gwy.leftDirection(), axisDirection_);
//         double rightDiff = angle_diff_abs_pi_2(gwy.rightDirection(), axisDirection_);
//         double direction =  leftDiff < rightDiff ? gwy.leftDirection() : gwy.rightDirection();
//
//         possibleEndpoints.push_back({gwy.center(), bnd, direction, true});
//     }

    if((possibleEndpoints.size() < 2) || (numLoopNodes < 2))
    {
        auto frontiers = getNodes(AreaNode::kDeadEnd);

        // If there aren't enough possible endpoints, then consider the junctions as well. By default they shouldn't be
        // included because they usually align very well with the axis and result in the path being too short.
        if(frontiers.size() + possibleEndpoints.size() < 2)
        {
            boost::push_back(frontiers, boost::as_array(getNodes(AreaNode::kJunction)));
        }

        // For each frontier point, need to find the direction. Use the orientation of the isovist at the cell.
        for(auto node : frontiers)
        {
            cell_t cell = utils::global_point_to_grid_cell_round(node->getPosition(), *grid_);
            if(isovistField_->contains(cell))
            {
                double direction = isovistField_->at(cell).scalar(utils::Isovist::kWeightedOrientation);
                possibleEndpoints.push_back({node->getPosition(), node, direction, 0.1, grid_->getMetricDistance(cell.x, cell.y), false});
            }
            else
            {
                std::cerr << "ERROR: No isovist found for skeleton cell at " << cell << '\n';
            }
        }

        // If there aren't enough points for two endpoints, then this area can't have them
        if(possibleEndpoints.size() < 2)
        {
            std::cerr << "ERROR: AreaHypothesis: Created area without enough endpoints. Must be at least two gateways + "
                << "frontiers/dead ends for a valid area." << " Areas:" << possibleEndpoints.size() << " Frontiers:"
                << frontiers.size() << " Nodes:" << nodes_.size() << '\n';
            for(auto n : nodes_)
            {
                std::cout << "Node:" << n->getPosition() << ':' << static_cast<int>(n->getType()) << '\n';
            }

            assert(possibleEndpoints.size() >= 2);
        }
    }

    PointPair endpoints;

    if(possibleEndpoints.size() != 2)
    {
        // NOTE: To ensure the correct inter-node, a new AreaGraph containing only the nodes in this area
        // would need to be computed. It's REALLY slow, so approximate with the original AreaGraph. In most cases,
        // these graphs will give the same inter-node distances, so the approximation is find for the 40% speedup.
        std::vector<Point<double>> boundaryIntersections;
        polygonBoundary_.intersections(centerLine, boundaryIntersections);
        if(boundaryIntersections.size() != 2)
        {
            if(boundaryIntersections.size() < 2)
            {
                std::cerr << "ERROR: AreaHypothesis: Axis line starting from center doesn't intersect the convex hull! Axis:"
                    << centerLine << " Boundary:" << polygonBoundary_ << '\n';
            }

            boundaryIntersections.resize(2);
            boundaryIntersections[0] = centerLine.a;
            boundaryIntersections[1] = centerLine.b;
        }

        endpoints = find_path_endpoints(possibleEndpoints,
                                        Line<double>(boundaryIntersections[0], boundaryIntersections[1]),
                                        axisDirection_,
                                        *areaGraph_);
    }
    else
    {
        endpoints.first = possibleEndpoints[0];
        endpoints.second = possibleEndpoints[1];
    }

    assignEndNodes(std::make_pair(endpoints.first.point, endpoints.second.point));

#ifdef DEBUG_ENDPOINTS
    std::cout << "DEBUG: AreaHypothesis: Selecting endpoint amongst gateways: ";
    for(auto& g : gateways)
    {
        std::cout << g.id() << ' ';
    }
    std::cout << '\n';

    std::cout << "DEBUG: AreaHypothesis: Endpoints: ";
    for(auto& node : endNodes_)
    {
        if(node->getType() & AreaNode::kGateway)
        {
            std::cout << "G:" << node->getGateway().id() << ':' << node->getGateway().boundary() << ' ';
        }
        else
        {
            std::cout << "F:" << node->getPosition() << ' ';
        }
    }
    std::cout << '\n';
#endif
}


void AreaHypothesis::mergeEndpoints(const std::vector<AreaHypothesis*>& areas)
{
    // Collect all the endpoints.
    std::vector<Point<double>> endpoints;
    std::vector<int8_t> hasMatch;
    std::vector<int> ids;
    for(auto& area : areas)
    {
        hasMatch.resize(area->endNodes_.size());
        std::fill(hasMatch.begin(), hasMatch.end(), 0);
        ids.clear();

        for(auto& node : area->endNodes_)
        {
            ids.push_back((node->getType() & AreaNode::kGateway) ? node->getGateway().id() : -1);
        }

        // We care about any endpoint that doesn't match another endpoint in one of the merged areas. Any matching
        // end gateways should be the merge point for the boundary.
        for(auto& other : areas)
        {
            if(other != area)
            {
                for(std::size_t n = 0; n < ids.size(); ++n)
                {
                    hasMatch[n] |= other->isEndGateway(ids[n]);
                }
            }
        }

        // There there weren't matches, then this endpoint is a new possible endpoint for the merged area, as it
        // won't be internal to the area like any matched gateway, which are now internal to the merged area
        for(std::size_t n = 0; n < ids.size(); ++n)
        {
            if(!hasMatch[n])
            {
                endpoints.push_back(area->endNodes_[n]->getPosition());
            }
        }
    }

    // Remove any endpoints where the count is more than one, as they are now internal to the area
    // Can't use erase_unique because unique still leaves one of the original values. Need to remove them all.
    auto pEnd = endpoints.end();
    for(auto pIt = endpoints.begin(); pIt < pEnd; ++pIt)
    {
        if(std::count(pIt, pEnd, *pIt) > 1)
        {
            pEnd = std::remove(pIt, pEnd, *pIt);
            pIt = endpoints.begin();
        }
    }
    endpoints.erase(pEnd, endpoints.end());

    // If only two endpoints remain, then a merge can be performed
    if(endpoints.size() == 2)
    {
        assignEndNodes(Endpoints(endpoints[0], endpoints[1]));
    }
    // Otherwise, need to do a full search for new endpoints
    else
    {
        findPathEndpoints();
    }
}


void AreaHypothesis::assignEndNodes(Endpoints endpoints)
{
    endNodes_.clear();

    // Once the actual position of the endpoints have been found, go through the nodes in the hypothesis
    // to determine which endpoint they correspond to
    for(auto& node : nodes_)
    {
        if(node->getPosition() == endpoints.first)
        {
            addEndNode(node);
        }

        if(node->getPosition() == endpoints.second)
        {
            addEndNode(node);
        }
    }

    if(endNodes_.size() < 2)
    {
        std::cerr << "ERROR: AreaHypothesis: Have a null end node. Endpoints:" << endpoints.first << "->"
        << endpoints.second << " Nodes:\n";
        for(auto& node : nodes_)
        {
            std::cerr << node->getPosition() << " Type:" << static_cast<int>(node->getType()) << '\n';

            if(node->getType() & AreaNode::kGateway)
            {
                std::cerr << "Gateway:" << node->getGateway().center() << '\n';
            }
        }
        assert(endNodes_.size() >= 2);
    }

    // Now check if either of the assigned end nodes leads to the same area as the established end node
    // If this node is associated with a boundary, then see if any other boundaries lead to the same area.
    // If so, those gateways are also end nodes, as the endpoint condition is area-based not gateway-based.
    for(int n = 0; n < 2; ++n)
    {
        const AreaNode* node = endNodes_[n];
        auto bndIt = std::find_if(boundaries_.begin(), boundaries_.end(), [node](auto& bnd) {
            return bnd->getNode() == node;
        });

        if(bndIt != boundaries_.end())
        {
            auto otherHyp = (*bndIt)->getOtherHypothesis(*this);

            for(auto& boundary : boundaries_)
            {
                if((boundary != *bndIt) && (boundary->getOtherHypothesis(*this) == otherHyp))
                {
                    endNodes_.push_back(boundary->getNode());
                }
            }
        }
    }
}


void AreaHypothesis::addEndNode(AreaNode* node)
{
    // This node will definitely be an end node
    endNodes_.push_back(node);
}


bool AreaHypothesis::isNonEndpointGateway(const AreaNode* node) const
{
    // If the node isn't on the boundary, then obviously, it isn't a gateway
    // NOTE: Can't just check if it is a gateway node because gateway nodes won't be on the boundary of merged areas!
    //       They'll be marked gateway because they are associated with a potential gateway, but only the boundaries of
    //       an area represent the final gateways
    auto boundaryNodeComp = [node](const AreaHypothesisBoundary* boundary) { return boundary->getNode() == node; };

    if(std::find_if(boundaries_.begin(), boundaries_.end(), boundaryNodeComp) == boundaries_.end())
    {
        return false;
    }

    return !utils::contains(endNodes_, node);
}


bool AreaHypothesis::isEndpoint(const AreaNode& node) const
{
    return utils::contains_if(endNodes_, [&node](auto& endNode) {
        return node.getPosition() == endNode->getPosition();
    });
}


std::vector<const AreaNode*> AreaHypothesis::boundaryNodes(void) const
{
    std::vector<const AreaNode*> bndNodes;

    // If boundaries exist, just use those nodes
    if(!boundaries_.empty())
    {
        for(auto& bnd : boundaries_)
        {
            bndNodes.push_back(bnd->getNode());
        }
    }
    // Otherwise internal gateway nodes must be valid.
    else
    {
        for(auto& node : nodes_)
        {
            if(node->getType() & AreaNode::kGateway)
            {
                bndNodes.push_back(node);
            }
        }
    }

    return bndNodes;
}


HypothesisContainment AreaHypothesis::singleBoundaryContainment(const AreaNode* lhsNode,
                                                                const AreaHypothesis& rhs,
                                                                const AreaNode* rhsNode) const
{
    auto otherLhsNode = findConnectedNode(lhsNode);
    auto otherRhsNode = rhs.findConnectedNode(rhsNode);

    auto lhsGateway = lhsNode->getGateway();

    if(lhsGateway.isPointToLeft(otherLhsNode->getPosition()) == lhsGateway.isPointToLeft(otherRhsNode->getPosition()))
    {
        return HypothesisContainment::complete;
    }
    else
    {
        return HypothesisContainment::none;
    }
}


const AreaNode* AreaHypothesis::findConnectedNode(const AreaNode* node) const
{
    auto edgeIt = std::find_if(edges_.begin(), edges_.end(),
                              [node](const AreaEdge* e) { return e->nodeIndex(node) >= 0; });
    assert(edgeIt != edges_.end());
    int otherIndex = ((*edgeIt)->nodeIndex(node) + 1) % 2;
    return (*edgeIt)->getEndpoint(otherIndex).get();
}


std::vector<AreaHypothesis*> extract_hypotheses(const std::vector<const AreaHypothesisBoundary*>& boundaries)
{
    std::set<AreaHypothesis*> areasToMerge;

    for(auto boundary : boundaries)
    {
        areasToMerge.insert(boundary->beginHypotheses(), boundary->endHypotheses());
    }

    return std::vector<AreaHypothesis*>(areasToMerge.begin(), areasToMerge.end());
}

} // namespace hssh
} // namespace vulcan
