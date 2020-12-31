/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_graph.cpp
* \author   Collin Johnson
*
* Implementation of HypothesisGraph.
*/

#include "hssh/local_topological/area_detection/labeling/hypothesis_graph.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "hssh/local_topological/area_extent.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/area_proposal.h"
#include <boost/graph/connected_components.hpp>
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <queue>
#include <iostream>

// #define DEBUG_SUBGRAPHS
// #define DEBUG_EDGE_CONSTRUCTION

namespace vulcan
{
namespace hssh
{

// Helpers for creating the hypotheses
void process_hypothesis_edge(AreaEdge* edge, std::set<AreaNode*>& subNodes, std::deque<AreaEdge*>& edgeQueue);
void enqueue_edges_on_same_side_of_gateway(const AreaNode*        gatewayNode,
                                           const AreaEdge*        edge,
                                           std::set<AreaNode*>&   subNodes,
                                           std::deque<AreaEdge*>& edgeQueue);
// Simple BFS for use in finding the mergeable regions
void enqueue_mergeable_boundaries(const std::vector<AreaHypothesisBoundary*>& boundaries, BoundaryMergeCondition condition, std::queue<AreaHypothesisBoundary*>& queue);


std::ostream& operator<<(std::ostream& out, const AreaEdge& edge)
{
    out << edge.getEndpoint(0)->getPosition() << "->" << edge.getEndpoint(1)->getPosition();
    return out;
}


HypothesisGraph::HypothesisGraph(AreaGraph& areaGraph,
                                 const VoronoiSkeletonGrid* grid,
                                 const VoronoiIsovistField* isovistField,
                                 std::shared_ptr<SmallScaleStarBuilder> starBuilder)
: grid_(grid)
, isovistField_(isovistField)
, visGraph_(areaGraph.toVisibilityGraph(1.0, *grid))
, skelGraph_(areaGraph.toGraph())
, starBuilder(std::move(starBuilder))
, nextId_(0)
{
    buildHypotheses(areaGraph);
    createBoundaries();

    // After constructing the graph, simplify it by merging frontiers and invalid regions
    simplify(BoundaryMergeCondition::kInvalid);
}


HypothesisGraph::~HypothesisGraph(void)
{
    // Only needed here for std::unique_ptr
}


HypothesisGraph::HypothesisConstIter HypothesisGraph::beginHypothesis(void) const
{
    return boost::make_transform_iterator<utils::SmartGetConstFunc<AreaHypothesis>>(hypotheses.begin());
}


HypothesisGraph::HypothesisConstIter HypothesisGraph::endHypothesis(void) const
{
    return boost::make_transform_iterator<utils::SmartGetConstFunc<AreaHypothesis>>(hypotheses.end());
}


const AreaHypothesis* HypothesisGraph::at(int id) const
{
    for(auto& hyp : hypotheses)
    {
        if(hyp->getId() == id)
        {
            return hyp.get();
        }
    }

    return nullptr;
}


HypothesisGraph::HypothesisIter HypothesisGraph::beginHypothesis(void)
{
    return boost::make_transform_iterator<utils::SmartGetFunc<AreaHypothesis>>(hypotheses.begin());
}


HypothesisGraph::HypothesisIter HypothesisGraph::endHypothesis(void)
{
    return boost::make_transform_iterator<utils::SmartGetFunc<AreaHypothesis>>(hypotheses.end());
}


HypothesisGraph::BoundaryConstIter HypothesisGraph::beginBoundary(void) const
{
    return boost::make_transform_iterator<utils::SmartGetConstFunc<AreaHypothesisBoundary>>(boundaries.begin());
}


HypothesisGraph::BoundaryConstIter HypothesisGraph::endBoundary(void) const
{
    return boost::make_transform_iterator<utils::SmartGetConstFunc<AreaHypothesisBoundary>>(boundaries.end());
}


HypothesisGraph::BoundaryIter HypothesisGraph::beginBoundary(void)
{
    return boost::make_transform_iterator<utils::SmartGetFunc<AreaHypothesisBoundary>>(boundaries.begin());
}


HypothesisGraph::BoundaryIter HypothesisGraph::endBoundary(void)
{
    return boost::make_transform_iterator<utils::SmartGetFunc<AreaHypothesisBoundary>>(boundaries.end());
}


int HypothesisGraph::simplify(BoundaryMergeCondition condition)
{
    int numRegionsMerged = 0;

    // NOTE: The end of boundaries will change if any regions are merged, so don't store it initially
    for(auto boundaryIt = boundaries.begin(); boundaryIt != boundaries.end();)
    {
        if((*boundaryIt)->shouldMerge(condition))
        {
            mergeRegion(findMergeableRegion(boundaryIt->get(), condition),
                        (*boundaryIt)->getHypotheses()[0]->getType(),
                        (*boundaryIt)->getHypotheses()[0]->getTypeDistribution());
            boundaryIt = boundaries.begin();
            ++numRegionsMerged;
        }
        else
        {
            ++boundaryIt;
        }
    }

    return numRegionsMerged;
}


std::vector<const AreaHypothesisBoundary*> HypothesisGraph::findMergeableRegion(const AreaHypothesis* start) const
{
    std::vector<AreaHypothesisBoundary*> mergeBoundaries;

    // Find one of the kSameType mergeable boundaries associated with the start hypothesis and use it as the basis for the
    // findMergeableRegion work method
    for(auto boundaryIt = start->beginBoundary(), boundaryEnd = start->endBoundary(); boundaryIt != boundaryEnd; ++boundaryIt)
    {
        if((*boundaryIt)->shouldMerge(BoundaryMergeCondition::kSameType))
        {
            mergeBoundaries = findMergeableRegion(*boundaryIt, BoundaryMergeCondition::kSameType);
            break;
        }
    }

    // Convert the non-const pointers to const pointer for the output. The helper internally needs the non-const for some of the
    // modifying operations
    return std::vector<const AreaHypothesisBoundary*>(mergeBoundaries.begin(), mergeBoundaries.end());
}


AreaHypothesis* HypothesisGraph::mergeRegion(const AreaHypothesis* start)
{
    // Why oh why can't I compare a const* to a *?
    AreaHypothesis* startHyp = const_cast<AreaHypothesis*>(start);

    if(hypToBoundaries.find(startHyp) == hypToBoundaries.end())
    {
        return nullptr;
    }

    // Find one of the kSameType mergeable boundaries associated with the start hypothesis and use it as the basis for the
    // mergeRegion work method
    for(auto boundary : hypToBoundaries[startHyp])
    {
        if(boundary->shouldMerge(BoundaryMergeCondition::kSameType))
        {
            return mergeRegion(findMergeableRegion(boundary, BoundaryMergeCondition::kSameType),
                               start->getType(),
                               start->getTypeDistribution());
        }
    }

    return nullptr;
}


AreaHypothesis* HypothesisGraph::mergeHypotheses(const std::vector<AreaHypothesis*>& toMerge, HypothesisType type)
{
    // If not requesting a merge, then it must be successful
    if(toMerge.empty())
    {
        std::cout << "WARNING: HypothesisGraph: Requested a merge with no areas.\n";
        return nullptr;
    }
    // If there's only a single area, then no actual merge needs to occur. Just return the same pointer back.
    else if(toMerge.size() == 1)
    {
        std::cout << "WARNING: HypothesisGraph: Requested a merge with only one area.\n";
        toMerge[0]->setType(type);
        return toMerge[0];
    }

    std::vector<AreaHypothesisBoundary*> boundariesToMerge;

    // Go through each boundary and if both hypotheses are in the provided set of hypotheses to merge, add the boundary
    // to the set of boundaries to be merged
    for(auto& b : boundaries)
    {
        bool hasLeft = std::find(toMerge.begin(), toMerge.end(), b->getHypotheses()[0]) != toMerge.end();
        bool hasRight = std::find(toMerge.begin(), toMerge.end(), b->getHypotheses()[1]) != toMerge.end();
        if(hasLeft && hasRight)
        {
            boundariesToMerge.push_back(b.get());
        }
    }

    if(!boundariesToMerge.empty())
    {
        return mergeRegion(boundariesToMerge, type, toMerge.front()->getTypeDistribution());
    }
    else
    {
        return nullptr;
    }
}


std::vector<AreaProposal> HypothesisGraph::createProposalsForCurrentHypotheses(const VoronoiSkeletonGrid& grid) const
{
    std::vector<AreaProposal> proposals;

    for(auto& hyp : hypotheses)
    {
        proposals.push_back(hyp->toAreaProposal());
    }

    return proposals;
}


std::vector<HypothesisGraph::HypVec> HypothesisGraph::findSubgraphs(AreaHypothesis* removed)
{
    using namespace boost;
    using Graph = adjacency_list<vecS, vecS, undirectedS, no_property, no_property, no_property, vecS>;

    // Create a mapping of hypothesis to vertex
    std::unordered_map<AreaHypothesis*, Graph::vertex_descriptor> hypToVertex;
    int nextId = 0;
    for(auto& hyp : hypotheses)
    {
        // Don't include the removed hypothesis in the graph
        if(hyp.get() != removed)
        {
            hypToVertex[hyp.get()] = nextId;
            ++nextId;
        }
    }

    // The vertex descriptors are automatically assigned and correspond to the vector indexes
    Graph graph(hypToVertex.size());

    // Create edges between all non-removed hypotheses
    for(auto& hypToBounds : hypToBoundaries)
    {
        // Ignore boundaries associated with the removed vertex
        if(hypToBounds.first == removed)
        {
            continue;
        }

        const std::vector<AreaHypothesisBoundary*>& bounds = hypToBounds.second;

        for(auto& b : bounds)
        {
            AreaHypothesis* otherHyp = b->getOtherHypothesis(*hypToBounds.first);

            // Ignore edges leading to the removed hypothesis
            if(otherHyp != removed)
            {
                add_edge(hypToVertex[hypToBounds.first], hypToVertex[otherHyp], graph);
            }
        }
    }

    // Once the edges are added, find the connected components in the graph
    std::vector<int> components(hypToVertex.size());
    int numComponents = connected_components(graph, &components[0]);

    // Create the subgraphs from the connected components
    std::vector<HypVec> subgraphs(numComponents);
    for(auto& hypToVert : hypToVertex)
    {
        // The component is stored in the particular index
        int component = components[hypToVert.second];
        subgraphs[component].push_back(hypToVert.first);
    }

    std::cout << "DEBUG: HypothesisGraph: Found " << numComponents << " subgraphs:\n";
    for(auto& sub : subgraphs)
    {
        for(auto& hyp : sub)
        {
            std::cout << hyp->center() << ' ';
        }
        std::cout << '\n';
    }

    return subgraphs;
}


void HypothesisGraph::buildHypotheses(AreaGraph& graph)
{
    // Create a hypothesis for every gateway node. Each edge should only be visited
    // a single time.
    auto gatewayNodes = graph.getNodes(AreaNode::kGateway);

    for(auto& node : gatewayNodes)
    {
        for(auto edgeIt = node->beginEdge(), edgeEnd = node->endEdge(); edgeIt != edgeEnd; ++edgeIt)
        {
            if(visitedEdges.find(edgeIt->get()) == visitedEdges.end())
            {
                extractHypothesisFromEdge(edgeIt->get(), graph);
            }
        }
    }

    // If there aren't any gateway nodes, then there's only a single area to extract
    if(gatewayNodes.empty() && (graph.beginEdges() != graph.endEdges()))
    {
        extractHypothesisFromEdge(graph.beginEdges()->get(), graph);
    }
}


void HypothesisGraph::extractHypothesisFromEdge(AreaEdge* edge, AreaGraph& graph)
{
    /*
    * Extract a hypothesis from this graph. The hypothesis starts at an edge, both endpoints of the
    * edge are considered. If the node matches the type mask, then it is not further expanded. If, however, the
    * node doesn't match the mask, then all of its other edges are enqueued and the process repeats until the edge
    * is empty.
    */

    std::set<AreaNode*> nodes;
    std::set<AreaEdge*> edges;

    std::deque<AreaEdge*> edgeQueue;
    edgeQueue.push_back(edge);

    while(!edgeQueue.empty())
    {
        auto currentEdge = edgeQueue.front();

        if(visitedEdges.find(currentEdge) == visitedEdges.end())
        {
            process_hypothesis_edge(currentEdge, nodes, edgeQueue);
            edges.insert(currentEdge);
            visitedEdges.insert(currentEdge);
        }

        edgeQueue.pop_front();

#ifdef DEBUG_EDGE_CONSTRUCTION
        std::cout << "DEBUG:HypothesisGraph: Current edge:" << *currentEdge << " Queue:\n";
        for(auto& e : edgeQueue)
        {
            std::cout << '\t' << *e << '\n';
        }
#endif
    }

#ifdef DEBUG_SUBGRAPHS
    std::cout<<"DEBUG:HypothesisGraph: New hypothesis: Started at: " << *edge << '\n';
    for(auto& node : nodes)
    {
        std::cout<<node->getPosition()<<'\n';
    }
#endif

    if(nodes.size() > 1)
    {
        hypotheses.emplace_back(new AreaHypothesis(nextId_,
                                                   &graph,
                                                   nodes,
                                                   edges,
                                                   grid_,
                                                   isovistField_,
                                                   &visGraph_,
                                                   &skelGraph_,
                                                   starBuilder.get()));
        ++nextId_;

        for(auto& node : nodes)
        {
            if(node->getType() & AreaNode::kGateway)
            {
                addHypothesisToNode(node, hypotheses.back().get());
            }
        }
    }
#ifdef DEBUG_SUBGRAPHS
    else
    {
        std::cerr << "WARNING:HypothesisGraph: Failed to create an area for edge. Only a single valid node:\n"
            << (*(nodes.begin()))->getPosition() << '\n';

    }
#endif
}


void HypothesisGraph::addHypothesisToNode(AreaNode* node, AreaHypothesis* hypothesis)
{
    // If the node hasn't been associated with a sub yet, then a new array needs to be
    // added to the map. Otherwise, we know the second spot in the array is unfilled and ready
    // for the new sub
    auto nodeIt = nodeToHypotheses.find(node);

    if(nodeIt == nodeToHypotheses.end())
    {
        nodeToHypotheses[node] = std::make_pair(hypothesis, nullptr);
    }
    else
    {
        if(nodeIt->second.second)
        {
            std::cerr<<"ERROR::HypothesisGraph: Trying to assign all sorts of things to a node: " << node->getPosition()
                << " : " << nodeIt->second.first->rectangleBoundary() << " and "
                << nodeIt->second.second->rectangleBoundary() << " Trying to add:" << hypothesis->rectangleBoundary()
                << '\n';

//             assert(!nodeIt->second.second);
        }
        nodeIt->second.second = hypothesis;
    }
}


void HypothesisGraph::createBoundaries(void)
{
    // Create the boundary and add it to both associated hypotheses
    for(auto& nodePair : nodeToHypotheses)
    {
        // If the node pairs
        if(!nodePair.second.first || !nodePair.second.second)
        {
            std::cout << "WARNING::HypothesisGraph: Attempted to create invalid gateway for boundary at "
                << nodePair.first->getPosition() << " Edges:\n";
            for(auto& edge : boost::make_iterator_range(nodePair.first->beginEdge(), nodePair.first->endEdge()))
            {
                std::cout << *edge << '\n';
            }
            continue;
        }
        else if(nodePair.second.first == nodePair.second.second)
        {
            std::cout<<"INFO::HypothesisGraph: Located invalid gateway at "<<nodePair.first->getPosition()<<". Both edges in the same hypothesis.\n";
            continue;
        }

        boundaries.emplace_back(new AreaHypothesisBoundary(nodePair.first, Hypotheses{{nodePair.second.first, nodePair.second.second}}));

        nodePair.second.first->addBoundary(boundaries.back().get());
        nodePair.second.second->addBoundary(boundaries.back().get());

        hypToBoundaries[nodePair.second.first].push_back(boundaries.back().get());
        hypToBoundaries[nodePair.second.second].push_back(boundaries.back().get());
    }
}


std::vector<AreaHypothesisBoundary*> HypothesisGraph::findMergeableRegion(const AreaHypothesisBoundary* start, BoundaryMergeCondition condition) const
{
    std::set<const AreaHypothesis*>      searched;
    std::vector<AreaHypothesisBoundary*> visited;
    std::queue<AreaHypothesisBoundary*>  boundaryQueue;

    AreaHypothesis* initialHyp = *(start->beginHypotheses());

    auto hypToBoundaryIt = hypToBoundaries.find(initialHyp);
    assert(hypToBoundaryIt != hypToBoundaries.end());
    enqueue_mergeable_boundaries(hypToBoundaryIt->second, condition, boundaryQueue);
    searched.insert(initialHyp);

    while(!boundaryQueue.empty())
    {
        auto boundary = boundaryQueue.front();
        boundaryQueue.pop();

        // Already been to this boundary? Just keep going
        if(std::find(visited.begin(), visited.end(), boundary) != visited.end())
        {
            continue;
        }

        visited.push_back(boundary);

        for(auto hyp : boundary->getHypotheses())
        {
            if(searched.find(hyp) == searched.end())
            {
                hypToBoundaryIt = hypToBoundaries.find(hyp);
                assert(hypToBoundaryIt != hypToBoundaries.end());
                enqueue_mergeable_boundaries(hypToBoundaryIt->second, condition, boundaryQueue);
                searched.insert(hyp);
            }
        }
    }

    return visited;
}


AreaHypothesis* HypothesisGraph::mergeRegion(const std::vector<AreaHypothesisBoundary*>& boundariesToMerge,
                                             HypothesisType type,
                                             HypothesisTypeDistribution distribution)
{
    // Find all boundaries contained within the mergeable region, then find the hypotheses associated with those boundaries.
    // The merge constructor for AreaHypothesis can be used to make the new area. Once created, all the boundaries adjacent
    // to the new area hypothesis need to have one of their areas changed to replace the old area with the newly merged area
    // After that operation is complete, there are no more references to either the merged hypotheses or merged boundaries
    // in the graph, so they can be safely removed.
    std::set<AreaHypothesis*> hypsToMerge;

    for(auto boundary : boundariesToMerge)
    {
        hypsToMerge.insert(boundary->beginHypotheses(), boundary->endHypotheses());
    }

    // Use the type from one of the starting areas for the type of the merged area. If the condition is kSameType, we know
    // they're already the same type. If they are frontiers or invalid, then the type will be Area anyhow.
    // The constructor wants const-boundaries, so create those. Easier to do it here than create two constructors that take a const or non-const
    // boundaries. Can only access const-boundaries outside of HypothesisGraph!
    std::vector<const AreaHypothesisBoundary*> constBoundaries(boundariesToMerge.begin(), boundariesToMerge.end());
    hypotheses.emplace_back(new AreaHypothesis(constBoundaries, type));

    AreaHypothesis* mergeHyp = hypotheses.back().get();

    // Go through all the hypotheses. For each hypothesis boundary that isn't one of the merged
    // boundaries, replace the hypothesis with the merged hypothesis.
    for(auto hyp : hypsToMerge)
    {
        for(auto boundary : hypToBoundaries[hyp])
        {
            if(std::find(boundariesToMerge.begin(), boundariesToMerge.end(), boundary) == boundariesToMerge.end())
            {
                boundary->changeArea(hyp, mergeHyp);
                hypToBoundaries[mergeHyp].push_back(boundary);
            }
        }
    }

    // At this point, all dependencies on the merged hypotheses and merged boundaries are handled, so no
    // references exist for them. We can go through and erase the boundaries and the hypotheses now
    for(auto hyp : hypsToMerge)
    {
        hypotheses.erase(std::find_if(hypotheses.begin(), hypotheses.end(), utils::UniqueToRawComp<AreaHypothesis>(hyp)));
        hypToBoundaries.erase(hyp);
    }

    for(auto boundary : boundariesToMerge)
    {
        boundaries.erase(std::find_if(boundaries.begin(), boundaries.end(), utils::UniqueToRawComp<AreaHypothesisBoundary>(boundary)));
    }

    mergeHyp->setType(type);
    mergeHyp->setTypeDistribution(distribution);

    return mergeHyp;
}


void process_hypothesis_edge(AreaEdge* edge, std::set<AreaNode*>& subNodes, std::deque<AreaEdge*>& edgeQueue)
{
    auto endpoints = edge->getEndpoints();

    for(std::size_t n = 0; n < endpoints.size(); ++n)
    {
        AreaNode* edgeNode = endpoints[n].get();
        subNodes.insert(edgeNode);

        // Any gateway node terminates the hypothesis, so only edges on the same side of the gateway should be added
        if(!(edgeNode->getType() & AreaNode::kGateway))
        {
            for(auto& edge : boost::make_iterator_range(edgeNode->beginEdge(), edgeNode->endEdge()))
            {
                edgeQueue.push_back(edge.get());
            }
        }
        // Otherwise, if there are more than two edges incident to the gateway, then only take the edges on the
        // same side of the gateway. For gateways with only two edges, then by definition there will only be one
        // on each side of the gateway, so there's no need to do the extra operation
        else // if(endpoints[n]->numEdges() > 2)
        {
            enqueue_edges_on_same_side_of_gateway(endpoints[n].get(), edge, subNodes, edgeQueue);
        }
    }
}


void enqueue_edges_on_same_side_of_gateway(const AreaNode*        gatewayNode,
                                           const AreaEdge*        edge,
                                           std::set<AreaNode*>&   subNodes,
                                           std::deque<AreaEdge*>& edgeQueue)
{
    // Determine which side of the gateway the current hypothesis is on. Only enqueue edges on the same side of the gateway
    // Usually there won't be any more edges. Only in the case where the gateway node is also a junction will there possibly
    // end up being an extra edge pushed into the queue for the area hypothesis
    int isOnLeft = edge->isLeftOfGateway(*gatewayNode);

#ifdef DEBUG_EDGE_CONSTRUCTION
    std::cout << "DEBUG: HypothesisGraph: Gateway:" << gatewayNode->getPosition() << " Edge:" << *edge << " Left? "
        << isOnLeft << '\n';
#endif

    for(auto edgeIt = gatewayNode->beginEdge(), edgeEnd = gatewayNode->endEdge(); edgeIt != edgeEnd; ++edgeIt)
    {
        auto nodeEdge = *edgeIt;

        if(nodeEdge->isLeftOfGateway(*gatewayNode) == isOnLeft)
        {
            auto otherEnd = (nodeEdge->getEndpoint(0).get() == gatewayNode) ? nodeEdge->getEndpoint(1) : nodeEdge->getEndpoint(0);

            edgeQueue.push_back(nodeEdge.get());
            subNodes.insert(otherEnd.get());
        }

#ifdef DEBUG_EDGE_CONSTRUCTION
        std::cout << "\tEdge:" << *nodeEdge << " Left:" << nodeEdge->isLeftOfGateway(*gatewayNode) << '\n';
#endif
    }
}


void enqueue_mergeable_boundaries(const std::vector<AreaHypothesisBoundary*>& boundaries, BoundaryMergeCondition condition, std::queue<AreaHypothesisBoundary*>& queue)
{
    // Any boundaries satisfying the merge condition should be put in the queue. Elsewhere in the search, duplicate boundaries in the
    // queue will be addressed
    for(auto boundary : boundaries)
    {
        if(boundary->shouldMerge(condition))
        {
            queue.push(boundary);
        }
    }
}

} // namespace hssh
} // namespace vulcan
