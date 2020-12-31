/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     alignment_network_filter.h
* \author   Collin Johnson
*
* Definition of class templates for various filters for the AlignmentNetwork graph:
*
*   - NonDestEdge/Vertex : filters to create a graph containing only path segments and decision points
*   - ActiveEdge/Vertex : filters to create a subset of the graph with only the specified nodes
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_NETWORK_FILTER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_NETWORK_FILTER_H

#include "utils/algorithm_ext.h"

namespace vulcan
{
namespace hssh
{

/////////////////// Functors for filtering out destinations from the graph //////////////////////
template <class Graph, class Network>
struct NonDestEdge
{
    Network* network;
    Graph* graph;

    template <typename Edge>
    bool operator()(Edge edge) const
    {
        return (network->assignedType(boost::source(edge, *graph)) != HypothesisType::kDest) &&
            (network->assignedType(boost::target(edge, *graph)) != HypothesisType::kDest);
    }
};

template <class Graph, class Network>
struct NonDestVertex
{
    Network* network;
    Graph* graph;

    template <typename Vertex>
    bool operator()(Vertex vertex) const
    {
        return network->assignedType(vertex) != HypothesisType::kDest;
    }
};

/////////////////// Functors for splitting an area //////////////////////
template <class Graph>
struct NonSplitActiveEdge
{
    const std::vector<int>* activeIds;
    std::size_t splitId;
    Graph* graph;

    template <typename Edge>
    bool operator()(Edge edge) const
    {
        bool isPartOfActive = utils::contains(*activeIds, boost::source(edge, *graph))
            || utils::contains(*activeIds, boost::target(edge, *graph));
        bool isNotSplit = (splitId != boost::source(edge, *graph)) && (splitId != boost::target(edge, *graph));

        return isPartOfActive && isNotSplit;
    }

    explicit NonSplitActiveEdge(const std::vector<int>* activeIds = nullptr,
                                int splitId = 0,
                                Graph* graph = nullptr)
    : activeIds(activeIds)
    , splitId(splitId)
    , graph(graph)
    {
    }
};

template <class Graph>
struct NonSplitGatewayActiveEdge
{
    const std::vector<int>* activeIds;
    std::pair<std::size_t, std::size_t> splitIds;
    Graph* graph;

    template <typename Edge>
    bool operator()(Edge edge) const
    {
        bool isPartOfActive = utils::contains(*activeIds, boost::source(edge, *graph))
            || utils::contains(*activeIds, boost::target(edge, *graph));
        bool isSplit = ((splitIds.first == boost::source(edge, *graph)) && (splitIds.second == boost::target(edge, *graph)))
            || ((splitIds.second == boost::source(edge, *graph)) && (splitIds.first == boost::target(edge, *graph)));

        return isPartOfActive && !isSplit;
    }

    explicit NonSplitGatewayActiveEdge(const std::vector<int>* activeIds = nullptr,
                                       std::pair<std::size_t, std::size_t> splitIds = { -1, -1 },
                                       Graph* graph = nullptr)
    : activeIds(activeIds)
    , splitIds(splitIds)
    , graph(graph)
    {
    }
};

template <class Graph>
struct ActiveVertex
{
    const std::vector<int>* activeIds;
    Graph* graph;

    template <typename Vertex>
    bool operator()(Vertex vertex) const
    {
        return utils::contains(*activeIds, vertex);
    }

    explicit ActiveVertex(const std::vector<int>* activeIds = nullptr,
                          Graph* graph = nullptr)
    : activeIds(activeIds)
    , graph(graph)
    {
    }
};

///////////// Functors for selecting only a subset of all vertices /////////////////
template <class Graph>
struct InSetVertex
{
    using IdSet = std::unordered_set<int>;

    const IdSet* idSet;
    Graph* graph;

    template <typename Vertex>
    bool operator()(Vertex vertex) const
    {
        return idSet->find(vertex) != idSet->end();
    }

    explicit InSetVertex(const IdSet* idSet= nullptr,
                         Graph* graph = nullptr)
    : idSet(idSet)
    , graph(graph)
    {
    }
};

template <class Graph>
struct InSetEdge
{
    using IdSet = std::unordered_set<int>;

    const IdSet* idSet;
    Graph* graph;

    template <typename Edge>
    bool operator()(Edge edge) const
    {
        return (idSet->find(boost::source(edge, *graph)) != idSet->end())
            && (idSet->find(boost::target(edge, *graph)) != idSet->end());
    }

    explicit InSetEdge(const IdSet* idSet = nullptr,
                       Graph* graph = nullptr)
    : idSet(idSet)
    , graph(graph)
    {
    }
};

//////////////////// Functors for selecting anything not in a subset of the vertices //////////////
template <class Graph>
struct InactiveVertex
{
    const std::vector<int>* inactiveIds;
    Graph* graph;

    template <typename Vertex>
    bool operator()(Vertex vertex) const
    {
        return !utils::contains(*inactiveIds, vertex);
    }

    explicit InactiveVertex(const std::vector<int>* inactiveIds = nullptr,
                            Graph* graph = nullptr)
    : inactiveIds(inactiveIds)
    , graph(graph)
    {
    }
};

template <class Graph>
struct InactiveEdge
{
    const std::vector<int>* inactiveIds;
    Graph* graph;

    template <typename Edge>
    bool operator()(Edge edge) const
    {
        return !utils::contains(*inactiveIds, boost::source(edge, *graph))
            && !utils::contains(*inactiveIds, boost::target(edge, *graph));
    }

    explicit InactiveEdge(const std::vector<int>* inactiveIds = nullptr,
                          Graph* graph = nullptr)
    : inactiveIds(inactiveIds)
    , graph(graph)
    {
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_NETWORK_FILTER_H
