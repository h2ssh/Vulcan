/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_graph.cpp
 * \author   Collin Johnson
 *
 * Definition of AreaGraph, AreaNode, and AreaEdge.
 */

#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "hssh/local_topological/area_detection/labeling/loops_and_trees.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/algorithm_ext.h"
#include <algorithm>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/make_shared.hpp>
#include <deque>
#include <iostream>
#include <limits>
#include <set>

// #define DEBUG_EDGE
// #define DEBUG_GRAPH

namespace vulcan
{
namespace hssh
{

// Check if the anchors associated with this skeleton cell are frontiers
bool is_frontier_skeleton(cell_t cell, const VoronoiSkeletonGrid& grid);

inline bool are_adjacent_cells(cell_t lhs, cell_t rhs)
{
    return (lhs.x - rhs.x >= -1) && (lhs.x - rhs.x <= 1) && (lhs.y - rhs.y >= -1) && (lhs.y - rhs.y <= 1);
}

inline bool is_cell_on_map_border(cell_t cell, const VoronoiSkeletonGrid& grid)
{
    int width = grid.getWidthInCells();
    int height = grid.getHeightInCells();
    return (cell.x == 0) || (cell.y == 0) || (cell.x + 1 == width) || (cell.y + 1 == height);
}


AreaNode::AreaNode(const Point<float>& position, cell_t cell, char typeMask, bool onBoundary)
: position(position)
, cell(cell)
, type(typeMask)
, onBoundary(onBoundary)
{
    assert(!(typeMask & kGateway));
}


AreaNode::AreaNode(const Gateway& gateway, Point<float> position)
: position(position)
, cell(gateway.skeletonCell())
, type(kGateway)
, onBoundary(false)
, gateway(gateway)
{
}


void AreaNode::addEdge(AreaEdgePtr edge)
{
    if (edge) {
        edges.push_back(edge);
    }
}


AreaEdge::AreaEdge(const EndpointArray& endpoints,
                   const std::vector<voronoi_cell_t>& cells,
                   float scale,
                   bool borderEdge)
: endpoints(endpoints)
, cells(cells)
, borderEdge(borderEdge)
{
    using namespace boost::accumulators;

    assert(endpoints[0]);
    assert(endpoints[1]);

    // The length is approximately the number of cells times the meters per cell. It isn't quite exact for curves, but
    // is close
    length = (cells.size() + 1) * scale;

    accumulator_set<double, stats<tag::mean, tag::variance, tag::max, tag::min>> statsAcc;

    // Go through and calculate the statistics on the width of the edge
    int numFrontierCells = 0;

    for (auto& cell : cells) {
        statsAcc(cell.distance * 2.0);

        if (cell.isFrontier) {
            ++numFrontierCells;
        }
    }

    frontierRatio = static_cast<float>(numFrontierCells) / cells.size();

    minWidth = min(statsAcc);
    maxWidth = max(statsAcc);
    averageWidth = mean(statsAcc);
    stdDevWidth = std::sqrt(variance(statsAcc));
}


int AreaEdge::nodeIndex(const AreaNode* node) const
{
    if (endpoints[0].get() == node) {
        return 0;
    } else if (endpoints[1].get() == node) {
        return 1;
    } else {
        return -1;
    }
}


int AreaEdge::isLeftOfGateway(const AreaNode& gatewayNode) const
{
    // Search in order through the cells, starting at the gateway node, until a cell is encountered that isn't on
    // the boundary.
    assert(gatewayNode.getType() & AreaNode::kGateway);
    assert((gatewayNode == *endpoints[0]) || (gatewayNode == *endpoints[1]));

    if (cells.empty()) {
        return (gatewayNode == *endpoints[0])
          ? (gatewayNode.getGateway().isPointToLeft(endpoints[1]->getPosition()) > 0)
          : (gatewayNode.getGateway().isPointToLeft(endpoints[0]->getPosition()) > 0);
    }

    Gateway gateway = gatewayNode.getGateway();
    int startIndex = 0;
    int endIndex = cells.size();
    int dir = 1;

    // If it isn't the first cell, then switch the direction of the iterations
    if (distance_between_points(gateway.skeletonCell(), cells.front().cell)
        > distance_between_points(gateway.skeletonCell(), cells.back().cell)) {
        startIndex = cells.size() - 1;
        endIndex = -1;
        dir = -1;
    }

    // Search until a dominant direction is found
    int numLeft = 0;

    for (int index = startIndex; (index != endIndex) && (std::abs(numLeft) < 20); index += dir) {
        numLeft += gateway.isCellToLeft(cells[index].cell);
    }

    if (numLeft == 0) {
        for (int index = startIndex; (index != endIndex) && (std::abs(numLeft) < 20); index += dir) {
            numLeft += gateway.isPointToLeft(cells[index].position);
        }
    }

#ifdef DEBUG_EDGE
    // All cells were on the boundary, which is curious, so output a message indicating the unusual state, though it
    // isn't an error
    if (numLeft == 0) {
        std::cerr << "WARNING::AreaEdge: All cells were evenly split between left and right on the gateway metric "
                     "boundary, which is curious. Gateway:"
                  << gateway.skeletonCell() << " Edge cells:\n";
        std::transform(cells.begin(),
                       cells.end(),
                       std::ostream_iterator<cell_t>(std::cerr, " "),
                       [](const voronoi_cell_t& c) {
                           return c.cell;
                       });
        std::cerr << '\n';
    }
#endif

    return (numLeft == 0) ? 0 : (numLeft > 0) ? 1 : -1;
}


AreaGraph::AreaGraph(const VoronoiSkeletonGrid& grid, const std::vector<Gateway>& gateways)
{
    initializeNodes(grid, gateways);

    NeighborArray neighbors;

    for (auto nodeIt = cellToNode.begin(), nodeEnd = cellToNode.end(); nodeIt != nodeEnd; ++nodeIt) {
        std::size_t numNeighbors = neighbor_cells_with_classification(nodeIt->first,
                                                                      SKELETON_CELL_REDUCED_SKELETON,
                                                                      grid,
                                                                      FOUR_THEN_EIGHT_WAY,
                                                                      neighbors);

        for (std::size_t n = 0; n < numNeighbors; ++n) {
            cell_t& neighbor = neighbors[n];

            // If two nodes are immediate neighbors, an edge still needs to be added between them!
            // All that will happen is a zero-length edge will be created
            // Only create this node, bypassing the normal check, if the node considered it less than the neighbor.
            // This ensures that the edge is only added once.
            auto neighborIt = cellToNode.find(neighbor);
            if ((neighborIt != cellToNode.end()) && (nodeIt->first < neighbor)) {
                createEdgeBetweenNeighbors(*nodeIt, *neighborIt, grid);
            } else if (visited.find(neighbor) == visited.end()) {
                extractSkeletonEdge(nodeIt->second, nodeIt->first, neighbor, grid);
            }
        }
    }

    // Go through all the gateways. If there's a gateway that doesn't have more than a single reduced neighbor,
    // turn it off because the gateway doesn't lead anywhere
    for (auto& gwy : gateways) {
        std::size_t numNeighbors = neighbor_cells_with_classification(gwy.skeletonCell(),
                                                                      SKELETON_CELL_REDUCED_SKELETON,
                                                                      grid,
                                                                      FOUR_THEN_EIGHT_WAY,
                                                                      neighbors);
        // If there's only one neighbor and it is a gateway node, set the type to dead end and not gateway because
        // there's nothing on the other side of the gateway, so it isn't bounding any structure
        if (numNeighbors == 1) {
            auto nodeIt = cellToNode.find(gwy.skeletonCell());
            if (nodeIt != cellToNode.end()) {
                std::cout << "Turning off gateway: " << nodeIt->first << " Not enough neighbors.\n";
                nodeIt->second->type &= ~AreaNode::kGateway;
            } else {
                std::cout << "Failed to detect gateway at " << gwy.skeletonCell() << '\n';
            }
        }
    }

    cellToNode.clear();
    visited.clear();

    LoopGraph loopGraph;
    construct_loop_graph(*this, loopGraph);
    label_loops_and_trees(loopGraph);
    compute_node_distances(*this, loopGraph);

#ifdef DEBUG_GRAPH
    std::cout << "DEBUG:AreaGraph: Final graph:\nNodes:\n";
    for (auto& n : nodes) {
        std::cout << '\t' << n->getPosition() << '\n';
    }
    std::cout << "Edges:\n";
    for (auto& e : edges) {
        std::cout << '\t' << e->getEndpoint(0)->getPosition() << "->" << e->getEndpoint(1)->getPosition() << '\n';
    }
#endif
}


AreaGraph::AreaGraph(const std::vector<AreaNodePtr>& nodes, const std::vector<AreaEdgePtr>& edges)
: nodes(nodes)
, edges(edges)
{
    LoopGraph loopGraph;
    construct_loop_graph(*this, loopGraph);
    compute_node_distances(*this, loopGraph);
}


std::vector<AreaNodePtr> AreaGraph::getNodes(char typeMask) const
{
    // Closure to check if a node matches the mask
    auto maskCheckF = [typeMask](const AreaNodePtr& node) {
        return node->getType() & typeMask;
    };

    // Create enough nodes by checking the count and then copy them over
    std::vector<AreaNodePtr> maskNodes(std::count_if(nodes.begin(), nodes.end(), maskCheckF));
    std::copy_if(nodes.begin(), nodes.end(), maskNodes.begin(), maskCheckF);

    return maskNodes;
}


bool AreaGraph::containsNode(const AreaNodePtr& node) const
{
    return std::find(nodes.begin(), nodes.end(), node) != nodes.end();
}


bool AreaGraph::containsEdge(const AreaEdgePtr& edge) const
{
    return std::find(edges.begin(), edges.end(), edge) != edges.end();
}


void AreaGraph::setNodeDistance(const AreaNode* nodeA, const AreaNode* nodeB, double distance)
{
    nodeDistances[nodeA][nodeB] = distance;
    nodeDistances[nodeB][nodeA] = distance;
}


double AreaGraph::distanceBetweenNodes(const AreaNode* from, const AreaNode* to) const
{
    auto distsIt = nodeDistances.find(from);

    if (distsIt != nodeDistances.end()) {
        auto nodeIt = distsIt->second.find(to);
        if (nodeIt != distsIt->second.end()) {
            return nodeIt->second;
        }
    }

    return 0.0;
}


utils::VisibilityGraph AreaGraph::toVisibilityGraph(double edgeNodeDensity, const VoronoiSkeletonGrid& skeleton) const
{
    // Extract the cells to be included in the visibility graph
    CellSet visibilityCells;

    // For each edge, distribute the cells evenly along the edge. There will always be at least two cells for the
    // endpoints plus however many are needed in the middle to get approximately the correctly density
    for (auto& e : edges) {
        assert(e->sizeCells() > 0);

        visibilityCells.insert(e->frontCell().cell);
        visibilityCells.insert(e->backCell().cell);
        int numInternalCells = e->getLength() / edgeNodeDensity;

        if (numInternalCells > 0) {
            // An edge of length edgeNode + epsilon will have three cells. The two ends plus one right in the middle
            // Thus the density is just the minimum density
            std::size_t stepSize = e->sizeCells() / (numInternalCells + 1);

            for (std::size_t n = stepSize; n < e->sizeCells(); n += stepSize) {
                visibilityCells.insert(e->at(n).cell);
            }
        }
    }

    // Return the constructed visibility graph
    return utils::VisibilityGraph(visibilityCells.begin(),
                                  visibilityCells.end(),
                                  20.0 * skeleton.cellsPerMeter(),
                                  skeleton,
                                  VoronoiSkeletonTerminationFunc());
}


utils::VisibilityGraph AreaGraph::toGraph(void) const
{
    std::vector<utils::VisGraphVertex> visNodes;

    for (auto& n : nodes) {
        visNodes.push_back(n->getCell());
    }

    std::vector<std::pair<int, int>> visEdges;

    for (auto& e : edges) {
        int zeroIdx = std::distance(nodes.begin(), std::find(nodes.begin(), nodes.end(), e->getEndpoint(0)));
        int oneIdx = std::distance(nodes.begin(), std::find(nodes.begin(), nodes.end(), e->getEndpoint(1)));

        visEdges.emplace_back(zeroIdx, oneIdx);
    }

    return utils::VisibilityGraph(visNodes, visEdges);
}


void AreaGraph::initializeNodes(const VoronoiSkeletonGrid& grid, const std::vector<Gateway>& gateways)
{
    for (auto& gateway : gateways) {
        createGatewayNode(gateway, grid);
    }

    for (auto cell : grid.getJunctionsPoints()) {
        createNodeForCell(cell, AreaNode::kJunction, grid);
    }

    for (auto cell : grid.getDeadEnds()) {
        createNodeForCell(cell, AreaNode::kDeadEnd, grid);
    }

    for (auto cell : grid.getExitPoints()) {
        createNodeForCell(cell, AreaNode::kFrontier, grid);
    }
}


void AreaGraph::createGatewayNode(const Gateway& gateway, const VoronoiSkeletonGrid& grid)
{
    // In addition to the official skeleton cell, the gateway may intersect one or more other edges of the skeleton.
    // Trace the gateway cells to see if this is the case. If so, create nodes for them as well
    CellVector gatewayCells(gateway.beginCells(), gateway.endCells());
    gatewayCells.push_back(gateway.skeletonCell());

    // Only want to add the skeleton cells as gateway nodes
    utils::erase_remove_if(gatewayCells, [&grid](cell_t c) {
        return ~grid.getClassification(c.x, c.y) & SKELETON_CELL_REDUCED_SKELETON;
    });

    // creating the node via the gateway form, so need to do the creation in here rather than createNodeForCell
    auto nodePtr = std::make_shared<AreaNode>(
      gateway,
      gateway.center());   // utils::grid_point_to_global_point(gateway.skeletonCell(), grid));
    nodes.push_back(nodePtr);

    for (auto cell : gatewayCells) {
        cellToNode[cell] = nodePtr;
        visited.insert(cell);   // add the node to the visited queue of cells
    }
}


void AreaGraph::createNodeForCell(cell_t cell, char type, const VoronoiSkeletonGrid& grid)
{
    // If a node already exists for this cell, then concatenate the types and create a new
    // node with the joint type

    if (cellToNode.find(cell) != cellToNode.end()) {
        cellToNode[cell]->type |= type;
        cellToNode[cell]->onBoundary |= is_cell_on_map_border(cell, grid);
    } else   // There was no node yet for this cell, so create one
    {
        auto nodePtr = std::make_shared<AreaNode>(utils::grid_point_to_global_point(cell, grid),
                                                  cell,
                                                  type,
                                                  is_cell_on_map_border(cell, grid));
        nodes.push_back(nodePtr);
        cellToNode[cell] = nodePtr;
        visited.insert(cell);   // add the node to the visited queue of cells
    }
}


void AreaGraph::createEdgeBetweenNeighbors(CellNodePair node, CellNodePair neighbor, const VoronoiSkeletonGrid& grid)
{
    auto endpoints = AreaEdge::EndpointArray{{node.second, neighbor.second}};
    std::vector<voronoi_cell_t> cells;
    cells.push_back({node.first,
                     utils::grid_point_to_global_point(node.first, grid),
                     grid.getMetricDistance(node.first.x, node.first.y),
                     false});

    cells.push_back({neighbor.first,
                     utils::grid_point_to_global_point(neighbor.first, grid),
                     grid.getMetricDistance(neighbor.first.x, neighbor.first.y),
                     false});

    addEdge(endpoints, cells, grid.metersPerCell(), node.second->isOnBoundary() | neighbor.second->isOnBoundary());
}


void AreaGraph::extractSkeletonEdge(AreaNodePtr startNode,
                                    cell_t nodeCell,
                                    cell_t cell,
                                    const VoronoiSkeletonGrid& grid)
{
    std::deque<cell_t> cellQueue;
    std::vector<voronoi_cell_t> edgeCells;
    bool isBorderEdge = startNode->isOnBoundary();
    NeighborArray neighbors;

    edgeCells.push_back({nodeCell,
                         utils::grid_point_to_global_point(nodeCell, grid),
                         grid.getMetricDistance(nodeCell.x, nodeCell.y),
                         is_frontier_skeleton(nodeCell, grid)});

    cellQueue.push_back(cell);

    while (!cellQueue.empty()) {
        int numAdded = 0;
        cell_t current = cellQueue.front();

        edgeCells.push_back({current,
                             utils::grid_point_to_global_point(current, grid),
                             grid.getMetricDistance(current.x, current.y),
                             is_frontier_skeleton(current, grid)});

        isBorderEdge |= is_cell_on_map_border(current, grid);

        std::size_t numNeighbors = neighbor_cells_with_classification(current,
                                                                      SKELETON_CELL_REDUCED_SKELETON,
                                                                      grid,
                                                                      FOUR_THEN_EIGHT_WAY,
                                                                      neighbors);

        for (std::size_t n = 0; n < numNeighbors; ++n) {
            cell_t& neighbor = neighbors[n];

            // Once another node has been reached, create the edge and add it to both of the nodes
            if ((cellToNode.find(neighbor) != cellToNode.end()) && (cellToNode[neighbor] != startNode)) {
#ifdef DEBUG_EDGE
                std::cout << "DEBUG: AreaGraph: Queue size when edge created: " << cellQueue.size() << '\n';
#endif
                auto otherNode = cellToNode[neighbor];
                isBorderEdge |= otherNode->isOnBoundary();
                auto endpoints = AreaEdge::EndpointArray{{startNode, otherNode}};
                edgeCells.push_back({neighbor,
                                     utils::grid_point_to_global_point(neighbor, grid),
                                     grid.getMetricDistance(neighbor.x, neighbor.y),
                                     is_frontier_skeleton(neighbor, grid)});
                addEdge(endpoints, edgeCells, grid.metersPerCell(), isBorderEdge);
                break;
            }
            // There should only be two neighbors and one should be visited because it was the parent. If that's
            // not the case, then display an appropriate warning
            else if (visited.find(neighbor) == visited.end()) {
                cellQueue.push_back(neighbor);
                ++numAdded;
            }

            if (numAdded > 1) {
                std::cerr << "WARNING:AreaGraph:Edge had more than one neighbor added, but wasn't a junction!\n";
            }
        }

        visited.insert(current);
        cellQueue.pop_front();
    }
}


void AreaGraph::addEdge(const AreaEdge::EndpointArray& endpoints,
                        const std::vector<voronoi_cell_t>& cells,
                        float scale,
                        bool borderEdge)
{
    // Don't add any self-edges
    if (endpoints[0] != endpoints[1]) {
        auto edgePtr = std::make_shared<AreaEdge>(endpoints, cells, scale, borderEdge);
        endpoints[0]->addEdge(edgePtr);
        endpoints[1]->addEdge(edgePtr);
        edges.push_back(edgePtr);

#ifdef DEBUG_EDGE
        std::cout << "DEBUG: AreaGraph: Adding edge between " << endpoints[0]->getCell() << " -> "
                  << endpoints[1]->getCell() << " :: ";
        for (auto& c : cells) {
            std::cout << c.cell << ' ';
        }
        std::cout << '\n';
#endif
    }
}


bool is_frontier_skeleton(cell_t cell, const VoronoiSkeletonGrid& grid)
{
    return std::find_if(grid.beginSourceCells(cell),
                        grid.endSourceCells(cell),
                        [&grid](cell_t source) {
                            return grid.getClassification(source.x, source.y) & SKELETON_CELL_FRONTIER;
                        })
      != grid.endSourceCells(cell);
}

}   // namespace hssh
}   // namespace vulcan
