/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_graph.h
* \author   Collin Johnson
*
* Declaration of AreaGraph and helper classes AreaNode and AreaEdge.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_GRAPH_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_GRAPH_H

#include "hssh/local_topological/gateway.h"
#include "hssh/types.h"
#include "core/point.h"
#include "utils/visibility_graph.h"
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <array>
#include <cassert>
#include <map>
#include <memory>
#include <set>

namespace vulcan
{
namespace hssh
{

class VoronoiSkeletonGrid;
class AreaNode;
class AreaEdge;

using AreaNodePtr = std::shared_ptr<AreaNode>;
using AreaEdgePtr = std::shared_ptr<AreaEdge>;

/**
* voronoi_cell_t describes a cell in the Voronoi skeleton -- the cell position in the grid and its distance from the nearest wall.
*/
struct voronoi_cell_t
{
    cell_t cell;                    ///< Position in the grid cells for this bit of the Voronoi skeleton
    Point<double> position;   ///< Position in metric coordinates
    float  distance;                ///< The distance to the closest wall  -- distance*2 = width of space at this point
    bool   isFrontier;              ///< Flag indicating if at least one of the cell's anchors is a frontier
};

/**
* AreaNode is a node in the AreaGraph. A node in the HSSH version of the Area graph occurs at either
* a junction point or a gateway. Creating nodes at gateways makes partitioning the graph into areas easier.
* The node contains pointers to the incident edges.
*/
class AreaNode
{
public:

    static const uint8_t kJunction = 0x01;
    static const uint8_t kGateway  = 0x02;
    static const uint8_t kFrontier = 0x04;
    static const uint8_t kDeadEnd  = 0x08;
    static const uint8_t kAll      = 0xFF;

    using EdgeIter = std::vector<AreaEdgePtr>::const_iterator;

    /**
    * Constructor for AreaNode.
    *
    * Creates a node where (typeMask & GATEWAY_NODE) == 0. If that condition isn't true, the program will crash itself.
    *
    * \param    position            Position of the node in the current map frame of reference
    * \param    cell                Cell in the grid associated with the position
    * \param    typeMask            Mask indicating the type of node
    * \param    onBoundary          Flag indicating if the node exists on the boundary/edge of the map (optional, default = false)
    */
    AreaNode(const Point<float>& position, cell_t cell, char typeMask, bool onBoundary = false);

    /**
    * Constructor for AreaNode.
    *
    * Creates a GATEWAY_NODE.
    *
    * \param    gateway         Gateway associated with the node
    * \param    position        Position of this node along the gateway
    */
    AreaNode(const Gateway& gateway, Point<float> position);

    // Iterators for the edges
    EdgeIter    beginEdge(void) const { return edges.begin(); }
    EdgeIter    endEdge(void)   const { return edges.end(); }
    std::size_t numEdges(void)  const { return edges.size(); }

    /**
    * getPosition retrieves the position of the node. The position is in the GLOBAL LPM reference frame.
    */
    Point<float> getPosition(void) const { return position; }

    /**
    * getCell retrieves the cell associated with this node.
    */
    cell_t getCell(void) const { return cell; }

    /**
    * getType retrieves the mask for the type of node. The type is a bitmask, so it can be both a junction and a gateway, for example.
    */
    char getType(void) const { return type; }

    /**
    * getGateway retrieves the gateway associated with a GATEWAY_NODE. If this isn't a gateway node, the
    * program will assert false.
    *
    * The gateway coordinates are in the GLOBAL LPM reference frame.
    */
    const Gateway& getGateway(void) const { assert(type & kGateway); return gateway; }

    /**
    * isOnBoundary retrieves the flag indicating if this node exists on the boundary of the current LPM.
    */
    bool isOnBoundary(void) const { return onBoundary; }

    /**
    * addEdge adds a new edge to the node connecting it to some other node in the graph.
    */
    void addEdge(AreaEdgePtr edge);

    /**
    * setLoop sets whether or not the node is on a loop in the environment.
    */
    void setLoop(bool loop) { onLoop = loop; }

    /**
    * setLoopDistance sets the distance of the loop going through this node.
    *
    * The loop distance is always the maximum of all loop distances for a given node.
    */
    void setLoopDistance(double dist) { loopDist = std::min(dist, loopDist); }

    /**
    * isLoop checks if the node is part of a loop in the environment.
    */
    bool isLoop(void) const { return onLoop; }

    /**
    * getLoopDistance retrieves the distance of the longest loop traveling through this node.
    */
    double getLoopDistance(void) const { return loopDist; }

    /**
    * setProbability sets the probability of the AreaNode.
    */
    void setProbability(double prob) { probability = prob; }

    /**
    * getProbability retrieves the associated probability.
    */
    double getProbability(void) const { return probability; }

private:

    friend class AreaGraph;

    Point<float>       position;
    cell_t                   cell;
    uint8_t                  type;
    double                   probability = 1.0;
    bool                     onBoundary;
    bool                     onLoop = false;
    double                   loopDist = 1e10;
    Gateway                  gateway;
    std::vector<AreaEdgePtr> edges;

    // Serialization support
    friend class ::cereal::access;

    AreaNode(void) { } // private default constructor to aid construction

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(position, cell, type, probability, onBoundary, onLoop, gateway, edges);
    }
};

// Operator overloads for AreaNode
/**
* Two AreaNodes are considered equal if they have the same position.
*/
inline bool operator==(const AreaNode& lhs, const AreaNode& rhs)
{
    return lhs.getPosition() == rhs.getPosition();
}

/**
* AreaEdge is an edge in the AreaGraph. The edge connects two AreaNodes. The edge describes the physical
* properties of the space through which it runs. A number of statistics can be queried like the min, max, and average
* distance from a wall along the edge. The cells from the skeleton associated with the edge can also be accessed if needed.
*/
class AreaEdge
{
public:

    using EndpointArray = std::array<AreaNodePtr, 2>;
    using VoronoiIter   = std::vector<voronoi_cell_t>::const_iterator;

    /**
    * Constructor for AreaEdge.
    *
    * \param    endpoints           Endpoints of the edge
    * \param    cells               Cells associated with the edge
    * \param    scale               Size of each cell (meters per cell)
    * \param    borderEdge          Flag indicating that the edge is on the border of the map
    *
    * \pre  for each cell[i], adjacent(cell[i-1], cell[i]) i.e. the cells are a connected sequence, as extracted from the skeleton
    * \pre  cells.size() > 1
    */
    AreaEdge(const EndpointArray& endpoints, const std::vector<voronoi_cell_t>& cells, float scale, bool borderEdge);

    /**
    * nodeIndex checks if one of the endpoints of the AreaEdge is the provided AreaNode.
    */
    int nodeIndex(const AreaNode* node) const;

    /**
    * getEndpoints retrieves the endpoints of the edge.
    */
    EndpointArray getEndpoints(void) const { return endpoints; }

    /**
    * getEndpoint retrieves the endpoint at the specified index -- only 0 or 1 are valid.
    */
    AreaNodePtr getEndpoint(int index) const { assert(index < 2); return endpoints[index]; }

    // Iterate through the Voronoi cells
    VoronoiIter beginCell(void) const { return cells.begin(); }
    VoronoiIter endCell(void) const { return cells.end(); }
    std::size_t sizeCells(void) const { return cells.size(); }
    voronoi_cell_t at(int index) const { return cells.at(index); }
    voronoi_cell_t frontCell(void) const { return cells.front(); }
    voronoi_cell_t backCell(void) const { return cells.back(); }

    /**
    * isBorderEdge checks if the edge hits the border of the current map.
    */
    bool isBorderEdge(void) const { return borderEdge; }

    /**
    * isFrontier checks to see if the edge has been classified as a frontier. A frontier edge
    * is an edge with >50% frontier skeleton cells.
    */
    bool isFrontier(void) const { return (frontierRatio > 0.5) | borderEdge; }

    /**
    * getFrontierRatio retrieves the ratio of frontierLength / totalLength. This ratio * the length is
    * the length of the frontier along the edge.
    */
    float getFrontierRatio(void) const { return frontierRatio; }

    /**
    * isLeftOfGateway checks to see if this edge is to the left of the provided gateway. An edge is considered
    * to the left of the gateway if the cell immediately adjacent to the gateway skeleton cell is to the left of
    * the gateway. If there are no skeleton cells, then the other endpoint of the edge is used.
    *
    * \param    gateway          Gateway to check for the isLeft condition
    * \return   1 if the edge is left of the gateway. -1 if the edge is right of the gateway. 0 if the edge has cells
    *   only on the gateway.
    *
    * \pre  The gatewayNode is one of the edge's endpoints.
    */
    int isLeftOfGateway(const AreaNode& gateway) const;

    /**
    * getType retrieves the type of the AreaEdge. An AreaEdge's type is the OR of the two endpoints' types.
    */
    char getType(void) const { return endpoints[0]->getType() | endpoints[1]->getType(); }

    // Various statistics about the edge -- all are metric, not cell coordinates
    double getLength(void)       const { return length; }
    double getAverageWidth(void) const { return averageWidth; }
    double getStdDevWidth(void)  const { return stdDevWidth; }
    double getMinWidth(void)     const { return minWidth; }
    double getMaxWidth(void)     const { return maxWidth; }

private:

    EndpointArray               endpoints;
    std::vector<voronoi_cell_t> cells;

    bool  borderEdge;
    float frontierRatio;

    double length;
    double averageWidth;
    double minWidth;
    double maxWidth;
    double stdDevWidth;

    // Serialization support
    friend class ::cereal::access;

    AreaEdge(void) { }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(endpoints[0],
           endpoints[1],
           cells,
           borderEdge,
           frontierRatio,
           length,
           averageWidth,
           minWidth,
           maxWidth,
           stdDevWidth);
    }
};

// Operator overloads for AreaEdge
/**
* Two edges are considered equal if they have equal endpoints.
*/
inline bool operator==(const AreaEdge& lhs, const AreaEdge& rhs)
{
    return (lhs.getEndpoints()[0] == rhs.getEndpoints()[0] && (lhs.getEndpoints()[1] == rhs.getEndpoints()[1])) ||
            (lhs.getEndpoints()[0] == rhs.getEndpoints()[1] && (lhs.getEndpoints()[1] == rhs.getEndpoints()[0]));
}

/**
* AreaGraph is an abstraction to a graph of nodes and edges of the Voronoi skeleton constructed via the brushfire algorithm.
* This graph consists of nodes located at the gateways and junction points. The edges are the portion of the
*/
class AreaGraph
{
public:

    /**
    * Default constructor for AreaGraph.
    *
    * Creates an empty graph.
    */
    AreaGraph(void) { }

    /**
    * Constructor for AreaGraph.
    *
    * \param    grid            Grid in which the skeleton exists
    * \param    gateways        Gateways found in the map
    */
    AreaGraph(const VoronoiSkeletonGrid& grid, const std::vector<Gateway>& gateways);

    /**
    * Constructor for AreaGraph.
    *
    * Creates an AreaGraph from an existing collection of nodes and edges.
    *
    * \param    nodes           Nodes for the graph
    * \param    edges           Edges that go in the graph
    */
    AreaGraph(const std::vector<AreaNodePtr>& nodes, const std::vector<AreaEdgePtr>& edges);

    /**
    * getNodes retrieves all nodes of the specified type.
    *
    * \param    typeMask        Type of nodes to get
    * \return   All nodes whose and-ing with the type mask is true will be returned
    */
    std::vector<AreaNodePtr> getNodes(char typeMask) const;

    /**
    * getAllNodes retrieves all nodes in the graph.
    */
    std::vector<AreaNodePtr> getAllNodes(void) const { return nodes; }

    /**
    * getAllEdges retrieves all the edges in the graph.
    */
    std::vector<AreaEdgePtr> getAllEdges(void) const { return edges; }

    /**
    * setNodeDistance sets the distance between two nodes.
    *
    * \param    nodeA       One node
    * \param    nodeB       The other node
    * \param    distance    Distance between the nodes
    */
    void setNodeDistance(const AreaNode* nodeA, const AreaNode* nodeB, double distance);

    /**
    * distanceBetweenNodes retrieves the compute distance between two nodes in the graph.
    *
    * \param    from        Starting node
    * \param    to          Node to reach
    * \return   Distance between these nodes.
    */
    double distanceBetweenNodes(const AreaNode* from, const AreaNode* to) const;

    /**
    * containsNode checks to see if the graph contains the node in question.
    *
    * \param    node            Node to check for existence in the graph
    * \return   True if the ndoe is in this graph.
    */
    bool containsNode(const AreaNodePtr& node) const;

    /**
    * containsEdge checks to see if the graph contains the edge in question.
    *
    * \param    edge            Edge to check for existence in the graph
    * \return   True if the edge is in the graph.
    */
    bool containsEdge(const AreaEdgePtr& edge) const;

    // Iteration support
    std::vector<AreaNodePtr>::const_iterator beginNodes(void) const { return nodes.begin(); }
    std::vector<AreaNodePtr>::const_iterator endNodes(void) const { return nodes.end(); }
    std::size_t sizeNodes(void) const { return nodes.size(); }

    std::vector<AreaEdgePtr>::const_iterator beginEdges(void) const { return edges.begin(); }
    std::vector<AreaEdgePtr>::const_iterator endEdges(void) const { return edges.end(); }
    std::size_t sizeEdges(void) const { return edges.size(); }

    /**
    * toVisibilityGraph converts the AreaGraph into a visibility graph. The visibility graph can be used to help
    * analyze and calculate appropriateness values for the area parsing algorithm.
    *
    * The visibility graph contains nodes for every node in the AreaGraph. Additional nodes are added to the
    * visibility graph by sampling positions along the AreaEdges. The density of this sampling is controlled by the
    * parameter edgeNodeDensity. Setting the density to a high number will reduce the visibility graph to a close
    * approximation of the AreaGraph itself.
    *
    * \param    edgeNodeDensity         Density of nodes to create along the edges (meters per node)
    * \param    skeleton                Skeleton grid in which the area graph exists
    * \return   A VoronoiVisibilityGraph constructed for the AreaGraph.
    */
    utils::VisibilityGraph toVisibilityGraph(double edgeNodeDensity, const VoronoiSkeletonGrid& skeleton) const;

    /**
    * toGraph converts the AreaGraph into a VisibilityGraph to allow for computation of centrality measures.
    */
    utils::VisibilityGraph toGraph(void) const;

private:

    using CellNodeMap = std::map<cell_t, AreaNodePtr>;
    using CellNodePair = CellNodeMap::value_type;
    using NodeDistMap = std::unordered_map<const AreaNode*, double>;

    std::vector<AreaNodePtr> nodes;
    std::vector<Point<int>> nodeCells;
    std::vector<AreaEdgePtr> edges;

    // Helper members used during initialization that are then cleared out after construction
    std::map<cell_t, AreaNodePtr> cellToNode;
    std::set<cell_t> visited;

    std::unordered_map<const AreaNode*, NodeDistMap> nodeDistances;

    void initializeNodes    (const VoronoiSkeletonGrid& grid, const std::vector<Gateway>& gateways);
    void createGatewayNode  (const Gateway& gateway, const VoronoiSkeletonGrid& grid);
    void createNodeForCell  (cell_t cell, char type, const VoronoiSkeletonGrid& grid);
    void createEdgeBetweenNeighbors(CellNodePair node, CellNodePair neighbor, const VoronoiSkeletonGrid& grid);
    void extractSkeletonEdge(AreaNodePtr startNode, cell_t nodeCell, cell_t cell, const VoronoiSkeletonGrid& grid);
    void addEdge(const AreaEdge::EndpointArray& endpoints,
                 const std::vector<voronoi_cell_t>& cells,
                 float scale,
                 bool borderEdge);

    // Serialization access
    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(nodes,
           edges);
    }
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, voronoi_cell_t& cell)
{
    ar(cell.cell,
       cell.distance,
       cell.isFrontier);
}

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_GRAPH_H
