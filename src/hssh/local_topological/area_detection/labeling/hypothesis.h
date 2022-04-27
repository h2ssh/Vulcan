/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis.h
 * \author   Collin Johnson
 *
 * Definition of AreaHypothesis.
 */

#ifndef HSSH_AREA_DETECTION_LABELING_HYPOTHESIS_H
#define HSSH_AREA_DETECTION_LABELING_HYPOTHESIS_H

#include "hssh/local_topological/area_detection/labeling/hypothesis_type.h"
#include "hssh/local_topological/area_detection/labeling/type_distribution.h"
#include "hssh/local_topological/small_scale_star.h"
#include "hssh/types.h"
#include "math/geometry/rectangle.h"
#include "utils/visibility_graph.h"
#include <array>
#include <memory>
#include <set>
#include <unordered_set>
#include <vector>

namespace vulcan
{
namespace hssh
{

class HypothesisFeatures;
class AreaExtent;
class Gateway;
class AreaGraph;
class AreaNode;
class AreaEdge;
class AreaProposal;
class DebugHypothesis;
class AreaHypothesisBoundary;
class VoronoiIsovistField;
class SmallScaleStar;
class SmallScaleStarBuilder;
class VoronoiSkeletonGrid;


using Endpoints = std::pair<Point<double>, Point<double>>;


/**
 * HypothesisContainment defines the possible levels of containment an AreaHypothesis can have with regards to another
 * AreaHypothesis. See AreaHypothesis::amountContained for full details.
 */
enum class HypothesisContainment
{
    none,      ///< No similar boundaries
    partial,   ///< Some boundaries are similar to nodes in the hypothesis
    complete   ///< All boundaries are similar to nodes in the hypothesis
};

/**
 * AreaHypothesis is a hypothesis of the full AreaGraph. The hypothesis contains a collection of nodes
 * and edges separated from the other parts of the graph by gateway nodes. Thus, the hypothesis represents
 * a potential area in the overall graph. These gateway nodes, represented as HypothesisBoundaries, provide
 * the link to other hypotheses. The adjacent areas important because they are used for classifying the
 * area.
 *
 * The toAreaProposal converts the nodes of the hypothesis into a proper AreaProposal representation.
 */
class AreaHypothesis
{
public:
    using Ptr = std::shared_ptr<AreaHypothesis>;
    using BoundaryIter = std::vector<const AreaHypothesisBoundary*>::const_iterator;
    using NodeIter = std::vector<AreaNode*>::const_iterator;
    using VisVertexIter = utils::VisVertexIter;

    /**
     * Constructor for AreaHypothesis.
     *
     * Create a new hypothesis from a set of nodes and edges extracted from the full AreaGraph.
     *
     * \param    id              Unique id assigned to the hypothesis
     * \param    nodes           Nodes the form the hypothesis
     * \param    edges           Edges in the hypothesis
     * \param    grid            Grid in which the hypothesis lives
     * \param    starBuilder     SmallScaleStarBuilder to use for calculating the local topology
     */
    AreaHypothesis(int id,
                   AreaGraph* areaGraph,
                   const std::set<AreaNode*>& nodes,
                   const std::set<AreaEdge*>& edges,
                   const VoronoiSkeletonGrid* grid,
                   const VoronoiIsovistField* isovistField,
                   const utils::VisibilityGraph* visGraph,
                   const utils::VisibilityGraph* skeletonGraph,
                   const SmallScaleStarBuilder* starBuilder);

    /**
     * Constructor for AreaHypothesis.
     *
     * Create a new hypothesis by merging the hypotheses on either side of all the provided boundaries.
     *
     * NOTE: This constructor does not modify the boundaries of the hypothesis. When iterating over the
     *       boundaries in the merge hypothesis, they will not contain a pointer to the merged hypothesis
     *       unless the boundaries were modified elsewhere to point to the merged hypothesis. This behavior
     *       allows for creating a temporary AreaHypothesis that is a composite of a number of AreaHypotheses
     *       that can be evaluated with the HypothesisClassifier but doesn't change the implicit graph
     *       of hypotheses and boundaries. This behavior is essential for evaluating the different assignments
     *       to hypotheses efficiently and correctly.
     *
     * NOTE: If a permanent alteration of the implicit graph is desired, then iterate over the AreaHypothesis'
     *       boundaries and call changeArea with this as the newHypothesis.
     *
     * \param    boundaries          Boundary whose hypotheses should be merged
     * \param    classification      Classification to be assigned to the newly created graph
     * \param    needExtent          The extent of an area is an expensive computation set a flag as to whether it is
     *                               needed or not. (optional, default = true)
     */
    AreaHypothesis(const std::vector<const AreaHypothesisBoundary*>& boundaries,
                   HypothesisType classification,
                   bool needExtent = true);

    /**
     * Constructor for AreaHypothesis.
     *
     * Create a new hypothesis by merging the provided hypotheses. It is assumed these areas are all adjacent.
     *
     * NOTE: This constructor does not modify the boundaries of the hypothesis. When iterating over the
     *       boundaries in the merge hypothesis, they will not contain a pointer to the merged hypothesis
     *       unless the boundaries were modified elsewhere to point to the merged hypothesis. This behavior
     *       allows for creating a temporary AreaHypothesis that is a composite of a number of AreaHypotheses
     *       that can be evaluated with the HypothesisClassifier but doesn't change the implicit graph
     *       of hypotheses and boundaries. This behavior is essential for evaluating the different assignments
     *       to hypotheses efficiently and correctly.
     *
     * NOTE: If a permanent alteration of the implicit graph is desired, then iterate over the AreaHypothesis'
     *       boundaries and call changeArea with this as the newHypothesis.
     *
     * \param    areas               Areas to be merged
     * \param    needExtent          The extent of an area is an expensive computation set a flag as to whether it is
     *                               needed or not. (optional, default = true)
     */
    AreaHypothesis(const std::vector<AreaHypothesis*>& areas, bool needExtent = true);


    virtual ~AreaHypothesis(void);

    // The type of the AreaHypothesis can be changed by assigning a different value via the csp::Variable base class

    // Observers
    /**
     * isSimilar checks for similarity between two hypotheses. Two hypotheses are considered to be similar if they share
     * one or more edges.
     *
     * \param    rhs         Hypothesis to be checked against this
     * \return   True if rhs and this have at least one edge in common.
     */
    bool isSimilar(const AreaHypothesis& rhs) const;

    /**
     * amountContained determines how much this hypothesis contains the input hypothesis. The containment is determined
     * by comparing the boundaries of rhs with gateway nodes in this hypothesis. If rhs.boundaries is a subset of the
     * gateway nodes in this hypothesis, the containment is complete. If only some of rhs.boundaries are gateway nodes,
     * the containment is partial. If none of rhs.boundaries are gateway nodes, there is no containment.
     *
     * \param    rhs         Hypothesis to check for containment
     * \return   Amount this hypothesis contains rhs.
     */
    HypothesisContainment amountContained(const AreaHypothesis& rhs) const;

    /**
     * length retrieves the estimated length of the area. It doesn't make a vast amount of sense for anything but paths.
     */
    double length(void) const { return length_; }

    /**
     * width retrieves the average width of the area.
     */
    double width(void) const { return width_; }

    /**
     * axisDirection retrieves the direction the axis points.
     */
    double axisDirection(void) const { return axisDirection_; }

    /**
     * center retrieves the center position of the area.
     */
    Point<float> center(void) const;

    /**
     * rectangleBoundary retrieves an approximation of the boundary represented by an axis-aligned bounding rectangle
     * of the true extent.
     */
    math::Rectangle<float> rectangleBoundary(void) const;

    /**
     * endpoints retrieves the endpoints calculated for the AreaHypothesis. If the AreaHypothesis is assigned to be a
     * path segment, these endpoints will define the constraints on destinations and areas.
     */
    Endpoints endpoints(void) const;

    /**
     * endNodes retrieves the end nodes for the AreaHypothesis. These are the AreaNodes corresponding to the endpoints.
     */
    const std::vector<const AreaNode*>& endNodes(void) const { return endNodes_; }

    /**
     * calculateStar calculates the small-scale star of the area using the specified mask for the matching hypothesis
     * types. If the Hypothesis doesn't have an associated SmallScaleStarBuilder, then an empty star will be returned.
     *
     * \param    typeMask        Mask for the assigned type
     * \return   SmallScaleStar of all gateways matching the typeMask.
     */
    SmallScaleStar calculateStar(HypothesisType typeMask) const;

    /**
     * pairwiseUnalignedGateways finds all gateways that are unaligned in the hypothesis. This condition for unaligned
     * differs from the condition used in constructing the full small-scale star. It simply considers if two gateways
     * are aligned without looking for ambiguous relationships. Thus, a single gateway can be aligned to two or more
     * other gateways.
     */
    std::vector<std::pair<const Gateway*, const Gateway*>> pairwiseUnalignedGateways(void) const;

    /**
     * pairwiseAlignedGateways finds all gateways that are aligned in the hypothesis. The condition for aligned
     * differs from the condition used in constructing the full small-scale star. It simply considers if two gateways
     * are aligned without looking for ambiguous relationships. Thus, a single gateway can be aligned to two or more
     * other gateways.
     */
    std::vector<std::pair<const Gateway*, const Gateway*>> pairwiseAlignedGateways(void) const;

    /**
     * numGatewaysAlignedToAxis checks if all gateways matching the type mask are aligned with the axis of skeleton
     * running through the area.
     *
     * Rather than building the star, this method finds the gateways with the type mask and uses the
     * areGatewaysAlignedToAxis method from the SmallScaleStarBuilder to determine if a single axis of motion exists
     * among the gateways.
     *
     * This method should be preferred for determining whether or not an area satisfies the path constraints. The reason
     * being that long paths are especially prone to being misclassified because the gateways have to be extremely
     * well-aligned on either end. The axis defines the best estimate of the angle of the path and thus is more stable
     * for long paths when determining the path status of an area.
     *
     * \param    typeMask            Type of gateways to consider for alignment
     * \return   Number of gateways aligned to the axis of the area.
     */
    int numGatewaysAlignedToAxis(HypothesisType typeMask) const;

    /**
     * numNonGatewayEnds retrieves the number of end nodes for the AreaHypothesis that aren't gateways. The method is
     * only meaningful for a hypothesis with type Path. The non-gateway end nodes will be dead ends or frontiers.
     */
    int numNonGatewayEnds(void) const;

    /**
     * numEndpointsWithType counds the number of endpoints with the specified hypothesis type.
     */
    int numEndpointsWithType(HypothesisType type) const;

    /**
     * isEndGateway checks if the specified gateway id is one of the endpoints of the path.
     */
    bool isEndGateway(int32_t gatewayId) const;

    /**
     * isValid checks if the hypothesis is valid. Invalid hypotheses have a single malformed edge. This edge will only
     * have cells along the edge.
     */
    bool isValid(void) const;

    /**
     * isFrontier checks if an area is a frontier.
     */
    bool isFrontier(void) const;

    /**
     * isBoundary checks if the area is on the boundary of the map. A boundary area contains cells on the border of the
     * map or has only one gateway shared with anything else on the map.
     */
    bool isBoundary(void) const;

    /**
     * countGatewaysWithMask determines the number of gateways that satisfy the provided mask.
     *
     * \param    typeMask        Mask to use for the gateway count
     * \return   Number of gateways whose type matches the mask.
     */
    std::size_t countGatewaysWithMask(HypothesisType typeMask) const;

    /**
     * findAdjacentHypotheses locates the adjacent hypotheses based on this hypothesis's boundaries.
     */
    std::vector<AreaHypothesis*> findAdjacentHypotheses(void) const;

    /**
     * adjacentArea finds the area adjacent to this area across the gateway.
     *
     * \param    gateway            Boundary between the areas
     * \return   The hypothesis on the other side of the gateway, if this gateway is part of the area. Otherwise, a
     *   nullptr is returned.
     */
    AreaHypothesis* adjacentArea(const Gateway& gateway);

    /**
     * Retrieve the AreaGraph of which this hypothesis is a part.
     */
    AreaGraph* areaGraph(void) const { return areaGraph_; }

    /**
     * hasMergeableBoundary checks if the hypothesis contains a mergeable boundary, which indicates that this hypothesis
     * could be merged into a larger structure.
     */
    bool hasMergeableBoundary(void) const;

    // Iterate over the boundaries in the area
    std::size_t numBoundaries(void) const { return boundaries_.size(); }
    BoundaryIter beginBoundary(void) const { return boundaries_.cbegin(); }
    BoundaryIter endBoundary(void) const { return boundaries_.cend(); }

    std::size_t numInternalBoundaries(void) const { return internalBoundaries_.size(); }
    BoundaryIter beginInternalBoundary(void) const { return internalBoundaries_.cbegin(); }
    BoundaryIter endInternalBoundary(void) const { return internalBoundaries_.cend(); }

    // Iterate over nodes in the area
    std::size_t numNodes(void) const { return nodes_.size(); }
    NodeIter beginNode(void) const { return nodes_.begin(); }
    NodeIter endNode(void) const { return nodes_.end(); }

    // Iterate over edges in the area


    // Iterate over the skeleton cells contained in the AreaHypothesis
    std::size_t numSkeleton(void) const { return cells_.size(); }
    CellConstIter beginSkeleton(void) const { return cells_.begin(); }
    CellConstIter endSkeleton(void) const { return cells_.end(); }

    // Iterate over the vertices contained in the visibility graph
    std::size_t numVisVertices(void) const { return visibilityVertices_.size(); }
    VisVertexIter beginVisVertices(void) const { return visibilityVertices_.begin(); }
    VisVertexIter endVisVertices(void) const { return visibilityVertices_.end(); }

    std::size_t numGraphVertices(void) const { return graphVertices_.size(); }
    VisVertexIter beginGraphVertices(void) const { return graphVertices_.begin(); }
    VisVertexIter endGraphVertices(void) const { return graphVertices_.end(); }

    /**
     * extent retrieves the extent of the hypothesis in the environment.
     */
    const AreaExtent& extent(void) const { return *extent_; }

    /**
     * features retrieves the features associated with the hypothesis.
     */
    const HypothesisFeatures& features(void) const;

    /**
     * getType retrieves the classification of the hypothesis. If the hypothesis has not been assigned a value, then it
     * will be the type that was assigned on construction, which is AREA by default, or via setType(type), but might be
     * different if constructed via the merge constructor.
     */
    HypothesisType getType(void) const;

    /**
     * getId retrieves the unique id assigned to this hypothesis.
     */
    int getId(void) const { return id_; }

    /**
     * setType sets the default type for the AreaHypothesis. This is the type returned when there has been no CSP
     * assignment made for the hypothesis. In the event that a CSP assignment exists, that is the type that will be
     * returned.
     */
    void setType(HypothesisType type) { type_ = type; }

    /**
     * setTypeDistribution sets the distribution across possible types for the area.
     *
     * \param    distribution            Appropriateness distribution across labels for this hypothesis
     */
    void setTypeDistribution(const HypothesisTypeDistribution& distribution) { distribution_ = distribution; }

    /**
     * setId sets a unique identifier for this area hypothesis.
     */
    void setId(int id) { id_ = id; }

    /**
     * findPathEndpoints forces a search for the endpoints for the hypothesis. These endpoints are computed on
     * construction, but changes in labels or probabilities may mean the optimal endpoints have changed as well.
     */
    void findPathEndpoints(void);

    /**
     * getTypeDistribution retrieves the type distribution associated with the AreaHypothesis.
     */
    HypothesisTypeDistribution getTypeDistribution(void) const { return distribution_; }

    /**
     * getGatewayType retrieves the type of a particular gateway associated with
     * this hypothesis. In most cases, this type will be the same as the overall type.
     * However, path segments have two classifications of gateways. The path segment has two distinct
     * endpoints, which are the PATH_ENDPOINTs. All other gateways are PATH_DESTs.
     *
     * \param    gateway         Gateway node for which to get the classification
     * \pre      gateway is an AreaNode::GATEWAY_NODE
     * \return   The type associated with the node.
     */
    HypothesisType getGatewayType(const AreaNode& gateway) const;

    // Producers
    /**
     * toAreaProposal converts the hypothesis representation into a full-fledged AreaProposal that can then
     * be used for tracking the areas.
     *
     * \return   AreaProposal representation of the the hypothesis.
     */
    AreaProposal toAreaProposal(void) const;

    /**
     * toDebug creates a debugging version of the AreaHypothesis. The debugging version is an immutable hypothesis that
     * maintains the type, approximate extent, and other relevant information.
     *
     * \param    distribution            Type distribution associated with the particular version of the hypothesis
     * \return   A simplified version of the debug graph to be used for visualization or logging purposes.
     */
    DebugHypothesis toDebug(const HypothesisTypeDistribution& distribution) const;

private:
    friend class HypothesisGraph;   // HypothesisGraph owns these pointers and can access their internal connections
                                    // to allow the public interface to be immutable

    using GatewayFunc = std::vector<Gateway> (AreaHypothesis::*)(HypothesisType) const;

    // Invariant: All values stored in the AreaHypothesis are in the GLOBAL reference frame.

    AreaGraph* areaGraph_;   // AreaGraph that this hypothesis is a subgraph of
    std::vector<AreaNode*> nodes_;
    std::unordered_set<AreaEdge*> edges_;
    std::vector<const AreaHypothesisBoundary*> boundaries_;
    std::vector<const AreaHypothesisBoundary*> internalBoundaries_;
    const VoronoiSkeletonGrid* grid_;
    const VoronoiIsovistField* isovistField_;
    const utils::VisibilityGraph* visGraph_;
    const utils::VisibilityGraph* skelGraph_;
    const SmallScaleStarBuilder* starBuilder_;
    std::unordered_map<const AreaNode*, double> maxPathDist_;

    // INVARIANT: The endNodes_[0] and endNodes_[1] are the selected end nodes. Any additional end nodes
    //  come from multiple gateways leading to the same boundary area.
    std::vector<const AreaNode*> endNodes_;   // For a path type, there must be nodes at each end
    mutable bool boundariesDirty_;            // Flag indicating a new boundary was added

    HypothesisTypeDistribution distribution_;   // Appropriatness distribution based on area features
    HypothesisType type_;                       // Current permanent type
    bool frontier_;   // Flag indicating if the majority of this hypothesis is associated with a frontier
    bool boundary_;   // Flag indicating the hypothesis is on the boundary of the map
    int id_;          // Unique identifier for this hypothesis -- to be applied to the AreaProposal

    CellVector cells_;
    float length_;
    float width_;   // mean width of all contained edges
    float axisRatio_;
    float axisDirection_;

    std::unique_ptr<AreaExtent> extent_;
    Point<double> center_;                                    // center of extent
    math::Polygon<double> polygonBoundary_;                   // boundary via the extent or hull of hulls
    math::Rectangle<double> rectangleBoundary_;               // rectangle boundary of the polygon boundary
    std::vector<utils::VisGraphVertex> visibilityVertices_;   // vertices for hyp contained in the visibility graph
    std::vector<utils::VisGraphVertex> graphVertices_;        // vertices for hyp contained in the graph
    mutable std::unique_ptr<HypothesisFeatures> features_;


    void initialize(void);

    // Mutators for use by HypothesisGraph
    /**
     * addBoundary adds a new AreaHypothesisBoundary to the representation of the hypothesis. Adding a boundary will
     * fail if the associated gateway node is not contained in the hypothesis.
     *
     * \param    boundary        Boundary to be added
     * \return   True if the boundary is added. False if no node exists in the hypothesis represented by the boundary.
     */
    bool addBoundary(const AreaHypothesisBoundary* boundary);

    // Helpers
    SmallScaleStar calculateStar(HypothesisType mask, GatewayFunc func) const;
    std::vector<Gateway> getGatewaysBasicType(HypothesisType type) const;   // calls getType()
    std::vector<Gateway> getGatewaysFullType(HypothesisType mask) const;    // calls getGatewayType()
    std::vector<Point<double>> getPoints(char mask) const;
    std::vector<const AreaNode*> getNodes(char mask) const;

    void extractSkeletonCells(void);
    void calculateAxis(void);
    void calculateFeaturesIfNeeded(void) const;

    void mergeEndpoints(const std::vector<AreaHypothesis*>& areas);
    void assignEndNodes(Endpoints endpoints);
    void addEndNode(AreaNode* node);
    bool isNonEndpointGateway(const AreaNode* node) const;
    bool isEndpoint(const AreaNode& node) const;

    std::vector<const AreaNode*> boundaryNodes(void) const;

    HypothesisContainment
      singleBoundaryContainment(const AreaNode* lhsNode, const AreaHypothesis& rhs, const AreaNode* rhsNode) const;
    const AreaNode* findConnectedNode(const AreaNode* node) const;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_AREA_DETECTION_LABELING_HYPOTHESIS_H
