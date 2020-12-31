/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_graph.h
 * \author   Collin Johnson
 *
 * Definition of HypothesisGraph.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_HYPOTHESIS_GRAPH_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_HYPOTHESIS_GRAPH_H

#include "hssh/local_topological/area_detection/labeling/type_distribution.h"
#include "utils/ptr.h"
#include "utils/visibility_graph.h"
#include <map>
#include <set>
#include <vector>

namespace vulcan
{
namespace utils
{
class VisibilityGraph;
}
namespace hssh
{

enum class BoundaryMergeCondition : unsigned char;
enum class HypothesisType : unsigned char;

class AreaGraph;
class AreaNode;
class AreaEdge;
class AreaHypothesis;
class AreaHypothesisBoundary;
class AreaProposal;
class SmallScaleStarBuilder;
class VoronoiIsovistField;
class VoronoiSkeletonGrid;

/**
 * HypothesisGraph
 */
class HypothesisGraph
{
public:
    using HypothesisIter = utils::SmartToRawIter<AreaHypothesis>;             // dereference type: AreaHypothesis*
    using HypothesisConstIter = utils::SmartToRawConstIter<AreaHypothesis>;   // dereference type: const AreaHypothesis*
    using BoundaryConstIter =
      utils::SmartToRawConstIter<AreaHypothesisBoundary>;   // dereference type: const AreaHypotheisBoundary*
    using BoundaryIter = utils::SmartToRawIter<AreaHypothesisBoundary>;   // dereference type: AreaHypotheisBoundary*
    using HypVec = std::vector<AreaHypothesis*>;

    /**
     * Constructor for HypothesisGraph.
     *
     * \param    areaGraph       AreaGraph containing the abstracted skeleton
     */
    HypothesisGraph(AreaGraph& areaGraph,
                    const VoronoiSkeletonGrid* grid,
                    const VoronoiIsovistField* isovistField,
                    std::shared_ptr<SmallScaleStarBuilder> starBuilder);

    /**
     * Destructor for HypothesisGraph.
     */
    ~HypothesisGraph(void);

    // Iterators across the hypotheses
    HypothesisIter beginHypothesis(void);
    HypothesisIter endHypothesis(void);

    HypothesisConstIter beginHypothesis(void) const;
    HypothesisConstIter endHypothesis(void) const;
    const AreaHypothesis* at(int id) const;
    std::size_t numHypotheses(void) const { return hypotheses.size(); }

    BoundaryConstIter beginBoundary(void) const;
    BoundaryConstIter endBoundary(void) const;
    std::size_t numBoundaries(void) const { return boundaries.size(); }

    BoundaryIter beginBoundary(void);
    BoundaryIter endBoundary(void);

    // Manipulators for the graph
    /**
     * simplify searches through the graph for all mergeable regions using the specified condition. The number of
     * regions merged is returned.
     *
     * NOTE: simplify calls mergeRegion if any regions are found. Therefore, if the number of merged regions returned is
     * not 0, then all post-conditions from mergeRegion apply.
     *
     * \param    condition           Condition to use for determining the regions to merge
     * \return   The number of regions merged.
     */
    int simplify(BoundaryMergeCondition condition);

    /**
     * findMergeableRegion searches through the graph for all reachable mergeable boundaries starting from
     * the provided hypothesis and only traversing mergeable boundaries/edges.
     *
     * \param    start           Start hypothesis for the mergeable region
     * \return   Collection of mergeable boundaries reachable from start.
     */
    std::vector<const AreaHypothesisBoundary*> findMergeableRegion(const AreaHypothesis* start) const;

    /**
     * mergeRegion merges all mergeable boundaries from the starting hypothesis. The mergeable boundaries
     * are those found by findMergeableRegion, but they are then merged into a single hypothesis.
     *
     * findMergeableRegion can be used for exploring the hypotheses that would be created by a merge without
     * actually modifying the graph. mergeRegion permanently changes the graph.
     *
     * NOTE: mergeRegion invalidates ALL existing handles to the graph's resources, which include all
     *       AmbiguousRegions and all iterators. Those resources should not be used across calls to mergeRegion.
     *
     * The safest way to merge all regions in a graph is to iterate through the hypotheses and merge whenever a
     * mergeable hypothesis is found. After merging this hypotheses, reset the iterator to beginHypothesis() and keep
     * doing this until no more mergeable hypotheses are found.
     *
     * The type of the merged region is start->getType() and the distribution is start->getTypeDistribution().
     *
     * \param    start           Start hypothesis for the mergeable region
     * \return   The merged hypothesis that now exists in the graph. nullptr if the merge failed.
     */
    AreaHypothesis* mergeRegion(const AreaHypothesis* start);

    /**
     * mergeHypotheses attempts to merge the provided hypotheses into a single hypothesis. The merge will be successful
     * if the provided hypotheses form a connected subgraph via their boundaries. If any hypothesis doesn't, then the
     * merge will fail.
     *
     * The merged hypothesis will be assigned the same type as the first hypothesis in toMerge,
     * toMerge.front()->getType(). The distribution will be assigned the same as the first type.
     *
     * \param    toMerge         Hypotheses to be merged
     * \param    type            Type to assign to the areas
     * \return   The merged hypothesis that now exists in the graph. nullptr if the merge failed. If toMerge is empty,
     *   then nullptr. If toMerge.size() == 1, then the single AreaHypothesis* is returned back.
     */
    AreaHypothesis* mergeHypotheses(const std::vector<AreaHypothesis*>& toMerge, HypothesisType type);

    /**
     * createProposalsForHypotheses creates an AreaProposal for each AreaHypothesis that exists in the graph.
     *
     * \param    grid            Grid to use for creating the AreaProposal via the toAreaProposal method in
     * AreaHypothesis
     */
    std::vector<AreaProposal> createProposalsForCurrentHypotheses(const VoronoiSkeletonGrid& grid) const;

    /**
     * findSubgraphs finds all connected subgraphs of the HypothesisGraph when a single area is removed. The subgraphs
     * will be the connected components of the full graph. The subgraphs are useful for determining which areas to fix
     * during the labeling process and which areas to allow to change. Any areas within the connected components of the
     * robot's current area should be changed. Areas in other connected components should be fixed in place.
     *
     * \param    removed         Area removed from the graph to generate the connected components
     * \return   The connected subgraphs of the full graph with the removed vertex removed.
     */
    std::vector<HypVec> findSubgraphs(AreaHypothesis* removed);

private:
    using HypothesisPair = std::pair<AreaHypothesis*, AreaHypothesis*>;

    std::vector<std::unique_ptr<AreaHypothesis>> hypotheses;
    std::vector<std::unique_ptr<AreaHypothesisBoundary>> boundaries;
    std::map<AreaHypothesis*, std::vector<AreaHypothesisBoundary*>>
      hypToBoundaries;   // keep track of edges/boundaries associated with a hypothesis

    std::map<AreaNode*, HypothesisPair>
      nodeToHypotheses;                 // temporary storage for efficiently constructing the boundaries
    std::set<AreaEdge*> visitedEdges;   // temporary storage for creating the area hypotheses

    const VoronoiSkeletonGrid* grid_;
    const VoronoiIsovistField* isovistField_;
    utils::VisibilityGraph visGraph_;
    utils::VisibilityGraph skelGraph_;
    std::shared_ptr<SmallScaleStarBuilder> starBuilder;
    int nextId_;

    void buildHypotheses(AreaGraph& graph);
    void extractHypothesisFromEdge(AreaEdge* edge, AreaGraph& graph);
    void addHypothesisToNode(AreaNode* node, AreaHypothesis* hypothesis);
    void createBoundaries(void);

    std::vector<AreaHypothesisBoundary*> findMergeableRegion(const AreaHypothesisBoundary* start,
                                                             BoundaryMergeCondition condition) const;
    AreaHypothesis* mergeRegion(const std::vector<AreaHypothesisBoundary*>& boundariesToMerge,
                                HypothesisType type,
                                HypothesisTypeDistribution distribution);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREAS_HYPOTHESIS_GRAPH_H
