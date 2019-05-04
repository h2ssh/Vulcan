/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mcmc_sampling.h
* \author   Collin Johnson
*
* Declaration of MCMCSampling.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_MCMC_SAMPLING_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_MCMC_SAMPLING_H

#include <hssh/local_topological/area_detection/labeling/alignment_constraint.h>
#include <hssh/local_topological/area_detection/labeling/alignment_graph.h>
#include <hssh/local_topological/area_detection/labeling/csp_solution.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis.h>
#include <hssh/local_topological/area_detection/labeling/type_distribution.h>
#include <hssh/local_topological/params.h>
#include <boost/graph/adjacency_list.hpp>
#include <deque>
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class AreaHypothesisBoundary;
class BoundaryClassifier;
struct CSPDebugInfo;

/**
* MCMCSampling is a network of constraints between area hypotheses. The network consists of a set of areas and
* constraints between them. The goal of the MCMCSampling is to find a solution to the constraint satisfaction
* problem defined by the network that minimizes the overall strain in the network. The strain in the network is
* determined by a combination of the number of currently failing constraints and the overall likelihood of the area
* hypotheses in the network.
*
* The difficulty of solving the network is that it requires both finding labels for the areas and determining the
* areas themselves. The input is areas bounded by gateway hypotheses. These hypotheses need to then be
*/
class MCMCSampling : public AlignmentGraph
{
public:

    /**
    * Constructor for MCMCSampling.
    *
    * \param    params              Parameters controlling the sampling algorithm
    * \param    bndClassifier       Boundary classifier for computing priors over the changed boundaries
    */
    MCMCSampling(const MCMCSamplingParams& params, const BoundaryClassifier* bndClassifier);

    /**
    * solve solves the network and returns a CSPSolution.
    *
    * \param[in]    doInitialMerge      Flag to indicate if the initial merge of areas should be performed
    * \param[out]   debug               Storage for generated debugging info (optional, default = no debug out)
    * \return   Solution to the CSP, if one exists.
    */
    CSPSolution solve(bool doInitialMerge, CSPDebugInfo* debug = nullptr);

    /**
    * activeId retrieves the id of the active area associated with the provided area id. As the network is solved, the
    * active id associated with an area via addArea will likely change as areas are merged and split during the solving
    * process.
    *
    * \param    areaId      Id of the area to find the active id for
    * \return   Active area id.
    */
    Id activeId(Id areaId) const { return original_[areaId].active; }

    // Various properties
    std::size_t numAreas(void) const { return original_.size(); }

    // AlignmentGraph interface
    Id addArea(AreaHypothesis* area, HypothesisType type) override;
    Id addFixedArea(AreaHypothesis* area, HypothesisType type, bool isConnected) override;
    void addConstraint(const AlignmentConstraint& constraint) override;
    HypothesisType assignedType(int area) const override { return active(area).type; }
    Id assignedArea(Id area) const override { return active(area).id; }
    bool isPathEndGateway(int areaId, int32_t gatewayId, int otherAreaId) const override;

private:

    using ConstraintVec = std::vector<AlignmentConstraint>;
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::no_property, boost::no_property, boost::vecS>;
    using AdjVec = std::vector<ConstraintAdjacentArea>;
    using TypeVec = std::vector<HypothesisType>;

    struct OriginalArea
    {
        Id id;
        Id active;
        bool isFixed;           // flag indicating if this area is completely fixed and unmodifiable
        bool mustBeConnected;   // flag indicating if this area must be part of the connected graph of areas
        AdjVec adjacent;        // Areas adjacent to the area via a constraint
        std::shared_ptr<AreaHypothesis> hypothesis;
        ConstraintVec constraints;
        TypeVec possibleTypes;
        HypothesisTypeDistribution distribution;
        IdVec endpoints;
        IdVec nonendpoints;

        OriginalArea(void)
        : active(-1)
        , isFixed(false)
        , mustBeConnected(true)
        , hypothesis(nullptr)
        {
        }

        bool operator==(const OriginalArea& rhs) const
        {
            return (rhs.id == id) && (rhs.active == active) && (rhs.hypothesis->getType() == hypothesis->getType());
        }
    };

    struct ActiveArea
    {
        Id id;
        IdVec areas;        // The ids are sorted in ascending order
        AdjVec adjacent;    // Areas adjacent to the area via a constraint
                            // Original ids, not active! Use active(adj.id) to access appropriate value
        HypothesisType type;
        std::shared_ptr<AreaHypothesis> hypothesis;
        bool isFixed = false;
        ConstraintVec constraints;       // constraints created due to merging
        TypeVec possibleTypes;

        double logProb = 0.0;

        explicit ActiveArea(Id id = -1)
        : id(id)
        , type(HypothesisType::kPathDest)
        , hypothesis(nullptr)
        {
        }

        ActiveArea(const OriginalArea& orig, HypothesisType type)
        : id(orig.id)
        , areas({orig.id})
        , adjacent(orig.adjacent)
        , type(type)
        , hypothesis(orig.hypothesis)
        , isFixed(orig.isFixed)
        , constraints(orig.constraints)
        , possibleTypes(orig.possibleTypes)
        {
        }

        bool operator==(const ActiveArea& rhs) const
        {
            return (type == rhs.type) && (areas == rhs.areas);
        }

        bool operator!=(const ActiveArea& rhs) const
        {
            return (type != rhs.type) || (areas != rhs.areas);
        }
    };

    struct NetworkChange
    {
        Id sourceId;    // active area from which this change was created
        std::vector<ActiveArea> newAreas; // size: 1 for label, 1 for merge, >1 for split
        std::vector<HypothesisType> newLabels;    // size: 1 = merge/relabel, >1 for split
        double logProb;        //
        int numFailing;
        int changeType; // 0 = label, 1 = merge, 2 = split, 3 = noop

        bool operator<(const NetworkChange& rhs) const { return logProb < rhs.logProb; }
        bool operator==(const NetworkChange& rhs) const
        {
            return (sourceId == rhs.sourceId)
                && (changeType == rhs.changeType)
                && (newAreas == rhs.newAreas);    // unlikely exact same areas involved twice in a row
        }
        bool operator!=(const NetworkChange& rhs) const { return !(*this == rhs); }
    };

    using ChangeVec = std::vector<NetworkChange>;

    // NetworkConfiguration holds the configuration of areas after applying a change.
    struct NetworkConfiguration
    {
        std::vector<OriginalArea> original;
        std::vector<ActiveArea> active;
        double logProb;
    };

    using ConfigVec = std::vector<NetworkConfiguration>;

    struct AreaSample
    {
        NetworkConfiguration config;
        ChangeVec changes;
        double probability;

        bool operator<(const AreaSample& rhs) { return probability < rhs.probability; }
    };

    enum class SplitMode
    {
        only_borders,
        all_areas,
    };

    std::vector<OriginalArea> original_;
    std::vector<ActiveArea> active_;        // contains original_.size() + 2 ActiveAreas (last two for hyp generation)
    std::vector<std::shared_ptr<AreaHypothesis>> mergedHypotheses_;
    std::vector<int> strainIdCache_;
    std::unordered_map<AreaHypothesis*, Id> hypToOrig_;

    double totalArea_ = 0.0;            // area of all original area extents

    std::vector<AreaSample> gibbsSamples_;  // Samples created for current iteration of Gibbs sampling

    Graph graph_;       // vertex descriptors in the graph correspond directly to the original ids.
    std::vector<int> activeComponent_;  // members of the currently active component
    std::vector<int> components_; // storage for connected components
    std::vector<Id> disconnectedAreas_; // storage for the areas that are disconnected

    ConstraintVec constraintCache_;
    std::vector<ChangeVec> changeCache_;
    std::vector<std::pair<IdVec, int>> mergedCache_;
    std::unordered_set<const AreaHypothesisBoundary*> boundaryCache_;
    std::unordered_set<int> dirtyAreaIds_;  // ids of all dirty active areas due to a change
    IdVec pathEndCache_;        // cache for path end check in isValidPath
    IdVec testAssignedActive_;
    std::size_t testSize_;          // number of areas in the test set

    NetworkConfiguration initialConfig_;
    NetworkConfiguration currentConfig_;
    ChangeVec appliedChanges_;      // all changes actually applied -- can convert into debug info

    ConfigVec iterConfigs_;    // configurations at the end of sampling iterations

    IdVec failingIdsCache_;     // stash ids for failing constraints here -- needs to be full for areaProb() to work
    IdVec failingActiveCache_;  // stash for dirty areas with failing active constraints

    const MCMCSamplingParams params_;
    double constraintPenalty;
    const BoundaryClassifier* bndClassifier_;

    void initializeProbabilities(void);
    void constructGraph(void);
    void mergeAdjacentSameTypeAreas(void);
    CSPSolution searchForSolution(CSPDebugInfo* debug);
    bool isActiveSolutionConsistent(void);
    AreaSample drawGibbsSample(const IdVec& activeIds, int failingId, bool stopWhenConsistent = true);
    int findInconsistentAreas(void);
    int generatePossibleChanges(ActiveArea toChange, SplitMode splitMode, ChangeVec& changes);
    int sampleAreaToChangeUniform(const IdVec& activeIds);
    int sampleAreaToChangeLogProb(const IdVec& activeIds);
    int sampleChangeToApplyUniform(const ChangeVec& changes);
    int sampleChangeToApplyLogProb(const ChangeVec& changes);
    int sampleGibbsToApply(void);
    bool isNewConfiguration(const NetworkChange& change);
    void invalidateCachedChanges(const NetworkChange& applied);
    int localSamplingForBetterAreas(void);
    int localSearchForBetterAreas(void);
    bool applyBestLocalChange(ChangeVec& changes);
    CSPSolution convertActiveToSolution(void);

    IdVec activeAreaIds(void) const;

    bool addNoOpChange(ActiveArea& area, ChangeVec& changes);
    bool addLabelChange(ActiveArea& area, HypothesisType newLabel, ChangeVec& changes);
    bool addMergeChange(ActiveArea& area,
                        const std::vector<ActiveArea*>& toMerge,
                        HypothesisType newLabel,
                        ChangeVec& changes);
    bool addSplitChange(ActiveArea& area, Id splitId, ChangeVec& changes);
    bool addGatewaySplitChange(ActiveArea& area, std::pair<Id, Id> splitIds, ChangeVec& changes);
    bool addHierarchyChanges(ActiveArea& area, ChangeVec& changes);
    double calculateChangeProb(NetworkChange& change);
    bool isValidPath(const ActiveArea& path, bool checkAllAreas);

    double networkProb(bool prediction = true); // flag indicates if predicting a change or wanting current assigned
    double areaProb(ActiveArea& area);

    void setActiveLabel(ActiveArea& area, HypothesisType label);

    void applyChange(const NetworkChange& change);
    void applyMerge(const ActiveArea& mergedArea, Id mergedId);
    void resetActiveToOriginal(Id area, HypothesisType type);

    ActiveArea createMergedArea(const IdVec& areas, HypothesisType type);
    std::shared_ptr<AreaHypothesis> createMergedHypothesis(const IdVec& toMerge);
    ConstraintVec createMergedConstraints(const ActiveArea& area);
    void addUnalignedConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints);
    void addAlignedConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints);
    void addAdjacencyConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints);
    void addAllNeighborsConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints);
    void addFixedTypeConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints);
    ConstraintAdjacentArea adjacentFromGateway(const ActiveArea& area, int32_t gatewayId) const;

    void setSampleToBestConsistent(AreaSample& sample);
    void applySample(const AreaSample& sample);
    void setConfiguration(const NetworkConfiguration& config);
    void pushChange(const NetworkChange& testChange);
    void popChange(void);

    ConfigVec::iterator findCurrentConfig(void); // use original_ and active_ to see if in previous iterConfigs_

    int numFailedGlobalConstraints(ConstraintVec* failed = nullptr);
    int numFailedActiveConstraints(const ActiveArea& area, ConstraintVec* failed = nullptr);
    void findDisconnectedAreas(void);

    void validateActiveAreas(void) const;
    void printSample(const AreaSample& sample) const;
    void printChange(const NetworkChange& change) const;
    void saveAreaExtents(CSPDebugInfo* debug);
    void saveIteration(const NetworkChange& change, CSPDebugInfo* debug);

    const ActiveArea& active(int origId) const { return active_[original_[origId].active]; }
    ActiveArea& active(int origId) { return active_[original_[origId].active]; }

    std::size_t sizeActive(void) const { return original_.size(); }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_MCMC_SAMPLING_H
