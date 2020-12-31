/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     alignment_network.h
 * \author   Collin Johnson
 *
 * Declaration of AlignmentNetwork.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_NETWORK_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_NETWORK_H

#include "hssh/local_topological/area_detection/labeling/alignment_constraint.h"
#include "hssh/local_topological/area_detection/labeling/alignment_graph.h"
#include "hssh/local_topological/area_detection/labeling/csp_solution.h"
#include "hssh/local_topological/area_detection/labeling/type_distribution.h"
#include <boost/graph/adjacency_list.hpp>
#include <deque>
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class AreaHypothesis;
class AreaHypothesisEvaluator;
class CSPDebugInfo;
enum class HypothesisType : unsigned char;


/**
 * strain_weights_t defines the weights for the different terms used for solving the AlignmentNetwork.
 *
 * No negative weights are allowed and at least one weight must be greater than 0 so the search can progress.
 */
struct strain_weights_t
{
    double appropriatenessWeight;   ///< Weight in strain for not being max-appropriate type
    double constraintWeight;        ///< Weight in strain for breaking constraints
};


/**
 * AlignmentNetwork is a network of constraints between area hypotheses. The network consists of a set of areas and
 * constraints between them. The goal of the AlignmentNetwork is to find a solution to the constraint satisfaction
 * problem defined by the network that minimizes the overall strain in the network. The strain in the network is
 * determined by a combination of the number of currently failing constraints and the overall likelihood of the area
 * hypotheses in the network.
 *
 * The difficulty of solving the network is that it requires both finding labels for the areas and determining the
 * areas themselves. The input is areas bounded by gateway hypotheses. These hypotheses need to then be
 */
class AlignmentNetwork : public AlignmentGraph
{
public:
    /**
     * Constructor for AlignmentNetwork.
     *
     * \param    evaluator           Evaluator to use for determining appropriateness of hypotheses
     * \param    strainWeights       Weights to use for evaluating strain
     */
    AlignmentNetwork(const AreaHypothesisEvaluator& evaluator, const strain_weights_t& strainWeights);

    /**
     * solve solves the network and returns a CSPSolution.
     *
     * \param[out]   debug               Storage for generated debugging info (optional, default = no debug out)
     * \return   Solution to the CSP, if one exists.
     */
    CSPSolution solve(CSPDebugInfo* debug = nullptr);

    /**
     * activeId retrieves the id of the active area associated with the provided area id. As the network is solved, the
     * active id associated with an area via addArea will likely change as areas are merged and split during the solving
     * process.
     *
     * \param    areaId      Id of the area to find the active id for
     * \return   Active area id.
     */
    Id activeId(Id areaId) const { return original_[areaId].active; }

    // AlignmentGraph interface
    Id addArea(AreaHypothesis* area, HypothesisType type) override;
    Id addFixedArea(AreaHypothesis* area, HypothesisType type, bool isConnected) override;
    void addConstraint(const AlignmentConstraint& constraint) override;
    HypothesisType assignedType(int area) const override { return active(area).type; }
    Id assignedArea(Id area) const override { return active(area).id; }
    bool isPathEndGateway(int areaId, int32_t gatewayId, int otherAreaId) const override;

    // Various properties
    std::size_t numAreas(void) const { return original_.size(); }

private:
    using ConstraintVec = std::vector<AlignmentConstraint>;
    using Graph = boost::adjacency_list<boost::vecS,
                                        boost::vecS,
                                        boost::undirectedS,
                                        boost::no_property,
                                        boost::no_property,
                                        boost::no_property,
                                        boost::vecS>;
    using AdjVec = std::vector<ConstraintAdjacentArea>;
    using TypeVec = std::vector<HypothesisType>;

    struct OriginalArea
    {
        Id id;
        Id active;
        bool isFixed;           // flag indicating if this area is completely fixed and unmodifiable
        bool mustBeConnected;   // flag indicating if this area must be part of the connected graph of areas
        AdjVec adjacent;        // Areas adjacent to the area via a constraint
        AreaHypothesis* hypothesis;
        ConstraintVec constraints;
        TypeVec possibleTypes;
        HypothesisTypeDistribution distribution;
        IdVec endpoints;
        IdVec nonendpoints;

        OriginalArea(void) : active(-1), isFixed(false), mustBeConnected(true), hypothesis(nullptr) { }
    };

    struct ActiveArea
    {
        Id id;
        IdVec areas;       // The ids are sorted in ascending order
        AdjVec adjacent;   // Areas adjacent to the area via a constraint
                           // Original ids, not active! Use active(adj.id) to access appropriate value
        HypothesisType type;
        AreaHypothesis* hypothesis;
        ConstraintVec constraints;   // constraints created due to merging
        TypeVec possibleTypes;
        HypothesisTypeDistribution distribution;

        explicit ActiveArea(Id id = -1) : id(id), type(HypothesisType::kPathDest), hypothesis(nullptr) { }

        ActiveArea(const OriginalArea& orig, HypothesisType type)
        : id(orig.id)
        , areas({orig.id})
        , adjacent(orig.adjacent)
        , type(type)
        , hypothesis(orig.hypothesis)
        , constraints(orig.constraints)
        , possibleTypes(orig.possibleTypes)
        , distribution(orig.distribution)
        {
        }

        bool operator==(const ActiveArea& rhs) const { return areas == rhs.areas; }
    };

    struct NetworkChange
    {
        Id sourceId;                             // active area from which this change was created
        std::vector<ActiveArea> newAreas;        // size: 1 for label, 1 for merge, >1 for split
        std::vector<HypothesisType> newLabels;   // size: 1 = merge/relabel, >1 for split
        double strain;
        int changeType;   // 0 = label, 1 = merge, 2 = split

        bool operator<(const NetworkChange& rhs) const { return strain < rhs.strain; }
        bool operator==(const NetworkChange& rhs) const
        {
            return (sourceId == rhs.sourceId) && (changeType == rhs.changeType)
              && (newAreas == rhs.newAreas);   // unlikely exact same areas involved twice in a row
        }
        bool operator!=(const NetworkChange& rhs) const { return !(*this == rhs); }
    };

    // NetworkConfiguration holds the configuration of areas at the end of an update. The configuration is always saved
    // to ensure the search doesn't backtrack on itself.
    using IdTypePair = std::pair<Id, HypothesisType>;
    using NetworkConfiguration = std::vector<IdTypePair>;
    using ChangeVec = std::vector<NetworkChange>;

    enum class SplitMode
    {
        only_borders,
        all_areas,
    };


    std::vector<OriginalArea> original_;
    std::vector<ActiveArea> active_;   // contains original_.size() + 2 ActiveAreas (last two for hyp generation)
    std::vector<std::unique_ptr<AreaHypothesis>> mergedHypotheses_;
    std::vector<int> strainIdCache_;

    std::vector<NetworkConfiguration> configurations_;

    Graph graph_;                         // vertex descriptors in the graph correspond directly to the original ids.
    std::vector<int> activeComponent_;    // members of the currently active component
    std::vector<int> components_;         // storage for connected components
    std::vector<Id> disconnectedAreas_;   // storage for the areas that are disconnected

    std::vector<ChangeVec> changeCache_;
    std::vector<std::pair<IdVec, AreaHypothesis*>> mergedCache_;
    ConstraintVec constraintCache_;
    IdVec testAssignedActive_;
    std::size_t testSize_;   // number of areas in the test set

    const AreaHypothesisEvaluator* evaluator_;
    const strain_weights_t strainWeights_;

    void constructGraph(void);
    CSPSolution searchForSolution(CSPDebugInfo* debug);
    bool isActiveSolutionConsistent(void);
    void findInconsistentAreas(ConstraintVec& constraints);
    bool splitInconsistentMergedAreas(const ConstraintVec& constraints);
    void selectActiveComponent(const ConstraintVec& constraints);
    void generatePossibleChanges(ActiveArea& toChange, SplitMode splitMode, ChangeVec& changes);
    int selectChangeToApply(const ChangeVec& changes, const std::deque<Id>& evalSequence);
    bool isNewConfiguration(const NetworkChange& change);
    void invalidateCachedChanges(const NetworkChange& applied);
    void localSearchForBetterAreas(CSPDebugInfo* debug);
    bool searchForLocalChanges(ActiveArea& area);
    CSPSolution convertActiveToSolution(void);

    NetworkConfiguration createConfiguration(void) const;
    IdVec activeAreaIds(void) const;

    bool addLabelChange(ActiveArea& area, HypothesisType newLabel, ChangeVec& changes);
    bool addMergeChange(ActiveArea& area,
                        const std::vector<ActiveArea*>& toMerge,
                        HypothesisType newLabel,
                        ChangeVec& changes);
    bool addSplitChange(ActiveArea& area, Id splitId, HypothesisType newLabel, ChangeVec& changes);
    double calculateChangeStrain(NetworkChange& change);
    bool isValidPath(const ActiveArea& path, bool checkAllAreas) const;

    double networkStrain(void);
    double areaStrain(ActiveArea& area);

    void setActiveLabel(ActiveArea& area, HypothesisType label);

    void applyChange(NetworkChange& change);
    void applyMerge(ActiveArea&& mergedArea, Id mergedId);
    void applySplit(IdVec areasToSplit);
    void resetActiveToOriginal(Id area, HypothesisType type);

    ActiveArea createMergedArea(const IdVec& areas, HypothesisType type);
    AreaHypothesis* createMergedHypothesis(const IdVec& toMerge);
    ConstraintVec createMergedConstraints(const ActiveArea& area);
    void addUnalignedConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints);
    void addAdjacencyConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints);
    void addAllNeighborsConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints);
    void addFixedTypeConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints);
    ConstraintAdjacentArea adjacentFromGateway(const ActiveArea& area, int32_t gatewayId) const;

    void setActiveChange(NetworkChange& testChange);
    void clearActiveChange(void);

    double numFailedGlobalConstraints(ConstraintVec* failed = nullptr);
    void findDisconnectedAreas(void);

    int numFailedActiveConstraints(const ActiveArea& area, ConstraintVec* failed = nullptr);
    int numFailedConstraints(const std::vector<AlignmentConstraint>& constraints,
                             HypothesisType insideType,
                             ConstraintVec* failed);

    void saveAreaExtents(CSPDebugInfo* debug);
    void saveInconsistentAreas(const ConstraintVec& failed, CSPDebugInfo* debug);
    void saveChangedArea(Id changedId, HypothesisType oldType, CSPDebugInfo* debug);

    const ActiveArea& active(int origId) const { return active_[original_[origId].active]; }
    ActiveArea& active(int origId) { return active_[original_[origId].active]; }

    std::size_t sizeActive(void) const
    {
        assert(active_.size() >= testSize_);
        return active_.size() - testSize_;
    }
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_NETWORK_H
