/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     alignment_network.cpp
* \author   Collin Johnson
*
* Definition of AlignmentNetwork.
*/

#include <hssh/local_topological/area_detection/labeling/alignment_network.h>
#include <hssh/local_topological/area_detection/labeling/alignment_network_filter.h>
#include <hssh/local_topological/area_detection/labeling/area_graph.h>
#include <hssh/local_topological/area_detection/labeling/csp_debug.h>
#include <hssh/local_topological/area_detection/labeling/evaluator.h>
#include <hssh/local_topological/area_detection/labeling/boundary.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis.h>
#include <hssh/local_topological/area_extent.h>
#include <utils/algorithm_ext.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/iterator_range.hpp>
#include <algorithm>
#include <ostream>
#include <random>
#include <cassert>

// #define DEBUG_UPDATES
// #define DEBUG_INITIAL_CONSTRAINTS
// #define DEBUG_FAILED_CONSTRAINTS
// #define DEBUG_CHANGES
// #define DEBUG_AREAS
// #define DEBUG_CONSTRAINTS
// #define DEBUG_STRAIN

namespace vulcan
{
namespace hssh
{

const int kChangeLabel = 0;
const int kChangeMerge = 1;
const int kChangeSplit = 2;

const std::size_t kMaxChangeSize = 200;  // can't merge more than this many areas in a single update


///////////////////// AlignmentNetwork implementation ///////////////////////////////////

AlignmentNetwork::AlignmentNetwork(const AreaHypothesisEvaluator& evaluator, const strain_weights_t& strainWeights)
: testSize_(0)
, evaluator_(&evaluator)
, strainWeights_(strainWeights)
{
}


AlignmentNetwork::Id AlignmentNetwork::addArea(AreaHypothesis* area, HypothesisType type)
{
    OriginalArea newArea;
    newArea.id = original_.size();
    newArea.active = original_.size();
    newArea.isFixed = false;
    newArea.mustBeConnected = true;
    newArea.hypothesis = area;
    newArea.distribution = evaluator_->calculateTypeDistribution(newArea.hypothesis->features());

    newArea.possibleTypes.reserve(3);
    newArea.possibleTypes.push_back(HypothesisType::kDest);
    newArea.possibleTypes.push_back(HypothesisType::kDecision);
    newArea.possibleTypes.push_back(HypothesisType::kPath);

    original_.push_back(newArea);
    active_.emplace_back(newArea, newArea.distribution.mostAppropriate());

    return newArea.active;
}


AlignmentNetwork::Id AlignmentNetwork::addFixedArea(AreaHypothesis* area, HypothesisType type, bool isConnected)
{
    OriginalArea newArea;
    newArea.id = original_.size();
    newArea.active = original_.size();
    newArea.isFixed = true;
    newArea.mustBeConnected = isConnected;
    newArea.hypothesis = area;
    newArea.distribution = evaluator_->calculateTypeDistribution(newArea.hypothesis->features());
    newArea.possibleTypes.push_back(type);

    original_.push_back(newArea);
    active_.emplace_back(newArea, type);

    return newArea.active;
}


void AlignmentNetwork::addConstraint(const AlignmentConstraint& constraint)
{
    OriginalArea& insideArea = original_[constraint.insideId()];
    assert(insideArea.active == constraint.insideId());

    insideArea.constraints.push_back(constraint);
    active(constraint.insideId()).constraints.push_back(constraint);

    // The inside area is adjacent to all outside areas in the constraint
    std::copy(constraint.beginAdjacent(), constraint.endAdjacent(), std::back_inserter(insideArea.adjacent));

    // Look at the adjacent gateways to see if they lead to an endpoint or nonendpoint. Used for the valid-path
    // check when creating new hypothesis areas
    for(auto& adj : boost::make_iterator_range(constraint.beginAdjacent(), constraint.endAdjacent()))
    {
        if(insideArea.hypothesis->isEndGateway(adj.gatewayId))
        {
            insideArea.endpoints.push_back(adj.id);
        }
        else
        {
            insideArea.nonendpoints.push_back(adj.id);
        }
    }

    // Ensure the constraint and the area type match and can be satisfied
    if(constraint.type() == AlignmentConstraint::fixed)
    {
        assert(constraint.fixedType() == active(constraint.insideId()).type);
    }

    // More than one gateway can connect two areas, so ensure only a single adjacency relation is saved
    std::sort(insideArea.adjacent.begin(), insideArea.adjacent.end());
    utils::erase_unique(insideArea.adjacent);

    std::sort(insideArea.endpoints.begin(), insideArea.endpoints.end());
    utils::erase_unique(insideArea.endpoints);

    std::sort(insideArea.nonendpoints.begin(), insideArea.nonendpoints.end());
    utils::erase_unique(insideArea.nonendpoints);

    // Add the adjacent areas to the associated active area
    active(constraint.insideId()).adjacent = insideArea.adjacent;

    if(insideArea.endpoints.size() > 2)
    {
        std::cerr << "ERROR: AlignmentNetwork: Somehow found more than two endpoints for an area: Id: "
            << insideArea.id << " Endpoints:\n";
        std::copy(insideArea.endpoints.begin(), insideArea.endpoints.end(), std::ostream_iterator<int>(std::cerr, ","));
        std::cout << '\n';
        assert(insideArea.endpoints.size() <= 2);
    }

#ifdef DEBUG_INITIAL_CONSTRAINTS
    std::cout << "Adding constraint:" << constraint << '\n';
#endif
}


CSPSolution AlignmentNetwork::solve(CSPDebugInfo* debug)
{
#ifdef DEBUG_AREAS
    std::cout << "DEBUG:AlignmentNetwork::solve: Solving network with the following areas: (id,boundary)\n";
    for(auto& area : active_)
    {
        std::cout << area.id << " : " << area.hypothesis->extent().rectangleBoundary(math::ReferenceFrame::GLOBAL)
            << '\n';
    }
#endif

    if(debug)
    {
        saveAreaExtents(debug);
    }

    constructGraph();

    // Toss some extra areas at the end of the active_ to make sure everything will fit in memory
    // Ensures that active_ is never reallocated, so pointers are always safe to use
    active_.reserve(active_.size() + kMaxChangeSize);
    changeCache_.resize(active_.capacity());

    // Finished with the labels.
    return searchForSolution(debug);
}


bool AlignmentNetwork::isPathEndGateway(int areaId, int32_t gatewayId, int otherAreaId) const
{
    const ActiveArea& area = active(areaId);

    // If the type is a path, check if this gateway is one of the endpoints
    return (area.type == HypothesisType::kPath) ? area.hypothesis->isEndGateway(gatewayId) : false;
}


void AlignmentNetwork::constructGraph(void)
{
    for(auto& orig : original_)
    {
        auto vertex = boost::add_vertex(graph_);
        assert(static_cast<int>(vertex) == orig.id);
    }

    for(auto& orig : original_)
    {
        for(auto& adj : orig.adjacent)
        {
            assert(orig.id < static_cast<int>(original_.size()));
            assert(adj.id < static_cast<int>(original_.size()));
            // Only add edges to those areas with a larger id, this keeps duplicate edges from being added
            if(adj.id > orig.id)
            {
                boost::add_edge(orig.id, adj.id, graph_);
            }
        }
    }
}


CSPSolution AlignmentNetwork::searchForSolution(CSPDebugInfo* debug)
{
    const int kMaxAttempts = 500;
    int numAttempts = 0;

    ConstraintVec failed;
    std::deque<Id> evalSequence;
    ChangeVec possibleChanges;
    IdVec areasToCheck;
    int failCount = 0;

    do
    {
        assert(original_.size() == sizeActive());   // confirm the graph is still valid

        // If the inconsistent areas don't change between runs, then a merged area needs to be split to increase
        // the search space
        if(failCount > 2)
        {
            evalSequence.clear();
            if(!splitInconsistentMergedAreas(failed))
            {
                // If there are no areas left to split and no changes were made on the previous update, then the
                // labeling algorithm has failed.
                std::cerr << "WARNING: MaxLikelihoodCSP::solve: Failed to find a solution to the problem. No areas "
                    << "left to split\n";
                break;
            }
        }

#ifdef DEBUG_UPDATES
        std::cout << "Starting iteration:" << numAttempts << '\n';
#endif

#ifdef DEBUG_CHANGES
        std::cout << "Active areas: (id,boundary)\n";
        for(auto& area : original_)
        {
            if(area.id == area.active)
            {
                auto& activeArea = active(area.id);
                std::cout << activeArea.id << " : "
                    << activeArea.hypothesis->extent().rectangleBoundary(math::ReferenceFrame::GLOBAL) << '\n';
            }
        }
#endif

        failed.clear();
        findInconsistentAreas(failed);

        if(debug)
        {
            saveInconsistentAreas(failed, debug);
        }

        // Break after saving the debugging info so the final solution is displayed when going to the end of the search
        // sequence in the DebugUI
        if(failed.empty())
        {
            break;
        }

#ifdef DEBUG_FAILED_CONSTRAINTS
        std::cout << "\nNumber of failed constraints:" << failed.size() << '\n';

        if(!failed.empty())
        {
            std::cout << "Failing constraints:\n";
            for(auto& c : failed)
            {
                std::cout << c << '\n';
            }
        }
#endif

        // Find all the active areas within the ambiguous region being changed
        areasToCheck.clear();
        for(auto& c : failed)
        {
            areasToCheck.push_back(active(c.insideId()).id);

            // If there was no solution on the last attempt, then try changing neighbors as well
            if(failCount > 0)
            {
                std::transform(c.beginIds(), c.endIds(), std::back_inserter(areasToCheck), [this](Id id) {
                    return active(id).id;
                });
            }
        }

        std::sort(areasToCheck.begin(), areasToCheck.end());
        utils::erase_unique(areasToCheck);

        // Erase any areas that aren't in the active component
        selectActiveComponent(failed);

        // Try some other component if we failed after including adjacent areas in the search
        if(failCount > 1)
        {
            utils::erase_remove_if(areasToCheck, [this](Id id) {
                return utils::contains(activeComponent_, id);
            });
        }
        // Clear out all areas not in the active component
        else
        {
            utils::erase_remove_if(areasToCheck, [this](Id id) {
                return !utils::contains(activeComponent_, id);
            });
        }

        possibleChanges.clear();

        // Create changes for each area with failing constraints
        for(auto& id : areasToCheck)
        {
            generatePossibleChanges(active_[id], SplitMode::only_borders, possibleChanges);
        }

        if(possibleChanges.empty())
        {
            std::cout << "WARNING: AlignmentNetwork: No valid changes found!\n";

            // If there are no changes and a fixed area has a failing constraint, then there's no way for the
            // graph to be solved
            for(auto& c : failed)
            {
                if(original_[c.insideId()].isFixed)
                {
                    std::cerr << "ERROR: AlignmentNetwork: Fixed area was causing network to fail.\n";
                    return CSPSolution(CSPSolution::fixed_area_failing_constraints);
                }
            }
        }

        std::sort(possibleChanges.begin(), possibleChanges.end());

#ifdef DEBUG_CHANGES
        for(auto& change : possibleChanges)
        {
            std::cout << "Change:" << change.sourceId << " to " << change.newLabels.front() << ": Strain: "
                << change.strain << " Type: ";
            switch(change.changeType)
            {
            case kChangeLabel:
                std::cout << " label\n";
                break;
            case kChangeMerge:
                std::cout << " merge\n";
                break;
            case kChangeSplit:
                std::cout << " split\n";
                break;
            }
        }
#endif

        int changeIndex = selectChangeToApply(possibleChanges, evalSequence);

        if(changeIndex != -1)
        {
            failCount = 0;
        }
        else
        {
            ++failCount;
        }

        if(failCount == 0)
        {
            auto& changeToApply = possibleChanges[changeIndex];

            Id changedId = changeToApply.sourceId;
            HypothesisType oldType = active_[changedId].type;

            invalidateCachedChanges(changeToApply);
            applyChange(changeToApply);
            configurations_.push_back(createConfiguration());
            evalSequence.push_back(changedId);

            if(evalSequence.size() > 30)
            {
                evalSequence.pop_front();
            }

            if(debug)
            {
                saveChangedArea(changedId, oldType, debug);
            }
        }

        ++numAttempts;
    } while((numAttempts < kMaxAttempts));

    if(isActiveSolutionConsistent())
    {
        std::cout << "INFO:AlignmentNetwork: Successfully found a consistent solution in " << numAttempts
            << " iterations. Beginning local search for better areas...\n";

        localSearchForBetterAreas(debug);

        // Add a last iteration so there's not a final change that makes the visualization look bad
        if(debug)
        {
            failed.clear();
            saveInconsistentAreas(failed, debug);
        }

        return convertActiveToSolution();
    }
    else if(numAttempts >= kMaxAttempts)
    {
        return CSPSolution(CSPSolution::too_many_attempts);
    }
    else
    {
        return CSPSolution(CSPSolution::no_solution_with_all_areas_split);
    }
}


bool AlignmentNetwork::isActiveSolutionConsistent(void)
{
    constraintCache_.clear();
    findInconsistentAreas(constraintCache_);
    return constraintCache_.empty();
}


void AlignmentNetwork::findInconsistentAreas(ConstraintVec& constraints)
{
    auto activeIds = activeAreaIds();
    // Search through every area in the active buffer
    for(auto id : activeIds)
    {
        // If there's an associated area and at least one constraint fails, then the overall solution isn't consistent
        numFailedActiveConstraints(active_[id], &constraints);
    }

    // Check for any areas disconnected from the main graph of decisions and paths
    numFailedGlobalConstraints(&constraints);
}


bool AlignmentNetwork::splitInconsistentMergedAreas(const ConstraintVec& constraints)
{
    IdVec toSplit;

    for(auto& c : constraints)
    {
        // Only add the areas if this area has yet to be added
        for(auto& origId : boost::make_iterator_range(c.beginIds(), c.endIds()))
        {
            if(!utils::contains(toSplit, origId))
            {
                boost::push_back(toSplit, boost::as_array(active(origId).areas));
            }
        }
    }

    // Clear the entire cache because many random changes will be applied
    for(auto& changes : changeCache_)
    {
        changes.clear();
    }

    // Any component should be changed if everything has been blown up
    activeComponent_.clear();

    applySplit(toSplit);
    return !toSplit.empty();
}


void AlignmentNetwork::selectActiveComponent(const ConstraintVec& constraints)
{
    // Get all areas associated with a failing constraint
    std::unordered_set<Id> failingSet;
    for(auto& c : constraints)
    {
        for(auto id : boost::make_iterator_range(c.beginIds(), c.endIds()))
        {
            failingSet.insert(active(id).areas.begin(), active(id).areas.end());
        }
    }

    // Find the connected components amongst parts of the graph with a failing constraint
    InSetEdge<Graph> edgeFilter(&failingSet, &graph_);
    InSetVertex<Graph> vertexFilter(&failingSet, &graph_);
    auto filtered = boost::make_filtered_graph(graph_, edgeFilter, vertexFilter);
    components_.resize(original_.size());
    std::fill(components_.begin(), components_.end(), -1);
    int numComponents = boost::connected_components(filtered, &components_[0]);

    int activeIndex = 0;    // by default, just use the first component

    // If there is more than one component, then find the one that most overlaps with the current active component
    if(numComponents > 1)
    {
        std::vector<int> overlapCounts(numComponents, 0);

        // If an area is contained within the active component, then increment the count for its component
        for(const auto& area : boost::make_iterator_range(boost::vertices(filtered)))
        {
            if(utils::contains(activeComponent_, area))
            {
                ++overlapCounts[components_[area]];
            }
        }

        activeIndex = std::distance(overlapCounts.begin(), std::max_element(overlapCounts.begin(),
                                                                            overlapCounts.end()));
    }

    activeComponent_.clear();

    for(int n = 0, end = components_.size(); n < end; ++n)
    {
        if(components_[n] == activeIndex)
        {
            activeComponent_.push_back(n);
        }
    }
}


void AlignmentNetwork::generatePossibleChanges(ActiveArea& toChange, SplitMode splitMode, ChangeVec& changes)
{
    // No changes are possible if the area contains a fixed constraint
    for(auto& origId : toChange.areas)
    {
        if(original_[origId].isFixed)
        {
            // Error if this area has ever been merged!
            assert(toChange.areas.size() == 1);
            return;
        }
    }

    // If this area has cached changes, then use them
    if(!changeCache_[toChange.id].empty())
    {
        // For each of the changes in the cache, recalculate the strain as the network is slightly different
        // than before
        for(auto& change : changeCache_[toChange.id])
        {
            change.strain = calculateChangeStrain(change);
            changes.push_back(change);
        }

        return;
    }

    // Otherwise need to do the full search
    std::vector<ActiveArea*> toMerge;
    std::vector<ActiveArea*> adjacentMerge(2);
    adjacentMerge[0] = &toChange;
    IdVec splitIds;

    if(splitMode == SplitMode::only_borders)
    {
        // One possible change is to split areas off that were merged from the edge of the area
        for(auto origId : toChange.areas)
        {
            // If an original area is adjacent to an adjacent area to the active area, then it must be on the outside
            // of the area and can possibly be split off, which would then let it be merged into some other area.
            for(auto adj : original_[origId].adjacent)
            {
                if(utils::contains(toChange.adjacent, adj))
                {
                    splitIds.push_back(origId);
                    break;
                }
            }
        }
    }
    else if(splitMode == SplitMode::all_areas)
    {
        splitIds = toChange.areas;
    }

    // Store the initial size of the change vec so all generated changes can be put into the cache
    std::size_t initialChangeSize = changes.size();

    // Generate both the option of merging all adjacent and of doing a step-by-step merge
    for(HypothesisType type : toChange.possibleTypes)
    {
        toMerge.clear();

        for(auto adj : toChange.adjacent)
        {
            // No fixed areas should be considered for possible changes
            if(original_[adj.id].isFixed)
            {
                continue;
            }

            ActiveArea& adjacent = active(adj.id);
            if(adjacent.type == type)
            {
                toMerge.push_back(&adjacent);
                adjacentMerge[1] = &adjacent;

                addMergeChange(toChange, adjacentMerge, type, changes);
            }
        }

        if(toChange.type != type && toMerge.empty())
        {
            addLabelChange(toChange, type, changes);
        }

        if(toMerge.size() > 1)  // the two-area merge is handled above
        {
            toMerge.push_back(&toChange);
            addMergeChange(toChange, toMerge, type, changes);
        }

        for(auto id : splitIds)
        {
            // Can split off an area into any other type of area. Don't split and keep the type the same though.
            if((active(id).areas.size() > 1) && (type != toChange.type))
            {
                addSplitChange(toChange, id, type, changes);
            }
        }
    }

    boost::push_back(changeCache_[toChange.id], boost::make_iterator_range(changes.begin() + initialChangeSize,
                                                                           changes.end()));
}


int AlignmentNetwork::selectChangeToApply(const ChangeVec& changes, const std::deque<Id>& evalSequence)
{
    std::set<Id> changed;
    std::transform(changes.begin(), changes.end(), std::inserter(changed, changed.end()), [](const NetworkChange& c) {
        return c.sourceId;
    });

    for(int i = 3; i <= 3; ++i)
    {
        const std::size_t kMinRepeat = 3 - i;
        for(std::size_t n = 0; n < changes.size(); ++n)
        {
            auto& change = changes[n];

            std::size_t lastChanged = std::distance(evalSequence.rbegin(),
                                                    std::find(evalSequence.rbegin(),
                                                              evalSequence.rend(),
                                                              change.sourceId));
            bool valid = !utils::contains(evalSequence, change.sourceId) ||
                (evalSequence.size() < kMinRepeat || (lastChanged >= kMinRepeat)
                || (changed.size() < kMinRepeat));
            valid &= isNewConfiguration(change);
    //         bool valid = isNewConfiguration(change);
            if(valid)
            {
                return n;
            }
#ifdef DEBUG_CHANGES
            else
            {
                bool inSequence = utils::contains(evalSequence, change.sourceId) && (evalSequence.size() >= kMinRepeat && (lastChanged < kMinRepeat)
                    && (changed.size() >= kMinRepeat));
                int numChanged = changed.size();
                bool isNew = isNewConfiguration(change);
                std::cout << "Ignoring change: Reason:\n"
                    << "In seq:" << inSequence << '\n'
                    << "Num changed:" << numChanged << '\n'
                    << "Is new:" << isNew << '\n';
            }
#endif
        }
    }

    // If we get to here, then no change takes us to a new configuration, so fail
    return -1;
}


bool AlignmentNetwork::isNewConfiguration(const AlignmentNetwork::NetworkChange& change)
{
    // Start from the current configuration of the network
    NetworkConfiguration config = createConfiguration();

    // Remove any areas in the current config that are part of the change
    for(auto& newArea : change.newAreas)
    {
        utils::erase_remove_if(config, [&newArea](IdTypePair area) {
            return utils::contains(newArea.areas, area.first);
        });
    }

    // Add each changed area
    for(auto& newArea : change.newAreas)
    {
        config.emplace_back(newArea.id, newArea.type);
    }

    // Use the increasing order of id sorting to allow direct comparison
    std::sort(config.begin(), config.end(), [](const IdTypePair& lhs, const IdTypePair& rhs) {
        return lhs.first < rhs.first;
    });

    // See if this updated configuration has existed before
    return !utils::contains(configurations_, config);
}


void AlignmentNetwork::invalidateCachedChanges(const NetworkChange& applied)
{
    // For each area and adjacent area in the new areas, clear out any changes because they will have different
    // neighbors now, so changes aren't the same as they were

    std::vector<Id> invalidIds;

    for(auto& area : applied.newAreas)
    {
        boost::push_back(invalidIds, boost::as_array(area.areas));
        for(auto& adj : area.adjacent)
        {
            invalidIds.push_back(adj.id);
        }
    }

    for(std::size_t n = 0; n < changeCache_.size(); ++n)
    {
        // Clear out if it is one of the changed areas
        if(utils::contains(invalidIds, n))
        {
            changeCache_[n].clear();
        }

        // Or clear out if one of the changes contains one of the bad ids
        bool shouldClear = false;
        for(auto& change : changeCache_[n])
        {
            for(auto& area : change.newAreas)
            {
                shouldClear |= utils::contains_if(area.areas, [&invalidIds](Id id) {
                    return utils::contains(invalidIds, id);
                });

                shouldClear |= utils::contains_if(area.adjacent, [&invalidIds](ConstraintAdjacentArea adj) {
                    return utils::contains(invalidIds, adj.id);
                });
            }

            if(shouldClear)
            {
                changeCache_[n].clear();
                break;
            }
        }
    }
}


void AlignmentNetwork::localSearchForBetterAreas(CSPDebugInfo* debug)
{
    // For each active area, if it has a better assignment that doesn't break any constraints, and yields a higher
    // overall appropriateness, then apply that assignment. Keep going for each area until the graph settles.
    bool madeChange = false;

    // Throw out all cached changes at the start because the options for the local search are different than for
    // the solution search. Thus the first iteration will be real slow, but the rest should be faster
    for(ChangeVec& changes : changeCache_)
    {
        changes.clear();
    }

    do
    {
        madeChange = false;
        auto allActiveIds = activeAreaIds();

        for(auto id : allActiveIds)
        {
            ActiveArea& active = active_[id];
            HypothesisType prevType = active.type;

            if(searchForLocalChanges(active))
            {
                // Restart the search from the beginning, as this change may cascade to other areas
                madeChange = true;

                if(debug)
                {
                    ConstraintVec failed;
                    saveInconsistentAreas(failed, debug);
                    saveChangedArea(id, prevType, debug);
                }

                // Jump out of this search and start over because the active areas will have changed
                break;
            }
        }
    } while(madeChange);
}


bool AlignmentNetwork::searchForLocalChanges(ActiveArea& area)
{
    int bestChangeIndex = -1;
    double lowestStrain = networkStrain();

    // Create all possible changes for this area.
    ChangeVec changes;
    generatePossibleChanges(area, SplitMode::all_areas, changes);

    for(std::size_t n = 0; n < changes.size(); ++n)
    {
        // Don't merge areas, as we don't want to eliminate existing places
//         if(changes[n].changeType == kChangeMerge)
//         {
//             continue;
//         }

        // If a change causes no constraints to fail and lowers the strain in the graph, then it should be
        // used instead of the current assignment
        setActiveChange(changes[n]);
        constraintCache_.clear();
        findInconsistentAreas(constraintCache_);

        if(constraintCache_.empty() && (changes[n].strain < lowestStrain))
        {
            bestChangeIndex = n;
            lowestStrain = changes[n].strain;
        }

        clearActiveChange();
    }

    if(bestChangeIndex >= 0)
    {
        invalidateCachedChanges(changes[bestChangeIndex]);
        applyChange(changes[bestChangeIndex]);
    }

    return bestChangeIndex >= 0;
}


CSPSolution AlignmentNetwork::convertActiveToSolution(void)
{
    std::vector<AreaLabel> labels;

    for(const auto& area : original_)
    {
        // If the original id matches the active id, then we'll call this the parent of the active area
        if(area.id == area.active)
        {
            std::vector<AreaHypothesis*> areaHyps;
            std::transform(active(area.id).areas.begin(), active(area.id).areas.end(), std::back_inserter(areaHyps),
                [this](Id id) { return original_[id].hypothesis; } );

            labels.emplace_back(std::move(areaHyps), active(area.id).type, active(area.id).distribution);
        }
    }

    return CSPSolution(labels, networkStrain());
}


AlignmentNetwork::NetworkConfiguration AlignmentNetwork::createConfiguration(void) const
{
    NetworkConfiguration config;
    IdVec activeIds = activeAreaIds();

    // Find the id->type of all active areas based on the original areas
    config.resize(activeIds.size());
    std::transform(activeIds.begin(), activeIds.end(), config.begin(), [this](Id id) {
        return std::make_pair(id, active_[id].type);
    });

    // Remove any duplicates
    std::sort(config.begin(), config.end(), [](const IdTypePair& lhs, const IdTypePair& rhs) {
        return lhs.first < rhs.first;
    });
    utils::erase_unique(config);

    return config;
}


AlignmentNetwork::IdVec AlignmentNetwork::activeAreaIds(void) const
{
    IdVec activeIds;
    activeIds.reserve(sizeActive());
    std::transform(original_.begin(),
                   original_.end(),
                   std::back_inserter(activeIds),
                   [](const OriginalArea& a) {
        return a.active;
    });

    std::sort(activeIds.begin(), activeIds.end());
    utils::erase_unique(activeIds);

    return activeIds;
}


bool AlignmentNetwork::addLabelChange(ActiveArea& area, HypothesisType newLabel, ChangeVec& changes)
{
#ifdef DEBUG_CHANGES
    std::cout << "***************************************\nLabel change:" << area.id << '\n';
#endif

    // Always check all areas for validity when changing an area to a path
//     if((newLabel == HypothesisType::kPath) && !isValidPath(area, true))
//     {
//         return false;
//     }

    NetworkChange change;
    change.sourceId = area.id;
    change.newAreas.push_back(area);
    change.newAreas.back().type = newLabel;
    change.newLabels.push_back(newLabel);
    change.changeType = kChangeLabel;
    change.strain = calculateChangeStrain(change);

#ifdef DEBUG_CHANGES
    std::cout << "Dist:" << area.distribution.path << ',' << area.distribution.destination << ',' << area.distribution.decision << '\n';
    std::cout << " New strain:" << change.strain << " Type:" << area.type << "->" << newLabel << '\n';
#endif

    changes.push_back(std::move(change));

    return true;
}


bool AlignmentNetwork::addMergeChange(ActiveArea& area,
                                      const std::vector<ActiveArea*>& toMerge,
                                      HypothesisType newLabel,
                                      ChangeVec& changes)
{
#ifdef DEBUG_CHANGES
    std::cout << "***************************************\nMerging ";
    for(auto area : toMerge)
    {
        std::cout << area->id << ' ';
    }
    std::cout << "to "<< newLabel << ": ";
#endif

    bool isMergingPath = newLabel == HypothesisType::kPath;

    IdVec idsToMerge;
    for(auto m : toMerge)
    {
        boost::push_back(idsToMerge, boost::as_array(m->areas));
        isMergingPath |= m->type == HypothesisType::kPath;
    }

    NetworkChange change;
    change.sourceId = area.id;
    change.changeType = kChangeMerge;
    change.newLabels.push_back(newLabel);
    change.newAreas.push_back(createMergedArea(idsToMerge, newLabel));

    // Only need to consider all areas if turning into a path, otherwise can just consider those areas that are
    // currently a path
//     bool invalidMerge = isMergingPath
//         && !isValidPath(change.newAreas.front(), newLabel == HypothesisType::kPath);
    bool invalidMerge = false;
#ifdef DEBUG_CHANGES
    std::cout << (invalidMerge ? "invalid" : "valid") << '\n';
#endif

    if(invalidMerge)
    {
        return false;
    }

    change.strain = calculateChangeStrain(change);

#ifdef DEBUG_CHANGES
    std::cout << "Distribution:" << change.newAreas.front().distribution.path << ','
        << change.newAreas.front().distribution.decision << ','
        << change.newAreas.front().distribution.destination << " Change:" << change.strain << '\n';
#endif

    changes.push_back(std::move(change));

    return true;
}


bool AlignmentNetwork::addSplitChange(ActiveArea& area, Id splitId, HypothesisType newLabel, ChangeVec& changes)
{
    IdVec splitMergeIds;
    splitMergeIds.push_back(splitId);

    for(auto adj : original_[splitId].adjacent)
    {
        ActiveArea& adjacent = active(adj.id);
        if((adjacent.type == newLabel) && !original_[adj.id].isFixed)
        {
            boost::push_back(splitMergeIds, boost::as_array(adjacent.areas));
        }
    }

    std::sort(splitMergeIds.begin(), splitMergeIds.end());
    utils::erase_unique(splitMergeIds);

#ifdef DEBUG_CHANGES
    std::cout << "***************************************\nSplitting " << splitId << " from ";
    for(auto id : area.areas)
    {
        std::cout << id << ' ';
    }
    std::cout << "and merging to ";
    for(auto id : splitMergeIds)
    {
        std::cout << id << ' ';
    }
    std::cout << ": ";
#endif

    ActiveArea splitMergeArea = createMergedArea(splitMergeIds, newLabel);

    // Only need to consider all areas if turning into a path, otherwise can just consider those areas that are
    // currently a path
//     bool invalidMerge = (newLabel == HypothesisType::kPath)
//         && !isValidPath(splitMergeArea, newLabel == HypothesisType::kPath);
    bool invalidMerge = false;

#ifdef DEBUG_CHANGES
    std::cout << (invalidMerge ? "invalid" : "valid") << '\n';
#endif

    if(invalidMerge)
    {
        return false;
    }

    NonSplitActiveEdge<Graph> edgeFilter(&area.areas, splitId, &graph_);
    ActiveVertex<Graph> vertexFilter(&area.areas, &graph_);
    boost::filtered_graph<Graph, NonSplitActiveEdge<Graph>, ActiveVertex<Graph>> filtered(graph_, edgeFilter, vertexFilter);
    components_.resize(original_.size());
    int numComponents = boost::connected_components(filtered, &components_[0]);

    assert(numComponents > 1);  // must be at least two components that are split off, as split isn't connected to any others

    std::unordered_map<int, IdVec> splitAreas;
    // Add each area in the graph to its connected components, which is the post-split area it belongs to
    for(Id area : boost::make_iterator_range(boost::vertices(filtered)))
    {
        // If the area is the split area, then add to the component all merge ids found above
        if(area == splitId)
        {
            boost::push_back(splitAreas[components_[area]], splitMergeIds);
        }
        else
        {
            splitAreas[components_[area]].push_back(area);
        }
    }

    NetworkChange change;
    change.sourceId = area.id;

    for(auto& split : splitAreas)
    {
        IdVec& ids = split.second;
        // If the ids contains the split id, then assign the new label.
        HypothesisType label = utils::contains(ids, splitId) ? newLabel : area.type;
        change.newLabels.push_back(label);
        change.newAreas.push_back(createMergedArea(ids, label));
    }

    change.changeType = kChangeSplit;
    change.strain = calculateChangeStrain(change);

    changes.push_back(std::move(change));

    // Splits are always allowed
    return true;
}


double AlignmentNetwork::calculateChangeStrain(NetworkChange& change)
{
    double strain = 0.0;

    // Label changes don't need the full change applied, just a quick label change
    if(change.changeType == kChangeLabel)
    {
        Id areaId = change.newAreas.front().id;
        assert(areaId == change.sourceId);
        auto oldLabel = active_[areaId].type;
        setActiveLabel(active_[areaId], change.newLabels.front());
        strain = networkStrain();
        setActiveLabel(active_[areaId], oldLabel);
    }
    // Otherwise need to apply the full change to calculate the strain
    else
    {
        setActiveChange(change);
        strain = networkStrain();
        clearActiveChange();
    }

    return strain;
}


bool AlignmentNetwork::isValidPath(const ActiveArea& path, bool checkAllAreas) const
{
    // When doing a relabeling, all areas will need to be checked for the non-endpoint condition. In the case
    // of doing a regular merge, then only the path elements need to be considered

    // Check each area to see if the active area contains any nonendpoint areas. If so, then the area can't be
    // a path.
    for(auto& id : path.areas)
    {
        if(!checkAllAreas && (active(id).type != HypothesisType::kPath))
        {
            continue;
        }

        for(auto& nonEndId : original_[id].nonendpoints)
        {
            if(utils::contains(path.areas, nonEndId))
            {
                return false;
            }
        }
    }

    return true;
}


double AlignmentNetwork::networkStrain(void)
{
    auto allActiveIds = activeAreaIds();

    double totalStrain = 0.0;

    // Go through each currently used active area and accumulate the strain
    for(auto& id : allActiveIds)
    {
        totalStrain += areaStrain(active_[id]);
    }

    // Check the global constraint by finding all areas that are disconnected from the main graph of decisions and paths
//     double areaDisconnected = numFailedGlobalConstraints();
//     totalStrain += areaDisconnected * strainWeights_.constraintWeight;

    return totalStrain;
}


double AlignmentNetwork::areaStrain(ActiveArea& area)
{
    double boundaryStrain = 0.0;
    std::unordered_set<const AreaHypothesisBoundary*> activeBoundaries;

    for(auto& bnd : boost::make_iterator_range(area.hypothesis->beginBoundary(), area.hypothesis->endBoundary()))
    {
        boundaryStrain += 1.0 - evaluator_->calculatePrior(*bnd, true);
        boundaryStrain += 1.0 - (bnd->getGateway().probability());
        activeBoundaries.insert(bnd);
    }

    double hypArea = area.hypothesis->extent().area();
    double maxAppropriatenessStrain = 1.0 - area.distribution.typeAppropriateness(area.type);
//     double maxAppropriatenessStrain = 0.0;
    maxAppropriatenessStrain *= hypArea;
//     double typeStrain = area.distribution.typeAppropriateness(area.type);

    double areaStrain = 0.0;
    areaStrain += maxAppropriatenessStrain;

    for(auto& origId : area.areas)
    {
        AreaHypothesis* hyp = original_[origId].hypothesis;
//         double appropriatenessStrain = original_[origId].distribution.maxAppropriateness()
//             - original_[origId].distribution.typeAppropriateness(area.type);
        double appropriatenessStrain = 1.0 - original_[origId].distribution.typeAppropriateness(area.type);
//         double appropriatenessStrain = area.distribution.typeAppropriateness(area.type);
//         double appropriatenessStrain = original_[origId].distribution.maxAppropriateness() - typeStrain;
        areaStrain += appropriatenessStrain * hyp->extent().area();

        // All inactive boundaries also add strain
        for(auto& bnd : boost::make_iterator_range(hyp->beginBoundary(), hyp->endBoundary()))
        {
            if(activeBoundaries.find(bnd) == activeBoundaries.end())
            {
                boundaryStrain += bnd->getGateway().probability();
            }
        }
    }

//     int numFailing = numFailedActiveConstraints(area);
//     double constraintStrain = (numFailing > 0) ? 1.0 : 0.0;
//     constraintStrain *= hypArea;

//     double totalStrain = (strainWeights_.appropriatenessWeight * maxAppropriatenessStrain)
//         + (strainWeights_.constraintWeight * constraintStrain);

    // Take half the boundary strain because it will be double-counted with the area on the other side
    double totalStrain = areaStrain + (0.5 * boundaryStrain);

#ifdef DEBUG_STRAIN
    std::cout << "Strain: (" << maxAppropriatenessStrain << ',' << constraintStrain << ") -> "
        << totalStrain << '\n';
#endif

    return totalStrain;
}


void AlignmentNetwork::setActiveLabel(ActiveArea& area, HypothesisType label)
{
    // Both the active hypothesis and all of the individual hypotheses need to have their labels changed to ensure
    // the underlying interaction between different hypotheses works. The hypothesis boundaries all point to original
    // AreaHypotheses, thus changing their types works equivalently to keeping track of the boundaries between the
    // active hypotheses, which requires lots more bookkeeping.
    area.type = label;
    area.hypothesis->setType(label);

    for(auto origId : area.areas)
    {
        original_[origId].hypothesis->setType(label);
    }
}


void AlignmentNetwork::applyChange(NetworkChange& change)
{
#ifdef DEBUG_UPDATES
    double currentStrain = networkStrain();

    if(change.changeType == kChangeLabel)
    {
        std::cout << "Applying label: " << change.sourceId << " to " << change.newLabels.front()
            << " Strain:" << (change.strain - currentStrain) << '\n';
    }
    else if(change.changeType == kChangeMerge)
    {
        std::cout << "Applying merge: " << change.sourceId << " Strain:" << (change.strain - currentStrain)
            << '\n';
        assert(change.newAreas.size() == 1);
    }
    else if(change.changeType == kChangeSplit)
    {
        std::cout << "Applying split: " << change.sourceId << " Strain:" << (change.strain - currentStrain)
            << '\n';
        assert(change.newAreas.size() > 1);
    }
    else // Error!
    {
        std::cerr << "ERROR:AlignmentNetwork:Unknown network change type:" << change.changeType << '\n';
    }
#endif

    for(auto& newArea : change.newAreas)
    {
        Id mergeId = newArea.id;
        applyMerge(std::move(newArea), mergeId);
    }
}


void AlignmentNetwork::applyMerge(ActiveArea&& mergedArea, Id mergedId)
{
    // After creation, point all the original areas at the new active id
    for(auto origId : mergedArea.areas)
    {
        // Clear the state from the previous active area for each of the originals
//         Id activeId = active(origId).id;
//         active(origId) = ActiveArea(activeId);
        original_[origId].active = mergedId;
        assert(!original_[origId].isFixed);
    }

    for(Id n = 0; n < static_cast<Id>(sizeActive()); ++n)
    {
        assert(n == active_[n].id);
    }

    // Assign the active area to the merged state
    active_[mergedId] = std::move(mergedArea);
    setActiveLabel(active_[mergedId], active_[mergedId].type);

    assert(active_[mergedId].type != HypothesisType::kPathDest);
}


// Take by value b/c associated ActiveArea.areas vector is cleared during the split process
void AlignmentNetwork::applySplit(IdVec areasToSplit)
{
    // For each of the merged areas in the active area, reset their active state to be the same as the original state
    // The active area associated with an original area has the same Id. Thus, each area associated with the active
    // area can just have the active_[id] be reset to the default state.
    for(auto a : areasToSplit)
    {
        HypothesisType type = original_[a].distribution.mostAppropriate();
        double typeRand = drand48();
        if(typeRand < 0.05)
        {
            type = HypothesisType::kPath;
        }
        else if(typeRand < 0.1)
        {
            type = HypothesisType::kDest;
        }
        else if(typeRand < 0.15)
        {
            type = HypothesisType::kDecision;
        }

        // If the randomly generated type is allowed, use it
        if(utils::contains(original_[a].possibleTypes, type))
        {
            resetActiveToOriginal(a, type);
        }
        // Otherwise this is a restricted type so just assign the first possible type
        else
        {
            resetActiveToOriginal(a, original_[a].possibleTypes.front());
        }
    }
}


void AlignmentNetwork::resetActiveToOriginal(Id area, HypothesisType type)
{
    assert(original_[area].hypothesis);

    ActiveArea& toReset = active_[area];
    toReset.id = area;
    toReset.areas.clear();
    toReset.areas.push_back(area);
    toReset.adjacent = original_[area].adjacent;
    toReset.hypothesis = original_[area].hypothesis;
    toReset.distribution = original_[area].distribution;
    toReset.constraints = original_[area].constraints;

    setActiveLabel(toReset, type);

    original_[area].active = area;
}


AlignmentNetwork::ActiveArea AlignmentNetwork::createMergedArea(const IdVec& areas, HypothesisType type)
{
    assert(!areas.empty());

    // The merged area contains the active area ids of all areas being merged.
    ActiveArea mergedArea;
    mergedArea.type = type;
    mergedArea.areas = areas;
    mergedArea.possibleTypes = original_[areas.front()].possibleTypes;

    for(auto origId : areas)
    {
        OriginalArea& area = original_[origId];

        // Copy all areas into the set of merged areas
        boost::push_back(mergedArea.adjacent, boost::as_array(area.adjacent));
        // New possible types are the intersection of the original types
        utils::erase_remove_if(mergedArea.possibleTypes, [&area](HypothesisType type) {
            return !utils::contains(area.possibleTypes, type);
        });
    }

    // Must always be at least one valid possible type -- otherwise an attempt to create an incorrect merge was made
    assert(!mergedArea.possibleTypes.empty());

    // Remove any areas from adjacent if they have been merged into the area
    utils::erase_remove_if(mergedArea.adjacent, [&mergedArea](ConstraintAdjacentArea adj) {
        return utils::contains(mergedArea.areas, adj.id);
    });

    std::sort(mergedArea.areas.begin(), mergedArea.areas.end());
    std::sort(mergedArea.adjacent.begin(), mergedArea.adjacent.end());

    utils::erase_unique(mergedArea.areas);
    utils::erase_unique(mergedArea.adjacent);

    // A new hypothesis consisting of all merged hypotheses needs to be created
    mergedArea.hypothesis = createMergedHypothesis(mergedArea.areas);
    mergedArea.distribution = evaluator_->calculateTypeDistribution(mergedArea.hypothesis->features());
    mergedArea.constraints = createMergedConstraints(mergedArea);

    mergedArea.id = mergedArea.areas.front();

    return mergedArea;
}


AreaHypothesis* AlignmentNetwork::createMergedHypothesis(const IdVec& toMerge)
{
    // Search the cache first before creating a new area
    for(auto& cachedArea : mergedCache_)
    {
        if(cachedArea.first == toMerge)
        {
            return cachedArea.second;
        }
    }

    // The cached value wasn't found so a new area needs to be created
    std::vector<AreaHypothesis*> areasToMerge;
    areasToMerge.reserve(toMerge.size());
    for(auto& origId : toMerge)
    {
        areasToMerge.push_back(original_[origId].hypothesis);
    }
    mergedHypotheses_.push_back(std::unique_ptr<AreaHypothesis>(new AreaHypothesis(areasToMerge)));
    // Store the newly created hypothesis for later use
    mergedCache_.push_back(std::make_pair(toMerge, mergedHypotheses_.back().get()));

    return mergedHypotheses_.back().get();
}


std::vector<AlignmentConstraint> AlignmentNetwork::createMergedConstraints(const ActiveArea& area)
{
    std::vector<AlignmentConstraint> mergedConstraints;

    constraintCache_.clear();

    for(Id origId : area.areas)
    {
        std::copy_if(original_[origId].constraints.begin(),
                     original_[origId].constraints.end(),
                     std::back_inserter(constraintCache_),
                     [origId](const AlignmentConstraint& c) { return c.insideId() == origId; });
    }

    addUnalignedConstraints(area, mergedConstraints);
    addAdjacencyConstraints(area, mergedConstraints);
    addAllNeighborsConstraint(area, mergedConstraints);
    addFixedTypeConstraint(area, mergedConstraints);

#ifdef DEBUG_CONSTRAINTS
    std::cout << "Initial constraints:\n";
    std::copy(constraintCache_.begin(),
              constraintCache_.end(),
              std::ostream_iterator<AlignmentConstraint>(std::cout, "\n"));
    std::cout << "Merged constraints:\n";
    std::copy(mergedConstraints.begin(),
              mergedConstraints.end(),
              std::ostream_iterator<AlignmentConstraint>(std::cout, "\n"));
    std::cout << "Areas:";
    std::copy(area.areas.begin(), area.areas.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << "\nAdjacent:";
    std::copy(area.adjacent.begin(),
              area.adjacent.end(),
              std::ostream_iterator<ConstraintAdjacentArea>(std::cout, " "));
    std::cout << "\n\n";
#endif // DEBUG_CONSTRAINTS

    return mergedConstraints;
}


void AlignmentNetwork::addUnalignedConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    // Find all pairwise-unaligned gateways in the new AreaHypothesis. The unaligned need to be recalculated because
    // the new pattern of gateways will yield different sets of aligned and unaligned. The main problem with making the
    // assumption of   unaligned separately -> unaligned merged is diagonal gateways cutting an area in half. In these
    // situations, unaligned gateways individually can be aligned when considered together. Conversely, these diagonal
    // gateways could create aligned gateways at an L-intersection, but then merging creates a normal unaligned
    // intersection. The downside is the occasional appearance of unaligned gateways in the middle of hallways. For
    // the most part, these are removed during the search

    AlignmentNetwork::Id insideId = area.areas.front();
    AlignmentConstraint::AdjVec adjacent;

    auto unalignedGateways = area.hypothesis->pairwiseUnalignedGateways();

    for(auto& u : unalignedGateways)
    {
        adjacent.clear();

        auto firstAdj = adjacentFromGateway(area, u.first->id());
        auto secondAdj = adjacentFromGateway(area, u.second->id());

        adjacent.push_back(firstAdj);
        adjacent.push_back(secondAdj);
        mergedConstraints.push_back(AlignmentConstraint::CreateUnalignedConstraint(adjacent, insideId));
    }
}


void AlignmentNetwork::addAdjacencyConstraints(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    AlignmentConstraint::AdjVec endpoints;

    for(auto& adj : area.adjacent)
    {
        if(area.hypothesis->isEndGateway(adj.gatewayId))
        {
            endpoints.push_back(adj);
        }
        else
        {
            // Doesn't matter which is marked as the inside. All areas work because they will point to the same
            // active area when the constraint is checked.
            mergedConstraints.push_back(AlignmentConstraint::CreateAdjacentConstraint(adj,
                                                                                      area.areas.front()));
        }
    }

    if(!endpoints.empty())
    {
        mergedConstraints.push_back(AlignmentConstraint::CreateEndpointsConstraint(endpoints,
                                                                                   area.areas.front()));
    }
}


void AlignmentNetwork::addAllNeighborsConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    // The adjacent areas are noted by their ConstraintAdjacentArea, thus all-neighbors just needs to create
    // a constraint amongst all of these areas.
    mergedConstraints.push_back(AlignmentConstraint::CreateAllNeighborsConstraint(area.adjacent,
                                                                                  area.areas.front()));
}


void AlignmentNetwork::addFixedTypeConstraint(const ActiveArea& area, ConstraintVec& mergedConstraints)
{
    for(auto& constraint : constraintCache_)
    {
        if(constraint.type() == AlignmentConstraint::fixed)
        {
            mergedConstraints.push_back(AlignmentConstraint::CreateFixedConstraint(area.adjacent,
                                                                                   area.areas.front(),
                                                                                   constraint.fixedType()));
        }
    }
}


ConstraintAdjacentArea AlignmentNetwork::adjacentFromGateway(const ActiveArea& area, int32_t gatewayId) const
{
    for(auto& adj : area.adjacent)
    {
        if(adj.gatewayId == gatewayId)
        {
            return adj;
        }
    }

    return ConstraintAdjacentArea{-1, -1};
}


void AlignmentNetwork::setActiveChange(NetworkChange& testChange)
{
    // Set the size of the change being activated
    testSize_ = testChange.newAreas.size();

    assert(testSize_ < kMaxChangeSize);

    // Add the new areas being tested to the active areas
    active_.insert(active_.end(), testChange.newAreas.begin(), testChange.newAreas.end());

    // Save the previously assignments for the areas being tested
    testAssignedActive_.clear();
    for(auto& active : testChange.newAreas)
    {
        for(auto origId : active.areas)
        {
            testAssignedActive_.push_back(original_[origId].active);
        }
    }

    // Reassign all original areas to point to the test active areas
    for(std::size_t n = 0; n < testSize_; ++n)
    {
        Id changedAreaId = sizeActive() + n;    // each test area is added on to the end of the active areas

        for(auto& origId : testChange.newAreas[n].areas)
        {
            original_[origId].active = changedAreaId;
        }

        active_[changedAreaId].id = changedAreaId;
        setActiveLabel(active_[changedAreaId], testChange.newLabels[n]);
    }
}


void AlignmentNetwork::clearActiveChange(void)
{
    assert(!testAssignedActive_.empty());
    assert(testSize_ > 0);

    // Point all original areas in the active test area back to their assigned area
    int nextTestAssignedIndex = 0;
    for(std::size_t n = sizeActive(); n < active_.size(); ++n)
    {
        IdVec& testAreas = active_[n].areas;
        for(std::size_t i = 0, end = testAreas.size(); i < end; ++i)
        {
            original_[testAreas[i]].active = testAssignedActive_[nextTestAssignedIndex];
            setActiveLabel(active(testAreas[i]), active(testAreas[i]).type);
            ++nextTestAssignedIndex;
        }
    }

    active_.resize(sizeActive());       // throw away the test active areas
    testSize_ = 0;                      // and reset back to no test areas
}


double AlignmentNetwork::numFailedGlobalConstraints(AlignmentNetwork::ConstraintVec* failed)
{
    disconnectedAreas_.clear();
    findDisconnectedAreas();

    double disconnectedArea = 0.0;

    for(Id id : disconnectedAreas_)
    {
        ActiveArea& area = active_[id];
        disconnectedArea += area.hypothesis->extent().area();

        // If failed, then create the all-neighbors constraint to indicate the massive badness that is created
        if(failed)
        {
            failed->push_back(AlignmentConstraint::CreateAllNeighborsConstraint(area.adjacent, area.areas.front()));
        }
    }

    return disconnectedArea;
}


int AlignmentNetwork::numFailedActiveConstraints(const ActiveArea& area, ConstraintVec* failed)
{
    int numFailed = 0;

    // Check the AlignmentConstraints for failures
    for(auto& c : area.constraints)
    {
        if(!c.isSatisfied(*this, area.type))
        {
            ++numFailed;

            if(failed)
            {
                failed->push_back(c);
            }
        }
    }

    // Check to see if the are enough unaligned paths for a decision point. If not, then mark it as failed as well.
    // The unaligned check has to live outside the AlignmentConstraint unfortunately.
    int numOffsetPaths = 0;
    int numUnalignedConstraints = 0;
    for(auto& c : area.constraints)
    {
        if(c.isUnalignedPathEnds(*this))
        {
            ++numOffsetPaths;
        }

        if(c.type() == AlignmentConstraint::unaligned)
        {
            ++numUnalignedConstraints;
        }
    }

    bool mustBeConnected = false;
    for(auto& origId : area.areas)
    {
        mustBeConnected |= original_[origId].mustBeConnected;
    }

    // Only a decision point sitting on the boundary of the map is allowed to not have offset paths
    // Can only fail this check if there are actually unaligned constraints to be failed.
    if(((numOffsetPaths == 0) || (numUnalignedConstraints == 0))
        && (area.type == HypothesisType::kDecision)
        && mustBeConnected)
    {
        if(!area.hypothesis->isBoundary())
        {
            ++numFailed;

            // If failed, then create the all-neighbors constraint to indicate something needs to change amongst the
            // neighbors
            if(failed)
            {
                failed->push_back(AlignmentConstraint::CreateAllNeighborsConstraint(area.adjacent, area.areas.front()));
            }
        }
    }

    return numFailed;
}


void AlignmentNetwork::findDisconnectedAreas(void)
{
    NonDestEdge<Graph, AlignmentNetwork> edgeFilter = { this, &graph_ };
    NonDestVertex<Graph, AlignmentNetwork> vertexFilter = { this, &graph_ };
    auto filtered = boost::make_filtered_graph(graph_, edgeFilter, vertexFilter);
    components_.resize(original_.size());
    std::fill(components_.begin(), components_.end(), -1);
    int numComponents = boost::connected_components(filtered, &components_[0]);

    // If there is more than one component, some subset of the path segments and decision points in the environment are
    // disconnected from the main connected graph.
    if(numComponents > 1)
    {
        // The areas in the largest component are accepted as not failing the constraint. All other areas not in the
        // largest component are failing the connected constraint
        std::vector<int> componentSizes(numComponents, 0);
        for(const auto& area : boost::make_iterator_range(boost::vertices(filtered)))
        {
            ++componentSizes[components_[area]];
        }

        int maxComponent = std::distance(componentSizes.begin(), std::max_element(componentSizes.begin(),
                                                                                  componentSizes.end()));

        // Go through all the original areas. If they aren't a destination, then they are disconnected from the main
        // graph of decisions and paths in the environment.
        for(auto& orig : original_)
        {
            if((active(orig.id).type != HypothesisType::kDest) // only paths and decisions can be disconnected
                && (components_[orig.id] != maxComponent))   // if not part of biggest component, then disconnected
            {
                if(orig.mustBeConnected)
                {
                    disconnectedAreas_.push_back(active(orig.id).id);
                }

                for(auto& adj : orig.adjacent)
                {
                    if(original_[adj.id].mustBeConnected)    // only worry if this area must be connected to the graph
                    {
                        disconnectedAreas_.push_back(active(adj.id).id);
                    }
                }
            }
        }
    }

    std::sort(disconnectedAreas_.begin(), disconnectedAreas_.end());
    utils::erase_unique(disconnectedAreas_);
}


void AlignmentNetwork::saveAreaExtents(CSPDebugInfo* debug)
{
    debug->extents.reserve(original_.size());

    for(OriginalArea& orig : original_)
    {
        debug->extents.push_back(orig.hypothesis->extent());
    }
}


void AlignmentNetwork::saveInconsistentAreas(const ConstraintVec& failed, CSPDebugInfo* debug)
{
    CSPIteration iteration;
    iteration.labels.reserve(original_.size());

    for(OriginalArea& orig : original_)
    {
        iteration.labels.push_back(active_[orig.active].type);
        iteration.strains.push_back(areaStrain(active_[orig.active]));
    }

    iteration.isFailing.resize(original_.size(), false);

    std::vector<Id> failedIds;
    failedIds.reserve(failed.size() * 3);
    for(auto& constraint : failed)
    {
        failedIds.insert(failedIds.end(), constraint.beginIds(), constraint.endIds());
    }
    std::sort(failedIds.begin(), failedIds.end());
    utils::erase_unique(failedIds);

    iteration.failedAreas.reserve(failedIds.size());
    for(Id id : failedIds)
    {
        ActiveArea& area = active(id);
        if(area.hypothesis)
        {
            CSPArea failedArea;
            failedArea.boundary = area.hypothesis->extent().rectangleBoundary(math::ReferenceFrame::GLOBAL);
            failedArea.oldType = area.type;
            failedArea.newType = area.type;
            iteration.failedAreas.push_back(std::move(failedArea));

            if(area.type == HypothesisType::kPath)
            {
                iteration.pathEndpoints.push_back(area.hypothesis->endpoints());
            }
        }

        // Mark the internal original areas as failing
        for(auto origId : area.areas)
        {
            iteration.isFailing[origId] = true;
        }
    }

    debug->iterations.push_back(std::move(iteration));
}


void AlignmentNetwork::saveChangedArea(Id changedId, HypothesisType oldType, CSPDebugInfo* debug)
{
    if(changedId >= 0)
    {
        ActiveArea& area = active_[changedId];
        if(area.hypothesis)
        {
            CSPArea failedArea;
            failedArea.boundary = area.hypothesis->extent().rectangleBoundary(math::ReferenceFrame::GLOBAL);
            failedArea.oldType = oldType;
            failedArea.newType = area.type;
            debug->iterations.back().updatedArea = failedArea;
        }
    }
}

} // namespace hssh
} // namespace vulcan
iledIds);

    iteration.failedAreas.reserve(failedIds.size());
    for(Id id : failedIds)
    {
        ActiveArea& area = active(id);
        if(area.hypothesis)
        {
            CSPArea failedArea;
            failedArea.boundary = area.hypothesis->extent().rectangleBoundary(math::ReferenceFrame::GLOBAL);
            failedArea.oldType = area.type;
            failedArea.newType = area.type;
            iteration.failedAreas.push_back(std::move(failedArea));

            if(area.type == HypothesisType::kPath)
            {
                iteration.pathEndpoints.push_back(area.hypothesis->endpoints());
            }
        }

        // Mark the internal original areas as failing
        for(auto origId : area.areas)
        {
            iteration.isFailing[origId] = true;
        }
    }

    debug->iterations.push_back(std::move(iteration));
}


void AlignmentNetwork::saveChangedArea(Id changedId, HypothesisType oldType, CSPDebugInfo* debug)
{
    if(changedId >= 0)
    {
        ActiveArea& area = active_[changedId];
        if(area.hypothesis)
        {
            CSPArea failedArea;
            failedArea.boundary = area.hypothesis->extent().rectangleBoundary(math::ReferenceFrame::GLOBAL);
            failedArea.oldType = oldType;
            failedArea.newType = area.type;
            debug->iterations.back().updatedArea = failedArea;
        }
    }
}

} // namespace hssh
} // namespace vulcan
