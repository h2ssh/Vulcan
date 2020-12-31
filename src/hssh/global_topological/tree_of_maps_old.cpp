/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tree_of_maps.cpp
* \author   Collin Johnson
*
* Definition of TreeOfMapsOld.
*/

#include "hssh/global_topological/tree_of_maps.h"
#include "hssh/global_topological/topological_map_hypothesis.h"
#include "hssh/global_topological/hypothesis_tree.h"
#include "core/angle_functions.h"
#include <iostream>
#include <queue>
#include <cassert>

// #define DEBUG_LEAVES
// #define DEBUG_CONNECT_PLACES
// #define DEBUG_HYPOTHESIS_TREE

namespace vulcan
{
namespace hssh
{

TreeOfMapsOld::TreeOfMapsOld(void)
    : nextHypothesisId(0)
{
}


bool TreeOfMapsOld::setRootHypothesis(uint32_t id)
{
    // Find the hypothesis to be the root and then kill off everything else
    // Start at the bottom of the tree because the desired operation is usually to throw away the
    // rest of the tree after determining the correct map among the most recent maps

    TopoMapPtr root;

    for(auto depthIt = depths.rbegin(), depthEnd = depths.rend(); depthIt != depthEnd; ++depthIt)
    {
        auto hypIt = std::find_if(depthIt->begin(), depthIt->end(), [&id](const TopoMapPtr& hyp) { return hyp->getId() == id; });

        if(hypIt != depthIt->end())
        {
            root = *hypIt;
            break;
        }
    }

    // If the map was found, then set it to be the new root. Otherwise, just pretend this method call never happened.
    if(root)
    {
        setRootHypothesis(root);
    }

    return root != 0;
}


void TreeOfMapsOld::setRootHypothesis(TopoMapPtr root)
{
    leaves.clear();
    depths.clear();
    idToMap.clear();

    root->id    = nextHypothesisId++;
    root->depth = 0;
    root->children.clear();

    std::set<TopoMapPtr> newDepth;
    newDepth.insert(root);
    depths.push_back(newDepth);
    leaves.insert(root);
    idToMap[root->id] = root;
}


TopoMapPtr TreeOfMapsOld::getMapFromId(uint32_t id) const
{
    TopoMapPtr mapPtr;

    auto mapIt = idToMap.find(id);

    if(mapIt != idToMap.end())
    {
        mapPtr = mapIt->second;
    }

    return mapPtr;
}


const std::set<TopoMapPtr>& TreeOfMapsOld::getLeaves(void) const
{
    #ifdef DEBUG_LEAVES
    std::cout<<"DEBUG:TreeOfMapsOld:Leaves:\n";

    for(auto leafIt = leaves.begin(), leafEnd = leaves.end(); leafIt != leafEnd; ++leafIt)
    {
        assert((*leafIt)->children.empty());

        std::cout<<leafIt->get()<<' '<<(*leafIt)->getDepth()<<' '<<(*leafIt)->getLogPosterior()<<' '<<(*leafIt)->getEstimatedLogPosterior()<<'\n';
    }
    #endif

    return leaves;
}


HypothesisTree TreeOfMapsOld::getHypothesisTree(void) const
{
    std::vector<hypothesis_tree_node_t> hypNodes;

    if(!depths.empty())
    {
        buildHypothesisTree(*(depths[0].begin()), hypNodes);
        return HypothesisTree(probabilityDescriptions, hypNodes, (*(depths[0].begin()))->getId(), leaves.size());
    }
    else
    {
        return HypothesisTree(probabilityDescriptions, hypNodes, 0, leaves.size());
    }
}


void TreeOfMapsOld::setMeasurementLikelihoodDescriptions(const std::vector<std::string>& descriptions)
{
    probabilityDescriptions.clear();

    // Put fixed descriptions first, then follow with the various measurement likelihoods
    probabilityDescriptions.push_back("log likelihood");
    probabilityDescriptions.push_back("log prior");
    probabilityDescriptions.push_back("log posterior");
    probabilityDescriptions.push_back("estimated log likelihood");
    probabilityDescriptions.push_back("estimated log prior");
    probabilityDescriptions.push_back("estimated log posterior");
    probabilityDescriptions.insert(probabilityDescriptions.end(), descriptions.begin(), descriptions.end());
}


void TreeOfMapsOld::createInitialHypothesis(const GlobalPlace& initialPlace)
{
    // Only allow this method to be called for an empty tree
    assert(depths.empty());
    assert(leaves.empty());

    setRootHypothesis(TopoMapPtr(new TopologicalMapHypothesis(initialPlace)));
}


TopoMapPtr TreeOfMapsOld::addNewPlace(const TopoMapPtr& map,
                                   GlobalPlace&      newPlace,
                                   int               entrySegmentId,
                                   const Lambda&     lambda)
{
    if(isValidParent(map))
    {
        TopoMapPtr childMap(new TopologicalMapHypothesis(*map.get()));

        addChildToMap(map, childMap);
        childMap->addNewPlace(newPlace, entrySegmentId, lambda);
        validateHypothesis(childMap);
        return childMap;
    }
    else
    {
        assert("bad parent add new place\n" && false);
        return TopoMapPtr();
    }
}


TopoMapPtr TreeOfMapsOld::connectPlaces(const TopoMapPtr&         map,
                                     const place_connection_t& previousPlace,
                                     const place_connection_t& currentPlace,
                                     const Lambda&             lambda,
                                     const pose_t&      transform)
{
    if(isValidParent(map))
    {
        TopoMapPtr childMap(new TopologicalMapHypothesis(*map.get()));

        #ifdef DEBUG_CONNECT_PLACES
        std::cout<<"DEBUG:TreeOfMapsOld:Connecting places in hypothesis:"<<previousPlace.placeId<<':'<<previousPlace.eventFragment.fragmentId
                 <<" to "<<currentPlace.placeId<<':'<<currentPlace.eventFragment.fragmentId<<'\n';
        #endif

        childMap->connectPlaces(currentPlace, previousPlace, lambda, transform);
        validateHypothesis(childMap);
        addChildToMap(map, childMap);
        return childMap;
    }
    else
    {
        assert(false && "bad parent connect places\n");
        return TopoMapPtr();
    }
}


TopoMapPtr TreeOfMapsOld::copyMapToNextDepth(const TopoMapPtr& map, const Lambda& lambda)
{
    if(isValidParent(map))
    {
        TopoMapPtr childMap(new TopologicalMapHypothesis(*map.get()));

        childMap->updateLambdaOnLastPathSegment(lambda);
        addChildToMap(map, childMap);
        return childMap;
    }
    else
    {
        assert(false && "bad parent copy to next\n");
        return TopoMapPtr();
    }
}


uint32_t TreeOfMapsOld::prune(TopoMapPtr toPrune)
{
    assert(toPrune.get());
    // Prune recursively until hitting a parent with a child
    if(!toPrune->children.empty() || (toPrune->depth == 0))
    {
        if(toPrune->depth == 0 && toPrune->children.empty())
        {
            std::cerr<<"WARNING:TreeOfMapsOld: Wanted to prune the root!\n";
        }

        return toPrune->depth;
    }

    // Erase all vestiges of map -- leaves, depth, and from parent
    assert(!toPrune->pruned);
    assert(!toPrune->parent->pruned);
    toPrune->pruned = true;
    leaves.erase(toPrune);
    depths[toPrune->depth].erase(toPrune);
    idToMap.erase(toPrune->getId());

    auto childIt = std::find(toPrune->parent->children.begin(), toPrune->parent->children.end(), toPrune);
    assert(childIt != toPrune->parent->children.end());

    toPrune->parent->children.erase(childIt);

    // Recurse to the parent, as it might now have no valid children either, thus invalidating itself!
    return prune(toPrune->parent);
}


void TreeOfMapsOld::buildHypothesisTree(const TopoMapPtr& hypothesis, std::vector<hypothesis_tree_node_t>& hypNodes) const
{
    // Recursively go through the all the nodes in the tree. Leaves have no children, so the recursion will successfully stop.
    hypothesis_tree_node_t hypNode;

    hypNode.id = hypothesis->id;
    hypNode.probabilities.push_back(hypothesis->probability.logLikelihood);
    hypNode.probabilities.push_back(hypothesis->probability.logPrior);
    hypNode.probabilities.push_back(hypothesis->probability.logPosterior);
    hypNode.probabilities.push_back(hypothesis->probability.estimatedLogLikelihood);
    hypNode.probabilities.push_back(hypothesis->probability.estimatedLogPrior);
    hypNode.probabilities.push_back(hypothesis->probability.estimatedLogPosterior);
    hypNode.probabilities.insert(hypNode.probabilities.begin(), hypothesis->probability.measurementLogLikelihoods.begin(), hypothesis->probability.measurementLogLikelihoods.end());

    for(auto childIt = hypothesis->children.begin(), childEnd = hypothesis->children.end(); childIt != childEnd; ++childIt)
    {
        hypNode.children.push_back((*childIt)->id);
        buildHypothesisTree(*childIt, hypNodes);
    }

    hypNodes.push_back(hypNode);

    #ifdef DEBUG_HYPOTHESIS_TREE
    std::cout<<"DEBUG:HypothesisTree:Node "<<hypNode.id<<" Children:"<<hypNode.children.size()<<'\n';
    #endif
}


bool TreeOfMapsOld::isValidParent(const TopoMapPtr& parent)
{
    assert(parent.get());

    uint32_t depth = parent->getDepth();

    if(depth < depths.size() && depths[depth].find(parent) != depths[depth].end() && !parent->pruned)
    {
        return true;
    }
    else if(depth >= depths.size())
    {
        std::cerr<<"ERROR:TreeOfMapsOld:Attempting to add place to a map with depth "<<depth<<" but tree height is "<<getHeight()<<". Ignoring.\n";
        assert(false);
    }
    else if(parent->pruned)
    {
        std::cerr<<"ERROR:TreeOfMapsOld:Attempting to add place to a map but parent was pruned. Ignoring.\n";
        assert(false);
    }
    else
    {
        std::cerr<<"ERROR:TreeOfMapsOld:Attempting to add place to a map with depth "<<depth<<" but map doesn't exist at that depth. Ignorning.\n";
        assert(false);
    }

    return false;
}


void TreeOfMapsOld::addChildToMap(const TopoMapPtr& parent, const TopoMapPtr& child)
{
    if(parent->children.empty())
    {
        leaves.erase(parent);
    }

    parent->children.push_back(child);
    child->id     = nextHypothesisId++;
    child->parent = parent;
    child->depth  = parent->depth + 1;

    leaves.insert(child);
    idToMap[child->id] = child;

    if(child->depth == depths.size())
    {
        std::set<TopoMapPtr> nextDepth;
        nextDepth.insert(child);
        depths.push_back(nextDepth);
    }
    else
    {
        depths[child->depth].insert(child);
    }
}


void TreeOfMapsOld::validateHypothesis(TopoMapPtr& map)
{
    for(auto pathIt = map->paths.begin(), pathEnd = map->paths.end(); pathIt != pathEnd; ++pathIt)
    {
        GlobalPath& currentPath = pathIt->second;

        const std::deque<GlobalPathSegment>& segments = currentPath.getGlobalPathSegments();

        for(auto segmentIt = segments.begin(), segmentEnd = segments.end(); segmentIt != segmentEnd; ++segmentIt)
        {
            if(segmentIt->getPlusTransition().placeId >= PATH_MIN_PLACE_ID && segmentIt->getMinusTransition().placeId >= PATH_MIN_PLACE_ID)
            {
                GlobalPlace& plusPlace  = map->places[segmentIt->getPlusTransition().placeId];
                GlobalPlace& minusPlace = map->places[segmentIt->getMinusTransition().placeId];

                assert(plusPlace.getStar().getFragmentWithId(segmentIt->getPlusTransition().transitionFragmentId).exploration == PATH_FRAGMENT_EXPLORED);
                assert(minusPlace.getStar().getFragmentWithId(segmentIt->getMinusTransition().transitionFragmentId).exploration == PATH_FRAGMENT_EXPLORED);
            }
            else if(segmentIt->getPlusTransition().placeId >= PATH_MIN_PLACE_ID && segmentIt->getMinusTransition().placeId == PATH_FRONTIER_ID)
            {
                GlobalPlace& plusPlace  = map->places[segmentIt->getPlusTransition().placeId];

                assert(plusPlace.getStar().getFragmentWithId(segmentIt->getPlusTransition().transitionFragmentId).exploration == PATH_FRAGMENT_FRONTIER);
            }
            else if(segmentIt->getPlusTransition().placeId == PATH_FRONTIER_ID && segmentIt->getMinusTransition().placeId >= PATH_MIN_PLACE_ID)
            {
                GlobalPlace& minusPlace  = map->places[segmentIt->getMinusTransition().placeId];

                assert(minusPlace.getStar().getFragmentWithId(segmentIt->getMinusTransition().transitionFragmentId).exploration == PATH_FRAGMENT_FRONTIER);
            }
        }
    }
}

} // namespace hssh
} // namespace vulcan
