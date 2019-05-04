/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     generator_queue.h
* \author   Collin Johnson
* 
* Declaration of GeneratorQueue abstract base class.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_GENERATOR_QUEUE_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_GENERATOR_QUEUE_H

#include <hssh/global_topological/mapping/probability_heuristics.h>
#include <hssh/utils/id.h>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace vulcan
{
namespace hssh 
{
    
class HypothesisGenerator;
struct TopoMapProbability;
struct TopologicalState;


/**
* GeneratorQueue is an abstract base class for the queue of map hypotheses maintained for TopologicalSLAM. The queue
* determines the order and stopping condition for a SLAM update performed when a new topological event occurs.
* 
* The queue works by selecting the next generator to use to create a map hypothesis. This generator creates a new map
* hypothesis, which is then returned. After construction, the computed posterior probability of the hypothesis is set
* in the queue to help determine when the current update can stop. If there are no hypotheses in the queue, then the
* update is done.
* 
* The basic use of GeneratorQueue goes as follows:
* 
*   - Add generators for all leaf hypotheses
*   - Set the depth of a complete map.
*   - while hasNext is true, keep generating maps
*   - for each map, add its posterior probability to help check the stopping condition
*   - additionally, probability heuristics that are used for ordering hypotheses must be provided
* 
* For a subclass, they need to implement the methods:
* 
*   - doUpdateCompletePosterior : incorporate a new complete posterior into the check for an update being complete
*   - doUpdateCompleteDepth : set the new depth for complete hypotheses, which is used to trigger a new update starting
*   - shouldGenerateMap : check if a new map needs to be generated, given the current state of the queue
*/
class GeneratorQueue
{
public:
    
    /**
    * Destructor for GeneratorQueue.
    */
    virtual ~GeneratorQueue(void);
    
    ///// Methods for initializing an update /////
    
    /**
    * hasGenerator checks if a generator exists for the specified TopologicalState. If no generator exists, one can
    * be added via addGenerator, otherwise, children will already be produced by the queue.
    * 
    * \param    state               State to check for an existing generator
    * \return   True if a generator for the provided state already exists.
    */
    bool hasGenerator(const TopologicalState& state) const;
    
    /**
    * addGenerator adds a new generator to be used by the queue. Generators should be added only to initialize the
    * queue for an update.
    * 
    * \param    generator           Generator that will be incorporated into the search queue
    */
    void addGenerator(std::unique_ptr<HypothesisGenerator> generator);
    
    /**
    * setProbabilityHeuristics sets the heuristics to use for determining the priority of a hypothesis. If the queue
    * has been constructed, then it will be reconstructed to incorporate the new heuristics. Set this before adding
    * generators to avoid that need.
    * 
    * \param    heuristics          Heuristics to use for determining generator priority
    */
    void setProbabilityHeuristics(const ProbabilityHeuristics& heuristics);
    
    /**
    * setCompleteDepth changes the depth of a complete map. Changing the depth will change the condition for whether
    * a map is complete and thus can change whether hasNext returns true or false. The depth should be set only at the
    * start of a given map update.
    * 
    * \pre  depth can only increase, never decrease. A new queue must be created if the depth needs to decrease.
    * \param    depth       New depth to set for a complete hypothesis in the queue
    */
    void setCompleteDepth(int depth);
    
    ///// Methods for running an update /////
    
    /**
    * hasNext checks if there is a another map hypothesis in the queue.
    * 
    * \return   True if there's another hypothesis that needs to be created.
    */
    bool hasNext(void) const;
    
    /**
    * nextMap pops and returns the next map in the queue. If there's no more maps, then it will be a null-ptr.
    * 
    * \return   A pair containing the next map in the priority ordering and a pointer to the parent state. A state 
    * with an invalid id and a null parent is returned if there aren't any more maps.
    */
    std::pair<TopologicalState, const TopologicalState*> nextMap(void);
    
    /**
    * addCompletePosterior adds information about the posterior probability of a complete hypothesis, which is needed
    * to determine whether or not an update has finished. As new complete maps are generated during an update, every
    * complete map should have its posterior passed to the generator queue.
    * 
    * \param    probability     New probability measurement for a complete (max-depth) hypothesis
    */
    void addCompleteProbability(const TopoMapProbability& probability);
    
    /**
    * exhaustedGenerators retrieves the list of ids of maps/generators that have no more children remaining. These
    * generators will not generate hypotheses anymore. If the tree of maps contains no children for one of these 
    * exhausted generators, then it needs to be pruned because it didn't generate any valid children.
    */
    std::vector<Id> exhaustedGenerators(void) const { return exhausted_; }
    
    /**
    * clearExhausted clears out the currently maintained list of exhausted generators.
    */
    void clearExhausted(void) { exhausted_.clear(); }
    
    
protected:
    
    /**
    * doUpdateCompleteDepth updates the depth of a map that is considered to be complete.
    * 
    * \param    depth           New depth for a complete map
    */
    virtual void doUpdateCompleteDepth(int depth) = 0;
    
    /**
    * doUpdateCompleteProbability should process a new complete probability provided by a user. The probability can
    * be incorporated however is needed to create the proper behavior for the queue.
    * 
    * \param    probability     New probability measurement for a complete (max-depth) hypothesis
    */
    virtual void doUpdateCompleteProbability(const TopoMapProbability& probability) = 0;
    
    /**
    * shouldGenerateMap checks if a map should be generated by the specified generator. The generator is guaranteed to
    * have at least one remaining hypothesis to generator, i.e. completed() == false.
    * 
    * \param    generator           Next generator in the queue, for which the desire to generate is unknown
    * \return   True if this generator should spawn a new map. False otherwise.
    */
    virtual bool shouldGenerateMap(const HypothesisGenerator& generator) const = 0;
    
private:
    
    struct GenPtrComparator
    {
        bool operator()(const HypothesisGenerator* lhs, const HypothesisGenerator* rhs) const;
    };

    using GenPtr = std::unique_ptr<HypothesisGenerator>;
    using GenQueue = std::priority_queue<HypothesisGenerator*, 
                                         std::vector<HypothesisGenerator*>,
                                         GenPtrComparator>;
    
    // INVARIANT: All generators in the queue are not completed.
    // INVARIANT: All generators ever created have their ids stored in genIds_.
    std::unordered_map<HypothesisGenerator*, GenPtr> generators_;
    std::unordered_set<Id> genIds_;
    std::vector<Id> exhausted_;
    ProbabilityHeuristics heuristics_;
    GenQueue queue_;
    int depth_ = 0;         // current depth of a complete map
    
    
    void rebuildPriorityQueue(void);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_GENERATOR_QUEUE_H
