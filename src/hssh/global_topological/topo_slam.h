/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topo_slam.h
* \author   Collin Johnson
*
* Declaration of TopologicalSLAM.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_TOPO_SLAM_H
#define HSSH_GLOBAL_TOPOLOGICAL_TOPO_SLAM_H

#include "hssh/global_topological/mapping/tree_of_maps.h"
#include "hssh/global_topological/mapping/probability_heuristics.h"
#include "hssh/global_topological/state.h"
#include "hssh/global_topological/utils/visit_sequence.h"
#include "hssh/global_topological/utils/metric_map_cache.h"
#include <fstream>
#include <memory>

namespace vulcan
{
namespace system { class DebugCommunicator; }
namespace hssh
{

class GeneratorQueue;
class HypothesisGeneratorFactory;
class HypothesisProbabilityEvaluator;
class MapOptimizer;
class TopologicalLocalizer;


/**
* TopologicalSLAM implements the basic multi-hypothesis topological SLAM algorithm. The basic algorithm searches the
* space of possible topological maps of an environment to find the best estimate of the current topological state
* (map + location).
*
* IMPORTANT: Turn around events are only applied when the path segment is exited for the SLAM process.
*/
class TopologicalSLAM
{
public:

    /**
    * Constructor for TopologicalSLAM.
    *
    * \param    localizer           Localizer to use for determining the robot's new topological location
    * \param    queue               Queue to use for selecting the next topological map to evaluate
    * \param    generatorFactory    Factory for creating generator for creating new map hypotheses during the search
    * \param    optimizer           Optimizer to use for computing Chi values
    * \param    evaluator           Evaluator to use for computing hypothesis probabilities
    * \param    resultsFile         Name of the file in which to write the results
    */
    TopologicalSLAM(std::unique_ptr<TopologicalLocalizer> localizer,
                    std::unique_ptr<GeneratorQueue> queue,
                    std::unique_ptr<HypothesisGeneratorFactory> generatorFactory,
                    std::unique_ptr<MapOptimizer> optimizer,
                    std::unique_ptr<HypothesisProbabilityEvaluator> evaluator,
                    const std::string& resultsFile);

    /**
    * Destructor for TopolgoicalSLAM.
    */
    ~TopologicalSLAM(void);

    /**
    * addEvent adds a new area event to the topological visit sequence. The information in this event will be
    * incorporated into the topological state estimate on the next call to updateMap.
    *
    * \param    event           LocalAreaEvent that was detected in the Local Topological layer
    * \param    localMap        LocalTopoMap in which the event occurred
    */
    void addEvent(const LocalAreaEvent& event, const LocalTopoMap& localMap);

    /**
    * updatePose provides a new pose estimate for the robot in the LPM. This pose estimate is used to help provide
    * information on the robot's travels through the world in the event of some sort of place detection error.
    *
    * \param    pose            Estimated pose in the LPM
    */
    void updatePose(const LocalPose& pose);

    /**
    * estimateTopologicalState updates the estimate of the topological state -- map + location -- based on all
    * topological events and local pose estimates made by the robot.
    *
    * The full update depends on the construction of the SLAM instance, in particular on the localizer, quuee, and
    * generator strategies employed by the algorithm.
    */
    void estimateTopologicalState(void);

    /**
    * bestEstimate retrieves the current best estimate of the topological state per the latest call to
    * estimateTopologicalState.
    */
    TopologicalState bestEstimate(void);

    /////   Debugging methods   /////

    /**
    * sendDebug sends out any debugging information generated during the course of the algorithm's latest update
    */
    void sendDebug(system::DebugCommunicator& communicator);

    /**
    * saveTreeOfMaps saves the TreeOfMaps containing all map hypotheses to a file.
    *
    * \param    filename            Name of the file in which to save the tree of maps
    * \return   True if the tree of maps was saved successfully.
    */
    bool saveTreeOfMaps(const std::string& filename) const;

    /**
    * saveVisitSequence saves the full visit sequence for the mapping process to a file so it can be loaded and
    * examined.
    *
    * \param    filename            Name of the file in which to save the visit sequence
    * \return   True if the visit sequence was saved successfully.
    */
    bool saveVisitSequence(const std::string& filename) const;

    /**
    * saveMapCache saves the map cache to the requested file.
    *
    * \param    filename            Name of the file in which to save the map cache
    * \return   True if the map cache was saved successfully.
    */
    bool saveMapCache(const std::string& filename) const;

private:

    TopologicalVisitSequence visits_;           // Sequence of areas visited by the robot
    TreeOfMaps tree_;                           // all map states contained and estimated
    ProbabilityHeuristics heuristics_;          // maintain the heuristics for the depths of the tree
    MetricMapCache mapCache_;                   // cache of all LPMs from events
    std::unique_ptr<TopologicalLocalizer> localizer_;
    std::unique_ptr<GeneratorQueue> queue_;
    std::unique_ptr<HypothesisGeneratorFactory> generatorFactory_;
    std::unique_ptr<MapOptimizer> optimizer_;
    std::unique_ptr<HypothesisProbabilityEvaluator> evaluator_;

    std::ofstream resultsOut_;

    void initializeRoot(void);
    void searchForNextState(void);
    bool enqueueState(TopologicalState* state);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_TOPO_SLAM_H
