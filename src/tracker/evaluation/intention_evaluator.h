/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* @file
* @author   Collin Johnson
*
* Declaration of IntentionEvaluator.
*/

#ifndef TRACKER_EVALUATION_INTENTION_EVALUATOR_H
#define TRACKER_EVALUATION_INTENTION_EVALUATOR_H

#include "tracker/dynamic_object.h"
#include "tracker/goal.h"
#include "hssh/local_topological/local_topo_map.h"
#include "core/pose.h"
#include <map>
#include <vector>

namespace vulcan
{
namespace tracker
{

/**
 * Maintain the intention estimates for all destinations while continuously moving in a single area. Later visits will
 * should be stored as a separate container for estimates.
 */
class AreaIntentionEstimates
{
public:

    struct Estimate
    {
        pose_t objPose;
        std::vector<double> destProbs;
        int maxProbIndex;
    };

    using const_iterator = std::vector<Estimate>::const_iterator;

    /**
    * Create storage for intention estimates for the given area with the initial goal distribution.
    *
    * @param    areaId                  Id of the area being investigated
    * @param    distribution            Initial goal distribution
    */
    AreaIntentionEstimates(int areaId, const ObjectGoalDistribution& distribution);

    /**
    * Add a new sample measurement to the estimates.
    *
    * @param    object          New object measurement
    */
    void addSample(const DynamicObject& object);

    /**
    * Save the goal estimate information to a file.
    *
    * The format is:
    *
    *  dest_0_prob dest_1_prob ... dest_N_prob
    *
    * Space-separated probability values for each loading into plotting utilities.
    *
    * @param   filename        Filename in which to store the estimate information
    */
    void saveToFile(const std::string& filename) const;

    /**
    * Retrieve the id of the area associated with the intention estimates.
    */
    int areaId(void) const { return areaId_; }

    /**
    * Retrieve the destinations associated with the area.
    */
    std::vector<ObjectDestination> destinations(void) const { return destinations_; }

    // Iterate through the stored estimates
    bool empty(void) const { return estimates_.empty(); }
    const_iterator begin(void) const { return estimates_.begin(); }
    const_iterator end(void) const { return estimates_.end(); }

private:

    int areaId_;                                    // id of the area in the local topo map
    std::vector<ObjectDestination> destinations_;   // actual destinations
    std::vector<Estimate> estimates_;               // state estimates for each sample
};

/**
* IntentionEvaulator maintains samples of goal estimates for the trajectory of a dynamic object. The evaluator segments
* the trajectory per area visited and maintains all intention estimates for each.
*/
class IntentionEvaluator
{
public:

    using const_iterator = std::vector<AreaIntentionEstimates>::const_iterator;

    /**
    * Constructor for IntentionEvaluator.
    *
    * @param    topoMap         Map containing the areas the object is moving through
    */
    IntentionEvaluator(const hssh::LocalTopoMap& topoMap);

    /**
    * Add a sample measurement to the evaluator.
    *
    * @param    object          Measurement of the DynamicObject being evaluated
    */
    void addSample(const DynamicObject& object);

    /**
    * Save all estimates to file. The output filenames are basename_areaid_timeinms.txt.
    *
    * @param    basename        Base filename to use
    */
    void saveEstimates(const std::string& basename) const;

    // Iterate over estimates for the object in the order the areas were visited
    bool empty(void) const { return estimates_.empty(); }
    std::size_t size(void) const { return estimates_.size(); }
    const_iterator begin(void) const { return estimates_.begin(); }
    const_iterator end(void) const { return estimates_.end(); }

private:

    hssh::LocalTopoMap topoMap_;
    std::vector<AreaIntentionEstimates> estimates_;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_EVALUATION_INTENTION_EVALUATOR_H
