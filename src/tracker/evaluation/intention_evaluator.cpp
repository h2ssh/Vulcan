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
* Definition of IntentionEvaluator.
*/

#include <tracker/evaluation/intention_evaluator.h>
#include <tracker/dynamic_object_visitor.h>
#include <tracker/objects/rigid.h>
#include <hssh/local_topological/area.h>

namespace vulcan
{
namespace tracker
{

struct PositionVisitor : public tracker::DynamicObjectVisitor
{
    tracker::Position position;

    // tracker::DynamicObjectVisitor interface
    void visitPerson(const tracker::Person& person)
    {
    }

    void visitPivotingObject(const tracker::PivotingObject& door)
    {
    }

    void visitRigid(const tracker::RigidObject& object)
    {
        auto slowState = object.slowMotionState();
        position.x = slowState[0];
        position.y = slowState[1];
    }

    void visitSlidingObject(const tracker::SlidingObject& door)
    {
    }

    void visitUnclassified(const tracker::UnclassifiedObject& object)
    {
    }
};

/////   AreaIntentionEstimates definition   /////

AreaIntentionEstimates::AreaIntentionEstimates(int areaId, const ObjectGoalDistribution& distribution)
: areaId_(areaId)
{
    // Populate the destinations
    for(auto& goal : distribution)
    {
        destinations_.push_back(goal.destination());
    }
}


void AreaIntentionEstimates::addSample(const DynamicObject& object)
{
    // Extract the probability for each estimate
    std::vector<double> destProbs(destinations_.size(), 0.0);
    for(auto& goal : object.goals())
    {
        auto destIt = std::find(destinations_.begin(), destinations_.end(), goal.destination());
        if(destIt == destinations_.end())
        {
            std::cerr << "ERROR: AreaIntentionEstimates: DynamicObject goals don't match area destinations."
                << " Ignorning object measurement.\n";
            return;
        }

        destProbs[std::distance(destinations_.begin(), destIt)] = goal.probability();
    }

    PositionVisitor visitor;
    object.accept(visitor);

    Estimate estimate;
    estimate.destProbs = destProbs;
    estimate.maxProbIndex = std::distance(destProbs.begin(),
                                          std::max_element(destProbs.begin(), destProbs.end()));
    auto state = object.motionState();
    estimate.objPose = pose_t(visitor.position.x, visitor.position.y, std::atan2(state.yVel, state.xVel));

    estimates_.push_back(estimate);
}


void AreaIntentionEstimates::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename);

    for(auto est : estimates_)
    {
        std::copy(est.destProbs.begin(), est.destProbs.end(), std::ostream_iterator<double>(out, " "));
        out << '\n';
    }
}


/////   IntentionEvaluator definition   /////

IntentionEvaluator::IntentionEvaluator(const hssh::LocalTopoMap& topoMap)
: topoMap_(topoMap)
{
}


void IntentionEvaluator::addSample(const DynamicObject& object)
{
    // If new area, create new series of area estimates
    auto area = topoMap_.areaContaining(object.position());

    // Ignore if there's no area associated with the position of the object
    if(!area)
    {
        return;
    }

    if(estimates_.empty() || (estimates_.back().areaId() != area->id()))
    {
        estimates_.emplace_back(area->id(), object.goals());
    }

    // Add the new object state to the estimates
    estimates_.back().addSample(object);
}


void IntentionEvaluator::saveEstimates(const std::string& basename) const
{
    for(auto& est : estimates_)
    {
        if(!est.empty())
        {
            std::ostringstream filename;
            filename << basename << '_' << est.areaId() << '_' << (est.begin()->objPose.timestamp / 1000) << ".txt";
            est.saveToFile(filename.str());
        }
    }
}

} // namespace tracker
} // namespace vulcan
