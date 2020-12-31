/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     interaction.cpp
* \author   Collin Johnson
*
* Definition of find_interactions.
*/

#include "mpepc/evaluation/interaction.h"
#include "mpepc/evaluation/mpepc_log.h"
#include "mpepc/social/social_norm_utils.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/local_topo_map.h"
#include "math/interpolation.h"
#include <boost/range/iterator_range.hpp>
#include <tuple>

namespace vulcan
{
namespace mpepc
{

std::tuple<pose_t, velocity_t> motion_state_at_time(int64_t time,
                                                                  MPEPCLog::motion_state_iterator begin,
                                                                  MPEPCLog::motion_state_iterator end);
interaction_t create_interaction(const pose_t& pose,
                                 const velocity_t& velocity,
                                 const tracker::DynamicObjectCollection& objects,
                                 const hssh::LocalTopoMap& topoMap,
                                 int numLateralBins,
                                 double maxDistance,
                                 double ignoreConeAngle);
bool is_interacting_with_object(const pose_t& pose,
                                const tracker::DynamicObject& object,
                                double maxDistance,
                                double ignoreConeAngle);



std::vector<interaction_t> find_interactions(MPEPCLog& log,
                                             const hssh::LocalTopoMap& topoMap,
                                             int numLateralBins,
                                             double maxDistance,
                                             double ignoreConeAngle)
{
    const int64_t kChunkDurationUs = 5000000;

    std::vector<interaction_t> interactions;

    // Process the log in chunks to ensure we don't run out of memory trying to load the big planning logs
    int64_t startTimeUs = 0;

    pose_t pose;
    velocity_t velocity;

    while(startTimeUs < log.durationUs())
    {
        log.loadTimeRange(startTimeUs, startTimeUs + (2 * kChunkDurationUs));

        // The objects are detected at a lower rate than the poses, so for each object collection, create an interaction
        // using the interpolated pose
        for(auto& objs : boost::make_iterator_range(log.beginObjects(startTimeUs),
                                                    log.endObjects(startTimeUs + kChunkDurationUs)))
        {
            std::tie(pose, velocity) = motion_state_at_time(objs.timestamp(),
                                                            log.beginMotionState(),
                                                            log.endMotionState());
            interactions.push_back(create_interaction(pose,
                                                      velocity,
                                                      objs,
                                                      topoMap,
                                                      numLateralBins,
                                                      maxDistance,
                                                      ignoreConeAngle));
        }

        startTimeUs += kChunkDurationUs;
    }

    return interactions;
}


std::tuple<pose_t, velocity_t> motion_state_at_time(int64_t time,
                                                                  MPEPCLog::motion_state_iterator begin,
                                                                  MPEPCLog::motion_state_iterator end)
{
    // If there's nothing to interpolate, then return one of the ends
    assert(begin != end);

    if(time <= begin->timestamp)
    {
        return std::make_tuple(begin->pose, begin->velocity);
    }
    else if(time >= (end - 1)->timestamp)
    {
        return std::make_tuple((end - 1)->pose, (end - 1)->velocity);
    }

    pose_t pose;
    velocity_t velocity;

    pose.timestamp = time;
    velocity.timestamp = time;

    for(auto next = begin + 1; next < end; ++begin, ++next)
    {
        // Find the range to interpolate between.
        if((begin->timestamp <= time) && (time <= next->timestamp))
        {
            // Ensure the timestamps are converted, since MPEPCLog doesn't deep-change the time
            pose_t startPose = begin->pose;
            startPose.timestamp = begin->timestamp;

            pose_t nextPose = next->pose;
            nextPose.timestamp = next->timestamp;

            pose = interpolate_pose(startPose, nextPose, time);

            double scale = static_cast<double>(time - begin->timestamp) / (next->timestamp - begin->timestamp);
            velocity.linear = math::linear_interpolation(begin->velocity.linear, next->velocity.linear, scale);
            velocity.angular = math::linear_interpolation(begin->velocity.angular, next->velocity.angular, scale);
        }
    }

    return std::make_tuple(pose, velocity);
}


interaction_t create_interaction(const pose_t& pose,
                                 const velocity_t& velocity,
                                 const tracker::DynamicObjectCollection& objects,
                                 const hssh::LocalTopoMap& topoMap,
                                 int numLateralBins,
                                 double maxDistance,
                                 double ignoreConeAngle)
{
    interaction_t interaction;
    interaction.timestamp = objects.timestamp();
    interaction.velocity = velocity;
    interaction.pose = pose;

    for(auto obj : objects)
    {
        if(is_interacting_with_object(pose, *obj, maxDistance, ignoreConeAngle))
        {
            if(auto agent = convert_to_topo_agent(obj, topoMap))
            {
                interaction.agents.push_back(*agent);
            }
        }
    }

    auto robotAgent = create_agent_for_robot(motion_state_t(pose, velocity), topoMap);

    if(topoMap.pathSegmentWithId(robotAgent.areaId))
    {
        interaction.pathSituation = PathSituation(robotAgent, interaction.agents, numLateralBins, topoMap);
    }
    else if(is_about_to_transition(robotAgent, topoMap))
    {
        interaction.placeSituation = PlaceSituation(robotAgent, interaction.agents, topoMap);
    }

    interaction.robotAgent = robotAgent;
    interaction.areaId = robotAgent.areaId;

    return interaction;
}


bool is_interacting_with_object(const pose_t& pose,
                                const tracker::DynamicObject& object,
                                double maxDistance,
                                double ignoreConeAngle)
{
    bool is_close_enough = distance_between_points(pose.toPoint(), object.position()) < maxDistance;
    bool is_in_cone = angle_diff_abs(angle_to_point(pose.toPoint(), object.position()), pose.theta)
        < ignoreConeAngle;

    return is_close_enough && is_in_cone;
}


} // namespace mpepc
} // namespace vulcan
