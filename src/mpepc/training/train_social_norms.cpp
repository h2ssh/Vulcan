/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <mpepc/training/agent_state.h>
#include <mpepc/cost/social_cost.h>
#include <mpepc/social/social_norm_utils.h>
#include <mpepc/social/topo_situation.h>
#include <mpepc/evaluation/mpepc_log.h>
#include <hssh/local_topological/area.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/areas/serialization.h>
#include <math/boundary.h>
#include <tracker/objects/serialization.h>
#include <utils/algorithm_ext.h>
#include <utils/pose_trace.h>
#include <utils/serialized_file_io.h>
#include <utils/timestamp.h>
#include <boost/range/iterator_range.hpp>
#include <fstream>
#include <map>

using namespace vulcan;
using namespace vulcan::mpepc;

struct training_log_t
{
    std::string logName;
    std::string ltmName;
};

struct training_state_t
{
    int numDistBins;
    int numLateralBins;
    std::vector<TopoSituationResponse> learnedResponses;
    TopoSituationResponse genericResponsePath;
    TopoSituationResponse genericResponsePlace;

    MotionObsMap pathObservations;
    MotionObsMap transObservations;

    int numPathExamples = 0;
    int numPlaceExamples = 0;
};

std::istream& operator>>(std::istream& in, training_log_t& log)
{
    in >> log.logName >> log.ltmName;
    return in;
}


void process_log(const training_log_t& log, training_state_t& state);
void learn_responses_for_time_step(const tracker::DynamicObjectCollection& objects,
                                   const motion_state_t& robotState,
                                   const hssh::LocalTopoMap& topoMap,
                                   training_state_t& state);
void filter_unimportant_agents(const topo_agent_t& agent, std::vector<topo_agent_t>& otherAgents);
double add_path_example(const topo_agent_t& agent,
                        const std::vector<topo_agent_t>& otherAgents,
                        const hssh::LocalTopoMap& topoMap,
                        training_state_t& state);
double add_place_example(const topo_agent_t& agent,
                         const std::vector<topo_agent_t>& otherAgents,
                         const hssh::LocalTopoMap& topoMap,
                         training_state_t& state);
void save_situations(const std::string& filename, const training_state_t& state);


template <class Situation>
void add_situation_distance(const Situation& situation, double normDistance, training_state_t& state);


int main(int argc, char** argv)
{
    if(argc < 5)
    {
        std::cout << "Expected: train_social_norms 'num dist bins' 'num lateral bins'"
                     "'logs file' 'learned norm output file'\n";
        exit(EXIT_FAILURE);
    }

    std::string pobsName = argv[4];
    pobsName += ".pobs";

    std::string tobsName = argv[4];
    tobsName += ".tobs";

    remove(pobsName.c_str());
    remove(tobsName.c_str());

    std::vector<training_log_t> trainingLogs;

    // Load all the logs and their associated LTMs
    std::ifstream logsIn(argv[3]);
    while(!logsIn.eof())
    {
        training_log_t log;
        logsIn >> log;

        // Check that there was enough information about the log
        if(!log.logName.empty() && !log.ltmName.empty())
        {
            trainingLogs.push_back(log);

            // Remove previous data stored with this log
            std::string logPobsName = log.ltmName + ".pobs";
            std::string logTobsName = log.ltmName + ".tobs";
            remove(logPobsName.c_str());
            remove(logTobsName.c_str());
        }
    }

    training_state_t state;
    state.numDistBins = std::atoi(argv[1]);
    state.numLateralBins = std::atoi(argv[2]);

    std::vector<double> initialDists(state.numDistBins, 1.0);
    state.genericResponsePlace = TopoSituationResponse(initialDists, TopoSituationResponse::place);
    state.genericResponsePath = TopoSituationResponse(initialDists, TopoSituationResponse::path);

    int64_t totalTime = 0;

    for(auto& log : trainingLogs)
    {
        std::cout << "Beginning training for " << log.logName << "...\n";
        int64_t startTime = utils::system_time_us();

        process_log(log, state);

        // Save map-specific observations
        save_motion_observations(log.ltmName + ".pobs", state.pathObservations);
        save_motion_observations(log.ltmName + ".tobs", state.transObservations);

        // Save general observations for easier evaluation
        save_motion_observations(pobsName, state.pathObservations);
        save_motion_observations(tobsName, state.transObservations);

        state.pathObservations.clear();
        state.transObservations.clear();

        int64_t duration = utils::system_time_us() - startTime;
        std::cout << "Completed training in " << (duration / 1000) << "ms\n";
        totalTime += duration;
    }

    std::cout << "Total training time:" << (totalTime / 1000) << "ms\n"
              << "Found: " << state.numPathExamples << " path examples.\n"
              << "       " << state.numPlaceExamples << " place examples.\n";

    for(auto& resp : state.learnedResponses)
    {
        resp.normalizeDistribution();
    }

    state.genericResponsePlace.normalizeDistribution();
    state.genericResponsePath.normalizeDistribution();

    std::cout << "Learned responses:\n";
    for(auto& resp : state.learnedResponses)
    {
        std::cout << resp << '\n';
    }

    std::cout << "Generic path:  " << state.genericResponsePath << '\n'
        << "Generic place: " << state.genericResponsePlace << '\n';

    learned_norm_cost_params_t learnedParams;
    learnedParams.defaultResponsePlace = state.genericResponsePlace;
    learnedParams.defaultResponsePath = state.genericResponsePath;
    learnedParams.responses = state.learnedResponses;
    learnedParams.numLateralBins = state.numLateralBins;

    if(!utils::save_serializable_to_file(argv[4], learnedParams))
    {
        std::cerr << "ERROR: Failed to saved the learned model to file: " << argv[4] << '\n';
        exit(EXIT_FAILURE);
    }

    std::string situationName = argv[4];
    situationName += ".situ";
    save_situations(situationName, state);

    return 0;
}


void process_log(const training_log_t& log, training_state_t& state)
{
    MPEPCLog mpepcLog(log.logName);
    hssh::LocalTopoMap topoMap;

    if(!utils::load_serializable_from_file(log.ltmName, topoMap))
    {
        std::cerr << "WARNING: Failed to load " << log.ltmName << " not processing log " << log.logName << '\n';
        return;
    }

    mpepcLog.loadAll();

    for(auto& objects : boost::make_iterator_range(mpepcLog.beginObjects(), mpepcLog.endObjects()))
    {
        auto stateIt = mpepcLog.beginMotionState(objects.timestamp());

        // Must have a motion state to account for robot in order for this state of objects to be valid
        if(stateIt != mpepcLog.endMotionState())
        {
            // Make sure the robot is included in the state because people are reacting to it!
            learn_responses_for_time_step(objects, *stateIt, topoMap, state);
        }
        else
        {
            std::cout << "Forced to ignore dynamic objects at " << objects.timestamp() << '\n';
        }
    }
}


void learn_responses_for_time_step(const tracker::DynamicObjectCollection& objects,
                                   const motion_state_t& robotState,
                                   const hssh::LocalTopoMap& topoMap,
                                   training_state_t& state)
{
    auto agents = find_topo_agents(objects, topoMap);
    auto robotAgent = create_agent_for_robot(robotState, topoMap);

    std::vector<topo_agent_t> otherAgents;

    for(std::size_t n = 0; n < agents.size(); ++n)
    {
        // Check if the agent is moving or not. Don't try learning from things that aren't moving or things that are
        // moving really fast
        double vel = std::sqrt(std::pow(agents[n].state.xVel, 2.0) + std::pow(agents[n].state.yVel, 2.0));
        if((vel < 0.25) || (vel > 2.0))
        {
            continue;
        }

        otherAgents = agents;
        otherAgents.push_back(robotAgent);  // add robot here to avoid it being included in agents that we are
                                            // learning from
        filter_unimportant_agents(agents[n], otherAgents);

        // If on a path segment, then learn that distribution
        if(topoMap.areaWithId(agents[n].areaId)->type() == hssh::AreaType::path_segment)
        {
            double dist = add_path_example(agents[n], otherAgents, topoMap, state);

            if(dist > 0.0)
            {
                ++state.numPathExamples;

                // Store the state of the agent for k-means processing
                state.pathObservations[agents[n].areaId].push_back(agent_state_t(agents[n].state, dist));
            }
        }

        // If about to transition to a new area, then learn the transition distribution
        if(is_about_to_transition(agents[n], topoMap))
        {
            double dist = add_place_example(agents[n], otherAgents, topoMap, state);

            if(dist > 0.0)
            {
                ++state.numPlaceExamples;

                // Store the state of the agent for k-means processing
                state.transObservations[agents[n].areaId].push_back(agent_state_t(agents[n].state, dist));
            }
        }


    }
}


void filter_unimportant_agents(const topo_agent_t& agent, std::vector<topo_agent_t>& otherAgents)
{
    // Only care about other agents within a certain distance, so the agent has had some time to react to
    // their presence
    const double kMaxImportantDist = 5.0;

    // Don't care about self
    utils::erase_remove(otherAgents, agent);

    utils::erase_remove_if(otherAgents, [&agent, kMaxImportantDist](auto& other) {
        return distance_between_points(Point<float>(agent.state.x, agent.state.y),
                                             Point<float>(other.state.x, other.state.y))
            > kMaxImportantDist;
    });
}


double add_path_example(const topo_agent_t& agent,
                        const std::vector<topo_agent_t>& otherAgents,
                        const hssh::LocalTopoMap& topoMap,
                        training_state_t& state)
{
    double normDistance = normalized_position_path(agent, topoMap);

    if((normDistance >= 0.0) && (normDistance <= 1.0))
    {
        state.genericResponsePath.addExample(normDistance);

        PathSituation situation(agent, otherAgents, state.numLateralBins, topoMap);
        add_situation_distance(situation, normDistance, state);
    }
    else
    {
        normDistance = -1.0;    // set negative to show we didn't use it
    }

    return normDistance;
}


double add_place_example(const topo_agent_t& agent,
                         const std::vector<topo_agent_t>& otherAgents,
                         const hssh::LocalTopoMap& topoMap,
                         training_state_t& state)
{
    auto normDistance = normalized_position_gateway(agent, topoMap);

    if((normDistance >= 0.0) && (normDistance <= 1.0))
    {
        // Always update the generic place response
        state.genericResponsePlace.addExample(normDistance);

        PlaceSituation situation(agent, otherAgents, topoMap);
        add_situation_distance(situation, normDistance, state);
    }
    else
    {
        normDistance = -1.0;    // set negative to show we didn't use it
    }

    return normDistance;
}


template <class Situation>
void add_situation_distance(const Situation& situation, double normDistance, training_state_t& state)
{
    // See if this situation matches a known response. Otherwise need to create a new response
    auto respIt = std::find_if(state.learnedResponses.begin(),
                               state.learnedResponses.end(),
                               [&situation](const auto& response) {
        return response.isResponseForSituation(situation);
    });

    if(respIt != state.learnedResponses.end())
    {
        respIt->addExample(normDistance);
    }
    else
    {
        // Initialize all counts at one to avoid zero probabilities
        std::vector<double> initialCount(state.numDistBins, 1.0);
        TopoSituationResponse newResponse(situation, initialCount);
        newResponse.addExample(normDistance);

        state.learnedResponses.push_back(newResponse);
    }
}


void save_situations(const std::string& filename, const training_state_t& state)
{
    std::ofstream out(filename);

    for(auto& response : state.learnedResponses)
    {
        response.saveExamples(out);
    }

    state.genericResponsePath.saveExamples(out);
    state.genericResponsePlace.saveExamples(out);
}
