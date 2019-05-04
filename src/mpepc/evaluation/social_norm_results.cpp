/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** @file
* @author   Collin Johnson
*
* social_norm_results is a program to pull information from one or more MPEPC logs and produce plots and figures that
* evaluate the success of the social norms planning vs. regular MPEPC.
*
* social_norm_results requires specifying a file with the logs to process -- one per line -- format:
*
*   'social'/'regular' log_name local_topo_map
*
* The produced results are:
*
*   - Plot of the histogram over normalized dist for path travel
*   - Plot of the histogram over normalized dist for area transitions
*   - File containing:
*       - stats for path interactions
*       - stats for place interactions
*       - stats for overall interactions
*
* Results are produced for social vs. regular logs.
*/

#include <mpepc/evaluation/interaction.h>
#include <mpepc/evaluation/mpepc_log.h>
#include <mpepc/evaluation/social_force.h>
#include <mpepc/evaluation/utils.h>
#include <mpepc/social/social_norm_utils.h>
#include <mpepc/social/topo_situation.h>
#include <mpepc/training/agent_state.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/areas/serialization.h>
#include <core/angle_functions.h>
#include <utils/histogram.h>
#include <utils/histogram_2d.h>
#include <math/t_test.h>
#include <math/z_test.h>
#include <utils/command_line.h>
#include <utils/serialized_file_io.h>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <boost/tuple/tuple.hpp>
#include <gnuplot-iostream.h>
#include <algorithm>
#include <iostream>
#include <cstdlib>


using namespace vulcan;
using namespace vulcan::mpepc;

using namespace boost::accumulators;
using StatsAcc = accumulator_set<double, stats<tag::min,
                                               tag::max,
                                               tag::mean,
                                               tag::variance,
                                               tag::median>>;

const std::string kDistArg("max-dist");
const std::string kConeArg("cone-angle");
const std::string kLogsArg("logs");
const std::string kTrainArg("train");

const int kNumHistBins = 20;


struct StatResults
{
    std::string name;

    double min;
    double max;
    double mean;
    double stdDev;
    double median;
};

struct SituationResults
{
    TopoSituationResponse situation;
    StatsAcc forceAcc;
    StatsAcc blameAcc;
    StatsAcc distPassingAcc;
    StatsAcc speedPassingAcc;

    int leftPassCount = 0;
    int rightPassCount = 0;
};

struct VersionResults
{
    utils::Histogram pathDist;
    utils::Histogram gwyDist;

    utils::Histogram2D pathInteractions;

    utils::Histogram pathDistAgent;
    utils::Histogram gwyDistAgent;

    std::vector<SituationResults> situations;

    SituationResults pathResults;
    SituationResults oncomingResults;
    SituationResults overtakingResults;
    SituationResults beingPassedResults;

    std::string basename;
    std::string version;

    VersionResults(void)
    : pathDist(0.0, 1.0, kNumHistBins)
    , gwyDist(0.0, 1.0, kNumHistBins)
    , pathInteractions(0.0, 1.0, 0.0, 1.0, kNumHistBins)
    , pathDistAgent(0.0, 1.0, kNumHistBins)
    , gwyDistAgent(0.0, 1.0, kNumHistBins)
    {
    }
};

struct OverallResults
{
    VersionResults socialResults;
    VersionResults regularResults;

    MotionObs agentObsPath;
    MotionObs agentObsTrans;
};


void add_distance_results(const ResultsLog& log, VersionResults& results);
void add_interaction_results(const ResultsLog& log,
                             double maxDistance,
                             double ignoreConeAngle,
                             VersionResults& results);
void add_motion_interaction_results(const std::vector<interaction_t>& interactions,
                                    const hssh::LocalTopoMap& topoMap,
                                    VersionResults& results);
void add_social_forces_results(const std::vector<interaction_t>& interactions, VersionResults& results);
std::vector<passing_object_t> prune_passing_events(const std::vector<passing_object_t>& passes);
void accumulate_pass_results(const std::vector<passing_object_t>& passes, SituationResults& results);

StatResults populate_stat_results(StatsAcc& acc, const std::string& name);

void produce_results(OverallResults& results);
void produce_path_dist_results(OverallResults& results);
void produce_gwy_dist_results(OverallResults& results);
void produce_interaction_results(VersionResults& results);
void produce_situation_results(SituationResults& results, const std::string& name);
SituationResults* find_situation(const interaction_t& interaction,
                                       std::vector<SituationResults>& situations);
void draw_dist_result_histogram(utils::Histogram& regularHist, utils::Histogram& socialHist, const std::string& name);
void histogram_analysis(utils::Histogram& regularHist,
                        utils::Histogram& socialHist,
                        utils::Histogram& trainingHist,
                        const std::string& name);
void draw_interaction_histogram(utils::Histogram2D& hist, const std::string& title, double maxCbrange);
void passing_analysis(SituationResults& regularPass, SituationResults& socialPass, const std::string& name);
double kl_divergence(const utils::Histogram& from, const utils::Histogram& to);
std::ostream& operator<<(std::ostream& out, const StatResults& results);


int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> args = {
        {kDistArg, "Maximum distance for an interaction (meters)", true, "5"},
        {kConeArg, "Half angle of the ignore cone behind the robot (degrees)", true, "30"},
        {kLogsArg, "File containing the logs to process", false, ""},
        {kTrainArg, "Base name for training *obs and .situ file", false, ""}
    };

    utils::CommandLine cmdLine(argc, argv, args);

    if(!cmdLine.verify())
    {
        cmdLine.printHelp();
        exit(EXIT_FAILURE);
    }

    double maxDistance = std::strtod(cmdLine.argumentValue(kDistArg).c_str(), 0);
    double ignoreConeAngle = wrap_to_pi(std::strtod(cmdLine.argumentValue(kConeArg).c_str(), 0) * M_PI / 180.0);
    ignoreConeAngle = angle_diff_abs(M_PI, ignoreConeAngle);

    auto logs = load_results_logs(cmdLine.argumentValue(kLogsArg));

    OverallResults allResults;
    allResults.socialResults.basename = cmdLine.argumentValue(kLogsArg);
    allResults.socialResults.version = version_to_public_name(MPEPCVersion::social);
    allResults.regularResults.basename = cmdLine.argumentValue(kLogsArg);
    allResults.regularResults.version = version_to_public_name(MPEPCVersion::regular);

    for(auto& log : logs)
    {
        VersionResults* results = nullptr;
        switch(log.version)
        {
        case MPEPCVersion::regular:
            results = &allResults.regularResults;
            break;
        case MPEPCVersion::social:
            results = &allResults.socialResults;
            break;
        case MPEPCVersion::unknown:
            std::cerr << "ERROR: Unknown MPEPC version for log.\n";
            break;
        }

        if(results)
        {
            add_distance_results(log, *results);
            add_interaction_results(log, maxDistance, ignoreConeAngle, *results);
        }
    }

    // Load training observations
    auto trainName = cmdLine.argumentValue(kTrainArg);

    // Check if there are .mobs for the log
    auto observations = load_motion_observations(trainName + ".pobs");

    for(auto& areaToObs : observations)
    {
        boost::push_back(allResults.agentObsPath, boost::as_array(areaToObs.second));
    }

    observations = load_motion_observations(trainName + ".tobs");

    for(auto& areaToObs : observations)
    {
        boost::push_back(allResults.agentObsTrans, boost::as_array(areaToObs.second));
    }

    std::cout << "Found " << allResults.agentObsPath.size() << " path observations and "
        << allResults.agentObsTrans.size() << " transition observations.\n";

    produce_results(allResults);

    return 0;
}


void add_distance_results(const ResultsLog& log, VersionResults& results)
{
    MPEPCLog logData(log.logName);

    const int64_t kChunkDurationUs = 5000000;

    // Process the log in chunks to ensure we don't run out of memory trying to load the big planning logs

    for(int64_t startTimeUs = 0; startTimeUs < logData.durationUs(); startTimeUs += kChunkDurationUs)
    {
        logData.loadTimeRange(startTimeUs, startTimeUs + (kChunkDurationUs * 2));

        for(auto& state : boost::make_iterator_range(logData.beginMotionState(startTimeUs),
                                                     logData.endMotionState(startTimeUs + kChunkDurationUs)))
        {
            auto agent = create_agent_for_robot(state, log.map);

            // If about to transition, process as a gateway distance
            if(is_about_to_transition(agent, log.map, 0.25))
            {
                auto distance = normalized_position_gateway(agent, log.map);
                if(distance >= 0.0)
                {
                    results.gwyDist.addValue(distance);
                }
            }
            // If on a path, also a path distance here
            if(log.map.pathSegmentWithId(agent.areaId))
            {
                auto distance = normalized_position_path(agent, log.map);
                if(distance >= 0.0)
                {
                    results.pathDist.addValue(distance);
                }
            }
        }
    }
}


void add_interaction_results(const ResultsLog& log,
                             double maxDistance,
                             double ignoreConeAngle,
                             VersionResults& results)
{
    MPEPCLog logData(log.logName);
    // TODO: Make lateral bins a parameter?
    auto interactions = find_interactions(logData, log.map, 3, maxDistance, ignoreConeAngle);

    add_motion_interaction_results(interactions, log.map, results);
    add_social_forces_results(interactions, results);
}


void add_motion_interaction_results(const std::vector<interaction_t>& interactions,
                                    const hssh::LocalTopoMap& topoMap,
                                    VersionResults& results)
{
    // For every other agent in the environment that the robot interacts with, save the results of their behavior
    for(auto& interaction : interactions)
    {
        double robotDistance = normalized_position_path(interaction.robotAgent, topoMap);

        for(auto& agent : interaction.agents)
        {
            double vel = std::sqrt(std::pow(agent.state.xVel, 2.0) + std::pow(agent.state.yVel, 2.0));

            if((vel < 0.25) || (vel > 2.0))
            {
                continue;
            }

            // If about to transition, process as a gateway distance
            if(is_about_to_transition(agent, topoMap, 0.25))
            {
                auto distance = normalized_position_gateway(agent, topoMap);
                if(distance >= 0.0)
                {
                    results.gwyDistAgent.addValue(distance);
                }
            }
            // If on a path, also a path distance
            if(topoMap.pathSegmentWithId(agent.areaId))
            {
                auto distance = normalized_position_path(agent, topoMap);
                if(distance >= 0.0)
                {
                    results.pathDistAgent.addValue(distance);

                    results.pathInteractions.addValue(distance, robotDistance);
                }
            }
        }
    }
}


void add_social_forces_results(const std::vector<interaction_t>& interactions, VersionResults& results)
{
    social_forces_params_t params;  // Use defaults, which are taken from Ferrer's paper.
    auto forces = trajectory_social_forces(interactions, params);

    std::vector<passing_object_t> pathPasses;
    std::vector<passing_object_t> oncomingPasses;
    std::vector<passing_object_t> overtakingPasses;
    std::vector<passing_object_t> beingPassedPasses;

    for(std::size_t n = 0; n < forces.size(); ++n)
    {
        auto& f = forces[n];
        auto& interaction = interactions[n];

        SituationResults* situResults = find_situation(interaction, results.situations);

        if(f.isInteracting && interaction.pathSituation)
        {
            results.pathResults.forceAcc(f.force);
            results.pathResults.blameAcc(f.blame);

            if(situResults)
            {
                situResults->forceAcc(f.force);
                situResults->blameAcc(f.blame);
            }
        }

        // Find the closest passing object
        auto minIt = std::min_element(f.passingObj.begin(), f.passingObj.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.passingDist < rhs.passingDist;
        });

        bool isPassing = minIt != f.passingObj.end();

        if(isPassing && interaction.pathSituation)
        {
            // Account for all passes
            boost::push_back(pathPasses, boost::as_array(f.passingObj));

            auto bins = interaction.pathSituation->situation();
            std::size_t towardCount = std::count_if(bins.begin(), bins.end(), [](PathSituation::BinState state) {
                return state == PathSituation::toward;
            });

            // If someone is coming toward us, then it is an oncoming pass
            if(towardCount > 0)
            {
                boost::push_back(oncomingPasses, boost::as_array(f.passingObj));

                results.oncomingResults.forceAcc(f.force);
                results.oncomingResults.blameAcc(f.blame);
            }
            // Otherwise, the relative speed determine who was doing the passing
            else if(minIt->passingSpeed > 0.0)   // Robot was overtaking the agent
            {
                boost::push_back(overtakingPasses, boost::as_array(f.passingObj));

                results.overtakingResults.forceAcc(f.force);
                results.overtakingResults.blameAcc(f.blame);
            }
            // Robot is being passed
            else
            {
                boost::push_back(beingPassedPasses, boost::as_array(f.passingObj));

                results.beingPassedResults.forceAcc(f.force);
                results.beingPassedResults.blameAcc(f.blame);
            }

            if(situResults)
            {
                situResults->distPassingAcc(minIt->passingDist);
                situResults->speedPassingAcc(minIt->passingSpeed);
            }
        }
    }

    pathPasses = prune_passing_events(pathPasses);
    accumulate_pass_results(pathPasses, results.pathResults);
    std::cout << "Found " << pathPasses.size() << " path passes.\n";

    oncomingPasses = prune_passing_events(oncomingPasses);
    accumulate_pass_results(oncomingPasses, results.oncomingResults);
    std::cout << "Found " << oncomingPasses.size() << " oncoming passes.\n";

    overtakingPasses = prune_passing_events(overtakingPasses);
    accumulate_pass_results(overtakingPasses, results.overtakingResults);
    std::cout << "Found " << overtakingPasses.size() << " overtaking passes.\n";

    beingPassedPasses = prune_passing_events(beingPassedPasses);
    accumulate_pass_results(beingPassedPasses, results.beingPassedResults);
    std::cout << "Found " << beingPassedPasses.size() << " being passed passes.\n";
}


std::vector<passing_object_t> prune_passing_events(const std::vector<passing_object_t>& passes)
{
    std::vector<passing_object_t> pruned;

    for(auto passIt = passes.begin(); passIt < passes.end();) // increment happens internally
    {
        // Find the first object not near the
        auto passEnd = std::find_if(passIt, passes.end(), [passIt](const auto& pass) {
            return pass.id != passIt->id;
        });

        // Find the closest point during passing
        auto minIt = std::min_element(passIt, passEnd, [](const auto& lhs, const auto& rhs) {
            return lhs.passingDist < rhs.passingDist;
        });

        auto totalDist = std::accumulate(passIt, passEnd, 0.0, [](double total, const auto& pass) {
            return total + pass.passingDist;
        });

        // Create a summarized view of the pass, where we consider the mean of the passing distance, since boundary
        // estimation is noisy and imprecise
        passing_object_t obj = *minIt;
        obj.passingDist = totalDist / std::distance(passIt, passEnd);

        pruned.push_back(obj);
        passIt = passEnd;   // increment the search
    }

    return pruned;
}


void accumulate_pass_results(const std::vector<passing_object_t>& passes, SituationResults& results)
{
    for(auto& pass : passes)
    {
        results.distPassingAcc(pass.passingDist);
        results.speedPassingAcc(pass.passingSpeed);

        if(pass.passingSide == math::RectSide::left)
        {
            ++results.leftPassCount;
        }
        else if(pass.passingSide == math::RectSide::right)
        {
            ++results.rightPassCount;
        }
    }
}


StatResults populate_stat_results(StatsAcc& acc, const std::string& name)
{
    StatResults results;

    results.name = name;
    results.min = min(acc);
    results.max = max(acc);
    results.mean = mean(acc);
    results.stdDev = std::sqrt(variance(acc));
    results.median = median(acc);

    return results;
}


void produce_results(OverallResults& results)
{
    produce_path_dist_results(results);
    produce_gwy_dist_results(results);

    produce_interaction_results(results.socialResults);
    produce_interaction_results(results.regularResults);

    passing_analysis(results.regularResults.pathResults,
                     results.socialResults.pathResults,
                     "Analysis of All Passing");

    passing_analysis(results.regularResults.oncomingResults,
                     results.socialResults.oncomingResults,
                     "Analysis of Oncoming Passing");

    passing_analysis(results.regularResults.overtakingResults,
                     results.socialResults.overtakingResults,
                     "Analysis of Overtaking Passing");

    results.regularResults.pathInteractions.normalize();
    results.socialResults.pathInteractions.normalize();
    double maxInteraction = std::max(results.regularResults.pathInteractions.max(),
                                     results.socialResults.pathInteractions.max());

    draw_interaction_histogram(results.regularResults.pathInteractions,
                               results.regularResults.version + " Robot-Pedestrian Path Interaction Locations",
                               maxInteraction);
    draw_interaction_histogram(results.socialResults.pathInteractions,
                               results.socialResults.version + " Robot-Pedestrian Path Interaction Locations",
                               maxInteraction);
}


void produce_path_dist_results(OverallResults& results)
{
    utils::Histogram trainingData(0.0, 1.0, kNumHistBins);
    for(auto& obs : results.agentObsPath)
    {
        trainingData.addValue(obs.normDist);
    }

    draw_dist_result_histogram(results.regularResults.pathDist,
                                results.socialResults.pathDist,
                                "Distribution of Lateral Path Position");
    // Draw twice because it often fails the first time, since it is the first Gnuplot instance to open
    draw_dist_result_histogram(results.regularResults.pathDist,
                                results.socialResults.pathDist,
                                "Distribution of Lateral Path Position");
    histogram_analysis(results.regularResults.pathDist, results.socialResults.pathDist, trainingData,
                       "Analysis of Path Distance");

    if((results.regularResults.pathDistAgent.numValues() > 0)
        && (results.socialResults.pathDistAgent.numValues() > 0))
    {
        std::cout << "Found " << results.regularResults.pathDistAgent.numValues() << " regular path agents and "
            << results.socialResults.pathDistAgent.numValues() << " social path agents.\n";

        draw_dist_result_histogram(results.regularResults.pathDistAgent,
                                   results.socialResults.pathDistAgent,
                                   "Distribution of Lateral Path Position for Pedestrians");
        histogram_analysis(results.regularResults.pathDistAgent, results.socialResults.pathDistAgent, trainingData,
                           "Analysis of Agent Path Distance");

    }
}


void produce_gwy_dist_results(OverallResults& results)
{
    utils::Histogram trainingData(0.0, 1.0, kNumHistBins);

    if(!results.agentObsTrans.empty())
    {

        for(auto& obs : results.agentObsTrans)
        {
            trainingData.addValue(obs.normDist);
        }
    }

    draw_dist_result_histogram(results.regularResults.gwyDist,
                                results.socialResults.gwyDist,
                                "Distribution of Lateral Transition Position");
    histogram_analysis(results.regularResults.gwyDist, results.socialResults.gwyDist, trainingData,
                       "Analysis of Transition Distance");

    if((results.regularResults.gwyDistAgent.numValues() > 0)
        && (results.socialResults.gwyDistAgent.numValues() > 0))
    {
        std::cout << "Found " << results.regularResults.gwyDistAgent.numValues() << " regular gateway agents and "
            << results.socialResults.gwyDistAgent.numValues() << " social gateway agents.\n";

        draw_dist_result_histogram(results.regularResults.gwyDistAgent,
                                   results.socialResults.gwyDistAgent,
                                   "Distribution of Lateral Transition Position for Pedestrians");
        histogram_analysis(results.regularResults.gwyDistAgent, results.socialResults.gwyDistAgent, trainingData,
                           "Analysis of Agent Transition Distance");
    }
}


void produce_interaction_results(VersionResults& results)
{
    produce_situation_results(results.pathResults, results.version + " path passing");
    produce_situation_results(results.oncomingResults, results.version + " oncoming passing");
    produce_situation_results(results.overtakingResults, results.version + " overtaking passing");
    produce_situation_results(results.beingPassedResults, results.version + " being passed passing");

//     for(auto& situation : results.situations)
//     {
//         std::cout << results.version << " -- " << situation.situation << " :"
//             << " count: " << count(situation.forceAcc) << "\n"
//             << populate_stat_results(situation.forceAcc, "Force") << '\n'
//             << populate_stat_results(situation.blameAcc, "Blame") << '\n';
//
//         if(count(situation.distPassingAcc) > 0)
//         {
//             std::cout << "Passing count: " << count(situation.distPassingAcc) << '\n'
//                 << populate_stat_results(situation.distPassingAcc, "Passing Distance (m)") << '\n'
//                 << populate_stat_results(situation.speedPassingAcc, "Passing Speed (m/s)") << '\n';
//         }
//     }
}


void produce_situation_results(SituationResults& results, const std::string& name)
{
    auto forceStats = populate_stat_results(results.forceAcc, "Force");
    auto blameStats = populate_stat_results(results.blameAcc, "Blame");
    auto distPassingStats = populate_stat_results(results.distPassingAcc, "Passing Distance (m)");
    auto speedPassingStats = populate_stat_results(results.speedPassingAcc, "Passing Speed (m/s)");

    std::cout << "\n==========   " << name << "   ==========\n\n"
        << forceStats << '\n'
        << blameStats << '\n';

    if(count(results.distPassingAcc) > 0)
    {
        std::cout << "Passing count:" << count(results.distPassingAcc) << '\n'
            << distPassingStats << '\n'
            << speedPassingStats << '\n';
    }

    int totalPass = results.leftPassCount + results.rightPassCount;
    if(totalPass > 0)
    {
        std::cout << "Left pass %:  " << (results.leftPassCount / static_cast<double>(totalPass)) << '\n'
            << "Right pass %: " << (results.rightPassCount / static_cast<double>(totalPass)) << '\n';
    }
}


SituationResults* find_situation(const interaction_t& interaction, std::vector<SituationResults>& situations)
{
    if(!interaction.pathSituation && !interaction.placeSituation)
    {
        return nullptr;
    }

    // See if this situation matches something already
    for(auto& s : situations)
    {
        if(interaction.pathSituation && s.situation.isResponseForSituation(*interaction.pathSituation))
        {
            return &s;
        }
        else if(interaction.placeSituation && s.situation.isResponseForSituation(*interaction.placeSituation))
        {
            return &s;
        }
    }

    // Otherwise, add a new situation
    SituationResults r;
    std::vector<double> nullDist(2, 0.5);   // don't actually care about these
    if(interaction.pathSituation)
    {
        r.situation = TopoSituationResponse(*interaction.pathSituation, nullDist);
    }
    else
    {
        assert(interaction.placeSituation);
        r.situation = TopoSituationResponse(*interaction.placeSituation, nullDist);
    }

    situations.push_back(r);
    return &situations.back();
}


void draw_dist_result_histogram(utils::Histogram& regularHist, utils::Histogram& socialHist, const std::string& name)
{
    Gnuplot plot;
    plot << "set yrange [0:25]\n";
    plot << "set xrange [0:1]\n";
    plot << "set style fill solid border lc black\n";
    plot << "set title '" << name << "'\n";
    plot << "set xlabel 'Normalized Dist'\n";
    plot << "set ylabel '% of trajectory'\n";

    std::vector<uint32_t> rawColors = {
        (255U << 24) | (135U << 16) | (206U << 8) | (235U),
        (255U << 24) | (255U << 16) | (99U << 8) | (71U),
        (255U << 24) | (106U << 16) | (90U << 8) | (205U),
    };

    std::vector<std::string> colors;
    for(auto& c : rawColors)
    {
        std::ostringstream str;
        str << '#' << std::hex << ((c >> 16) & 0xFF) << ((c >> 8) & 0xFF) << (c  & 0xFF);
        colors.push_back(str.str());
    }

    const std::string lineStyle(" lw 2 pi 1 ps 1");

    plot << "plot "
        << "'-' using 2:1 with lp lt 5 lc rgb \"" << colors[0] << "\"" << lineStyle
        << " title '" << version_to_public_name(MPEPCVersion::regular) << "'"
        << ", '-' using 2:1 with lp lt 7 dt 2 lc rgb \"" << colors[1] << "\"" << lineStyle
        << " title '" << version_to_public_name(MPEPCVersion::social) << "'";
    plot << std::endl;

    // Data is col 1 = value, col 2 = tic value
    std::vector<double> regularVals;
    std::vector<double> socialVals;
    std::vector<double> xtics;

    regularHist.normalize();
    for(auto& bin : regularHist)
    {
        regularVals.push_back(bin.count * 100);
        xtics.push_back(bin.minValue);
    }

    socialHist.normalize();
    xtics.clear();
    for(auto& bin : socialHist)
    {
        socialVals.push_back(bin.count * 100);
        xtics.push_back(bin.minValue);
    }

    plot.send1d(boost::make_tuple(regularVals, xtics));
    plot.send1d(boost::make_tuple(socialVals, xtics));

    sleep(1);
}


void histogram_analysis(utils::Histogram& regularHist,
                        utils::Histogram& socialHist,
                        utils::Histogram& trainingHist,
                        const std::string& name)
{
    regularHist.normalize();
    socialHist.normalize();
    trainingHist.normalize();

    auto regularDist = regularHist.toGaussian();
    auto socialDist = socialHist.toGaussian();

    math::t_test_sample_t sampleRegular = {
        regularDist.mean(),
        regularDist.variance(),
        static_cast<int>(regularHist.numValues())
    };

    math::t_test_sample_t sampleSocial = {
        socialDist.mean(),
        socialDist.variance(),
        static_cast<int>(socialHist.numValues())
    };

    auto tTest = math::independent_t_test(sampleSocial, sampleRegular, 0.05);

    std::cout << "\n\n=====    " << name << "   =====\n"
        << "KLD to regular: " << kl_divergence(trainingHist, regularHist) << '\n'
        << "KLD to social: " << kl_divergence(trainingHist, socialHist) << '\n'
        << "Regular dist (mean, var): " << regularDist.mean() << ',' << regularDist.variance() << '\n'
        << "Social dist (mean, var): " << socialDist.mean() << ',' << socialDist.variance() << '\n';

    // A value being greater than another here means it moves to the right of the other
    std::cout << "T-test results:\n"
        << "Num samples: Social: " << sampleSocial.numSamples << " Regular: " << sampleRegular.numSamples << '\n'
        << "Dists are different:      " << tTest.areDifferent << " p-value:" << tTest.pValueDifferent << " t-value: " << tTest.tValue << '\n'
        << "Social right of regular:  " << tTest.isGreater << " p-value: " << tTest.pValueGreater << " t-value: " << tTest.tValue << '\n'
        << "Social left of regular:   " << tTest.isLess << " p-value: " << tTest.pValueLess << " t-value: " << tTest.tValue << '\n';
    std::cout << "====================\n\n";
}


void draw_interaction_histogram(utils::Histogram2D& hist, const std::string& title, double maxCbrange)
{
    hist.plot(title, "Pedestrian Normalized Path Distance", "Robot Normalized Path Distance", maxCbrange);
}


void passing_analysis(SituationResults& regularPass, SituationResults& socialPass, const std::string& name)
{
    std::cout << "\n\n=====    " << name << "   =====\n";

    math::z_test_sample_t regular = {
        regularPass.leftPassCount,
        regularPass.leftPassCount + regularPass.rightPassCount
    };

    math::z_test_sample_t social = {
        socialPass.leftPassCount,
        socialPass.leftPassCount + socialPass.rightPassCount
    };

    auto zTest = math::binomial_proportion_z_test(social, regular, 0.05);

    // A value being greater than another here means it moves to the right of the other
    std::cout << "Z-test results:\n"
        << "Num samples: Social: " << social.numSamples << " Regular: " << regular.numSamples << '\n'
        << "Dists are different:       " << zTest.areDifferent << " p-value:" << zTest.pValueDifferent << " z-value: " << zTest.zValue << '\n'
        << "Social more left passes:  " << zTest.isGreater << " p-value: " << zTest.pValueGreater << " z-value: " << zTest.zValue << '\n'
        << "Social more right passes: " << zTest.isLess << " p-value: " << zTest.pValueLess << " z-value: " << zTest.zValue << '\n';
    std::cout << "====================\n\n";

    math::t_test_sample_t sampleRegular = {
        mean(regularPass.distPassingAcc),
        variance(regularPass.distPassingAcc),
        static_cast<int>(count(regularPass.distPassingAcc))
    };

    math::t_test_sample_t sampleSocial = {
        mean(socialPass.distPassingAcc),
        variance(socialPass.distPassingAcc),
        static_cast<int>(count(socialPass.distPassingAcc))
    };

    auto tTest = math::independent_t_test(sampleSocial, sampleRegular, 0.05);

    // A value being greater than another here means it moves to the right of the other
    std::cout << "T-test results for passing distance:\n"
        << "Num samples: Social: " << sampleSocial.numSamples << " Regular: " << sampleRegular.numSamples << '\n'
        << "Dists are different:     " << tTest.areDifferent << " p-value:" << tTest.pValueDifferent << " t-value: " << tTest.tValue << '\n'
        << "Social passes further:  " << tTest.isGreater << " p-value: " << tTest.pValueGreater << " t-value: " << tTest.tValue << '\n'
        << "Social passes closer:   " << tTest.isLess << " p-value: " << tTest.pValueLess << " t-value: " << tTest.tValue << '\n';
    std::cout << "====================\n\n";
}


double kl_divergence(const utils::Histogram& from, const utils::Histogram& to)
{
    assert(from.size() == to.size());

    const double kMinProb = 1e-4;

    double kld = 0.0;

    for(std::size_t n = 0; n < from.size(); ++n)
    {
        double fromProb = std::max(from.bin(n).count, kMinProb);
        double toProb = std::max(to.bin(n).count, kMinProb);

        kld += toProb * std::log(toProb / fromProb);
    }

    return kld;
}


std::ostream& operator<<(std::ostream& out, const StatResults& results)
{
    out << std::fixed << std::setprecision(3) << results.name << '\t' << results.min << '\t' << results.max << '\t'
        << results.mean << '\t' << results.stdDev << '\t' << results.median;

    return out;
}
