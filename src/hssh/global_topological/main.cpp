/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "system/module.h"
#include "hssh/global_topological/director.h"
#include "hssh/global_topological/params.h"
#include "hssh/global_topological/topo_slam.h"
#include "hssh/global_topological/localization/localizer.h"
#include "hssh/global_topological/mapping/generator_queue.h"
#include "hssh/global_topological/mapping/generator_queue_impl.h"
#include "hssh/global_topological/mapping/hypothesis_generator.h"
#include "hssh/global_topological/mapping/hypothesis_generator_factory.h"
#include "hssh/global_topological/mapping/hypothesis_generator_factory_impl.h"
#include "hssh/global_topological/mapping/likelihood_evaluator.h"
#include "hssh/global_topological/mapping/map_optimizer.h"
#include "hssh/global_topological/mapping/prior_evaluator.h"
#include "hssh/global_topological/mapping/probability_evaluator.h"
#include "hssh/global_topological/utils/event_log_runner.h"
#include "hssh/local_topological/areas/serialization.h"
#include "hssh/local_topological/events/serialization.h"
#include "utils/config_file.h"
#include "utils/command_line.h"
#include "utils/stub.h"
#include <vector>

using namespace vulcan;
using namespace vulcan::hssh;

const std::string kFromLogArg("from-log");
const std::string kInteractiveArg("interactive");
const std::string kPosesArg("poses");
const std::string kEventsArg("events");
const std::string kResultsArg("results");

// Initialize the global_topo_hssh via dependency injection
std::unique_ptr<TopologicalSLAM> create_topo_slam(const utils::ConfigFile& config,
                                                  const utils::CommandLine& commandLine);
std::unique_ptr<GeneratorQueue> create_queue(const utils::ConfigFile& config,
                                                       const utils::CommandLine& commandLine);
std::unique_ptr<HypothesisGeneratorFactory> create_generator(const utils::ConfigFile& config,
                                                             const utils::CommandLine& commandLine);
std::unique_ptr<TopologicalLocalizer> create_localizer(const utils::ConfigFile& config,
                                                       const utils::CommandLine& commandLine);
std::unique_ptr<MapOptimizer> create_optimizer(const utils::ConfigFile& config,
                                               const utils::CommandLine& commandLine);
std::unique_ptr<HypothesisProbabilityEvaluator> create_evaluator(const utils::ConfigFile& config,
                                                                 const utils::CommandLine& commandLine);


int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, ""});
    arguments.push_back({kFromLogArg, "Run the module using the provided stability log, rather than LCM.", true, ""});
    arguments.push_back({kInteractiveArg, "Run the stability log mode interactively, (hit entered to go to next event)", true, ""});
    arguments.push_back({kPosesArg, "Run the module with these poses", true, ""});
    arguments.push_back({kEventsArg, "Run the module with these events", true, ""});
    arguments.push_back({kResultsArg, "Run the module with these events", true, "topo_slam_results.txt"});

    utils::CommandLine commandLine(argc, argv, arguments);
    if(!commandLine.verify())
    {
        return 1;
    }

    utils::ConfigFile config(commandLine.configName());
    auto director = std::make_unique<GlobalTopoDirector>(create_topo_slam(config, commandLine),
                                                         commandLine,
                                                         config);

    if(commandLine.argumentExists(kFromLogArg))
    {
        EventLogRunner runner(commandLine.argumentValue(kFromLogArg));
        runner.run(*director, commandLine.argumentExists(kInteractiveArg));
    }
    else if(commandLine.argumentExists(kPosesArg) && commandLine.argumentExists(kEventsArg))
    {
        EventLogRunner runner(commandLine.argumentValue(kEventsArg), commandLine.argumentValue(kPosesArg));
        runner.run(*director, commandLine.argumentExists(kInteractiveArg));
    }
    else
    {
        system::Module<GlobalTopoDirector> module(std::move(director), commandLine, config);
        module.run();
    }

    return 0;
}


std::unique_ptr<TopologicalSLAM> create_topo_slam(const utils::ConfigFile& config,
                                                  const utils::CommandLine& commandLine)
{
    return std::make_unique<TopologicalSLAM>(create_localizer(config, commandLine),
                                             create_queue(config, commandLine),
                                             create_generator(config, commandLine),
                                             create_optimizer(config, commandLine),
                                             create_evaluator(config, commandLine),
                                             commandLine.argumentValue(kResultsArg));
}


std::unique_ptr<GeneratorQueue> create_queue(const utils::ConfigFile& config,
                                             const utils::CommandLine& commandLine)
{
    generator_queue_params_t params(config);

    if(params.queueType == kExhaustiveGeneratorQueueType)
    {
        return std::make_unique<ExhaustiveGeneratorQueue>();
    }
    else if(params.queueType == kLazyGeneratorQueueType)
    {
        return std::make_unique<LazyGeneratorQueue>();
    }

    std::cerr << "ERROR: create_queue: Unknown queue type: " << params.queueType << " Known types:\n"
        << '\t' << kExhaustiveGeneratorQueueType << '\n'
        << '\t' << kLazyGeneratorQueueType << '\n';
    return std::unique_ptr<GeneratorQueue>();
}


std::unique_ptr<HypothesisGeneratorFactory> create_generator(const utils::ConfigFile& config,
                                                             const utils::CommandLine& commandLine)
{
    hypothesis_generator_params_t params(config);

    if(params.generatorType == kExhaustiveGeneratorType)
    {
        return std::make_unique<ExhaustiveGeneratorFactory>();
    }
    else if(params.generatorType == kLazyGeneratorType)
    {
        return std::make_unique<LazyGeneratorFactory>();
    }

    std::cerr << "ERROR: create_generator: Unknown generator type: " << params.generatorType << " Known types:\n"
        << '\t' << kExhaustiveGeneratorType << '\n'
        << '\t' << kLazyGeneratorType << '\n';
    return std::unique_ptr<HypothesisGeneratorFactory>();
}


std::unique_ptr<TopologicalLocalizer> create_localizer(const utils::ConfigFile& config,
                                                       const utils::CommandLine& commandLine)
{
    // NOTE: Only one type of TopologicalLocalizer exists at the moment.
    return std::make_unique<TopologicalLocalizer>();
}


std::unique_ptr<MapOptimizer> create_optimizer(const utils::ConfigFile& config,
                                               const utils::CommandLine& commandLine)
{
    map_optimizer_params_t params(config);
    return create_map_optimizer(params.type, params);
}


std::unique_ptr<HypothesisProbabilityEvaluator> create_evaluator(const utils::ConfigFile& config,
                                                                 const utils::CommandLine& commandLine)
{
    hypothesis_probability_evaluator_params_t params(config);

    std::string evaluatorString = params.likelihoodEvaluators;
    auto commaPosition   = evaluatorString.find(',');
    std::string evaluatorType;

    std::vector<std::unique_ptr<HypothesisLikelihoodEvaluator>> likelihoodEvaluators;

    std::cout<<"INFO:HypothesisProbabilityEvaluator:Loading likelihood evaluators:\n";
    while(evaluatorString.length() > 0)
    {
        evaluatorType = evaluatorString.substr(0, commaPosition);
        likelihoodEvaluators.push_back(create_hypothesis_likelihood_evaluator(evaluatorType, params));

        std::cout << evaluatorType << '\n';

        if(commaPosition != std::string::npos)
        {
            evaluatorString = evaluatorString.substr(commaPosition+1);
            commaPosition = evaluatorString.find(',', commaPosition+1);
        }
        else
        {
            break;
        }
    }

    return std::make_unique<HypothesisProbabilityEvaluator>(std::move(likelihoodEvaluators),
                                                            create_hypothesis_prior_evaluator(params.priorEvaluator,
                                                                                              params));
}
