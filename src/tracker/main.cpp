/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_topological/areas/serialization.h"
#include "hssh/local_topological/local_topo_map.h"
#include "system/module.h"
#include "tracker/director.h"
#include "tracker/goal_predictor.h"
#include "tracker/goals/goal_estimator_factory.h"
#include "tracker/motions/classifier.h"
#include "tracker/object_detector.h"
#include "tracker/object_tracker.h"
#include "tracker/objects/object_factory.h"
#include "tracker/tracking/data_association.h"
#include "tracker/tracking/object_factory.h"
#include "utils/command_line.h"
#include "utils/config_file.h"
#include "utils/serialized_file_io.h"
#include <vector>


const std::string kUpdateRateArgument("output-period-ms");
const std::string kDefaultPeriod("40");
const std::string kLocalTopoMapArg("topo-map");
const std::string kMetricMapArg("map");


int main(int argc, char** argv)
{
    using namespace vulcan;
    using namespace vulcan::tracker;

    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, ""});
    arguments.push_back({kUpdateRateArgument,
                         "Update period for output of DynamicObjectCollection in milliseconds. Default = 40",
                         true,
                         kDefaultPeriod});
    arguments.push_back({kLocalTopoMapArg, "LocalTopoMap to use for goal estimation. Default = none", true, ""});

    utils::CommandLine commandLine(argc, argv, arguments);
    if (!commandLine.verify()) {
        exit(-1);
    }

    utils::ConfigFile config(commandLine.configName());

    // Create the dependencies for the Director, which will be passed into the module
    // The complete dependencies are in object_graph_di in docs/design
    object_motion_classifier_params_t classifierParams(config);
    tracking_filter_params_t filterParams(config);
    auto motionClassifier = std::make_unique<ObjectMotionClassifier>(classifierParams, filterParams);

    object_tracker_params_t trackerParams(config);

    auto associationStrategy = create_data_association_strategy(trackerParams.dataAssociationType, config);
    auto trackingObjFactory =
      std::make_unique<TrackingObjectFactory>(std::move(motionClassifier), std::move(associationStrategy));

    auto tracker = std::make_unique<ObjectTracker>(trackerParams, std::move(trackingObjFactory));

    object_detector_params_t detectorParams(config);
    auto detector = std::make_unique<ObjectDetector>(detectorParams);

    gateway_goal_estimator_params_t gatewayParams(config);
    auto goalFactory = std::make_unique<GoalEstimatorFactory>(gatewayParams);
    auto dynamicObjFactory = std::make_unique<DynamicObjectFactory>();
    auto predictor = std::make_unique<GoalPredictor>(std::move(goalFactory), std::move(dynamicObjFactory));

    int32_t outputPeriodMs = std::atoi(commandLine.argumentValue(kUpdateRateArgument).c_str());

    hssh::LocalTopoMap ltm;
    bool haveLTM = commandLine.argumentExists(kLocalTopoMapArg);
    if (haveLTM) {
        haveLTM = utils::load_serializable_from_file(commandLine.argumentValue(kLocalTopoMapArg), ltm);
    }

    hssh::LocalPerceptualMap lpm;
    bool haveLPM = commandLine.argumentExists(kMetricMapArg);
    if (haveLPM) {
        haveLPM = utils::load_serializable_from_file(commandLine.argumentValue(kMetricMapArg), lpm);
    }

    auto director = std::make_unique<ObjectTrackerDirector>(outputPeriodMs,
                                                            std::move(detector),
                                                            std::move(tracker),
                                                            std::move(predictor),
                                                            haveLPM ? &lpm : nullptr,
                                                            haveLTM ? &ltm : nullptr);

    system::Module<ObjectTrackerDirector> module(std::move(director), commandLine, config);
    module.run();

    return 0;
}
