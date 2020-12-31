/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_event_detector.cpp
 * \author   Collin Johnson
 *
 * Definition of create_area_event_detectors.
 */

#include "hssh/local_topological/event_detection/area_event_detector.h"
#include "hssh/local_topological/event_detection/area_transition_detector.h"
#include "hssh/local_topological/event_detection/path_direction_detector.h"

namespace vulcan
{
namespace hssh
{

std::vector<std::unique_ptr<AreaEventDetector>>
  create_area_event_detectors(const std::vector<std::string>& detectorTypes, const event_detector_params_t& params)
{
    std::vector<std::unique_ptr<AreaEventDetector>> detectors;

    for (auto& type : detectorTypes) {
        if (type == kAreaTransitionDetectorType) {
            detectors.emplace_back(new AreaTransitionDetector(params.transitionParams));
            std::cout << "INFO:create_area_event_detectors: Created AreaTransitionDetector.\n";
        } else if (type == kPathDirectionDetectorType) {
            detectors.emplace_back(new PathDirectionDetector(params.pathDirectionParams));
            std::cout << "INFO:create_area_event_detectors: Created PathDirectionDetector.\n";
        } else {
            std::cerr << "WARNING::create_area_event_detectors: Attempted to create detector of unknown type:" << type
                      << '\n';
        }
    }

    return detectors;
}

}   // namespace hssh
}   // namespace vulcan
