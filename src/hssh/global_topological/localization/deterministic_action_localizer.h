/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     deterministic_action_localizer.h
* \author   Collin Johnson
*
* Declaration of DeterministicActionLocalizer.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_DETERMINISTIC_ACTION_LOCALIZER_H
#define HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_DETERMINISTIC_ACTION_LOCALIZER_H

#include <hssh/global_topological/localization/localizer.h>
#include <string>

namespace vulcan
{
namespace hssh
{

const std::string kDeterministicActionLocalizerType("deterministic-action");

/**
* DeterministicActionLocalizer is the simplest topological localization strategy.
* Every action taken is assumed to be correctly executed. The robot will always
* exit onto the indicated path. When the robot enters a place, it enters on the
* correct path. If on a frontier path, then indeed the robot has entered a frontier.
*/
class DeterministicActionLocalizer : public TopologicalLocalizer
{
public:

    /**
    * Constructor for DeterministicActionLocalizer.
    */
    DeterministicActionLocalizer(void);
    
    // TopologicalLocalizer interface
    GlobalLocationDistribution localize(const TopologicalState& state, const TopologicalVisit& visit) override;

private:

    GlobalLocationDistribution handleEnteredEvent(const TopologicalState& state,
                                                  const TopologicalVisit& visit);
    
    GlobalLocationDistribution handleExitedEvent(const TopologicalState& state,
                                                 const TopologicalVisit& visit);
    
    GlobalLocationDistribution handleTurnAroundEvents(const TopologicalState& state,
                                                      const TopologicalVisit& visit);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_DETERMINISTIC_ACTION_LOCALIZER_H
