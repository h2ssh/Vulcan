/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     progress_checker.h
 * \author   Jong Jin Park and Collin Johnson
 *
 * Declaration of ProgressChecker.
 */

#ifndef MPEPC_METRIC_PLANNER_PROGRESS_CHECKER_H
#define MPEPC_METRIC_PLANNER_PROGRESS_CHECKER_H

#include "core/motion_state.h"

namespace vulcan
{
namespace mpepc
{

/**
 * ProgressChecker
 */
class ProgressChecker
{
public:
    void reset(void);
    void run(const motion_state_t& state);

    //         bool robotIsNotProgressingForSomeTime(void) { return robotIsStuckCounter_ > 10; };
    bool robotIsNotProgressingForSomeTime(void) { return false; };

private:
    int32_t progressCounter_;
    int32_t robotIsStuckCounter_;
    bool robotIsStuck_;

    velocity_t slowAverage_;
    velocity_t fastAverage_;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_METRIC_PLANNER_PROGRESS_CHECKER_H
