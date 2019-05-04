/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     result.h
* \author   Collin Johnson
*
* Declaration of DecisionResult.
*/

#ifndef PLANNER_DECISION_RESULT_H
#define PLANNER_DECISION_RESULT_H

#include <boost/optional.hpp>
#include <memory>

namespace vulcan
{
namespace system { class SystemCommunicator; }
namespace planner
{

class DecisionStatus;
class DecisionException;
class ControlTask;

/**
* DecisionResult holds the results of a planning update. The result consists of one or more different types of data:
*
*   1) DecisionStatus proides basic diagnostic information on progress of the current task, if any.
*   2) DecisionException is triggered if a task cannot be completed to inform of layers of the failure, if any.
*   3) MetricPlannerTask is sent to execute the decision-level task by telling the robot to drive to the appropriate
*       pose.
*
* The result contains a subset of the above. The result is sent using a SystemCommunicator.
*/
class DecisionResult
{
public:

    /**
    * Default constructor for DecisionResult.
    */
    DecisionResult(void);

    /**
    * Constructor for DecisionResult.
    *
    * \param    status          Status of the DecisionPlanner
    * \param    exception       Exception generated for another level
    * \param    task            Task to be sent to the result
    */
    DecisionResult(const boost::optional<DecisionStatus>& status,
                   const boost::optional<DecisionException>& exception,
                   std::unique_ptr<ControlTask> task);

    /**
    * Destructor for DecisionResult.
    *
    * For unique_ptr
    */
    ~DecisionResult(void);

    // Moveable
    DecisionResult(DecisionResult&& rhs);
    DecisionResult& operator=(DecisionResult&& rhs);

    // No copying
    DecisionResult& operator=(const DecisionResult&) = delete;
    DecisionResult(const DecisionResult&)            = delete;

    /**
    * send sends off the result of a using the provided communicator.
    *
    * \param    communicator            Communicator for sending the result
    */
    void send(system::SystemCommunicator& communicator) const;

private:

    boost::optional<DecisionStatus> status_;
    boost::optional<DecisionException> exception_;
    std::unique_ptr<ControlTask> task_;
};

}
}

#endif // PLANNER_DECISION_RESULT_H
