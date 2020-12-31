/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_topo_explorer.h
 * \author   Collin Johnson
 *
 * Declaration of LocalTopoExplorer.
 */

#ifndef PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORER_H
#define PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORER_H

#include "hssh/local_metric/pose.h"
#include "hssh/local_topological/event.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/location.h"
#include "mpepc/metric_planner/messages.h"
#include "planner/exploration/local_topo/exploration_map.h"
#include "planner/exploration/map_explorer.h"
#include "utils/mutex.h"
#include <atomic>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}
namespace planner
{

const std::string kLocalTopoExplorerType("local_topo");


/**
 * local_topo_explorer_params_t
 */
struct local_topo_explorer_params_t
{
    local_topo_explorer_params_t(void) = default;
    local_topo_explorer_params_t(const utils::ConfigFile& config);
};

/**
 * LocalTopoExplorer explores a LocalTopoMap by visiting every area in the map and crossing every gateway at least once.
 * The exploration approach is as follows:
 *
 *   1) After waking up, drive to a random area in the environment.
 *   2) Start exploring by selecting a random unvisited area and driving there.
 *   3) Keep driving to unvisited areas until they have all been visited.
 *   4) Indicate that exploration is complete.
 *
 * During exploration, the LocalTopoExplorer publishes MetricPlannerTasks for commanding the robot to drive to the
 * defined goals in the map.
 *
 * As debugging output, a LocalTopoExplorationMap will be published to show exploration progress -- where the robot has
 * been, where it still needs to go, and where it is currently going.
 */
class LocalTopoExplorer : public MapExplorer
{
public:
    /**
     * Constructor for LocalTopoExplorer.
     *
     * \param    map         Map to be explored
     * \param    params      Parameters controlling the exploration behavior
     */
    LocalTopoExplorer(const hssh::LocalTopoMap& map, const local_topo_explorer_params_t& params);

    /**
     * Destructor for LocalTopoExplorer.
     */
    virtual ~LocalTopoExplorer(void);

    // MapExplorer interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    void unsubscribeFromData(system::ModuleCommunicator& communicator) override;
    bool hasNewData(void) override;
    bool isFinishedExploring(void) const override;
    void startExploring(system::ModuleCommunicator& communicator) override;
    void continueExploring(system::ModuleCommunicator& communicator) override;
    void stopExploring(system::ModuleCommunicator& communicator) override;

    // Data handlers
    void handleData(const hssh::LocalPose& pose, const std::string& channel);
    void handleData(const hssh::LocalLocation& location, const std::string& channel);
    void handleData(const hssh::LocalAreaEventVec& events, const std::string& channel);
    void handleData(const mpepc::metric_planner_status_message_t& status, const std::string& channel);

private:
    enum State
    {
        waiting_for_initialization,
        driving_to_initial_area,
        exploring_map,
        driving_to_start,
        finished_exploring,
    };

    enum class PlannerTaskStatus
    {
        unknown,
        executing,
        not_executing,
    };

    State state_;
    hssh::LocalPose startPose_;

    hssh::LocalTopoMap map_;
    LocalTopoExplorationMap explorationMap_;
    local_topo_explorer_params_t params_;

    LocalAreaTarget* currentTarget_;
    std::shared_ptr<mpepc::NavigationTask> currentPlannerTask_;
    hssh::LocalPose pose_;
    hssh::LocalLocation location_;
    hssh::LocalAreaEventVec events_;
    std::vector<mpepc::metric_planner_status_message_t>
      taskStatus_;   // can't miss one, as it might be the message needed, so store all status messages

    std::atomic<bool> haveNewData_;
    bool haveExplorationUpdate_;   // Flag indicating if new area was visited or target changed

    utils::Mutex dataLock_;

    // INVARIANT: If state_ == exploring_map, currentTarget_ != nullptr


    void goToTarget(LocalTopoExplorationTarget* target, system::ModuleCommunicator& communicator);
    bool haveReachedCurrentTarget(void);
    PlannerTaskStatus checkPlannerTaskStatus(void);   // -1 = no info available, 0 = no, 1 = yes

    void driveToInitialArea(system::ModuleCommunicator& communicator);
    void exploreMap(system::ModuleCommunicator& communicator);
    void driveToStart(system::ModuleCommunicator& communicator);
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORER_H
