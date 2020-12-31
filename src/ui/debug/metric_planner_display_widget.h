/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     metric_planner_display_widget.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of the MetricPlannerDisplayWidget which is an OpenGL-based widget
 * that handles rendering all state related to the local metric planner.
 */

#ifndef UI_DEBUG_METRIC_PLANNER_DISPAY_WIDGET_H
#define UI_DEBUG_METRIC_PLANNER_DISPAY_WIDGET_H

#include "hssh/local_metric/lpm.h"
#include "mpepc/cost/cost_map.h"
#include "mpepc/cost/social_cost.h"
#include "mpepc/grid/navigation_grid.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "mpepc/grid/visibility_analysis.h"
#include "utils/fixed_duration_buffer.h"
#include "utils/mutex.h"
// #include "mpepc/rrt/debug_rrt.h"
#include "mpepc/metric_planner/messages.h"
#include "mpepc/simulator/dynamic_object_trajectory.h"
#include "mpepc/trajectory/robot_trajectory_info.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "tracker/dynamic_object_collection.h"
#include "ui/common/color_interpolator.h"
#include "ui/common/ui_forward_declarations.h"
#include "ui/common/ui_params.h"
#include "ui/components/grid_based_display_widget.h"
#include "ui/components/rrt_renderer.h"

namespace vulcan
{
namespace hssh
{
class LocalTopoMap;
}
namespace mpepc
{
class RobotSimulator;
class TrajectoryEvaluator;
class TaskManifold;
}   // namespace mpepc

namespace mpepc
{
struct metric_planner_params_t;
}

namespace ui
{

class CellGridAlphaRenderer;
class CostMapRenderer;
class GradientGridRenderer;
class ObjectTrajectoryRenderer;
class OccupancyGridRenderer;
class PathRenderer;
class PoseTraceRenderer;
class PoseTargetRenderer;
class RobotRenderer;
class RobotTrajectoryRenderer;
class TopoSituationRenderer;
class TrackedObjectRenderer;


enum class MPEPCMapType
{
    lpm,
    obstacles,
    costs,
    navigation,
    none,
};

/**
 * MetricPlannerDisplayWidget handles rendering the state of the local metric planner
 * module. The metric planner uses a number of grids for planning. Intermediate state
 * of the planner may be output at some point in the future as well.
 */
class MetricPlannerDisplayWidget : public GridBasedDisplayWidget
{
public:
    /**
     * Constructor for MetricPlannerDisplayWidget.
     */
    MetricPlannerDisplayWidget(wxWindow* parent,
                               wxWindowID id = wxID_ANY,
                               const wxPoint& pos = wxDefaultPosition,
                               const wxSize& size = wxDefaultSize,
                               long style = 0,
                               const wxString& name = wxString((const wxChar*)("GLCanvas")),
                               const wxPalette& palette = wxNullPalette);

    virtual ~MetricPlannerDisplayWidget(void);

    void setWidgetParams(const metric_planner_display_params_t& params, const lpm_display_params_t& lpmParams);

    virtual std::string printCellInformation(Point<int> cell) override;

    // Text updates
    wxArrayString setupCostDescriptions(void);
    size_t getNumTrjEvaluatedToShow(void);
    std::string getEvaluatedCostLabelString(void);
    float getEvaluatedCostToShow(void);

    mpepc::control_law_coordinates_t getControlLawCoords(void) { return controlLawCoords_; };
    mpepc::motion_target_t getMotionTargetCoords(void) { return motionTargetToEvaluate_; };
    float getMotionTargetCostToShow(void) { return trajectoryToMotionTarget_.expectedCost; };

    // Marker updates
    void setHoverDestinationPose(const pose_t& target);
    void setDestinationPose(const pose_t& target);
    void clearHoverDestinationPose(void) { haveHoverDestinationPose_ = false; };
    void clearDestinationPose(void);

    void setHoverMotionTargetPose(const pose_t& target);
    void setMotionTargetPose(const pose_t& target);
    void clearHoverMotionTargetPose(void) { haveHoverMotionTargetPose_ = false; };
    void clearMotionTarget(void);

    void clearTrace(void);

    // Main display options
    void showRobotPose(bool checked) { showRobot_ = checked; };
    void showDestinationPose(bool checked) { showDestination_ = checked; };
    void showTrackedObjects(bool checked) { showTrackedObjects_ = checked; };
    void showEstimatedObjectMotions(bool checked) { showEstimatedObjectMotion_ = checked; };
    void showOptimalPath(bool checked) { showOptimalPath_ = checked; }
    void showVisibilityAnalysis(bool checked) { showVisibilityAnalysis_ = checked; }
    void showTopoSituation(bool checked) { showTopoSituation_ = checked; }

    // MPEPC Options
    void showMap(MPEPCMapType type) { mapType_ = type; }

    void showCurrentTrajectories(void) { trjGroup_ = TRJ_CURRENT; }
    void showOptimalTrajectory(void) { trjGroup_ = TRJ_OPTIMAL; }
    void showExplorationHistory(void) { trjGroup_ = TRJ_HISTORY; }
    void showNoTrajectories(void) { trjGroup_ = TRJ_NONE; }
    void showThreeSecHistory(void) { trjGroup_ = TRJ_THREE_SEC; }
    void showFiveSecHistory(void) { trjGroup_ = TRJ_FIVE_SEC; }

    void setTrajectoryCostToShow(size_t selection)
    {
        trjCostType_ = static_cast<mpepc::robot_trajectory_debug_cost_type_t>(selection);
    };

    void showAllTrajectories(void) { trjDispMode_ = DISP_ALL; }
    void showLastNTrajectories(void) { trjDispMode_ = DISP_LAST_N; }
    void showSingleTrajectory(void) { trjDispMode_ = DISP_SINGLE; }

    void setTrajectoryNumber(size_t num) { trajectoryNumber_ = num; };

    void showMotionTargets(bool checked) { showMotionTargets_ = checked; };
    void overlayRobotPosesOverTrajectory(bool checked) { overlayRobotPosesOverTrajectory_ = checked; };

    // Motion Target Options
    void setMotionTargetR(double coord);
    void setMotionTargetTheta(double coord);
    void setMotionTargetDelta(double coord);
    void setMotionTargetGain(double coord);
    void setMotionTargetK1(double coord);
    void setMotionTargetK2(double coord);

    void evaluateMotionTarget(void);

    // RRT Options
    void showRRT(bool checked) { showRRT_ = checked; };
    void showRRTTarget(bool checked) { showRRTTarget_ = checked; };
    void showRRTPath(bool checked) { showRRTGoalPath_ = checked; };
    void showRRTGoal(bool checked) { showRRTGoalRegion_ = checked; };

    // Script Options
    void setDestinationPoses(const std::vector<pose_t>& waypointPoses);

    // robot state and command data
    void handleData(const motion_state_t& robotState, const std::string& channel);
    void handleData(const robot::commanded_joystick_t& commandedJoystick, const std::string& channel);
    void handleData(const robot::commanded_velocity_t& commandedVelocity, const std::string& channel);
    void handleData(const robot::motion_command_t& motionCommand, const std::string& channel);

    // LPMDataConsumer interface
    void handleData(const pose_t& pose, const std::string& channel);
    void handleData(const hssh::LocalPerceptualMap& occGrid, const std::string& channel);
    void handleData(const hssh::LocalTopoMap& topoMap, const std::string& channel);
    void handleData(const tracker::DynamicObjectCollection& tracked, const std::string& channel);

    // MetricPlannerDataConsumer interface
    void handleData(const mpepc::ObstacleDistanceGrid& distGrid, const std::string& channel);
    void handleData(const mpepc::NavigationGrid& navGrid, const std::string& channel);
    void handleData(const mpepc::CostMap& costMap, const std::string& channel);
    void handleData(const mpepc::VisibilityAnalysis& visibility, const std::string& channel);
    void handleData(const std::vector<mpepc::dynamic_object_trajectory_debug_info_t>& objectTrajectories,
                    const std::string& channel);
    void handleData(const std::vector<mpepc::dynamic_object_trajectory_t>& objectTrajectories,
                    const std::string& channel);
    void handleData(const mpepc::trajectory_planner_debug_info_t& mpepcInfo, const std::string& channel);
    void handleData(const mpepc::metric_planner_status_message_t& plannerStatus, const std::string& channel);
    void handleData(const mpepc::learned_norm_info_t& normInfo, const std::string& channel);
    //     void handleData(const mpepc::rrt_info_t&                               rrtInfo,            const std::string&
    //     channel);
    void handleData(const std::shared_ptr<mpepc::MetricPlannerTask>& task, const std::string& channel);

private:
    MetricPlannerDisplayWidget(const MetricPlannerDisplayWidget& copy) = delete;
    MetricPlannerDisplayWidget& operator=(const MetricPlannerDisplayWidget& rhs) = delete;

    // GridBasedDisplayWidget interface
    void renderWidget(void);
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;

    // Local definitions
    using robot_trajectories_t = std::vector<mpepc::robot_trajectory_debug_info_t>;
    using robot_trajectories_simple_t = std::vector<mpepc::robot_trajectory_debug_info_simple_t>;
    using object_trajectories_t = std::vector<mpepc::dynamic_object_trajectory_debug_info_t>;
    using trj_cost_type_t = mpepc::robot_trajectory_debug_cost_type_t;

    enum trj_group_t
    {
        TRJ_CURRENT,
        TRJ_OPTIMAL,
        TRJ_HISTORY,
        TRJ_NONE,
        TRJ_THREE_SEC,
        TRJ_FIVE_SEC,
    };

    enum trj_disp_mode_t   // sub-option for the SHOW_CURRENT and SHOW_ENTIRE_HISTORY mode.
    {
        DISP_ALL,
        DISP_LAST_N,
        DISP_SINGLE
    };

    struct dynamic_objects_t
    {
        int64_t timestamp;
        std::vector<mpepc::dynamic_object_state_t> states;
    };

    // Methods for data processing
    void sortTrajectories(void);

    // Redering functions
    void associatePlannerStatusToRobotColor(mpepc::metric_planner_status_t status);

    void renderVisibility(void);
    void renderSituations(void);

    void displayTrajectoryPlannerInfo(void);
    void setTrajectoryColors(void);

    void renderEvaluatedTrajectories(const robot_trajectories_t& evaluatedTrajectories);
    void renderEvaluatedTrajectory(const mpepc::robot_trajectory_debug_info_t& evaluatedTrajectory);
    void renderPlannedTrajectory(const mpepc::robot_trajectory_debug_info_t& plannedTajectory);

    void renderEvaluationHistory(const std::deque<robot_trajectories_simple_t>& pastTrajectories);
    void renderRecentHistory(const utils::FixedDurationBuffer<pose_t>& poseHistory,
                             const utils::FixedDurationBuffer<dynamic_objects_t>& objectHistory);

    // Renderers
    std::unique_ptr<OccupancyGridRenderer> occGridRenderer_;
    std::unique_ptr<CellGridAlphaRenderer> distGridRenderer_;
    std::unique_ptr<CostMapRenderer> costMapRenderer_;
    std::unique_ptr<CellGridAlphaRenderer> navGridRenderer_;
    std::unique_ptr<PoseTargetRenderer> targetRenderer_;
    std::unique_ptr<PoseTraceRenderer> traceRenderer_;
    std::unique_ptr<RobotTrajectoryRenderer> robotTrajectoryRenderer_;
    std::unique_ptr<RobotRenderer> robotRenderer_;
    //     std::unique_ptr<RRTRenderer>             rrtRenderer_;
    std::unique_ptr<TrackedObjectRenderer> objectRenderer_;
    std::unique_ptr<PathRenderer> pathRenderer_;
    std::unique_ptr<TopoSituationRenderer> situationRenderer_;

    // Evaluators
    std::unique_ptr<mpepc::RobotSimulator> robotSimulator_;
    std::unique_ptr<mpepc::TrajectoryEvaluator> trajectoryEvaluator_;

    // Mutex for handling data
    utils::Mutex dataLock_;

    // Variables for each option group in the debug ui:
    // Main display options
    bool showRobot_;
    bool showDestination_;
    bool showTrackedObjects_;
    bool showEstimatedObjectMotion_;
    bool showOptimalPath_;
    bool showVisibilityAnalysis_;
    bool showTopoSituation_;

    std::deque<pose_t> robotTrace_;   // trace of past robot motion
    uint16_t maxTraceLength_;

    motion_state_t robotState_;
    std::vector<pose_t> destinationPoses_;
    tracker::DynamicObjectCollection trackedObjects_;
    object_trajectories_t objectTrajectories_;

    std::vector<mpepc::dynamic_object_trajectory_t> objectTrajectoriesWithLaserObjects_;

    // Storage for the snapshot of time view that shows current state plus prior states
    utils::FixedDurationBuffer<dynamic_objects_t> threeSecObjHistory_;
    utils::FixedDurationBuffer<dynamic_objects_t> fiveSecObjHistory_;

    utils::FixedDurationBuffer<pose_t> threeSecRobotHistory_;
    utils::FixedDurationBuffer<pose_t> fiveSecRobotHistory_;

    bool newPlannerStatusMessage_;

    mpepc::metric_planner_status_message_t plannerStatusMessage_;   // controls robot color

    // MPEPC Options
    MPEPCMapType mapType_ = MPEPCMapType::lpm;
    trj_group_t trjGroup_ = TRJ_CURRENT;
    trj_cost_type_t trjCostType_ = mpepc::EXPECTED_COST;
    trj_disp_mode_t trjDispMode_ = DISP_ALL;

    size_t trajectoryNumber_;

    bool showMotionTargets_;
    bool overlayRobotPosesOverTrajectory_;

    bool haveHoverDestinationPose_;
    bool haveSelectedDestinationPose_;

    pose_t hoverDestinationPose_;

    bool newLPM_;
    bool newDistGrid_;
    bool newCostMap_;
    bool newNavGrid_;

    hssh::LocalPerceptualMap lpmGrid_;
    mpepc::ObstacleDistanceGrid distGrid_;
    mpepc::NavigationGrid navGrid_;
    mpepc::CostMap costMap_;
    mpepc::VisibilityAnalysis visibility_;
    std::shared_ptr<hssh::LocalTopoMap> topoMap_;

    bool newTrajectoryPlannerInfo_;

    mpepc::trajectory_planner_debug_info_t mpepcInfo_;
    robot_trajectories_simple_t sortedEvaluatedTrajectories_;
    std::deque<robot_trajectories_simple_t> trajectoryEvaluationHistory_;

    mpepc::learned_norm_info_t situationInfo_;

    float minTrajectoryCost_;
    float maxTrajectoryCost_;

    // Motion Target Options
    bool haveHoverMotionTargetPose_;
    bool haveSelectedMotionTargetPose_;
    bool haveTrajectoryToMotionTarget_;

    pose_t robotPoseForControlLawCoords_;
    mpepc::control_law_coordinates_t controlLawCoords_;
    mpepc::motion_target_t motionTargetToEvaluate_;
    mpepc::robot_trajectory_info_t trajectoryToMotionTarget_;

    pose_t hoverMotionTargetPose_;

    // NOTE: Later I may want to send this out directly, so I don't have to create
    //       unnecessary copies of NavGrid and RRT.
    std::shared_ptr<mpepc::TaskManifold> taskManifold;

    // RRT Options
    bool showRRT_;
    bool showRRTTarget_;
    bool showRRTGoalPath_;
    bool showRRTGoalRegion_;

    bool newRRT_;
    //     mpepc::rrt_info_t rrtInfo_;

    // Script Options
    // nothing for Script Options yet.


    // Colors
    LinearColorInterpolator interpolator_;

    GLColor trajectoryRed_;
    GLColor trajectoryBlue_;
    GLColor trajectoryGreen_;

    // Parameters
    metric_planner_display_params_t params_;
    std::unique_ptr<mpepc::metric_planner_params_t> mpepcParams_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_METRIC_PLANNER_DISPAY_WIDGET_H
