/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_planner_display_widget.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of MetricPlannerDisplayWidget.
*/

#include <ui/debug/metric_planner_display_widget.h>
#include <ui/common/ui_params.h>
#include <ui/common/gl_shapes.h>
#include <ui/common/default_colors.h>
#include <ui/components/cell_grid_renderer.h>
#include <ui/components/cost_map_renderer.h>
#include <ui/components/gradient_grid_renderer.h>
#include <ui/components/occupancy_grid_renderer.h>
#include <ui/components/path_renderer.h>
#include <ui/components/pose_target_renderer.h>
#include <ui/components/pose_trace_renderer.h>
#include <ui/components/robot_trajectory_renderer.h>
#include <ui/components/robot_renderer.h>
#include <ui/components/rrt_renderer.h>
#include <ui/components/topo_situation_renderer.h>
#include <ui/components/tracked_object_renderer.h>
#include <hssh/local_topological/local_topo_map.h>
#include <mpepc/grid/navigation_grid_utils.h>
#include <mpepc/metric_planner/params.h>
#include <mpepc/metric_planner/task/navigation.h>
#include <mpepc/manifold/navigation.h>
#include <mpepc/simulator/robot_simulator.h>
#include <mpepc/trajectory/trajectory_evaluator.h>
#include <robot/model/params.h>
#include <utils/auto_mutex.h>
#include <utils/config_file.h>
#include <boost/range/iterator_range.hpp>
#include <algorithm>
#include <iostream>

namespace vulcan
{

namespace ui
{

MetricPlannerDisplayWidget::MetricPlannerDisplayWidget(wxWindow* parent,
                                                       wxWindowID id,
                                                       const wxPoint& pos,
                                                       const wxSize& size,
                                                       long style,
                                                       const wxString& name,
                                                       const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, occGridRenderer_(new OccupancyGridRenderer)
, distGridRenderer_(new CellGridAlphaRenderer(250, true))
, costMapRenderer_(new CostMapRenderer)
, navGridRenderer_(new CellGridAlphaRenderer(2048))
, targetRenderer_(new PoseTargetRenderer)
, traceRenderer_(new PoseTraceRenderer)
, robotTrajectoryRenderer_(new RobotTrajectoryRenderer)
, robotRenderer_(new RobotRenderer)
// , rrtRenderer_(new RRTRenderer)
, objectRenderer_(new TrackedObjectRenderer)
, pathRenderer_(new PathRenderer)
, situationRenderer_(std::make_unique<TopoSituationRenderer>())
, showRobot_(true)
, showDestination_(true)
, showTrackedObjects_(true)
, showEstimatedObjectMotion_(true)
, showOptimalPath_(false)
, showVisibilityAnalysis_(false)
, showTopoSituation_(false)
, threeSecObjHistory_(3000000)
, fiveSecObjHistory_(5000000)
, threeSecRobotHistory_(3000000)
, fiveSecRobotHistory_(5000000)
, newPlannerStatusMessage_(true)
, trajectoryNumber_(0)
, showMotionTargets_(false)
, overlayRobotPosesOverTrajectory_(false)
, haveHoverDestinationPose_(false)
, haveSelectedDestinationPose_(false)
, newLPM_(false)
, newDistGrid_(false)
, newCostMap_(false)
, newNavGrid_(false)
, newTrajectoryPlannerInfo_(false)
, minTrajectoryCost_(0.0f)
, maxTrajectoryCost_(0.0f)
, haveHoverMotionTargetPose_(false)
, haveSelectedMotionTargetPose_(false)
, haveTrajectoryToMotionTarget_(false)
, showRRT_(false)
, showRRTTarget_(false)
, showRRTGoalPath_(false)
, showRRTGoalRegion_(false)
, newRRT_(false)
{
    // default values for simulated motion target evaluator
    motionTargetToEvaluate_.velocityGain = 0.7;
    motionTargetToEvaluate_.k1 = 1.2;
    motionTargetToEvaluate_.k2 = 3.0;

    enableScrolling();
}


MetricPlannerDisplayWidget::~MetricPlannerDisplayWidget(void)
{
}


void MetricPlannerDisplayWidget::setWidgetParams(const metric_planner_display_params_t& plannerDisplayParams,
                                                 const lpm_display_params_t&            lpmParams)
{
    utils::ConfigFile mpepcConfig(plannerDisplayParams.mpepcConfig);
    mpepcParams_ = std::make_unique<mpepc::metric_planner_params_t>(mpepc::load_metric_planner_params(mpepcConfig));

    utils::ConfigFile collisionConfig(plannerDisplayParams.collisionModelConfig);
    robot::collision_model_params_t collisionParams(collisionConfig);

    // Use the same settings as for the LPM widget to keep things consistent
    maxTraceLength_ = lpmParams.maxTraceLength;

    traceRenderer_->setRenderColor(lpmParams.traceColor);

    GLColor white(1.0f, 1.0f, 1.0f, 1.0f);

    // Only want to see the occupied cells as something other than white. Don't care about the classifications,
    // as they just distort and confuse what is actually happening.
    occGridRenderer_->setDynamicColor(white);
    occGridRenderer_->setHazardColor(occupied_color());
    occGridRenderer_->setLimitedVisibilityColor(occupied_color());
    occGridRenderer_->setQuasiStaticColor(quasi_static_color());
    targetRenderer_->setRenderColors(lpmParams.targetCircleColor, lpmParams.targetArrowColor);

    robotRenderer_->setRobotColor(plannerDisplayParams.robotColorGrey); // default robot color
    robotRenderer_->setRobotShape(collisionParams); // robot shape

    objectRenderer_->setRenderColor(lpmParams.activeTrackingColor);

    robotTrajectoryRenderer_->setRenderColor(plannerDisplayParams.trajectoryColorBlue); // default trajectory color

//     rrtRenderer_->setRenderColors(plannerDisplayParams.edgeColor,
//                                   plannerDisplayParams.nodeColor,
//                                   plannerDisplayParams.startColor,
//                                   plannerDisplayParams.targetColor,
//                                   plannerDisplayParams.goalPathColor,
//                                   plannerDisplayParams.goalRegionColor);

    robotSimulator_.reset(new mpepc::RobotSimulator(mpepcParams_->trajectoryPlannerParams.simulatorParams));
    trajectoryEvaluator_.reset(new mpepc::TrajectoryEvaluator(mpepcParams_->trajectoryPlannerParams.evaluatorParams,
                                                              mpepcParams_->trajectoryPlannerParams.robotBodyParams));

    mpepcInfo_.clear();

    std::vector<GLColor> trajectoryColors = { plannerDisplayParams.trajectoryColorGreen,
                                              plannerDisplayParams.trajectoryColorBlue,
                                              plannerDisplayParams.trajectoryColorRed };
    interpolator_.setColors(trajectoryColors);

    params_ = plannerDisplayParams;
}


std::string MetricPlannerDisplayWidget::printCellInformation(Point<int> cell)
{
    float cost = 0.0;

    switch(mapType_)
    {
        case MPEPCMapType::lpm:
            cost = lpmGrid_.getCost(cell);
            break;

        case MPEPCMapType::obstacles:
            cost = distGrid_.getObstacleDistance(cell);
            break;

        case MPEPCMapType::costs:
            cost = costMap_.getValue(cell.x, cell.y);
            break;

        case MPEPCMapType::navigation:
            cost = navGrid_.getCostToGo(cell);
            break;

//         case SHOW_FLOW_GRID:
//             break;

        case MPEPCMapType::none:
            break;
    }

    std::string description = "  Cost: ";
    std::string costString  = std::to_string(cost);

    return description.append(costString);
}


wxArrayString MetricPlannerDisplayWidget::setupCostDescriptions(void)
{
    wxArrayString costDescriptions;
    costDescriptions.Alloc(static_cast<int>(mpepc::NUM_COST_TYPES));

    for(int costEnum = 0; costEnum != mpepc::NUM_COST_TYPES; costEnum++)
    {
        mpepc::robot_trajectory_debug_cost_type_t costType = static_cast<mpepc::robot_trajectory_debug_cost_type_t>(costEnum);
        costDescriptions.Add(wxString(mpepc::get_trj_debug_cost_text(costType)));
    }

    // NOTE: Iterating over enum this way gives me some irks but I don't know a simpler alternative to this.
    //       I want to specify cost types in robot_trajectory_info_t and read them over her to generate description lists.

    return costDescriptions;
}


size_t MetricPlannerDisplayWidget::getNumTrjEvaluatedToShow(void)
{
    if(trjGroup_ == TRJ_HISTORY)
    {
        size_t totalNumTrjEvaluated = 0;
        for(auto trjIter = trajectoryEvaluationHistory_.begin(); trjIter != trajectoryEvaluationHistory_.end(); trjIter++)
        {
            totalNumTrjEvaluated += trjIter->size();
        }

        return totalNumTrjEvaluated;
    }
    else
    {
        return mpepcInfo_.trajectories.size();
    }
}


std::string MetricPlannerDisplayWidget::getEvaluatedCostLabelString(void)
{
    switch(trjGroup_)
    {
    case TRJ_CURRENT:
    {
        switch(trjDispMode_)
        {
        case DISP_ALL:
            return "Evaluated Cost (Optimal)";

        case DISP_LAST_N: // intentional fall-through

        case DISP_SINGLE:
            return "Evaluated Cost (Single)";

        default:
            return " ";
        }
    } // no break needed due to return

    case TRJ_OPTIMAL:
        return "Evaluated Cost (Optimal)";

    case TRJ_HISTORY: // intentional fall-through

    case TRJ_NONE: // intentional fall-through

    default:
        return " ";
    }
}


float MetricPlannerDisplayWidget::getEvaluatedCostToShow(void)
{
    if(mpepcInfo_.iteration == 0)
    {
        return 0.0f;
    }
    else
    {
        switch(trjGroup_)
        {
        case TRJ_CURRENT:
        {
            switch(trjDispMode_) // this is only relevant when TRJ_CURRENT is selected.
            {
            case DISP_ALL:
            {
                return mpepcInfo_.plannedTrajectory.expectedCost;
            }

            case DISP_LAST_N: // intentional fall-through

            case DISP_SINGLE:
            {
                if(trajectoryNumber_ == 0)
                {
                    return 0.0f;
                }
                else
                {
                    auto   trjIt = mpepcInfo_.trajectories.rbegin();
                    size_t n     = std::min(trajectoryNumber_, mpepcInfo_.trajectories.size()) - 1;

                    return (trjIt + n)->expectedCost;
                }
            }

            default:
            {
                return 0.0f;
            }
            }
        } // no break needed due to return

        case TRJ_OPTIMAL:
            return mpepcInfo_.plannedTrajectory.expectedCost;

        case TRJ_HISTORY: // intentional fall-through

        case TRJ_NONE: // intentional fall-through

        default:
            return 0.0f;
        }
    }
}


void MetricPlannerDisplayWidget::setHoverDestinationPose(const pose_t& target)
{
    haveHoverDestinationPose_ = true;
    hoverDestinationPose_     = target;
}


void MetricPlannerDisplayWidget::setDestinationPose(const pose_t& target)
{
    utils::AutoMutex autoLock(dataLock_);
    haveSelectedDestinationPose_ = true;
    showDestination_ = true; // this only changes the internal state, not the checkbox in the gui.

    destinationPoses_.clear();
    destinationPoses_.push_back(target);
}


void MetricPlannerDisplayWidget::clearDestinationPose(void)
{
    haveHoverDestinationPose_    = false;
    haveSelectedDestinationPose_ = false;
}


void MetricPlannerDisplayWidget::setHoverMotionTargetPose(const pose_t& target)
{
    haveHoverMotionTargetPose_ = true;
    hoverMotionTargetPose_     = target;

    if(!haveSelectedMotionTargetPose_)
    {
        controlLawCoords_ = mpepc::control_law_coordinates_t(robotPoseForControlLawCoords_, target);
    }
}


void MetricPlannerDisplayWidget::setMotionTargetPose(const pose_t& target)
{
    haveSelectedMotionTargetPose_     = true;
    controlLawCoords_                 = mpepc::control_law_coordinates_t(robotPoseForControlLawCoords_, target);
    motionTargetToEvaluate_.pose      = target;
    motionTargetToEvaluate_.direction = mpepc::FORWARD;
}


void MetricPlannerDisplayWidget::clearMotionTarget(void)
{
    haveHoverMotionTargetPose_    = false;
    haveSelectedMotionTargetPose_ = false;
    haveTrajectoryToMotionTarget_ = false;
    controlLawCoords_             = mpepc::control_law_coordinates_t();
}


void MetricPlannerDisplayWidget::clearTrace(void)
{
    utils::AutoMutex autoLock(dataLock_);

    robotTrace_.clear();
    objectTrajectories_.clear();
    mpepcInfo_.clear();
    sortedEvaluatedTrajectories_.clear();
    trajectoryEvaluationHistory_.clear();
}


void MetricPlannerDisplayWidget::setMotionTargetR(double coord)
{
    haveSelectedMotionTargetPose_ = true;
    controlLawCoords_.r           = coord;
    motionTargetToEvaluate_.pose  = controlLawCoords_.toTargetPose(robotPoseForControlLawCoords_);

    if(controlLawCoords_.r < 0.0)
    {
        motionTargetToEvaluate_.direction = mpepc::BACKWARD;
    }
    else
    {
        motionTargetToEvaluate_.direction = mpepc::FORWARD;
    }
}


void MetricPlannerDisplayWidget::setMotionTargetTheta(double coord)
{
    haveSelectedMotionTargetPose_ = true;
    controlLawCoords_.theta       = coord;
    motionTargetToEvaluate_.pose  = controlLawCoords_.toTargetPose(robotPoseForControlLawCoords_);
}


void MetricPlannerDisplayWidget::setMotionTargetDelta(double coord)
{
    haveSelectedMotionTargetPose_ = true;
    controlLawCoords_.delta       = coord;
    motionTargetToEvaluate_.pose  = controlLawCoords_.toTargetPose(robotPoseForControlLawCoords_);
}


void MetricPlannerDisplayWidget::setMotionTargetGain(double coord)
{
    haveSelectedMotionTargetPose_ = true;
    motionTargetToEvaluate_.velocityGain = coord;
}


void MetricPlannerDisplayWidget::setMotionTargetK1(double coord)
{
    haveSelectedMotionTargetPose_ = true;
    motionTargetToEvaluate_.k1 = coord;
}


void MetricPlannerDisplayWidget::setMotionTargetK2(double coord)
{
    haveSelectedMotionTargetPose_ = true;
    motionTargetToEvaluate_.k2 = coord;
}


void MetricPlannerDisplayWidget::evaluateMotionTarget(void)
{
    // make sure we have non-empty mpepcInfo and the task
    if(mpepcInfo_.iteration == 0 || !taskManifold)
    {
        robotSimulator_->estimateRobotTrajectoryFromIntialState(motionTargetToEvaluate_,
                                                                robotState_,
                                                                0,
                                                                mpepcParams_->simulatorTimeStep,
                                                                mpepcParams_->trajectoryTimeLength,
                                                                trajectoryToMotionTarget_);
    }
    else
    {
        robotSimulator_->estimateRobotTrajectoryFromIntialState(motionTargetToEvaluate_,
                                                                mpepcInfo_.trajectoryToInitialState.back(),
                                                                0,
                                                                mpepcParams_->simulatorTimeStep,
                                                                mpepcParams_->trajectoryTimeLength,
                                                                trajectoryToMotionTarget_);

        // Trajectory evaluator needs pointers to various pieces of data.
        trajectoryEvaluator_->setTaskEnvironment(*taskManifold,
                                                 distGrid_,
                                                 objectTrajectoriesWithLaserObjects_);

        std::cout<<"NumObjects : "<<objectTrajectoriesWithLaserObjects_.size()<<'\n';

        trajectoryEvaluator_->evaluateTrajectory(trajectoryToMotionTarget_);

        // various couts on the evaluated trajectory
        // set precision
        int prevCoutPrecision = std::cout.precision();
        std::cout.precision(4);
        std::cout.setf(std::ios::fixed, std:: ios::floatfield);

        std::cout<<"Evaluated Trajectory to Motion Target: Expected cost          : "<<trajectoryToMotionTarget_.expectedCost <<"\n";
        std::cout<<"Evaluated Trajectory to Motion Target: Total survivability    : "<<trajectoryToMotionTarget_.totalSurvivability <<"\n";
        std::cout<<"Evaluated Trajectory to Motion Target: Expected progress      : "<<trajectoryToMotionTarget_.expectedProgress <<"\n";
        std::cout<<"Evaluated Trajectory to Motion Target: Expected collision cost: "<<trajectoryToMotionTarget_.expectedCollisionCost <<"\n";
        std::cout<<"Evaluated Trajectory to Motion Target: Expected action cost   : "<<trajectoryToMotionTarget_.expectedActionCost <<"\n";
        std::cout<<"Evaluated Trajectory to Motion Target: Used heuristic cost    : "<<trajectoryToMotionTarget_.heuristicCost <<"\n";

        std::cout<<"Evaluated Trajectory to Motion Target: Distance to the nearest static object: \n(";
        for(auto sampleIt = trajectoryToMotionTarget_.distanceToStaticObj.begin(), sampleEnd = trajectoryToMotionTarget_.distanceToStaticObj.end();
            sampleIt != sampleEnd;
            sampleIt++)
        {
            if(*sampleIt >= 0)
            {
                std::cout<<' ';
            }
            std::cout<<*sampleIt;
            (sampleIt+1 == sampleEnd) ? std::cout<<")\n" : std::cout<<',';
        }

        std::cout<<"Evaluated Trajectory to Motion Target: Distance to the nearest dynamic object: \n(";
        for(auto sampleIt = trajectoryToMotionTarget_.distanceToDynamicObj.begin(), sampleEnd = trajectoryToMotionTarget_.distanceToDynamicObj.end();
            sampleIt != sampleEnd;
            sampleIt++)
        {
            if(*sampleIt >= 0)
            {
                std::cout<<' ';
            }
            std::cout<<*sampleIt;
            (sampleIt+1 == sampleEnd) ? std::cout<<")\n" : std::cout<<',';
        }

        std::cout<<"Evaluated Trajectory to Motion Target: Piecewise survivability values: \n(";
        for(auto sampleIt = trajectoryToMotionTarget_.piecewiseSurvivability.begin(), sampleEnd = trajectoryToMotionTarget_.piecewiseSurvivability.end();
            sampleIt != sampleEnd;
            sampleIt++)
        {
            if(*sampleIt >= 0)
            {
                std::cout<<' ';
            }
            std::cout<<*sampleIt;
            (sampleIt+1 == sampleEnd) ? std::cout<<")\n" : std::cout<<',';
        }

        std::cout<<"Evaluated Trajectory to Motion Target: Piecewise progress values: \n(";
        for(auto sampleIt = trajectoryToMotionTarget_.piecewiseRawProgress.begin(), sampleEnd = trajectoryToMotionTarget_.piecewiseRawProgress.end();
            sampleIt != sampleEnd;
            sampleIt++)
        {
            if(*sampleIt >= 0)
            {
                std::cout<<' ';
            }
            std::cout<<*sampleIt;
            (sampleIt+1 == sampleEnd) ? std::cout<<")\n" : std::cout<<',';
        }

        std::cout<<"Evaluated Trajectory to Motion Target: Piecewise collision cost values: \n(";
        for(auto sampleIt = trajectoryToMotionTarget_.piecewiseCollisionCost.begin(), sampleEnd = trajectoryToMotionTarget_.piecewiseCollisionCost.end();
            sampleIt != sampleEnd;
            sampleIt++)
        {
            if(*sampleIt >= 0)
            {
                std::cout<<' ';
            }
            std::cout<<*sampleIt;
            (sampleIt+1 == sampleEnd) ? std::cout<<")\n" : std::cout<<',';
        }

        std::cout<<"Evaluated Trajectory to Motion Target: Piecewise action cost values: \n(";
        for(auto sampleIt = trajectoryToMotionTarget_.piecewiseActionCost.begin(), sampleEnd = trajectoryToMotionTarget_.piecewiseActionCost.end();
            sampleIt != sampleEnd;
            sampleIt++)
        {
            if(*sampleIt >= 0)
            {
                std::cout<<' ';
            }
            std::cout<<*sampleIt;
            (sampleIt+1 == sampleEnd) ? std::cout<<")\n" : std::cout<<',';
        }

        std::cout<<"MPEPC Info: Clearance to static obstacles :"<<mpepcInfo_.clearanceToStaticObs<<"\n";
        std::cout<<"MPEPC Info: Clearance to dynamic obstacles:"<<mpepcInfo_.clearanceToDynObs<<"\n";

        std::cout<<"MPEPC Info: Distance to static obstacles from robot center:"<< distGrid_.getObstacleDistance(robotState_.pose.toPoint()) <<"\n\n";

        std::cout.precision(prevCoutPrecision);
        std::cout.unsetf(std:: ios::floatfield);
    }

    haveTrajectoryToMotionTarget_ = true;
}


void MetricPlannerDisplayWidget::setDestinationPoses(const std::vector<pose_t>& waypointPoses)
{
    utils::AutoMutex autoLock(dataLock_);
    haveSelectedDestinationPose_ = true;

    destinationPoses_ = waypointPoses;
}


void MetricPlannerDisplayWidget::handleData(const motion_state_t& robotState, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    robotState_ = robotState;

    robotTrace_.push_front(robotState.pose);

    if(robotTrace_.size() > maxTraceLength_)
    {
        robotTrace_.pop_back();
    }

    setCameraFocalPoint(robotState_.pose.toPoint());

    // Store the current pose state for the brief-history view
    threeSecRobotHistory_.push(robotState.pose);
    fiveSecRobotHistory_.push(robotState.pose);
}


void MetricPlannerDisplayWidget::handleData(const robot::commanded_joystick_t& commandedJoystick, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
}


void MetricPlannerDisplayWidget::handleData(const robot::commanded_velocity_t& commandedVelocity, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
}


void MetricPlannerDisplayWidget::handleData(const robot::motion_command_t& motionCommand, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
}


void MetricPlannerDisplayWidget::handleData(const pose_t& pose, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
}


void MetricPlannerDisplayWidget::handleData(const hssh::LocalPerceptualMap& occGrid, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    lpmGrid_ = occGrid;
    newLPM_ = true;

    setGridDimensions(occGrid.getWidthInMeters(), occGrid.getHeightInMeters());
}


void MetricPlannerDisplayWidget::handleData(const hssh::LocalTopoMap& topoMap, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    topoMap_ = std::make_shared<hssh::LocalTopoMap>(topoMap);
}


void MetricPlannerDisplayWidget::handleData(const tracker::DynamicObjectCollection& tracked, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    trackedObjects_ = tracked;
}


void MetricPlannerDisplayWidget::handleData(const mpepc::trajectory_planner_debug_info_t& mpepcInfo, const std::string& channel)
{
    dataLock_.lock();
    mpepcInfo_ = mpepcInfo;
    // Clear out the extra trajectories since the memory optimization isn't needed here
    mpepcInfo_.trajectories.erase(mpepcInfo_.trajectories.begin() + mpepcInfo_.numTrajectories,
                                  mpepcInfo_.trajectories.end());

    if(mpepcInfo_.iteration != 0)
    {
        robotPoseForControlLawCoords_ = mpepcInfo.trajectoryToInitialState.back().pose;

        sortedEvaluatedTrajectories_.resize(mpepcInfo.numTrajectories);

        for(std::size_t n = 0; n < mpepcInfo_.numTrajectories; ++n)
        {
            // new set of simplified trajectories for sorting.
            sortedEvaluatedTrajectories_[n] = mpepc::robot_trajectory_debug_info_simple_t(mpepcInfo_.trajectories[n]);
        }

        // sortTrajectories acquires the dataLock, so make sure unlocked before going into that function
        dataLock_.unlock();
        sortTrajectories();
        dataLock_.lock();
    }
    else
    {
        sortedEvaluatedTrajectories_.clear();
    }

    // Keep recent trajectory history.
    if(trajectoryEvaluationHistory_.size() > 600) // Don't keep around too many trajectories! At 5 hz 1 min ~ 300.
    {
        trajectoryEvaluationHistory_.pop_front();
    }

    trajectoryEvaluationHistory_.push_back(sortedEvaluatedTrajectories_);
    dataLock_.unlock();

    newTrajectoryPlannerInfo_ = true;
}


void MetricPlannerDisplayWidget::handleData(const mpepc::ObstacleDistanceGrid& distGrid, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    distGrid_ = distGrid;
    newDistGrid_ = true;
}


void MetricPlannerDisplayWidget::handleData(const std::vector<mpepc::dynamic_object_trajectory_debug_info_t>& objectTrajectories, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    objectTrajectories_ = objectTrajectories;

    // Store the trajectory state for the brief history view
    if(!objectTrajectories.empty())
    {
        dynamic_objects_t objs;
        objs.timestamp = objectTrajectories.front().timestamp;

        for(auto& traj : objectTrajectories)
        {
            objs.states.push_back(traj.states.front());
        }

        threeSecObjHistory_.push(objs);
        fiveSecObjHistory_.push(objs);
    }
}


void MetricPlannerDisplayWidget::handleData(const std::vector<mpepc::dynamic_object_trajectory_t>& objectTrajectoriesWithLaserObjects, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    objectTrajectoriesWithLaserObjects_ = objectTrajectoriesWithLaserObjects;
}


void MetricPlannerDisplayWidget::handleData(const mpepc::NavigationGrid& navGrid, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    navGrid_ = navGrid;
    taskManifold = std::make_shared<mpepc::NavigationTaskManifold>(navGrid,
                                                                   mpepcParams_->taskParams.navigationTaskParams,
                                                                   mpepcParams_->taskManifoldBuilderParams.navigationTaskManifoldParams);
    newNavGrid_ = true;
}


void MetricPlannerDisplayWidget::handleData(const mpepc::CostMap& costMap, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    costMap_ = costMap;
    newCostMap_ = true;
}


void MetricPlannerDisplayWidget::handleData(const mpepc::VisibilityAnalysis& visibility, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    visibility_ = visibility;
}


void MetricPlannerDisplayWidget::handleData(const mpepc::metric_planner_status_message_t& plannerStatus, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    plannerStatusMessage_ = plannerStatus;
    newPlannerStatusMessage_    = true;
}


void MetricPlannerDisplayWidget::handleData(const mpepc::learned_norm_info_t& normInfo, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    situationInfo_ = normInfo;
}


void MetricPlannerDisplayWidget::handleData(const std::shared_ptr<mpepc::MetricPlannerTask>& task, const std::string& channel)
{
    auto manifold = task->createTaskManifold();
    setDestinationPose(manifold->target());
}


void MetricPlannerDisplayWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(dataLock_);

    // redering grids
    switch(mapType_)
    {
    case MPEPCMapType::lpm:
    {
        if(newLPM_)
        {
            occGridRenderer_->setGrid(lpmGrid_);
            newLPM_ = false;
        }

        occGridRenderer_->renderGrid();
    }
    break;

    case MPEPCMapType::obstacles:
    {
        if(newDistGrid_)
        {
//             distGridRenderer_->setMaxValue(distGrid_.getMaxGridDist());
            distGridRenderer_->setMaxValue(200);
            distGridRenderer_->setCellGrid(distGrid_);
            newDistGrid_ = false;
        }

        distGridRenderer_->renderGrid(true);
    }
    break;

    case MPEPCMapType::costs:
    {
        if(newCostMap_)
        {
            costMapRenderer_->setCostMap(costMap_);
            newCostMap_ = false;
        }

        costMapRenderer_->renderCosts();
    }
    break;

    case MPEPCMapType::navigation:
    {
        if(newNavGrid_)
        {
            navGridRenderer_->setMaxValue(navGrid_.getMaxGridCostToGo());
            navGridRenderer_->setCellGrid(navGrid_);
            newNavGrid_ = false;
        }

        navGridRenderer_->renderGrid(true);
    }
    break;

    case MPEPCMapType::none:
    {
    }
    break;
    }

    // rendering rrts
    if(showRRT_ && newRRT_)
    {
//         rrtRenderer_->renderRRT(rrtInfo_);
    }

    if(showTopoSituation_ && topoMap_)
    {
        renderSituations();
    }

    // redering dynamic objects and their estimated trajectories
    if(showTrackedObjects_)
    {
        // direct tracker outputs
        for(const auto& object : trackedObjects_)
        {
            objectRenderer_->renderObject(*object);
        }

        // tracker output used by the trajectory planner
        for(const mpepc::dynamic_object_trajectory_t& objectTrajectory : objectTrajectoriesWithLaserObjects_)
        {
            objectRenderer_->renderObject(*objectTrajectory.laserObject);
        }
    }

    if(showEstimatedObjectMotion_)
    {
        // info reported by the trajectory planner
        for(const mpepc::dynamic_object_trajectory_debug_info_t& objectTrajectory : objectTrajectories_)
        {
            objectRenderer_->renderEstimatedObjectTrajectory(objectTrajectory);
        }
    }

    if(showOptimalPath_)
    {
        auto path = mpepc::minimum_cost_path(robotState_.pose.toPoint(), navGrid_);
        pathRenderer_->render(path, navGrid_.metersPerCell());
    }

    if(showVisibilityAnalysis_)
    {
        renderVisibility();
    }

    // rendering destination poses
    if(showDestination_ && haveSelectedDestinationPose_)
    {
        for(auto targetIt = destinationPoses_.begin(), targetEnd = destinationPoses_.end(); targetIt != targetEnd; targetIt++)
        {
            targetRenderer_->renderTargetRectangle(*targetIt);
        }
    }

    if(haveHoverDestinationPose_) // draw hover pose on top
    {
        targetRenderer_->renderTargetRectangle(hoverDestinationPose_);
    }

    // rendering motion target for evaulation
    if(haveSelectedMotionTargetPose_)
    {
        targetRenderer_->renderTargetTriangle(motionTargetToEvaluate_.pose);
    }

    if(haveHoverMotionTargetPose_)
    {
        targetRenderer_->renderTargetTriangle(hoverMotionTargetPose_);
    }

    if(haveTrajectoryToMotionTarget_)
    {
        if(trjGroup_ == TRJ_CURRENT && trjDispMode_ == DISP_ALL)
        {
            renderPlannedTrajectory(mpepc::robot_trajectory_debug_info_t(trajectoryToMotionTarget_));
        }
        else
        {
            renderEvaluatedTrajectory(mpepc::robot_trajectory_debug_info_t(trajectoryToMotionTarget_));

            if(overlayRobotPosesOverTrajectory_)
            {
                std::vector<pose_t> poses;
                trajectoryToMotionTarget_.getPoses(poses);
                robotTrajectoryRenderer_->renderRobotsOverTrajectory(poses, *robotRenderer_, 5);
            }
        }
    }

    // rendering robot trajectories
    if(newTrajectoryPlannerInfo_)
    {
        displayTrajectoryPlannerInfo();
    }

    // select robot color
    if(newPlannerStatusMessage_)
    {
        associatePlannerStatusToRobotColor(plannerStatusMessage_.status);
        newPlannerStatusMessage_ = false;
    }

    // render robot pose
    if(showRobot_)
    {
        // render robot using the specified color.
        robotRenderer_->renderRobot(robotState_.pose);

        // draw a bounding box around the robot if there is an obstacle nearby.
//         if(mpepcInfo_.clearanceToStaticObs < 2.0)
//         {
//             // TODO (maybe): add code here to determine color.
//             robotRenderer_->renderBoundingBox(robotState_.pose, mpepcInfo_.clearanceToStaticObs);
//         }
//
//         if(!objectTrajectories_.empty() && mpepcInfo_.clearanceToDynObs < 2.0)
//         {
//             // TODO (maybe): add code here to determine color.
//             robotRenderer_->renderBoundingBox(robotState_.pose, mpepcInfo_.clearanceToDynObs);
//         }

        // render robot trace
//         traceRenderer_->setPoseTrace(robotTrace_);
//         traceRenderer_->renderTrace();
    }
}


Point<int> MetricPlannerDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    return utils::global_point_to_grid_cell(world, lpmGrid_);
}


void MetricPlannerDisplayWidget::sortTrajectories(void)
{
    utils::AutoMutex autoLock(dataLock_);

    // simply sort using the expected cost.
    std::sort(sortedEvaluatedTrajectories_.begin(),
              sortedEvaluatedTrajectories_.end(),
              [](const mpepc::robot_trajectory_debug_info_simple_t& lhs, const mpepc::robot_trajectory_debug_info_simple_t& rhs)
              {
                  return lhs.expectedCost < rhs.expectedCost; // smaller is better
              });
}


void MetricPlannerDisplayWidget::associatePlannerStatusToRobotColor(mpepc::metric_planner_status_t status)
{
    // reset robot color if new status message has been received.
    switch(status)
    {
        case mpepc::IDLE:
        {
            robotRenderer_->setRobotColor(params_.robotColorGrey);
        }
        break;

        case mpepc::PAUSED:
        {
            robotRenderer_->setRobotColor(params_.robotColorGrey);
        }
        break;

        case mpepc::ACTIVE_NORMAL:
        {
            robotRenderer_->setRobotColor(params_.robotColorGreen);
        }
        break;

        case mpepc::ACTIVE_SPECIAL:
        {
            robotRenderer_->setRobotColor(params_.robotColorLightBlue);
        }
        break;

        case mpepc::ACTIVE_RRT:
        {
            robotRenderer_->setRobotColor(params_.robotColorLightBlue);
        }
        break;

        case mpepc::SUCCESS_REACHED_POSE:
        {
            robotRenderer_->setRobotColor(params_.robotColorBlue);
        }
        break;

        case mpepc::FAILURE_CANNOT_ASSIGN_TASK:
        {
            robotRenderer_->setRobotColor(params_.robotColorRed);
        }
        break;

        case mpepc::FAILURE_UNABLE_TO_PROGRESS:
        {
            robotRenderer_->setRobotColor(params_.robotColorViolet);
        }   break;

        case mpepc::FAILURE_CANNOT_FIND_SOLUTION:
        {
            robotRenderer_->setRobotColor(params_.robotColorRed);
        }
        break;
    }
}


void MetricPlannerDisplayWidget::renderVisibility(void)
{
    hazard_color().set(0.33);

    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(visibility_.origin().x, visibility_.origin().y);
    for(auto ray : boost::make_iterator_range(visibility_.beginStatic(), visibility_.endStatic()))
    {
        glVertex2f(ray.x, ray.y);
    }
    glEnd();

    dynamic_color().set(0.33);

    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(visibility_.origin().x, visibility_.origin().y);
    for(auto ray : boost::make_iterator_range(visibility_.beginDynamic(), visibility_.endDynamic()))
    {
        glVertex2f(ray.x, ray.y);
    }
    glEnd();
}


void MetricPlannerDisplayWidget::renderSituations(void)
{
    // Need the topo map to draw this information
    if(!topoMap_)
    {
        if(!situationInfo_.pathSituations.empty())
        {
            std::cout << "INFO: Have situations, but need topo map.\n";
        }
        return;
    }

    for(auto& path : situationInfo_.pathSituations)
    {
        situationRenderer_->renderSituationPath(path, *topoMap_);
    }

    for(auto& place : situationInfo_.placeSituations)
    {
        situationRenderer_->renderSituationPlace(place, *topoMap_);
    }
}


void MetricPlannerDisplayWidget::displayTrajectoryPlannerInfo(void)
{
    // converting intial states to initial poses
    std::vector<pose_t> initialPoses;
    for(auto stateIt =  mpepcInfo_.trajectoryToInitialState.begin();
             stateIt != mpepcInfo_.trajectoryToInitialState.end();
           ++stateIt)
    {
            initialPoses.push_back(stateIt->pose);
    }

    switch(trjGroup_)
    {
        case TRJ_CURRENT:
        {
            // render trajectory to the beginning of the next plannign cycle (the initial state for the planner)
            robotTrajectoryRenderer_->renderTrajectory(initialPoses);

            // render trajectories without sorting so that we can iterate through optimizer action
            setTrajectoryColors();
            renderEvaluatedTrajectories(mpepcInfo_.trajectories);

            // render optimizer outputs
    //        robotTrajectoryRenderer_->renderTrajectory(mpepcInfo_.coarseOptimizerOutput.poses, ui::GLColor(0,0,0,125),     5.0);
            robotTrajectoryRenderer_->renderTrajectory(mpepcInfo_.localOptimizerOutput.poses, GLColor(200,200,0,125), 5.0);
        }
        break;

        case TRJ_OPTIMAL:
        {
            robotTrajectoryRenderer_->renderTrajectory(initialPoses);

            // render optimizer outputs
    //        robotTrajectoryRenderer_->renderTrajectory(mpepcInfo_.coarseOptimizerOutput.poses, ui::GLColor(0,0,0,255));

            // render final planned trajectory
            renderPlannedTrajectory(mpepcInfo_.plannedTrajectory);
        }
        break;

        case TRJ_HISTORY:
        {
            renderEvaluationHistory(trajectoryEvaluationHistory_);
        }
        break;

        case TRJ_NONE:
        {
            // do nothing
        }
        break;

        case TRJ_THREE_SEC:
            renderRecentHistory(threeSecRobotHistory_, threeSecObjHistory_);
            break;

        case TRJ_FIVE_SEC:
            renderRecentHistory(fiveSecRobotHistory_, fiveSecObjHistory_);
            break;

        default:
            std::cerr << "ERROR: Invalid trajectory display type: " << trjGroup_ << '\n';
            assert(false);
            break;
    }
}


void MetricPlannerDisplayWidget::renderEvaluatedTrajectories(const robot_trajectories_t& evaluatedTrajectories)
{
    switch(trjDispMode_)
    {
        case DISP_ALL:
        {
            for(auto trjIt = evaluatedTrajectories.begin(); trjIt != evaluatedTrajectories.end(); trjIt++)
            {
                renderEvaluatedTrajectory(*trjIt);
            }
        }
        break;

        case DISP_LAST_N:
        {
            // we know trajectoryNumber_ is at least zero but need to check if it is out of bound. Draw the last trajectory at the end.
            for(auto trjIt = evaluatedTrajectories.end() - std::min(trajectoryNumber_, evaluatedTrajectories.size()); trjIt != evaluatedTrajectories.end(); trjIt++)
            {
                renderEvaluatedTrajectory(*trjIt);
            }
        }
        break;

        case DISP_SINGLE:
        {
            if(trajectoryNumber_ > 0)
            {
                auto trjIt = evaluatedTrajectories.rbegin() + std::min(trajectoryNumber_, evaluatedTrajectories.size()) - 1;
                renderEvaluatedTrajectory(*trjIt);
            }
        }
        break;

        default:
        {
            // TODO: send out a warning here
        }
        break;
    }

}


void MetricPlannerDisplayWidget::setTrajectoryColors(void)
{
    // Low values will be displayed in green, and high values will be in red.
    switch(trjCostType_)
    {
        case mpepc::EXPECTED_COST:
        {
            float kNominalGoodProgress = -mpepcParams_->trajectoryTimeLength *
                                          mpepcParams_->trajectoryPlannerParams.optimizerParams.maxVelocityGain;
            minTrajectoryCost_ = std::max(0.8f * mpepcInfo_.plannedTrajectory.expectedCost, kNominalGoodProgress); // take value from the planned one, but limit it by some nominal value.

            // use absolute scale when everyting is bad
            if(minTrajectoryCost_ > 0.0)
            {
                minTrajectoryCost_ = 0.0;
            }

            float kNominalBadCollisionCost = mpepcParams_->trajectoryTimeLength *
                                             mpepcParams_->trajectoryPlannerParams.evaluatorParams.baseCostOfCollision;
            maxTrajectoryCost_ = std::max(0.8f * kNominalBadCollisionCost, minTrajectoryCost_ + 1.0f); // ensure the max is larger than min by meaningful amount.
        }
        break;

        case mpepc::TOTAL_SURVIVABILITY:
        {
            minTrajectoryCost_ = 1.0;
            maxTrajectoryCost_ = 0.0;
        }
        break;

        case mpepc::EXPECTED_PROGRESS:
        {
            float kNominalGoodProgress = -mpepcParams_->trajectoryTimeLength *
                                          mpepcParams_->trajectoryPlannerParams.optimizerParams.maxVelocityGain;
            minTrajectoryCost_ = std::max(0.8f * mpepcInfo_.plannedTrajectory.expectedCost, kNominalGoodProgress); // take value from the planned one, but limit it by some nominal value.

            // use absolute scale when everything is bad
            if(minTrajectoryCost_ > -0.5)
            {
                minTrajectoryCost_ = -0.5;
            }

            maxTrajectoryCost_ = 0.0;
        }
        break;

        case mpepc::EXPECTED_COLLISION_COST:
        {
            minTrajectoryCost_ = 0.0;

            float kNominalBadCollisionCost = mpepcParams_->trajectoryTimeLength *
                                             mpepcParams_->trajectoryPlannerParams.evaluatorParams.baseCostOfCollision;
            maxTrajectoryCost_ = kNominalBadCollisionCost * 1.2f;
        }
        break;

        case mpepc::EXPECTED_ACTION_COST:
        {
            minTrajectoryCost_ = 0.0;

            float kNominalLargeActionCost =  mpepcParams_->trajectoryTimeLength *
                                             mpepcParams_->trajectoryPlannerParams.evaluatorParams.linearVelocityActionWeight;
            maxTrajectoryCost_ = kNominalLargeActionCost;
        }
        break;

        // Nothing to be done for piecewise components
        case mpepc::PIECEWISE_SURVIVABILITY:
        {
        }
        break;

        case mpepc::PIECEWISE_PROGRESS:
        {
        }
        break;

        case mpepc::PIECEWISE_RAW_PROGRESS:
        {
        }
        break;

        case mpepc::PIECEWISE_COLLSION_COST:
        {
        }
        break;

        case mpepc::PIECEWISE_ACTION_COST:
        {
        }
        break;

        case mpepc::NUM_COST_TYPES: // intentional fall-through

        default:
        {
        }
        break;
    }
}


void MetricPlannerDisplayWidget::renderEvaluatedTrajectory(const mpepc::robot_trajectory_debug_info_t& evaluatedTrajectory)
{
    // NOTE: This requires the minTrajectoryCost and maxTrajectory costs to be computed beforehand according to the appropriate
    //       showTrajectoriesCost type. That is currently done in in the sortTrajectories(). That means, if renderRobotTrajectory
    //       runs without proper preceeding run of sortTrajectories() the colors will be messed up.
    float              piecewiseCost;
    std::vector<float> costsToDisplay;

    float timeBetweenSamples        = mpepcParams_->trajectoryPlannerParams.evaluatorParams.timeBetweenSamples;
    float nominalLargeCollisionCost = mpepcParams_->trajectoryPlannerParams.evaluatorParams.baseCostOfCollision * timeBetweenSamples;
    float nominalLargeActionCost    = mpepcParams_->trajectoryPlannerParams.evaluatorParams.linearVelocityActionWeight * timeBetweenSamples;

    switch(trjCostType_)
    {
    // Low values will be displayed in green, and high values will be in red.
        case mpepc::EXPECTED_COST:
        {
            robotTrajectoryRenderer_->renderTrajectory(evaluatedTrajectory.poses,
                                                       interpolator_.calculateColor((evaluatedTrajectory.expectedCost - minTrajectoryCost_) / (maxTrajectoryCost_ - minTrajectoryCost_)));
        }
        break;

        case mpepc::TOTAL_SURVIVABILITY:
        {
            robotTrajectoryRenderer_->renderTrajectory(evaluatedTrajectory.poses,
                                                       interpolator_.calculateColor(1.0 - evaluatedTrajectory.totalSurvivability)); // [0,1]
        }
        break;

        case mpepc::EXPECTED_PROGRESS:
        {
            robotTrajectoryRenderer_->renderTrajectory(evaluatedTrajectory.poses,
                                                       interpolator_.calculateColor((evaluatedTrajectory.expectedProgress - minTrajectoryCost_) / (maxTrajectoryCost_ - minTrajectoryCost_)));
        }
        break;

        case mpepc::EXPECTED_COLLISION_COST:
        {
            robotTrajectoryRenderer_->renderTrajectory(evaluatedTrajectory.poses,
                                                       interpolator_.calculateColor((evaluatedTrajectory.expectedCollisionCost - minTrajectoryCost_) / (maxTrajectoryCost_ - minTrajectoryCost_)));
        }
        break;

        case mpepc::EXPECTED_ACTION_COST:
        {
            robotTrajectoryRenderer_->renderTrajectory(evaluatedTrajectory.poses,
                                                       interpolator_.calculateColor((evaluatedTrajectory.expectedActionCost - minTrajectoryCost_) / (maxTrajectoryCost_ - minTrajectoryCost_)));
        }
        break;

        // TODO: lots of add-hoc values floating around below...
        case mpepc::PIECEWISE_SURVIVABILITY:
        {
            for(auto costIt = evaluatedTrajectory.piecewiseSurvivability.begin(); costIt != evaluatedTrajectory.piecewiseSurvivability.end(); ++costIt)
            {
                piecewiseCost = 1.0 - (*costIt);
                if(piecewiseCost > 1.0f)
                {
                    piecewiseCost = 1.0f;
                }
                else if (piecewiseCost < 0.0f)
                {
                    piecewiseCost = 0.0f;
                }
                costsToDisplay.push_back(piecewiseCost);
            }
            robotTrajectoryRenderer_->renderPiecewiseCosts(evaluatedTrajectory.poses, costsToDisplay);
        }
        break;

        case mpepc::PIECEWISE_PROGRESS:
        {
            for(auto progressIt = evaluatedTrajectory.piecewiseRawProgress.begin(), survivabilityIt = evaluatedTrajectory.piecewiseSurvivability.begin();
                progressIt != evaluatedTrajectory.piecewiseRawProgress.end();
                ++progressIt, ++survivabilityIt)
            {
                piecewiseCost = ((*progressIt) * (*survivabilityIt) + timeBetweenSamples) / timeBetweenSamples;
                if(piecewiseCost > 1.0f)
                {
                    piecewiseCost = 1.0f;
                }
                else if (piecewiseCost < 0.0f)
                {
                    piecewiseCost = 0.0f;
                }
                costsToDisplay.push_back(piecewiseCost);
            }
            robotTrajectoryRenderer_->renderPiecewiseCosts(evaluatedTrajectory.poses, costsToDisplay);
        }
        break;

        case mpepc::PIECEWISE_RAW_PROGRESS:
        {
            for(auto costIt = evaluatedTrajectory.piecewiseRawProgress.begin(); costIt != evaluatedTrajectory.piecewiseRawProgress.end(); ++costIt)
            {
                piecewiseCost = ((*costIt) + timeBetweenSamples) / timeBetweenSamples;
                if(piecewiseCost > 1.0f)
                {
                    piecewiseCost = 1.0f;
                }
                else if (piecewiseCost < 0.0f)
                {
                    piecewiseCost = 0.0f;
                }
                costsToDisplay.push_back(piecewiseCost);
            }
            robotTrajectoryRenderer_->renderPiecewiseCosts(evaluatedTrajectory.poses, costsToDisplay);
        }
        break;

        case mpepc::PIECEWISE_COLLSION_COST:
        {
            for(auto costIt = evaluatedTrajectory.piecewiseCollisionCost.begin(); costIt != evaluatedTrajectory.piecewiseCollisionCost.end(); ++costIt)
            {
                piecewiseCost = (*costIt) / nominalLargeCollisionCost;
                if(piecewiseCost > 1.0f)
                {
                    piecewiseCost = 1.0f;
                }
                else if (piecewiseCost < 0.0f)
                {
                    piecewiseCost = 0.0f;
                }
                costsToDisplay.push_back(piecewiseCost);
            }
            robotTrajectoryRenderer_->renderPiecewiseCosts(evaluatedTrajectory.poses, costsToDisplay);
        }
        break;

        case mpepc::PIECEWISE_ACTION_COST:
        {
            for(auto costIt = evaluatedTrajectory.piecewiseActionCost.begin(); costIt != evaluatedTrajectory.piecewiseActionCost.end(); ++costIt)
            {

                piecewiseCost = (*costIt) / nominalLargeActionCost;
                if(piecewiseCost > 1.0f)
                {
                    piecewiseCost = 1.0f;
                }
                else if (piecewiseCost < 0.0f)
                {
                    piecewiseCost = 0.0f;
                }
                costsToDisplay.push_back(piecewiseCost);
            }
            robotTrajectoryRenderer_->renderPiecewiseCosts(evaluatedTrajectory.poses, costsToDisplay);
        }
        break;

        case mpepc::NUM_COST_TYPES: // intentional fall-through

        default:
        {
        }
        break;
    }

    if(showMotionTargets_)
    {
        robotTrajectoryRenderer_->renderMotionTarget(evaluatedTrajectory.motionTarget);
    }

    if(trjGroup_ == TRJ_CURRENT && trjDispMode_ == DISP_SINGLE && overlayRobotPosesOverTrajectory_)
    {
        robotTrajectoryRenderer_->renderRobotsOverTrajectory(evaluatedTrajectory.poses, *robotRenderer_, 5);
    }
}


void MetricPlannerDisplayWidget::renderPlannedTrajectory(const mpepc::robot_trajectory_debug_info_t& plannedTajectory)
{
    if(plannedTajectory.hasCollision)
    {
        robotTrajectoryRenderer_->renderTrajectory(plannedTajectory.poses, params_.trajectoryColorRed);
    }
    else
    {
        robotTrajectoryRenderer_->renderTrajectory(plannedTajectory.poses, params_.trajectoryColorBlue);
    }

    if(showMotionTargets_)
    {
        robotTrajectoryRenderer_->renderMotionTarget(plannedTajectory.motionTarget);
    }

    if(overlayRobotPosesOverTrajectory_)
    {
        robotTrajectoryRenderer_->renderRobotsOverTrajectory(plannedTajectory.poses, *robotRenderer_, 5);
    }
}


void MetricPlannerDisplayWidget::renderEvaluationHistory(const std::deque<robot_trajectories_simple_t>& pastTrajectories)
{
    // use some absolute scale
    float kMaxTrajectoryCost = -4.5;
    float kMinTrajectoryCost =  0.75;
    float kCostScale = kMaxTrajectoryCost - kMinTrajectoryCost;

    for(auto planningCycleIt = pastTrajectories.begin(); planningCycleIt != pastTrajectories.end(); ++planningCycleIt)
    {

        size_t numTrjsToShow;
        switch(trjDispMode_)
        {
            case(DISP_ALL):
            {
                numTrjsToShow = planningCycleIt->size();
            }
            break;

            case(DISP_LAST_N):
            {
                numTrjsToShow = std::min(trajectoryNumber_, planningCycleIt->size());
            }
            break;

            case(DISP_SINGLE):
            {
                numTrjsToShow = 1;
            }
            break;

            default:
            {
                numTrjsToShow = 0;
            }
            break;
        }

        auto trjEnd = planningCycleIt->begin() + numTrjsToShow;
        for(auto evaluatedTrajectoryIt = planningCycleIt->begin(); evaluatedTrajectoryIt != trjEnd; ++evaluatedTrajectoryIt)
        {
            robotTrajectoryRenderer_->renderTrajectory(evaluatedTrajectoryIt->poses,
                                                       interpolator_.calculateColor((evaluatedTrajectoryIt->expectedCost - kMinTrajectoryCost) / kCostScale));
        }
    }
}


void MetricPlannerDisplayWidget::renderRecentHistory(const utils::FixedDurationBuffer<pose_t>& poseHistory,
                                                     const utils::FixedDurationBuffer<dynamic_objects_t>& objectHistory)
{
    const int64_t kRenderIntervalUs = 1000000;

//     if(!objectHistory.empty())
//     {
//         int64_t lastTime = 0;
//         for(auto& objects : objectHistory)
//         {
//             if(objects.timestamp - lastTime > kRenderIntervalUs/2)
//             {
//                 for(auto& obj : objects.states)
//                 {
//                     Point<float> center(obj.x, obj.y);
//                     glColor4f(0.5f, 0.1f, 0.6f, 0.5);
//                     gl_draw_line_circle(center, 0.2); // draw small circle
//                 }
//
//                 lastTime = objects.timestamp;
//             }
//         }
//     }

    if(!poseHistory.empty())
    {
        int64_t lastTime = 0;
        for(auto& pose : poseHistory)
        {
            if(pose.timestamp - lastTime > kRenderIntervalUs)
            {
                robotRenderer_->renderBoundary(pose);
                lastTime = pose.timestamp;
            }
        }

        glBegin(GL_LINE_STRIP);
        frontier_color().set();
        for(auto& pose : poseHistory)
        {
            glVertex2f(pose.x, pose.y);
        }
        glEnd();
    }

    robotRenderer_->renderRobot(robotState_.pose);

    robotTrajectoryRenderer_->renderTrajectory(mpepcInfo_.plannedTrajectory.poses, params_.trajectoryColorBlue);
//     robotTrajectoryRenderer_->renderRobotsOverTrajectory(mpepcInfo_.plannedTrajectory.poses, *robotRenderer_, 5);
}

} // namespace ui
} // namespace vulcan
