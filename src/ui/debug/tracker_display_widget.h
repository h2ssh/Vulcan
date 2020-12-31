/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     tracker_display_widget.h
 * \author   Collin Johnson
 *
 * Declaration of TrackerDisplayWidget.
 */

#ifndef UI_DEBUG_TRACKER_DISPLAY_WIDGET_H
#define UI_DEBUG_TRACKER_DISPLAY_WIDGET_H

#include "core/motion_state.h"
#include "hssh/local_metric/lpm.h"
#include "tracker/dynamic_object_collection.h"
#include "tracker/evaluation/intention_evaluator.h"
#include "tracker/laser_object_collection.h"
#include "ui/components/grid_based_display_widget.h"
#include "utils/mutex.h"

namespace vulcan
{
namespace ui
{

class RobotRenderer;
class LaserObjectRenderer;
class DynamicObjectRenderer;
class ObjectIntentionRenderer;
class OccupancyGridRenderer;
struct ui_params_t;

/**
 * RigidObjectState specifies the possible rigid object states to be rendered by the widget.
 */
enum class RigidObjectState
{
    fast,
    slow,
};

/**
 * TrackingUncertainty specifies the different types of uncertainty maintained by a tracked object. The type to show
 * is given here.
 */
enum class TrackingUncertainty
{
    none,
    position,
    velocity,
    acceleration,
};

/**
 * PredictionType defines the different predictions available for a tracked object.
 */
enum class PredictionType
{
    none,
    best,
    all,
};


/**
 * TrackerDisplayWidget visualizes the state estimated by the object_tracker module. The following are rendered:
 *
 *   - Laser objects
 *   - Dynamic objects
 *   - Dynamic object goal predictions
 *   - Dynamic object trajectory predictions
 */
class TrackerDisplayWidget : public GridBasedDisplayWidget
{
public:
    const static int kRectangleModel = 0;
    const static int kCircleModel = 1;
    const static int kTwoCircleModel = 2;

    TrackerDisplayWidget(wxWindow* parent,
                         wxWindowID id = wxID_ANY,
                         const wxPoint& pos = wxDefaultPosition,
                         const wxSize& size = wxDefaultSize,
                         long style = 0,
                         const wxString& name = wxString((const wxChar*)("GLCanvas")),
                         const wxPalette& palette = wxNullPalette);

    virtual ~TrackerDisplayWidget(void);

    void setWidgetParams(const ui_params_t& params);

    void shouldFollowRobot(bool follow) { shouldFollowRobot_ = follow; }

    void showLaserObjects(bool show) { shouldShowLaserObjects_ = show; }
    void showLaserPoints(bool show) { shouldShowLaserPoints_ = show; }
    void showLaserUncertainty(bool show) { shouldShowLaserUncertainty_ = show; }

    void showTrackedObjects(bool show) { shouldShowDynamicObjects_ = show; }
    void showAccleration(bool show) { shouldShowAccleration_ = show; }
    void showRecentTrajectory(bool show) { shouldShowRecentTrajectory_ = show; }
    void setBoundaryToShow(tracker::BoundaryType show) { boundaryToShow_ = show; }
    void setRigidObjectStateToShow(RigidObjectState show) { rigidObjectStateToShow_ = show; }
    void setTrackingUncertaintyToShow(TrackingUncertainty show) { trackingUncertaintyToShow_ = show; }

    void shouldEvaluateIntentions(bool evaluate);
    void setGoalPredictionToShow(PredictionType show) { goalsToShow_ = show; }

    void setTrajectoryPredictionToShow(PredictionType show) { trajectoryPredictionToShow_ = show; }
    void setPredictedTrajectoryDuration(int durationMs) { predictedTrajDurationMs_ = durationMs; }

    void handleData(const motion_state_t& state, const std::string& channel);
    void handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel);
    void handleData(const tracker::LaserObjectCollection& objects, const std::string& channel);
    void handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel);

private:
    std::vector<tracker::LaserObjectCollection> laserObjects_;
    tracker::DynamicObjectCollection dynObjCollection_;
    hssh::LocalPerceptualMap lpm_;
    motion_state_t robotState_;

    std::unique_ptr<tracker::AreaIntentionEstimates> intentions_;
    bool shouldEvaluateIntentions_ = false;

    bool haveNewLPM_;
    int laserObjectsIndex_;

    utils::Mutex dataLock_;

    std::unique_ptr<OccupancyGridRenderer> lpmRenderer_;
    std::unique_ptr<DynamicObjectRenderer> dynamicObjectRenderer_;
    std::unique_ptr<LaserObjectRenderer> laserObjectRenderer_;
    std::unique_ptr<RobotRenderer> robotRenderer_;
    std::unique_ptr<ObjectIntentionRenderer> intentionRenderer_;

    bool shouldFollowRobot_;
    bool shouldShowLaserObjects_;
    bool shouldShowLaserPoints_;
    bool shouldShowLaserUncertainty_;

    bool shouldShowDynamicObjects_;
    bool shouldShowAccleration_;
    bool shouldShowRecentTrajectory_;

    tracker::BoundaryType boundaryToShow_;
    RigidObjectState rigidObjectStateToShow_;
    TrackingUncertainty trackingUncertaintyToShow_;

    PredictionType goalsToShow_;

    PredictionType trajectoryPredictionToShow_;
    int predictedTrajDurationMs_;

    // GridBasedDisplayWidget interface
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;
    virtual void renderWidget(void);

    int createLaserOptions(void) const;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_TRACKER_DISPLAY_WIDGET_H
