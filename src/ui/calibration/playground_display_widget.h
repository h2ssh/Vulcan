/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     playground_display_widget.h
* \author   Collin Johnson
*
* Declaration of PlaygroundDisplayWidget.
*/

#ifndef UI_CALIBRATION_PLAYGROUND_DISPLAY_WIDGET_H
#define UI_CALIBRATION_PLAYGROUND_DISPLAY_WIDGET_H

#include <ui/components/grid_based_display_widget.h>
#include <ui/common/ui_color.h>
#include <hssh/local_metric/lpm.h>

namespace vulcan
{
namespace ui
{

struct calibration_ui_params_t;

/**
* PlaygroundDisplayWidget displays the playground configuration and current status for the Playground
* tab of the CalibrationUI. The widget does the following things:
*
*   1) Displays the LPM in which the robot is moving.
*   2) Allows creating the playground boundary via specifying a polygon on the screen.
*   3) Allows creating the target region by specifying a polygon on the screen.
*   4) Displays the currently generated target, along with the trajectory taken by the robot along the way.
*/
class PlaygroundDisplayWidget : public GridBasedDisplayWidget
{
public:

    /**
    * Constructor for PlaygroundDisplayWidget.
    */
    PlaygroundDisplayWidget(const calibration_ui_params_t& params);
    
    // Set target region and set playground boundary accept a flag indicating if they are final as the final color showing the
    // region will likely be different than the intermediate color -- probably darker going to lighter
    void setTargetRegion      (const math::Shape<float>& target,     bool final);
    void setPlaygroundBoundary(const math::Shape<float>& playground, bool final);

    // Methods for providing the data to be rendered
    void setLPM(const hssh::LocalPerceptualMap& lpm);
    void setPose(const pose_t& pose);
    void setPath(const mpepc::metric_waypoint_path_t& path);

private:
    
    // GridBasedDisplayWidget interface
    virtual Point<uint16_t> convertWorldToGrid(const Point<float>& world) const;
    virtual void                  renderWidget(void);
    
    // Helper renderers
    void drawPolygonBoundary(const std::vector<Point<float>>& boundary, const GLColor& color);
    
    hssh::LocalPerceptualMap lpm;
    pose_t            pose;
    
    std::vector<Point<float>> targetRegion;
    std::vector<Point<float>> playgroundBoundary;
    
    bool targetIsFinal;
    bool playgroundIsFinal;
    
    GLColor targetColor;
    GLColor playgroundColor;
};

}
}

#endif // UI_CALIBRATION_PLAYGROUND_DISPLAY_WIDGET_H
