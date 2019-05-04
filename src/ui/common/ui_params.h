/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ui_params.h
* \author   Collin Johnson
*
* Definition of params structs for the ENTIRE UI. Might need to be
* separated into different files based functionality in the future.
*/

#ifndef UI_COMMON_UI_PARAMS_H
#define UI_COMMON_UI_PARAMS_H

#include <ui/common/ui_color.h>
#include <cstdint>
#include <string>
#include <vector>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace ui
{

struct lpm_display_params_t
{
    GLColor frontLaserColor;
    GLColor backLaserColor;
    GLColor warningLaserColor;
    GLColor criticalLaserColor;
    GLColor intensityLaserColor;
    GLColor extractedLinesColor;

    GLColor occupiedColor;
    GLColor dynamicColor;
    GLColor limitedVisibilityColor;
    GLColor hazardColor;
    GLColor quasiStaticColor;

    GLColor robotColor;
    GLColor traceColor;
    GLColor odometryTraceColor;

    GLColor particlesColor;

    GLColor targetCircleColor;
    GLColor targetArrowColor;
    GLColor metricPathColor;
    GLColor reachedTargetCircleColor;
    GLColor reachedTargetArrowColor;

    GLColor acquiringTrackingColor;
    GLColor activeTrackingColor;
    GLColor inactiveTrackingColor;

    uint16_t maxTraceLength;

    std::string messageChannel;
    std::string metricPathChannel;
};

struct local_topo_display_params_t
{
    GLColor frontierColor;
    GLColor junctionColor;
    GLColor deadEndColor;
    GLColor gatewayColor;
    GLColor sourceColor;

    GLColor skeletonCellColor;
    GLColor reducedCellColor;

    GLColor pathColor;
    GLColor decisionColor;
    GLColor destinationColor;
    GLColor unknownColor;
    GLColor pathDestColor;
    GLColor pathDecisionColor;
    GLColor destDecisionColor;

    GLColor              isovistRayColor;
    std::vector<GLColor> isovistFieldColors;

    // Units are CELLS
    float minAnchorPointRadius;
    float maxAnchorPointRadius;

    GLColor exploredColor;
    GLColor gatewayEndpointColor;

    GLColor pathStartColor;
    GLColor pathEndColor;
};

struct global_topo_display_params_t
{
    GLColor placeColor;
    GLColor pathColor;
    GLColor frontierColor;
    GLColor locationColor;

    GLColor nodeColor;
    GLColor edgeColor;

    GLColor bestNodeColor;
    GLColor hoverNodeColor;
    GLColor selectedMapAColor;
    GLColor selectedMapBColor;
    GLColor selectedMapCColor;

    std::string correctMapOutputChannel;
};

struct relocalization_display_params_t
{
    std::string requestChannel;
};

struct scripting_display_params_t
{
    GLColor hoverColor;
    GLColor selectedColor;
    GLColor finalizedColor;
};

struct calibration_display_params_t
{

};

struct metric_planner_display_params_t
{
    GLColor trajectoryColorRed;
    GLColor trajectoryColorBlue;
    GLColor trajectoryColorGreen;

    GLColor robotColorRed;
    GLColor robotColorBlue;
    GLColor robotColorGreen;
    GLColor robotColorGrey;
    GLColor robotColorLightBlue;
    GLColor robotColorViolet;

    GLColor flowTailColor;
    GLColor flowHeadColor;

    // RRT params
    GLColor nodeColor;
    GLColor edgeColor;
    GLColor startColor;
    GLColor targetColor;
    GLColor goalRegionColor;
    GLColor goalPathColor;

    std::string mpepcConfig;
    std::string collisionModelConfig;
};

struct decision_planner_display_params_t
{
    std::string targetSequenceChannel;

    GLColor placeEntryFragmentColor;
    GLColor placeExitFragmentColor;
    GLColor pathExitPointColor;
};

struct goal_planner_display_params_t
{
    GLColor globalLocationColor;
    GLColor globalTargetColor;

    GLColor placeVertexColor;
    GLColor pathSegmentVertexColor;
    GLColor edgeColor;
    GLColor graphPathColor;

    GLColor routeElementColor;
    GLColor visitedElementColor;
    GLColor activeElementColor;
    GLColor remainingElementColor;

    std::string routeCommandOutputChannel;
    std::string goalTargetOutputChannel;
    std::string setGlobalLocationOutputChannel;
};

struct vision_display_params_t
{
    GLColor groundPlaneColor;
};

struct main_frame_params_t
{
    int8_t framesPerSecond;
};

struct ui_params_t
{
    lpm_display_params_t              lpmParams;
    local_topo_display_params_t       localTopoParams;
    global_topo_display_params_t      globalTopoParams;
    relocalization_display_params_t   relocalizationParams;
    scripting_display_params_t        scriptingParams;
    calibration_display_params_t      calibrationDisplayParams;
    metric_planner_display_params_t   metricPlannerDisplayParams;
    decision_planner_display_params_t decisionPlannerParams;
    goal_planner_display_params_t     goalPlannerParams;
    vision_display_params_t           visionParams;
    main_frame_params_t               mainFrameParams;
};


/**
* load_ui_params loads the parameters for the current user interface from
* the provided configuration file.
*/
ui_params_t load_ui_params(const utils::ConfigFile& config);

}
}

#endif // UI_COMMON_UI_PARAMS_H
