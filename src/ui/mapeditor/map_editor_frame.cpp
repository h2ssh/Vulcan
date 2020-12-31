/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     map_editor_frame.cpp
 * \author   Collin Johnson
 *
 * Implementation of MapEditorFrame.
 */

#include "ui/mapeditor/map_editor_frame.h"
#include "ui/common/ui_params.h"
#include "ui/mapeditor/global_topo_editor_panel.h"
#include "ui/mapeditor/local_topo_editor_panel.h"
#include "ui/mapeditor/metric_editor_panel.h"
#include "ui/mapeditor/metric_editor_widget.h"

namespace vulcan
{
namespace ui
{

MapEditorFrame::MapEditorFrame(const ui_params_t& params, const hssh::local_topology_params_t& localTopoParams)
: EditorFrame(0)
{
    setupMetricPanel(params);
    setupLocalTopoPanel(params, localTopoParams);
    setupGlobalTopoPanel(params);

    // Now that all panels are setup, do the final initialization for the frame
    initialize(mapEditorNotebook, params.mainFrameParams.framesPerSecond, metricEditorWidget, editorStatusBar);
}


MapEditorFrame::~MapEditorFrame(void)
{
}


void MapEditorFrame::setupMetricPanel(const ui_params_t& params)
{
    metric_editor_panel_widgets_t widgets;
    widgets.editorWidget = metricEditorWidget;
    widgets.cellEditModeButton = cellEditModeButton;
    widgets.editCellTypeRadio = editCellTypeRadioBox;
    widgets.floodEditModeButton = floodEditMetricButton;
    widgets.lineEditModeButton = lineEditMetricButton;

    metricPanel_ = new MetricEditorPanel(params, widgets);

    addPanel(metricPanel_, localMetricNotebookPanel);
}


void MapEditorFrame::setupLocalTopoPanel(const ui_params_t& params,
                                         const hssh::local_topology_params_t& localTopoParams)
{
    local_topo_editor_panel_widgets_t widgets;
    widgets.widget = localTopoEditorWidget;
    widgets.mapNameText = currentMapNameText;
    widgets.assignLabelsButton = assignLabelsButton;
    widgets.clearLabelsButton = clearLocalAreaLabelsButton;
    widgets.clearSelectedButton = clearSelectedAreasButton;
    widgets.editModeRadio = localTopoEditModeRadio;
    widgets.generateGatewaysButton = generateGatewaysButton;
    widgets.createGatewaysButton = handCreateGatewaysButton;
    widgets.loadGatewaysButton = loadGatewaysButton;
    widgets.createAreasButton = createAreasFromGatewaysButton;
    widgets.labelToAssignRadio = labelToAssignRadio;
    widgets.mergedSelectedButton = mergeSelectedAreasButton;
    widgets.resetMergedButton = resetMergedAreasButton;
    widgets.selectAreasButton = selectLocalAreasButton;
    widgets.labelRemainingButton = labelAllAreasButton;
    widgets.simplifyViaLabelsButton = simplifyViaLabelsButton;
    widgets.labeledDataList = labelDataList;
    widgets.trainingDataList = trainingDataList;
    widgets.testDataList = testDataList;

    localTopoPanel_ = new LocalTopoEditorPanel(widgets, params, localTopoParams);

    addPanel(localTopoPanel_, localTopoNotebookPanel);
}


void MapEditorFrame::setupGlobalTopoPanel(const ui_params_t& params)
{
    // TODO

    //     addPanel(globalTopoPanel.get());
}

}   // namespace ui
}   // namespace vulcan
