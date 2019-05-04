/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     vision_panel.cpp
* \author   Collin Johnson
*
* Definition of VisionPanel.
*/

#include <ui/debug/vision_panel.h>
#include <ui/debug/vision_display_widget.h>
#include <ui/debug/debug_ui.h>
#include <vision/felzenszwalb_segmenter.h>
#include <vision/wassenberg_segmenter.h>
#include <utils/auto_mutex.h>
#include <system/module_communicator.h>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(VisionPanel, wxEvtHandler)
    EVT_CHECKBOX(ID_SHOW_SEGMENTS_BOX,      VisionPanel::showSegmentsChecked)
    EVT_SLIDER(ID_MIN_EDGE_WEIGHT_SLIDER,   VisionPanel::changedMinEdgeWeight)
    EVT_SLIDER(ID_MAX_EDGE_WEIGHT_SLIDER,   VisionPanel::changedMaxEdgeWeight)
    EVT_SLIDER(ID_PIXEL_SIGMA_SLIDER,       VisionPanel::changedPixelSigma)
    EVT_SLIDER(ID_CREDIT_MULTIPLIER_SLIDER, VisionPanel::changedCreditMultiplier)
    EVT_SLIDER(ID_FILTER_WIDTH_SLIDER,      VisionPanel::changedFilterWidth)
END_EVENT_TABLE()


VisionPanel::VisionPanel(const ui_params_t& params, const vision_panel_widgets_t& widgets)
: widgets_(widgets)
, haveNewImage_(false)
, shouldFindSegments_(false)
{
    widgets_.displayWidget->setWidgetParams(params.visionParams);

    reloadSegmenter();
}


void VisionPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.displayWidget->setStatusBar(statusBar);
    widgets_.displayWidget->setRenderContext(context);
}


void VisionPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<Image>(this);
}


void VisionPanel::setConsumer(system::ModuleCommunicator* consumer)
{
}


void VisionPanel::update(void)
{
    if(haveNewImage_)
    {
        utils::AutoMutex autoLock(imageLock_);

        if(shouldFindSegments_)
        {
            std::vector<vision::image_segment_t> segments;
            imageSegmenter_->segmentImage(image_, segments);
            widgets_.displayWidget->setImageSegments(std::move(segments));
        }

        widgets_.displayWidget->setImage(std::move(image_));
        haveNewImage_ = false;
    }

    widgets_.displayWidget->Refresh();
}


void VisionPanel::saveSettings(utils::ConfigFileWriter& config)
{

}


void VisionPanel::loadSettings(const utils::ConfigFile& config)
{

}


void VisionPanel::handleData(const Image& image, const std::string& channel)
{
    utils::AutoMutex autoLock(imageLock_);
    image_        = image;
    haveNewImage_ = true;
}


void VisionPanel::reloadSegmenter(void)
{
    // Initial params that seem to work pretty well
    /*
    * [ FelzenszwalbSegmenterParameters]
    * k                = 882
    * sigma            = 1.3
    * min_segment_size = 100
    *
    * [WassenbergSegmenterParameters]
    * sigma            = 0.8
    * min_segment_size = 50
    *
    * min_edge_weight   = 3
    * max_edge_weight   = 10
    * pixel_sigma       = 2
    * credit_multiplier = 1.0
    */

    const float kWeightPerTick = 0.1f;

//     vision::wassenberg_params_t wassParams;
//     wassParams.creditMultiplier = widgets_.creditMultiplierSlider->GetValue() * kWeightPerTick;
//     wassParams.pixelSigma = widgets_.pixelSigmaSlider->GetValue() * kWeightPerTick;
//     wassParams.maxEdgeWeight = widgets_.maxEdgeWeightSlider->GetValue();
//     wassParams.minEdgeWeight = widgets_.minEdgeWeightSlider->GetValue();
//     wassParams.graphParams.sigma = widgets_.filterWidthSlider->GetValue() * kWeightPerTick;
//     wassParams.graphParams.minSegmentSize = 25;
//     wassParams.graphParams.initialThreshold = 1000.0; // just need to be larger than max component distance

    vision::felzenszwalb_params_t felzParams;
    felzParams.k = widgets_.minEdgeWeightSlider->GetValue() * widgets_.maxEdgeWeightSlider->GetValue();
    felzParams.graphParams.minSegmentSize = 100;
    felzParams.graphParams.sigma = widgets_.filterWidthSlider->GetValue() * kWeightPerTick;;
    felzParams.graphParams.initialThreshold = 300;

    utils::AutoMutex autoLock(imageLock_);
//     imageSegmenter_.reset(new vision::WassenbergSegmenter(wassParams));
    imageSegmenter_.reset(new vision::FelzenszwalbSegmenter(felzParams));

    haveNewImage_ = true;
}


void VisionPanel::showSegmentsChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showImageSegments(event.IsChecked());
    shouldFindSegments_ = event.IsChecked();
}


void VisionPanel::changedMinEdgeWeight(wxCommandEvent& event)
{
    reloadSegmenter();
}


void VisionPanel::changedMaxEdgeWeight(wxCommandEvent& event)
{
    reloadSegmenter();
}


void VisionPanel::changedPixelSigma(wxCommandEvent& event)
{
    reloadSegmenter();
}


void VisionPanel::changedCreditMultiplier(wxCommandEvent& event)
{
    reloadSegmenter();
}


void VisionPanel::changedFilterWidth(wxCommandEvent& event)
{
    reloadSegmenter();
}

} // namespace ui
} // namespace vulcan
