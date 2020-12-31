/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     relocalization_panel.cpp
 * \author   Collin Johnson
 *
 * Definition of RelocalizationPanel.
 */

#include "ui/debug/relocalization_panel.h"
#include "hssh/global_metric/debug_info.h"
#include "hssh/global_metric/map.h"
#include "hssh/global_metric/messages.h"
#include "hssh/local_metric/commands/relocalize_in_lpm.h"
#include "hssh/local_metric/lpm_io.h"
#include "hssh/metrical/relocalization/filter_initializer_impl.h"
#include "hssh/metrical/relocalization/scan_matching_initializer.h"
#include "system/module_communicator.h"
#include "ui/common/shape_creators.h"
#include "ui/debug/debug_ui.h"
#include "ui/debug/relocalization_display_widget.h"
#include "utils/timestamp.h"
#include <cassert>
#include <iostream>


// Put event table in its own box to help the KDevelop parser with the file
namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(RelocalizationPanel, wxEvtHandler)
EVT_RADIOBOX(ID_RELOCALIZATION_MODE_RADIO, RelocalizationPanel::modeChanged)
EVT_BUTTON(ID_RELOCALIZE_LOAD_LPM_BUTTON, RelocalizationPanel::loadLPMPressed)
EVT_BUTTON(ID_RELOCALIZE_LOAD_GMM_BUTTON, RelocalizationPanel::loadGMMPressed)
EVT_CHECKBOX(ID_RELOCALIZE_SHOW_LASER_BOX, RelocalizationPanel::showLaserChecked)
EVT_CHECKBOX(ID_RELOCALIZE_SHOW_ERROR_BOX, RelocalizationPanel::showErrorChecked)
EVT_CHECKBOX(ID_RELOCALIZE_SHOW_PARTICLES_BOX, RelocalizationPanel::showParticlesChecked)
EVT_TOGGLEBUTTON(ID_SET_REGION_RELOCALIZE_BUTTON, RelocalizationPanel::setRegionPressed)
EVT_BUTTON(ID_SEND_REGION_MESSAGE_BUTTON, RelocalizationPanel::sendRegionMessagePressed)
EVT_BUTTON(ID_SEND_FREE_SPACE_MESSAGE_BUTTON, RelocalizationPanel::sendFreeSpaceMessagePressed)
EVT_BUTTON(ID_SEND_SCAN_MATCHING_MESSAGE_BUTTON, RelocalizationPanel::sendScanMatchingMessagePressed)
END_EVENT_TABLE()

}   // namespace ui
}   // namespace vulcan

namespace vulcan
{
namespace ui
{

const int kLocalRelocalizationIndex = 0;
const int kGlobalRelocalizationIndex = 1;

RelocalizationPanel::RelocalizationPanel(const ui_params_t& params, const relocalization_widgets_t widgets)
: mode_(kLocal)
, regionCreator_(new RectangleCreator())
, shouldUpdateInitialRegion(false)
, widgets_(widgets)
, consumer_(nullptr)
{
    assert(widgets_.displayWidget);
    assert(widgets_.loadLPMButton);
    assert(widgets_.loadGMMButton);
    assert(widgets_.showLaserBox);
    assert(widgets_.showErrorBox);
    assert(widgets_.showParticlesBox);
    assert(widgets_.regionNumSamplesText);
    assert(widgets_.regionPosesPerPosText);
    assert(widgets_.freeSpacePosesPerCellText);
    assert(widgets_.freeSpaceStrideText);

    widgets_.displayWidget->setWidgetParams(params);
    widgets_.displayWidget->showLaser(widgets_.showLaserBox->IsChecked());
    widgets_.displayWidget->showErrorEllipse(widgets_.showErrorBox->IsChecked());
    widgets_.displayWidget->showParticles(widgets_.showParticlesBox->IsChecked());
}


void RelocalizationPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.displayWidget->setStatusBar(statusBar);
    widgets_.displayWidget->setRenderContext(context);
}


void RelocalizationPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::local_metric_relocalization_debug_info_t>(this);
    producer.subscribeTo<hssh::global_metric_relocalization_info_t>(this);
}


void RelocalizationPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    consumer_ = consumer;
}


void RelocalizationPanel::update(void)
{
    if (shouldUpdateInitialRegion) {
        initialRegion_ = regionCreator_->getRectangle();
        widgets_.displayWidget->setInitializerRegion(initialRegion_);

        if (regionCreator_->isFinished()) {
            disableRegionSelection();
        }
    }

    widgets_.displayWidget->Refresh();
}


void RelocalizationPanel::saveSettings(utils::ConfigFileWriter& config)
{
    // TODO: Support saving settings
}


void RelocalizationPanel::loadSettings(const utils::ConfigFile& config)
{
    // TODO: Support loading settings
}


void RelocalizationPanel::handleData(const hssh::local_metric_relocalization_debug_info_t& info,
                                     const std::string& channel)
{
    if (mode_ == kLocal) {
        widgets_.displayWidget->setRelocalizationInfo(info.info);
    }
}


void RelocalizationPanel::handleData(const hssh::global_metric_relocalization_info_t& info, const std::string& channel)
{
    if (mode_ == kGlobal) {
        widgets_.displayWidget->setRelocalizationInfo(info.info);
    }
}


void RelocalizationPanel::loadLPM(void)
{
    wxFileDialog loadDialog(widgets_.displayWidget,
                            ("Load LPM..."),
                            "",   // default directory
                            "",   // default filename
                            "Local Perceptual Map (*.lpm)|*.lpm",
                            wxFD_OPEN);

    if (loadDialog.ShowModal() == wxID_OK) {
        wxString path = loadDialog.GetPath();

        hssh::LocalPerceptualMap lpm;
        if (!loadedLPM_) {
            loadedLPM_.reset(new hssh::LocalPerceptualMap());
        }

        hssh::load_lpm_1_0(std::string(path.mb_str()), *loadedLPM_);

        changeDisplayedMap();
    }
}


void RelocalizationPanel::loadGMM(void)
{
    wxFileDialog loadDialog(widgets_.displayWidget,
                            ("Load GMM..."),
                            "",   // default directory
                            "",   // default filename
                            "Global Metric Map (*.gmm)|*.gmm",
                            wxFD_OPEN);

    if (loadDialog.ShowModal() == wxID_OK) {
        wxString path = loadDialog.GetPath();

        if (!loadedGMM_) {
            loadedGMM_.reset(new hssh::GlobalMetricMap());
        }

        loadedGMM_->loadFromFile(std::string(path.mb_str()));

        changeDisplayedMap();
    }
}


void RelocalizationPanel::changeDisplayedMap(void)
{
    if ((mode_ == kLocal) && loadedLPM_) {
        widgets_.displayWidget->setMap(loadedLPM_);
    } else if ((mode_ == kGlobal) && loadedGMM_) {
        widgets_.displayWidget->setMap(loadedGMM_);
    }
}


void RelocalizationPanel::enableRegionSelection(void)
{
    shouldUpdateInitialRegion = true;
    widgets_.displayWidget->showInitialRegion(true);

    regionCreator_->reset();
    widgets_.displayWidget->pushMouseHandler(regionCreator_.get());
}


void RelocalizationPanel::disableRegionSelection(void)
{
    widgets_.displayWidget->removeMouseHandler(regionCreator_.get());
    shouldUpdateInitialRegion = false;
}


void RelocalizationPanel::sendRelocalizationMessage(const std::shared_ptr<hssh::FilterInitializer>& initializer)
{
    if (mode_ == kLocal) {
        sendLocalRelocalizationMessage(initializer);
    } else if (mode_ == kGlobal) {
        sendGlobalRelocalizationMessage(initializer);
    }
}


void RelocalizationPanel::sendLocalRelocalizationMessage(const std::shared_ptr<hssh::FilterInitializer>& initializer)
{
    assert(consumer_);

    if (!loadedLPM_) {
        std::cerr << "ERROR: RelocalizationPanel: No loaded LPM to relocalization in.\n";
        return;
    }

    std::shared_ptr<hssh::LocalMetricCommand> message =
      std::make_shared<hssh::RelocalizeInLpmCommand>("DebugUI", *loadedLPM_, initializer);
    consumer_->sendMessage(message);
}


void RelocalizationPanel::sendGlobalRelocalizationMessage(const std::shared_ptr<hssh::FilterInitializer>& initializer)
{
    assert(consumer_);

    if (!loadedGMM_) {
        std::cerr << "ERROR: RelocalizationPanel: No loaded GMM to relocalization in.\n";
        return;
    }

    hssh::global_metric_relocalization_request_message_t msg;
    msg.initializer = initializer;
    msg.map = *loadedGMM_;

    consumer_->sendMessage(msg);
}


void RelocalizationPanel::modeChanged(wxCommandEvent& event)
{
    if (event.GetSelection() == kLocalRelocalizationIndex) {
        mode_ = kLocal;
    } else if (event.GetSelection() == kGlobalRelocalizationIndex) {
        mode_ = kGlobal;
    } else {
        std::cerr << "ERROR: RelocalizationPanel: Unknown mode: " << event.GetSelection() << '\n';
        assert(false);
    }

    changeDisplayedMap();
}


void RelocalizationPanel::loadLPMPressed(wxCommandEvent& event)
{
    loadLPM();
}


void RelocalizationPanel::loadGMMPressed(wxCommandEvent& event)
{
    loadGMM();
}


void RelocalizationPanel::showLaserChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showLaser(event.IsChecked());
}


void RelocalizationPanel::showErrorChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showErrorEllipse(event.IsChecked());
}


void RelocalizationPanel::showParticlesChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showParticles(event.IsChecked());
}


void RelocalizationPanel::setRegionPressed(wxCommandEvent& event)
{
    if (event.IsChecked()) {
        enableRegionSelection();
    } else {
        disableRegionSelection();
    }
}


void RelocalizationPanel::sendRegionMessagePressed(wxCommandEvent& event)
{
    // To send the region message:
    //  - Grab the rectangle defined by the user
    //  - Convert the samples text to a number
    //  - Convert the poses per position text to a number
    //  - Create a RegionFilterInitializer
    //  - Pass the initializer to the method for sending the actual message

    long numSamples = 0;
    long posesPerPosition = 0;

    if (!widgets_.regionNumSamplesText->GetValue().ToLong(&numSamples) || (numSamples <= 0)) {
        std::cerr << "ERROR: RelocalizationPanel: Invalid value for numSamples: " << numSamples
                  << " Using default of 30,000\n";
        numSamples = 30000;
    }

    if (!widgets_.regionPosesPerPosText->GetValue().ToLong(&posesPerPosition) || (posesPerPosition <= 0)) {
        std::cerr << "ERROR: RelocalizationPanel: Invalid value for poses per position: " << posesPerPosition
                  << " Using default of 60\n";
        posesPerPosition = 60;
    }

    sendRelocalizationMessage(
      std::make_shared<hssh::RegionFilterInitializer>(initialRegion_, numSamples, posesPerPosition));
}


void RelocalizationPanel::sendFreeSpaceMessagePressed(wxCommandEvent& event)
{
    // To send the free space message:
    //  - Convert the stride text to a number
    //  - Convert the poses per position text to a number
    //  - Create a FreeSpaceInitializer
    //  - Pass the initializer to the method for sending the actual message

    long cellStride = 0;
    long posesPerCell = 0;

    if (!widgets_.freeSpaceStrideText->GetValue().ToLong(&cellStride) || (cellStride <= 0)) {
        std::cerr << "ERROR: RelocalizationPanel: Invalid value for cell stride: " << cellStride << '\n';
        return;
    }

    if (!widgets_.freeSpacePosesPerCellText->GetValue().ToLong(&posesPerCell) || (posesPerCell <= 0)) {
        std::cerr << "ERROR: RelocalizationPanel: Invalid value for poses per cell: " << posesPerCell << '\n';
        return;
    }

    sendRelocalizationMessage(std::make_shared<hssh::FreeSpaceFilterInitializer>(cellStride, posesPerCell));
}


void RelocalizationPanel::sendScanMatchingMessagePressed(wxCommandEvent& event)
{
    // To send the scan matching message:
    //  - Create a ScanMatchingInitializer
    //  - Pass the initializer to the method for sending the actual message

    sendRelocalizationMessage(std::make_shared<hssh::ScanMatchingInitializer>());
}

}   // namespace ui
}   // namespace vulcan
