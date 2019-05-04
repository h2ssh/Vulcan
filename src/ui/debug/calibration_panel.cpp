/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     calibration_panel.cpp
* \author   Jong Jin Park
*
* Definition of CalibrationPanel.
*/

#include <ui/debug/calibration_panel.h>
#include <ui/debug/calibration_display_widget.h>
#include <ui/debug/debug_ui.h>
#include <ui/common/file_dialog_settings.h>
#include <ui/common/ui_params.h>
#include <calibration/laser/tilt_calibration.h>
#include <core/laser_scan.h>
#include <system/module_communicator.h>
#include <utils/auto_mutex.h>
#include <fstream>
#include <wx/msgdlg.h>
#include <cassert>

// The parsing of EVT_TEXT is broken in KDevelop 4, so put in a separate namespace to have the rest of the file still
// be parsed correctly
namespace vulcan
{
namespace ui
{
BEGIN_EVENT_TABLE(CalibrationPanel, wxEvtHandler)
    EVT_CHECKBOX(ID_SHOW_FRONT_LASER_BOX,              CalibrationPanel::showFrontLaserChecked)
    EVT_CHECKBOX(ID_SHOW_BACK_LASER_BOX,               CalibrationPanel::showBackLaserChecked)
    EVT_TEXT(ID_FRONT_LASER_COORDS_X,                  CalibrationPanel::setFrontLaserX)
    EVT_TEXT(ID_FRONT_LASER_COORDS_Y,                  CalibrationPanel::setFrontLaserY)
    EVT_TEXT(ID_FRONT_LASER_COORDS_THETA,              CalibrationPanel::setFrontLaserTheta)
    EVT_TEXT(ID_FRONT_LASER_PITCH_TEXT,                CalibrationPanel::setFrontLaserPitch)
    EVT_TEXT(ID_FRONT_LASER_ROLL_TEXT,                 CalibrationPanel::setFrontLaserRoll)
    EVT_TEXT(ID_BACK_LASER_COORDS_X,                   CalibrationPanel::setBackLaserX)
    EVT_TEXT(ID_BACK_LASER_COORDS_Y,                   CalibrationPanel::setBackLaserY)
    EVT_TEXT(ID_BACK_LASER_COORDS_THETA,               CalibrationPanel::setBackLaserTheta)
    EVT_TEXT(ID_BACK_LASER_PITCH_TEXT,                 CalibrationPanel::setBackLaserPitch)
    EVT_TEXT(ID_BACK_LASER_ROLL_TEXT,                  CalibrationPanel::setBackLaserRoll)
    EVT_BUTTON(ID_LOAD_TILT_DATA_BUTTON, CalibrationPanel::loadTiltDataPressed)
    EVT_RADIOBOX(ID_TILT_LASER_RADIO, CalibrationPanel::tiltLaserChanged)
    EVT_TEXT(ID_LINE_START_INDEX_TEXT, CalibrationPanel::lineStartIndexChanged)
    EVT_TEXT(ID_LINE_END_INDEX_TEXT, CalibrationPanel::lineEndIndexChanged)
    EVT_BUTTON(ID_CALIBRATE_PITCH_BUTTON, CalibrationPanel::calibratePitchPressed)
    EVT_BUTTON(ID_CALIBRATE_ROLL_BUTTON, CalibrationPanel::calibrateRollPressed)
END_EVENT_TABLE()
}
}

namespace vulcan
{
namespace ui
{

CalibrationPanel::CalibrationPanel(const ui_params_t& params, const calibration_panel_widgets_t& widgets)
: mode_(manual)
, newMode_(manual)
, widgets(widgets)
, haveFrontLaserPose_(false)
, haveBackLaserPose_(false)
, tiltLaser_(front)
, params(params.calibrationDisplayParams)
{
    assert(widgets.displayWidget);
    assert(widgets.showFrontLaser);
    assert(widgets.showBackLaser);
    assert(widgets.frontLaserTextX);
    assert(widgets.frontLaserTextY);
    assert(widgets.frontLaserTextTheta);
    assert(widgets.frontLaserTextPitch);
    assert(widgets.frontLaserTextRoll);
    assert(widgets.backLaserTextX);
    assert(widgets.backLaserTextY);
    assert(widgets.backLaserTextTheta);
    assert(widgets.backLaserTextPitch);
    assert(widgets.backLaserTextRoll);
    assert(widgets.tiltLaserToShowRadio);
    assert(widgets.tiltStartIndex);
    assert(widgets.tiltEndIndex);
    assert(widgets.calibratePitchButton);
    assert(widgets.calibrateRollButton);
    
    widgets.displayWidget->setWidgetParams(params);
    
    enterManualMode();
}


void CalibrationPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets.displayWidget->setRenderContext(context); // no status bar for openGLwidgets.
    widgets.displayWidget->enablePanning(); // allow scrolling and zooming.
    widgets.displayWidget->enableZooming();
    widgets.displayWidget->showFrontLaserScans(true); // show laser scans by default
    widgets.displayWidget->showBackLaserScans(true);
}

void CalibrationPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<polar_laser_scan_t>(this);
}


void CalibrationPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    if(consumer)
    {
        this->consumer = consumer;
    }
}

void CalibrationPanel::update(void)
{    
    if(newMode_ != mode_)
    {
        mode_ = newMode_;
        switch(mode_)
        {
        case manual:
            enterManualMode();
            break;
            
        case tilt:
            enterTiltMode();
            break;
        }   
    }
    
    switch(mode_)
    {
    case manual:
        updateManualMode();
        break;
        
    case tilt:
        updateTiltMode();
        break;
    }
    
    widgets.displayWidget->Refresh();
}

void CalibrationPanel::saveSettings(utils::ConfigFileWriter& config)
{

}


void CalibrationPanel::loadSettings(const utils::ConfigFile& config)
{

}


void CalibrationPanel::handleData(const polar_laser_scan_t& scan, const std::string& channel)
{
    utils::AutoMutex autoLock(laserLock_);
    
    if(scan.laserId == kFrontLaserId)
    {
        frontScan_ = scan;
        
        if(!haveFrontLaserPose_)
        {
            frontLaserPoseInRobotFrame = frontScan_.offset.toPose();
        }
    }
    else if(scan.laserId == kBackLaserId)
    {
        backScan_ = scan;
        
        if(!haveBackLaserPose_)
        {
            backLaserPoseInRobotFrame = backScan_.offset.toPose();
        }
    }
    
    // When laser data is received, the UI should be in manual mode
   newMode_ = manual;
}


void CalibrationPanel::enterManualMode(void)
{
    stopTiltMode();
    startManualMode();
}


void CalibrationPanel::updateManualMode(void)
{
    {
        utils::AutoMutex autoLock(laserLock_);
        widgets.displayWidget->setFrontLaser(frontScan_);
        widgets.displayWidget->setBackLaser(backScan_);
        
        if(!haveFrontLaserPose_ && (frontScan_.numRanges > 0))
        {
            widgets.frontLaserTextX->SetValue(wxString::FromDouble(frontLaserPoseInRobotFrame.x, 4));
            widgets.frontLaserTextY->SetValue(wxString::FromDouble(frontLaserPoseInRobotFrame.y, 4));
            widgets.frontLaserTextTheta->SetValue(wxString::FromDouble(frontLaserPoseInRobotFrame.theta, 4));
            widgets.displayWidget->setFrontLaserPoseInRobotFrame(frontLaserPoseInRobotFrame);
            
            haveFrontLaserPose_ = true;
        }
        
        if(!haveBackLaserPose_ && (backScan_.numRanges > 0))
        {
            widgets.backLaserTextX->SetValue(wxString::FromDouble(backLaserPoseInRobotFrame.x, 4));
            widgets.backLaserTextY->SetValue(wxString::FromDouble(backLaserPoseInRobotFrame.y, 4));
            widgets.backLaserTextTheta->SetValue(wxString::FromDouble(backLaserPoseInRobotFrame.theta, 4));
            widgets.displayWidget->setBackLaserPoseInRobotFrame(backLaserPoseInRobotFrame);
            
            haveBackLaserPose_ = true;
        }
    }
}


void CalibrationPanel::startManualMode(void)
{
    toggleManualWidgets(true);
}


void CalibrationPanel::stopManualMode(void)
{
    toggleManualWidgets(false);
}


void CalibrationPanel::toggleManualWidgets(bool enable)
{
    // Enable all manual widgets
    widgets.showFrontLaser->Enable(enable);
    widgets.showBackLaser->Enable(enable);
    widgets.frontLaserTextX->Enable(enable);
    widgets.frontLaserTextY->Enable(enable);
    widgets.frontLaserTextTheta->Enable(enable);
    widgets.frontLaserTextPitch->Enable(enable);
    widgets.frontLaserTextRoll->Enable(enable);
    widgets.backLaserTextX->Enable(enable);
    widgets.backLaserTextY->Enable(enable);
    widgets.backLaserTextTheta->Enable(enable);
    widgets.backLaserTextPitch->Enable(enable);
    widgets.backLaserTextRoll->Enable(enable);
}


void CalibrationPanel::enterTiltMode(void)
{
    stopManualMode();
    startTiltMode();
}


void CalibrationPanel::updateTiltMode(void)
{
    // In tilt mode, the front laser is the points on the line being fit and the back laser is the points not on the
    // line
    setTiltLines(frontScan_, backScan_);
    widgets.displayWidget->setFrontLaser(frontScan_);
    widgets.displayWidget->setBackLaser(backScan_);
}


void CalibrationPanel::startTiltMode(void)
{
    // Reset the pose offsets to 0
    widgets.displayWidget->setFrontLaserPoseInRobotFrame(pose_t());
    widgets.displayWidget->setBackLaserPoseInRobotFrame(pose_t());
    
    toggleTiltWidgets(true);
}


void CalibrationPanel::stopTiltMode(void)
{
    toggleTiltWidgets(false);
}


void CalibrationPanel::toggleTiltWidgets(bool enable)
{
    widgets.tiltLaserToShowRadio->Enable(enable);
    widgets.tiltStartIndex->Enable(enable);
    widgets.tiltEndIndex->Enable(enable);
    widgets.calibratePitchButton->Enable(enable);
    widgets.calibrateRollButton->Enable(enable);
}



void CalibrationPanel::calibrateTilt(sensors::SensorLog::laser_iterator begin, 
                                     sensors::SensorLog::laser_iterator end, 
                                     bool doPitch)
{
    using namespace calibration;
    
    tilt_calibration_results_t results;
    
    if(doPitch)
    {
        results = calibrate_laser_pitch(begin, end, tiltRange_.first, tiltRange_.second, 1e-4, 0.05);
    }
    else // calibrate roll
    {
        results = calibrate_laser_roll(begin, end, tiltRange_.first, tiltRange_.second, 1e-4, 0.05);
    }
    
    wxString resultString;
    resultString << "Best tilt (pitch, roll): (" << results.bestTilt.pitch << ',' << results.bestTilt.roll << ")\n";
    
    std::string resultName = tiltData_.name();
    resultName += "_tilt.txt";
    std::cout << "Saving results of calibration to " << resultName << '\n';
    std::ofstream pitchResults(resultName);
    for(auto& result : results.tiltErrors)
    {
        pitchResults << result.first.pitch << ' ' << result.first.roll << ' ' << result.second << '\n';
    }
    
    wxMessageDialog dialog(widgets.displayWidget, resultString);
    dialog.ShowModal();
}


void CalibrationPanel::setTiltLines(polar_laser_scan_t& lineScan, polar_laser_scan_t& notLineScan)
{
    assert(lineScan.numRanges == notLineScan.numRanges);
    
    // Just negate all the bad looking data so it isn't shown
    for(int n = 0; n < static_cast<int>(lineScan.ranges.size()); ++n)
    {
        if(n >= tiltRange_.first && n < tiltRange_.second)
        {
            lineScan.ranges[n] = std::copysign(lineScan.ranges[n], 1);
            notLineScan.ranges[n] = std::copysign(notLineScan.ranges[n], -1);
        }
        else 
        {
            notLineScan.ranges[n] = std::copysign(notLineScan.ranges[n], 1);
            lineScan.ranges[n] = std::copysign(lineScan.ranges[n], -1);
        }
    }
}


void CalibrationPanel::showFrontLaserChecked(wxCommandEvent& event)
{
    widgets.displayWidget->showFrontLaserScans(event.IsChecked());
}


void CalibrationPanel::showBackLaserChecked(wxCommandEvent& event)
{
    widgets.displayWidget->showBackLaserScans(event.IsChecked());
}


void CalibrationPanel::setFrontLaserX(wxCommandEvent& event)
{
    double x;
    wxString coordString = widgets.frontLaserTextX->GetLineText(0);
    
    if(coordString.ToDouble(&x))
    {
        frontLaserPoseInRobotFrame.x = x;
        widgets.displayWidget->setFrontLaserPoseInRobotFrame(frontLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setFrontLaserY(wxCommandEvent& event)
{
    double y;
    wxString coordString = widgets.frontLaserTextY->GetLineText(0);
    
    if(coordString.ToDouble(&y))
    {
        frontLaserPoseInRobotFrame.y = y;
        widgets.displayWidget->setFrontLaserPoseInRobotFrame(frontLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setFrontLaserTheta(wxCommandEvent& event)
{
    double theta;
    wxString coordString = widgets.frontLaserTextTheta->GetLineText(0);
    
    if(coordString.ToDouble(&theta))
    {
        frontLaserPoseInRobotFrame.theta = theta;
        widgets.displayWidget->setFrontLaserPoseInRobotFrame(frontLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setFrontLaserPitch(wxCommandEvent& event)
{
    double pitch;
    wxString coordString = widgets.frontLaserTextPitch->GetLineText(0);
    
    if(coordString.ToDouble(&pitch))
    {
        frontLaserPoseInRobotFrame.phi = pitch;
        widgets.displayWidget->setFrontLaserPoseInRobotFrame(frontLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setFrontLaserRoll(wxCommandEvent& event)
{
    double roll;
    wxString coordString = widgets.frontLaserTextRoll->GetLineText(0);
    
    if(coordString.ToDouble(&roll))
    {
        frontLaserPoseInRobotFrame.rho = roll;
        widgets.displayWidget->setFrontLaserPoseInRobotFrame(frontLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setBackLaserX(wxCommandEvent& event)
{
    double x;
    wxString coordString = widgets.backLaserTextX->GetLineText(0);
    
    if(coordString.ToDouble(&x))
    {
        backLaserPoseInRobotFrame.x = x;
        widgets.displayWidget->setBackLaserPoseInRobotFrame(backLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setBackLaserY(wxCommandEvent& event)
{
    double y;
    wxString coordString = widgets.backLaserTextY->GetLineText(0);
    
    if(coordString.ToDouble(&y))
    {
        backLaserPoseInRobotFrame.y = y;
        widgets.displayWidget->setBackLaserPoseInRobotFrame(backLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setBackLaserTheta(wxCommandEvent& event)
{
    double theta;
    wxString coordString = widgets.backLaserTextTheta->GetLineText(0);
    
    if(coordString.ToDouble(&theta))
    {
        backLaserPoseInRobotFrame.theta = theta;
        widgets.displayWidget->setBackLaserPoseInRobotFrame(backLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setBackLaserPitch(wxCommandEvent& event)
{
    double pitch;
    wxString coordString = widgets.backLaserTextPitch->GetLineText(0);
    
    if(coordString.ToDouble(&pitch))
    {
        backLaserPoseInRobotFrame.phi = pitch;
        widgets.displayWidget->setBackLaserPoseInRobotFrame(backLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::setBackLaserRoll(wxCommandEvent& event)
{
    double roll;
    wxString coordString = widgets.backLaserTextRoll->GetLineText(0);
    
    if(coordString.ToDouble(&roll))
    {
        backLaserPoseInRobotFrame.rho = roll;
        widgets.displayWidget->setBackLaserPoseInRobotFrame(backLaserPoseInRobotFrame);
    }
}


void CalibrationPanel::loadTiltDataPressed(wxCommandEvent& event)
{
    // Create file dialog and load the appropriate log file
    wxFileDialog loadDialog(widgets.displayWidget,
                            wxT("Select log file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.log"),
                            kFileOpenFlags);
    
    if(loadDialog.ShowModal() == wxID_OK)
    {
        auto path = std::string{loadDialog.GetPath().mb_str()};
        tiltData_ = sensors::SensorLog(path);
        
        // Set the front and rear laser scans to be the first members of the log
        if(tiltData_.sizeFrontLaser() > 0)
        {
            frontScan_ = *tiltData_.beginFrontLaser();
            backScan_ = frontScan_;
        }
        else if(tiltData_.sizeBackLaser() > 0)
        {
            backScan_ = *tiltData_.beginBackLaser();
            frontScan_ = backScan_;
        }
        
        newMode_ = tilt;
    }
}


void CalibrationPanel::tiltLaserChanged(wxCommandEvent& event)
{
    const int kFrontLaser = 0;
    const int kBackLaser = 1;
    
    switch(event.GetSelection())
    {
    case kFrontLaser:
        if(tiltData_.sizeFrontLaser() > 0)
        {
            frontScan_ = *tiltData_.beginFrontLaser();
            backScan_ = frontScan_;
        }
        tiltLaser_ = front;
        break;
        
    case kBackLaser:
        if(tiltData_.sizeBackLaser() > 0)
        {
            frontScan_ = *tiltData_.beginBackLaser();
            backScan_ = frontScan_;
        }
        tiltLaser_ = back;
        break;
        
    default:
        std::cerr << "ERROR:CalibrationPanel: You forgot to update the radio box handler after changing its options!\n";
        assert(false);
    }
    
    widgets.displayWidget->showFrontLaserScans(true);
    widgets.displayWidget->showBackLaserScans(true);
}


void CalibrationPanel::lineStartIndexChanged(wxCommandEvent& event)
{
    long startIndex = 0;
    
    wxString indexString = widgets.tiltStartIndex->GetLineText(0);
    
    if(indexString.ToLong(&startIndex) && (startIndex > 0))
    {
        tiltRange_.first = startIndex;
    }
}


void CalibrationPanel::lineEndIndexChanged(wxCommandEvent& event)
{
    long endIndex = 0;
    
    wxString indexString = widgets.tiltEndIndex->GetLineText(0);
    
    if(indexString.ToLong(&endIndex) && (endIndex > 0))
    {
        tiltRange_.second = endIndex;
    }
}


void CalibrationPanel::calibratePitchPressed(wxCommandEvent& event)
{
    switch(tiltLaser_)
    {
    case front:
        calibrateTilt(tiltData_.beginFrontLaser(), tiltData_.endFrontLaser(), true);
        break;
        
    case back:
        calibrateTilt(tiltData_.beginBackLaser(), tiltData_.endBackLaser(), true);
        break;
    }
}


void CalibrationPanel::calibrateRollPressed(wxCommandEvent& event)
{
    switch(tiltLaser_)
    {
        case front:
            calibrateTilt(tiltData_.beginFrontLaser(), tiltData_.endFrontLaser(), false);
            break;
            
        case back:
            calibrateTilt(tiltData_.beginBackLaser(), tiltData_.endBackLaser(), false);
            break;
    }
}

} // namespace ui
} // namespace vulcan
