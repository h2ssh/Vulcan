/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     vision_display_widget.h
* \author   Collin Johnson
* 
* Declaration of VisionDisplayWidget.
*/

#ifndef UI_DEBUG_VISION_DISPLAY_WIDGET_H
#define UI_DEBUG_VISION_DISPLAY_WIDGET_H

#include <ui/components/grid_based_display_widget.h>
#include <ui/common/ui_params.h>
#include <core/image.h>
#include <vision/image_segment.h>
#include <utils/mutex.h>
#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class ImageRenderer;
class GroundPlaneBoundaryRenderer;
class ImageSegmentRenderer;

/**
* VisionDisplayWidget is a widget that displays the various output produced by
* the vision modules used by Vulcan. The
*/
class VisionDisplayWidget : public GridBasedDisplayWidget
{
public:

    /**
    * Constructor for VisionDisplayWidget.
    */
    VisionDisplayWidget(wxWindow* parent,
                        wxWindowID id = wxID_ANY,
                        const wxPoint& pos = wxDefaultPosition,
                        const wxSize& size = wxDefaultSize,
                        long style = 0,
                        const wxString& name = wxString((const wxChar*)("GLCanvas")),
                        const wxPalette& palette = wxNullPalette);

    void setWidgetParams(const vision_display_params_t& params);

    // Set flags to indicate which things to show on top of the image, which is always shown
    void showImageSegments(bool show) { shouldShowImageSegments_ = show; }

    // Handle the various types of incoming vision data
    void setImage        (Image image);
    void setImageSegments(std::vector<vision::image_segment_t> segments);

private:

    Image                        image_;
    std::vector<vision::image_segment_t> segments_;
    
    std::unique_ptr<ImageRenderer>        imageRenderer_;
    std::unique_ptr<ImageSegmentRenderer> segmentRenderer_;
    
    bool shouldShowImageSegments_;
    bool haveNewSegments_;
    bool initialized_;

    utils::Mutex dataLock_;
    
    // GridBasedDisplayWidget interface
    virtual void renderWidget(void);
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_DEBUG_VISION_DISPLAY_WIDGET_H
