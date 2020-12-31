/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     vision_display_widget.cpp
* \author   Collin Johnson
* 
* Definition of VisionDisplayWidget.
*/

#include "ui/debug/vision_display_widget.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include "utils/auto_mutex.h"
#include "core/point.h"
#include "ui/common/gl_utilities.h"
#include "ui/components/image_renderer.h"
#include "ui/components/image_segment_renderer.h"

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(VisionDisplayWidget, GridBasedDisplayWidget)
END_EVENT_TABLE()


VisionDisplayWidget::VisionDisplayWidget(wxWindow* parent,
                                         wxWindowID id,
                                         const wxPoint& pos,
                                         const wxSize& size,
                                         long style,
                                         const wxString& name,
                                         const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, imageRenderer_(new ImageRenderer)
, segmentRenderer_(new ImageSegmentRenderer)
, shouldShowImageSegments_(false)
, haveNewSegments_(false)
, initialized_(false)
{
}


void VisionDisplayWidget::setWidgetParams(const vision_display_params_t& params)
{
}


void VisionDisplayWidget::setImage(Image image)
{
    utils::AutoMutex autoLock(dataLock_);

    image_ = std::move(image);

    // Need to maintain the aspect ratio of the image, so set the height accordingly
    setViewRegion(image.getWidth(), image.getHeight());
    setGridDimensions(image.getWidth(), image.getHeight());
    setCameraFocalPoint(Point<float>(image.getWidth()/2, image.getHeight()/2));
}


void VisionDisplayWidget::setImageSegments(std::vector<vision::image_segment_t> segments)
{
    utils::AutoMutex autoLock(dataLock_);

    segments_        = std::move(segments);
    haveNewSegments_ = true;
}


void VisionDisplayWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(dataLock_);

    if(haveNewSegments_)
    {
        segmentRenderer_->setImageSegments(segments_, image_.getWidth(), image_.getHeight());
        haveNewSegments_ = false;
    }

    imageRenderer_->renderImage(image_);

    if(shouldShowImageSegments_)
    {
        segmentRenderer_->renderImageSegments();
    }
}


Point<int> VisionDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    // For rendering the image, the world and grid coordinates are identical
    return Point<int>(world.x, world.y);
}

} // namespace ui
} // namespace vulcan
