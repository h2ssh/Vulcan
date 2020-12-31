/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_COMPONENTS_LINES_RENDERER_H
#define UI_COMPONENTS_LINES_RENDERER_H

#include <vector>
#include "core/line.h"
#include "ui/common/ui_color.h"

namespace vulcan
{

namespace robot
{
    struct pose_t;
}
    
namespace ui
{

/**
* LinesRenderer renders a set of lines onto the screen using the specified color.
*/
class LinesRenderer
{
public:
    
    /**
    * setRenderColor sets the color to use for the rendering.
    */
    void setRenderColor(const GLColor& color);
    
    /**
    * renderLines renders the provided lines. The provided lines are assumed to be in local coordinate frame, so they
    * will be transformed to the provided global coordinate frame.
    */
    void renderLines(const std::vector<Line<float>>& lines, const pose_t& globalRobotPose, float lineWidth);

private:
    
    GLColor lineColor;
};


}
}

#endif // UI_COMPONENTS_LINES_RENDERER_H
