/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* @file
* @author   Collin Johnson
*
* Declaration of ObjectIntentionRenderer.
*/

#ifndef UI_COMPONENTS_OBJECT_INTENTION_RENDERER_H
#define UI_COMPONENTS_OBJECT_INTENTION_RENDERER_H

#include <vector>

namespace vulcan
{
namespace tracker { class AreaIntentionEstimates; }
namespace ui
{

/**
* Renderer for AreaIntentionEstimates. The renderer does the following:
*
*   - Each goal is renderer as a separate color.
*   - Every one meter (configurable) the pose of the object is drawn with color the same
*       as the most probable goal.
*   - The full trajectory of the object is drawn.
*/
class ObjectIntentionRenderer
{
public:

    /**
    * Create a renderer with the specified spacing between drawn poses.
    *
    * @param    distPerPose         Gap between the full pose rendering -- meters (optional, default =1m)
    */
    explicit ObjectIntentionRenderer(double distPerPose);

    /**
    * Render the provided intentions.
    */
    void renderIntentions(const tracker::AreaIntentionEstimates& intentions);

private:

    double distPerPose_;
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMPONENTS_OBJECT_INTENTION_RENDERER_H
