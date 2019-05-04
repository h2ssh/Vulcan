/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_COMPONENTS_TRACKED_OBJECT_RENDERER_H
#define UI_COMPONENTS_TRACKED_OBJECT_RENDERER_H

#include <ui/common/ui_color.h>
#include <vector>

namespace vulcan
{
namespace mpepc   { struct dynamic_object_trajectory_debug_info_t; };
namespace tracker { class DynamicObject; }
namespace tracker { class DynamicObjectCollection; }
namespace tracker { class ObjectBoundary; }

namespace ui
{

/**
* TrackedObjectRenderer renders a tracked object onto the screen.
*/
class TrackedObjectRenderer
{
public:
    
    static const int kShowPositionUncertainy  = 0x01;   ///< Show the laser points associated with the laser object
    static const int kShowVelocityUncertainty = 0x02;   ///< Show the position uncertainty ellipse
    static const int kShowRecentTrajectory    = 0x04;   ///< Show the recent trajectory of the object
    static const int kDrawRectangle           = 0x08;   ///< Draw the rectangle boundary for the object
    static const int kDrawCircle              = 0x10;   ///< Draw the circle boundary for the object
    
    TrackedObjectRenderer(void);

    /**
    * setRenderColor sets the color to be used for rendering the objects.
    */
    void setRenderColor(const GLColor& objectColor);

    /**
    * renderObjects draws a collection of TrackedObjects.
    */
    void renderObjects(const tracker::DynamicObjectCollection& objects, int options = kDrawRectangle);

    /**
    * renderObject renders the tracked object as described above.
    */
    void renderObject(const tracker::DynamicObject& object, int options = kDrawRectangle);

    /**
    * renderObjectMotion renders the object at future times based on the estimated velocity.
    *
    * The trajectory is drawn as a line. Along the trajectory, at certain steps, the full object
    * will be rendered with tuned down alpha to give an idea of the area occupied by the object
    * in the future. The number of steps is configurable, but defaults to 3.
    *
    * \param    object          Object to be rendered
    * \param    time            Amount of time into the future to render the object
    * \param    steps           Number of time steps (optional, default=3)
    */
    void renderObjectMotion(const tracker::DynamicObject& object, float time, std::size_t steps = 3);
    
    void renderEstimatedObjectTrajectory(const mpepc::dynamic_object_trajectory_debug_info_t& estimatedObjectTrajectory);

private:

    void renderObject(const tracker::DynamicObject& object, int options, const GLColor& color);
    void drawBoundary(const tracker::ObjectBoundary& boundary, const GLColor& color, bool filled);
    float calculateObjectAlpha(int32_t trackedTime, int32_t maxTrackedTime);

    GLColor objectColor_;
    std::vector<float> trajectoryPoses;
    
    // trajectory rendering
    void   allocatePoseArraysIfNeeded(size_t size);
  
};

}
}

#endif // UI_COMPONENTS_TRACKED_OBJECT_RENDERER_H
