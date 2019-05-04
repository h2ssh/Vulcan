/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_object_renderer.h
* \author   Collin Johnson
*
* Declaration of LaserObjectRenderer.
*/

#ifndef UI_COMPONENTS_LASER_OBJECT_RENDERER_H
#define UI_COMPONENTS_LASER_OBJECT_RENDERER_H

#include <ui/common/ui_color.h>
#include <tracker/types.h>
#include <vector>

namespace vulcan
{
namespace tracker { class LaserObject; }
namespace tracker { class LaserObjectCollection; }
namespace tracker { class ObjectBoundary; }
namespace ui
{

/**
* LaserObjectRenderer renders LaserObjects. A LaserObject can be drawn using the rectangle, one-circle, or two-circle boundaries.
* The laser points associated with the object can be drawn. The covariance matrix for the object can also be drawn.
*
* When the bulk renderObjects method is called, the colors to use for the objects are randomly generated.
*/
class LaserObjectRenderer
{
public:

    // Flags for the render options
    static const int kShowPoints      = 0x01;   ///< Show the laser points associated with the laser object
    static const int kShowUncertainty = 0x02;   ///< Show the position uncertainty ellipse

    /**
    * renderObjects draws a collection of LaserObjects onto the screen. The color for the objects
    * is generated randomly. A different color is used for each object. See renderObject for the
    * rendering options associated with the options flag.
    *
    * \param    objects         Objects to be rendered
    * \param    boundary        Type of boundary to be drawn for the objects
    * \param    options         Rendering options (default = none)
    */
    void renderObjects(const tracker::LaserObjectCollection& objects,
                       tracker::BoundaryType boundary,
                       int options = 0);

    /**
    * renderObject draws a single LaserObject. The object is drawn with the provided color and options.
    * The options for rendering the object come from the flags defined above.
    *
    * \param    object          Object to be rendered
    * \param    boundary        Type indicating which boundary should be drawn
    * \param    color           Color of the object (default = black)
    * \param    options         Options for rendering the object (default = none)
    */
    void renderObject(const tracker::LaserObject& object,
                      tracker::BoundaryType boundary,
                      const GLColor& color = GLColor(0,0,0,1),
                      int options = 0);

private:

    GLColor activeColor;

    void drawBoundary   (const tracker::ObjectBoundary& boundary, int options);
    void drawUncertainty(const tracker::LaserObject& object);
    void drawPoints     (const tracker::LaserObject& object);
};

}
}

#endif // UI_COMPONENTS_LASER_OBJECT_RENDERER_H
