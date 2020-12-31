/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dynamic_object_renderer.h
* \author   Collin Johnson
*
* Declaration of DynamicObjectRenderer.
*/

#ifndef UI_COMPONENTS_DYNAMIC_OBJECT_RENDERER_H
#define UI_COMPONENTS_DYNAMIC_OBJECT_RENDERER_H

#include "tracker/dynamic_object_visitor.h"
#include "ui/common/ui_color.h"
#include <vector>

namespace vulcan
{
namespace tracker { class DynamicObject; }
namespace tracker { class DynamicObjectCollection; }
namespace tracker { class ObjectBoundary; }
namespace ui
{

/**
* DynamicObjectRenderer draws DynamicObjects using a different color for each type of object.
*
* The colors are:
*
*   - black : unclassified
*   - brown : pivoting
*   - grey  : sliding
*   - green : striding
*   - blue  : rigid
*/
class DynamicObjectRenderer : public tracker::DynamicObjectVisitor
{
public:

    // Flags defining the uncertainty type to render to the rigid object state estimates
    static const uint32_t kShowPositionUncertainty = 0x01;
    static const uint32_t kShowVelocityUncertainty = 0x02;
    static const uint32_t kShowAccelerationUncertainty = 0x04;
    static const uint32_t kShowFastState = 0x08;
    static const uint32_t kShowSlowState = 0x10;
    static const uint32_t kShowAcceleration = 0x20;

    /**
    * Constructor for DynamicObjectRenderer.
    */
    DynamicObjectRenderer(void);

    /**
    * renderCollectionStateEstimates draws the state estimate (x, y, v_x, v_y, a_x, a_y) for a DynamicObjectCollection.
    *
    * Options for which type of uncertainty to draw are provided as input flags.
    */
    void renderCollectionStateEstimates(const tracker::DynamicObjectCollection& collection,
                                        uint32_t uncertaintyOptions = 0);

    /**
    * renderObjectStateEstimate draws the state estimate (x, y, v_x, v_y, a_x, a_y) for a single DynamicObject.
    *
    * Options for which type of uncertainty to draw are provided as input flags.
    */
    void renderObjectStateEstimate(const tracker::DynamicObject& object, uint32_t uncertaintyOptions = 0);

    /**
    * renderObjectGoals draws the goals associated with an object. The boundary goals are line segments with an
    * arrow pointing in the heading direction. The position goals are small circles with a heading arrow.
    *
    * If the full distribution is drawn, then the boundary goals are shaded triangles while the position goals are
    * just shaded
    */
    void renderObjectGoals(const tracker::DynamicObject& object, bool showFullDistribution);

    // tracker::DynamicObjectVisitor interface
    void visitPerson        (const tracker::Person& person) override;
    void visitPivotingObject(const tracker::PivotingObject& door) override;
    void visitRigid         (const tracker::RigidObject& object) override;
    void visitSlidingObject (const tracker::SlidingObject& door) override;
    void visitUnclassified  (const tracker::UnclassifiedObject& object) override;

private:

    uint32_t uncertaintyFlags_;

    GLColor unclassifiedColor_;
    GLColor pivotingColor_;
    GLColor slidingColor_;
    GLColor stridingColor_;
    GLColor rigidColor_;

    void drawObject(const tracker::DynamicObject& object, const GLColor& color);
    void drawBoundary(const tracker::ObjectBoundary& boundary, const GLColor& color, bool filled);
    void drawVelocity(const tracker::DynamicObject& object, const GLColor& color);
    void drawRigidObjectState(const tracker::RigidObject& object, const GLColor& color);
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMPONENTS_DYNAMIC_OBJECT_RENDERER_H
