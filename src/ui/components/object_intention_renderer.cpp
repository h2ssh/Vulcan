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
* Definition of ObjectIntentionRenderer.
*/

#include "ui/components/object_intention_renderer.h"
#include "ui/common/color_generator.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/ui_color.h"
#include "tracker/dynamic_object_visitor.h"
#include "tracker/evaluation/intention_evaluator.h"
#include "tracker/objects/rigid.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

struct GoalRenderer : boost::static_visitor<>
{
    GLColor color;

    void operator()(const Point<double>& position)
    {
        color.set(0.9);
        gl_draw_filled_circle(position, 0.125);
        color.set();
        gl_draw_line_circle(position, 0.125, 1.0);
    }

    void operator()(const Line<double>& boundary)
    {
        glLineWidth(6.0);
        color.set(0.9);
        glBegin(GL_LINES);
        glVertex2f(boundary.a.x, boundary.a.y);
        glVertex2f(boundary.b.x, boundary.b.y);
        glEnd();
    }
};


ObjectIntentionRenderer::ObjectIntentionRenderer(double distPerPose)
: distPerPose_(distPerPose)
{
}


void ObjectIntentionRenderer::renderIntentions(const tracker::AreaIntentionEstimates& intentions)
{
    if(intentions.empty())
    {
        return;
    }

    auto colors = generate_colors(intentions.destinations().size(), 1.0, false);

    // Draw the trajectory
    GLColor black;
    black.set();
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for(auto& est : intentions)
    {
        glVertex2f(est.objPose.x, est.objPose.y);
    }
    glEnd();

    // Draw the destinations
    auto dests = intentions.destinations();
    GoalRenderer renderer;
    for(std::size_t n = 0; n < dests.size(); ++n)
    {
        renderer.color = colors[n];
        dests[n].apply_visitor(renderer);
    }

    double distToNext = 0.0;
    pose_t lastPose = intentions.begin()->objPose;
    for(auto& est : intentions)
    {
        distToNext -= distance_between_points(lastPose.toPoint(), est.objPose.toPoint());
        lastPose = est.objPose;

        if(distToNext < 0.0)
        {
            colors[est.maxProbIndex].set();
            gl_draw_filled_triangle(est.objPose.toPoint(), 0.25, 0.45, est.objPose.theta);

            distToNext = distPerPose_;
        }
    }
    glEnd();
}

} // namespace ui
} // namespace vulcan
