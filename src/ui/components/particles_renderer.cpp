/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     particles_renderer.h
* \author   Collin Johnson
*
* Definition of ParticlesRenderer.
*/

#include <ui/components/particles_renderer.h>
#include <ui/common/color_interpolator.h>
#include <hssh/metrical/localization/debug_info.h>
#include <hssh/metrical/localization/particle.h>
#include <algorithm>
#include <iostream>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void ParticlesRenderer::setRenderColor(const GLColor& color)
{
    this->color = color;
}


void ParticlesRenderer::renderParticles(const std::vector<hssh::particle_t>& particles)
{
    if(particles.empty())
    {
        return;
    }

    LinearColorInterpolator interpolator(GLColor(0.0f, 0.0f, 1.0f, 0.75f), GLColor(1.0f, 0.0f, 0.0f, 0.75f));

    float maxWeight = std::max_element(particles.begin(), particles.end())->weight;

    glPointSize(3.0f);

    glBegin(GL_POINTS);
    for(auto particleIt = particles.begin(), particleEnd = particles.end(); particleIt != particleEnd; ++particleIt)
    {
        GLColor particleColor = interpolator.calculateColor(particleIt->weight/maxWeight);
        particleColor.set();
        glVertex2f(particleIt->pose.x, particleIt->pose.y);
    }
    glEnd();
}


void ParticlesRenderer::renderParticleScores(const pose_t& pose, 
                                             hssh::particle_grid_score_t& scores,
                                             double maxScore)
{
    LinearColorInterpolator interpolator(GLColor(0.0f, 0.0f, 1.0f, 0.75f), GLColor(1.0f, 0.0f, 0.0f, 0.75f));
    
    glLineWidth(0.75f);
    
    glBegin(GL_LINES);
    for(std::size_t n = 0; n < scores.scores.size(); ++n)
    {
        GLColor particleColor = interpolator.calculateColor(scores.scores[n] / maxScore);
        particleColor.set();
        glVertex2f(pose.x, pose.y);
        glVertex2f(scores.endpoints[n].x, scores.endpoints[n].y);
    }
    glEnd();
}

} // namespace hssh
} // namespace vulcan
