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
* Declaration of ParticlesRenderer.
*/

#ifndef UI_COMPONENTS_PARTICLES_RENDERER_H
#define UI_COMPONENTS_PARTICLES_RENDERER_H

#include "ui/common/ui_color.h"
#include <vector>
#include <cstdint>

namespace vulcan
{
namespace hssh { struct particle_t; }
namespace hssh { struct particle_grid_score_t; }
struct pose_t;

namespace ui
{

/**
* ParticlesRenderer is used for rendering the particles contained in a
* std::vector<particle_t>. The particles are represented by (x, y, weight).
*/
class ParticlesRenderer
{
public:

    void setRenderColor(const GLColor& color);
    void renderParticles(const std::vector<hssh::particle_t>& samples);
    void renderParticleScores(const pose_t& pose, hssh::particle_grid_score_t& scores, double maxScore = 1.0);

private:

    GLColor color;
};

}
}

#endif // UI_COMPONENTS_PARTICLES_RENDERER_H
