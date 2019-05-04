/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     cost_map_renderer.cpp
* \author   Collin Johnson
*
* Definition of CostMapRenderer.
*/

#include <ui/components/cost_map_renderer.h>
#include <ui/common/gl_texture_helpers.h>
#include <mpepc/cost/cost_map.h>

namespace vulcan
{
namespace ui
{

enum CostTextures
{
    type_idx,
    cost_idx,
    num_textures,
};


void CostMapRenderer::setCostMap(const mpepc::CostMap& costMap)
{
    if(textureNames_.empty())
    {
        textureNames_.resize(num_textures);
        glGenTextures(num_textures, textureNames_.data());
    }

    if((gridWidth_ != static_cast<int>(costMap.getWidthInCells()))
        || (gridHeight_ != static_cast<int>(costMap.getHeightInCells())))
    {
        gridWidth_ = costMap.getWidthInCells();
        gridHeight_ = costMap.getHeightInCells();
        metricWidth_ = costMap.getWidthInMeters();
        metricHeight_ = costMap.getHeightInMeters();

        initializeGridTextures();
        initialized_ = true;
    }

    gridOrigin_ = costMap.getBottomLeft();
    updateGridTextures(costMap);
    typeTextureChanged_ = false;
}


void CostMapRenderer::renderCosts(void)
{
    if(initialized_)
    {
        if(typeTextureChanged_)
        {
            activate_texture(textureNames_[type_idx], GL_TEXTURE1, GL_DECAL);
            set_sub_texture(typeTexture_.data(), gridWidth_, gridHeight_, GL_RGB);
            disable_texture(GL_TEXTURE1);
        }

        enableTextures();

        float textureXMax = gridWidth_  / static_cast<float>(textureWidth_);
        float textureYMax = gridHeight_ / static_cast<float>(textureHeight_);

        glColor4f(0.0f, 0.0f, 0.0f, 0.8f);
        draw_two_textures_on_rectangle(gridOrigin_.x, gridOrigin_.y, metricWidth_, metricHeight_, textureXMax, textureYMax);

        disableTextures();

        typeTextureChanged_ = false;
    }
}


void CostMapRenderer::initializeGridTextures(void)
{
    textureWidth_  = round_to_power_of_two(gridWidth_);
    textureHeight_ = round_to_power_of_two(gridHeight_);

    typeTexture_.resize(textureWidth_ * textureHeight_ * 3);
    costTexture_.resize(textureWidth_ * textureHeight_);

    initialize_texture(textureNames_[type_idx], typeTexture_.data(), textureWidth_, textureHeight_, GL_RGB, GL_RGB);
    initialize_texture_16(textureNames_[cost_idx], costTexture_.data(), textureWidth_, textureHeight_, GL_ALPHA, GL_ALPHA);
}


void CostMapRenderer::updateGridTextures(const mpepc::CostMap& costs)
{
    activate_texture(textureNames_[type_idx], GL_TEXTURE1, GL_DECAL);
    convertTypesToTexture(costs);
    set_sub_texture(typeTexture_.data(), gridWidth_, gridHeight_, GL_RGB);
    disable_texture(GL_TEXTURE1);

    activate_texture(textureNames_[cost_idx], GL_TEXTURE0, GL_REPLACE);
    convertCostsToTexture(costs);
    set_sub_texture_16(costTexture_.data(), gridWidth_, gridHeight_, GL_ALPHA);
    disable_texture(GL_TEXTURE0);
}


void CostMapRenderer::enableTextures(void)
{
    activate_texture(textureNames_[type_idx], GL_TEXTURE1, GL_DECAL);
    activate_texture(textureNames_[cost_idx], GL_TEXTURE0, GL_REPLACE);
}


void CostMapRenderer::disableTextures(void)
{
    disable_texture(GL_TEXTURE1);
    disable_texture(GL_TEXTURE0);
}


void CostMapRenderer::convertTypesToTexture(const mpepc::CostMap& costs)
{
    for(int y = 0; y < gridHeight_; ++y)
    {
        for(int x = 0; x < gridWidth_; ++x)
        {
            setCellColor(x, y, costs(x, y), costs.getMaxCost());
        }
    }
}


void CostMapRenderer::setCellColor(int x, int y, int32_t cost, int32_t maxCost)
{
    int textureIndex = ((y * gridWidth_) + x) * 3;
    // No cost means free cell
    if(cost == 0)
    {
        typeTexture_[textureIndex] = freeColor_[0];
        typeTexture_[textureIndex + 1] = freeColor_[1];
        typeTexture_[textureIndex + 2] = freeColor_[2];
    }
    // A collision is occupied
    else if(cost >= mpepc::kMinCollisionCost)
    {
        typeTexture_[textureIndex] = occupiedColor_[0];
        typeTexture_[textureIndex + 1] = occupiedColor_[1];
        typeTexture_[textureIndex + 2] = occupiedColor_[2];
    }
    // Otherwise, various additive costs have been used
    else
    {
        typeTexture_[textureIndex] = costColor_[0];
        typeTexture_[textureIndex + 1] = costColor_[1];
        typeTexture_[textureIndex + 2] = costColor_[2];
    }
}


void CostMapRenderer::convertCostsToTexture(const mpepc::CostMap& costs)
{
    const auto maxCost = costs.getMaxCost();

    const float scale = (maxCost > 0) ? 65536.0f / maxCost : 1.0;
    int textureIndex = 0;

    for(int y = 0; y < gridHeight_; ++y)
    {
        for(int x = 0; x < gridWidth_; ++x)
        {
            const auto cost = costs(x, y);
            costTexture_[textureIndex++] = (cost >= maxCost) ? 65536 : cost * scale;
        }
    }
}

} // namespace ui
} // namespace vulcan
