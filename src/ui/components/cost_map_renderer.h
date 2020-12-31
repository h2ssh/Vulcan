/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     cost_map_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of CostMapRenderer.
 */

#ifndef UI_COMPONENTS_COST_MAP_RENDERER_H
#define UI_COMPONENTS_COST_MAP_RENDERER_H

#include "core/point.h"
#include <GL/gl.h>
#include <cstdint>
#include <vector>

namespace vulcan
{
namespace mpepc
{
class CostMap;
}
namespace ui
{

/**
 * CostMapRenderer renders a CostMap from MPEPC with the following color scheme:
 *
 *   - Free cells are white
 *   - Collision cells are black
 *   - Other cost cells are red
 */
class CostMapRenderer
{
public:
    /**
     * setCostMap sets the cost map to be rendered by future calls to renderCosts.
     */
    void setCostMap(const mpepc::CostMap& costMap);

    /**
     * renderCosts renders the stored cost map.
     */
    void renderCosts(void);

private:
    // Flags indicating which parts of the grid should be rendered
    bool typeTextureChanged_ = false;

    uint8_t freeColor_[4] = {255, 255, 255, 255};
    uint8_t occupiedColor_[4]{0, 0, 0, 255};
    uint8_t costColor_[4] = {255, 0, 0, 255};

    int gridWidth_;
    int gridHeight_;

    float metricWidth_;
    float metricHeight_;

    Point<float> gridOrigin_;

    int textureWidth_;
    int textureHeight_;

    std::vector<GLuint> textureNames_;
    std::vector<uint16_t> costTexture_;
    std::vector<uint8_t> typeTexture_;

    bool initialized_ = false;

    void initializeGridTextures(void);
    void updateGridTextures(const mpepc::CostMap& costs);
    void enableTextures(void);
    void disableTextures(void);

    void convertTypesToTexture(const mpepc::CostMap& costs);
    void setCellColor(int x, int y, int32_t cost, int32_t maxCost);
    void convertCostsToTexture(const mpepc::CostMap& costs);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_COST_MAP_RENDERER_H
