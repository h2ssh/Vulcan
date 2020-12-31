/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateways_renderer.h
* \author   Collin Johnson
*
* Declaration of GatewaysRenderer.
*/

#ifndef UI_COMPONENTS_GATEWAYS_RENDERER_H
#define UI_COMPONENTS_GATEWAYS_RENDERER_H

#include <vector>
#include "ui/common/ui_color.h"

namespace vulcan
{
namespace hssh { class Gateway; }

namespace ui
{

/**
* GatewaysRenderer renders a set of gateways onto the screen. Gateways are drawn as dashed lines that connect two
* circular endpoints. The endpoints will fall within an anchor point, so there will be obvious and clear
* overlap between the two if they are being rendered simultaneously.
*/
class GatewaysRenderer
{
public:

    /**
    * setRenderColors sets the colors to use for rendering the gateways.
    */
    void setRenderColors(const GLColor& exploredColor, const GLColor& frontierColor, const GLColor& endpointColor);

    /**
    * renderGateways renders the gateways onto the screen.
    */
    void renderGateways(const std::vector<hssh::Gateway>& gateways, bool shouldRenderNormals = false);

    /**
    * renderGateway renders a single gateway onto the screen.
    * 
    * \param    gateway                 Gateway to be drawn
    * \param    shouldRenderNormals     Flag indicating if the normals for the gateway should be rendered (optional, default = false)
    * \param    color                   Color to draw the gateway if different than global default (optional, default = nullptr)
    */
    void renderGateway(const hssh::Gateway& gateway, 
                       bool shouldRenderNormals = false,
                       const GLColor* color = nullptr);

private:

    void renderGatewayNormals(const hssh::Gateway& gateway);

    GLColor exploredColor;
    GLColor frontierColor;
    GLColor endpointColor;
};

}
}

#endif // UI_COMPONENTS_GATEWAYS_RENDERER_H
