/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file
 * \author   Collin Johnson
 *
 * Declaration of TopoSituationRenderer.
 */

#ifndef UI_COMPONENTS_TOPO_SITUATION_RENDERER_H
#define UI_COMPONENTS_TOPO_SITUATION_RENDERER_H

namespace vulcan
{
namespace hssh
{
class LocalTopoMap;
}
namespace mpepc
{
class PathSituation;
}
namespace mpepc
{
class PlaceSituation;
}
namespace ui
{

/**
 *
 */
class TopoSituationRenderer
{
public:
    /**
     * Constructor for TopoSituationRenderer.
     */
    TopoSituationRenderer(void);

    /**
     * Render the path situation based on the topological map.
     */
    void renderSituationPath(const mpepc::PathSituation& situation, const hssh::LocalTopoMap& topoMap) const;

    /**
     * Render the place situation based on the topological map.
     *
     */
    void renderSituationPlace(const mpepc::PlaceSituation& situation, const hssh::LocalTopoMap& topoMap) const;

private:
    double robotRadius_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_TOPO_SITUATION_RENDERER_H
