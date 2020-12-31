/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     exploration_map_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of ExplorationMapRenderer.
 */

#ifndef UI_COMPONENTS_EXPLORATION_MAP_RENDERER_H
#define UI_COMPONENTS_EXPLORATION_MAP_RENDERER_H

namespace vulcan
{
namespace planner
{
class LocalTopoExplorationMap;
}
namespace ui
{

/**
 * ExplorationMapRenderer draws various exploration maps used by the map_exploration module. The currently supported
 * exploration maps are:
 *
 *   - LocalTopoExplorationMap : map used for exploring every area in a LocalTopoMap
 */
class ExplorationMapRenderer
{
public:
    /**
     * renderLocalTopoExplorationMap renders the information in a LocalTopoExplorationMap.
     *
     * \param    map             Exploration map of the current environment
     * \param    currentArea     Area robot is currently in
     * \param    targetArea      Target area robot is currently driving to
     */
    void renderLocalTopoExplorationMap(const planner::LocalTopoExplorationMap& map, int currentArea, int targetArea);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_EXPLORATION_MAP_RENDERER_H
