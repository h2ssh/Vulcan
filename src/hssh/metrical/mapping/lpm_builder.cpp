/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     lpm_builder.cpp
 * \author   Collin Johnson
 *
 * Definition of LPMBuilder.
 */

#include "hssh/metrical/mapping/lpm_builder.h"
#include "core/laser_scan.h"
#include "core/pose.h"
#include "hssh/metrical/mapping/mapping_params.h"
#include "utils/tiled_cell_grid.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

LPMBuilder::LPMBuilder(const lpm_params_t& params, const pose_t& currentPose) : map(params, currentPose.toPoint())
{
    // On construction, erase the approximate area of the robot + blindspots from the map
    // NOTE: This could potentially erase obstacles that would then get filled in, but the opposite effect
    // of the robot not being able to drive right away in the map is worse since quasi-static + glass mapping
    // will easily fill in the blanks

    // TODO: Make this a parameter, since not all robots have the same blind spot!
    // Blind spot area derived from looking at an initial LPM.
    Point<float> bottomLeft(currentPose.x - 0.35, currentPose.y - 0.45);
    Point<float> topRight(currentPose.x + 0.5, currentPose.y + 0.5);
    auto blCell = utils::global_point_to_grid_cell_round(bottomLeft, map);
    auto trCell = utils::global_point_to_grid_cell_round(topRight, map);
    for (int y = blCell.y; y <= trCell.y; ++y) {
        for (int x = blCell.x; x <= trCell.x; ++x) {
            map.setCost(Point<int>(x, y), 0);
        }
    }
}


void LPMBuilder::reset(void)
{
    // No special resetting needed
}


void LPMBuilder::boundaryChanged(const math::Rectangle<float>& boundary)
{
    // Nothing needed when the boundary changes
}


void LPMBuilder::update(const map_update_data_t& data)
{
    utils::boundary_intersection_t coords(
      map.getBoundary(),
      data.scanRaster.getBoundary(),
      map.cellsPerMeter(),
      std::make_pair(map.getWidthInCells(), map.getHeightInCells()),
      std::make_pair(data.scanRaster.getWidthInCells(), data.scanRaster.getHeightInCells()));

    Point<int> lpmCell(coords.gridStartCell);
    Point<int> updateCell(coords.updateStartCell);

    map.setTimestamp(data.timestamp);

    for (std::size_t y = 0; y < coords.updateHeight; ++y, ++lpmCell.y, ++updateCell.y) {
        lpmCell.x = coords.gridStartCell.x;
        updateCell.x = coords.updateStartCell.x;

        for (std::size_t x = 0; x < coords.updateWidth; ++x, ++lpmCell.x, ++updateCell.x) {
            map.updateCostNoCheck(lpmCell, data.scanRaster.getValueNoCheck(updateCell.x, updateCell.y));
        }
    }
}


void LPMBuilder::rotate(float radians)
{
    map.rotate(radians);
}

}   // namespace hssh
}   // namespace vulcan
