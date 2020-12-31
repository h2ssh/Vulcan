/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     glass_walls.h
 * \author   Collin Johnson
 *
 * Declaration of GlassWall and associated predict_glass_walls function.
 */

#ifndef HSSH_METRICAL_MAPPING_GLASS_WALLS_H
#define HSSH_METRICAL_MAPPING_GLASS_WALLS_H

#include "core/line.h"
#include "core/point.h"
#include "hssh/types.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

class GlassMap;

/**
 * GlassWall contains information about a glass wall in the environment.
 */
class GlassWall
{
public:
    using Iter = std::vector<Point<float>>::const_iterator;

    /**
     * Constructor for GlassWall.
     *
     * \param    cells           Glass map cells associated with the wall
     * \param    map             GlassMap in which the wall was found
     */
    GlassWall(const CellVector& cells, const GlassMap& map);

    /**
     * Constructor for GlassWall.
     *
     * \param    walls           Create a GlassWall by merging multiple existing walls
     * \param    map             GlassMap in which the walls were found
     */
    GlassWall(const std::vector<GlassWall>& walls, const GlassMap& map);

    /**
     * wall retrieves the location of the actual wall that was measured.
     */
    Line<float> wall(void) const { return wall_; }

    /**
     * predictedWall retrieves the location of the predicted wall, which may extend in both directions beyond the
     * ends of the measured wall.
     */
    Line<float> predictedWall(void) const { return predicted_; }

    /**
     * normal retrieves the estimated normal to the line based on the measured angle bins. This normal isn't necessarily
     * the same as the normal of the wall() line.
     */
    double normal(void) const { return normal_; }

    /**
     * error retrieves the average distance from a cell in the wall to the fitted wall line. It is a measure of how good
     * the wall model is.
     */
    double error(void) const { return error_; }

    // Iterate over cells in the GlassWall
    Iter begin(void) const { return cells_.begin(); }
    Iter end(void) const { return cells_.end(); }
    std::size_t size(void) const { return cells_.size(); }
    Point<float> at(int index) const { return cells_.at(index); }
    Point<float> operator[](int index) const { return cells_[index]; }

private:
    std::vector<Point<float>> cells_;
    Line<float> wall_;
    Line<float> predicted_;
    double normal_;
    double error_;

    void computeWall(const GlassMap& map);
};


/**
 * predict_glass_walls
 */
std::vector<GlassWall> predict_glass_walls(const GlassMap& map);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_METRICAL_MAPPING_GLASS_WALLS_H
