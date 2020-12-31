/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     lpm.h
 * \author   Collin Johnson
 *
 * Declaration of LPM.
 */

#ifndef HSSH_LOCAL_METRIC_LPM_H
#define HSSH_LOCAL_METRIC_LPM_H

#include "core/pose.h"
#include "hssh/metrical/occupancy_grid.h"
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

struct lpm_params_t;

/**
 * LocalPerceptualMap
 */
class LocalPerceptualMap : public OccupancyGrid
{
public:
    // Methods for creating/destroying the grid
    LocalPerceptualMap(void);

    /**
     * Constructor to create a new LocalPerceptualMap from a parameters struct.
     *
     * \param    gridParams          Parameters for the grid to be created
     * \param    globalCenter        Global center of the map
     */
    LocalPerceptualMap(const lpm_params_t& gridParams, const Point<float>& globalCenter);

    /**
     * Constructor to create a new LocalPerceptualMap.
     *
     * \param    gridWidth           Width of the grid in cells
     * \param    gridHeight          Height of the grid in cells
     * \param    gridScale           Meters per grid cell
     * \param    globalCenter        Global center of the map
     * \param    occupiedCellCost    Threshold cost for a cell to be considered occupied or dynamic
     * \param    freeCellCost        Threshold cost for a cell to be considered free
     */
    LocalPerceptualMap(std::size_t widthInCells,
                       std::size_t heightInCells,
                       float cellsToMeters,
                       const Point<float>& globalCenter,
                       uint8_t occupiedCellCost,
                       uint8_t freeCellCost);

    /**
     * Copy constructor to create from another OccupancyGrid. Doesn't have to be a LocalPerceptualMap though.
     *
     * \param    grid            OccupancyGrid to use for creating this LPM
     */
    LocalPerceptualMap(const OccupancyGrid& grid);

    LocalPerceptualMap(const LocalPerceptualMap& gridToCopy) = default;
    LocalPerceptualMap(LocalPerceptualMap&& toMove) = default;
    LocalPerceptualMap& operator=(const LocalPerceptualMap& rhs) = default;
    LocalPerceptualMap& operator=(LocalPerceptualMap&& rhs) = default;


    // Methods for accessing grid parameters
    int64_t getTimestamp(void) const { return timestamp; }
    int32_t getId(void) const { return mapId; }
    int32_t getReferenceFrameIndex(void) const { return frameIndex; }
    pose_t getTransformFromPreviousFrame(void) const { return transformFromLastFrame; }

    // Methods for mutating the grid
    void setTimestamp(int64_t timestamp) { this->timestamp = timestamp; }
    void setId(int32_t id) { mapId = id; }
    void setReferenceIndex(int32_t index) { frameIndex = index; }
    void setFrameTransform(const pose_t& transform) { transformFromLastFrame = transform; }

    // Methods for modifying position of the grid
    /**
     * changeReferenceFrame changes the reference frame of the LPM. The map is rotated to align to the
     * axes and the boundary is redefined in the new coordinate system. Changing the reference frame
     * is distinct from changing the global center or boundary. Those operations shift the grid to encapsulate
     * a new portion of the environment. Chaging the reference frame merely transforms the coordinates of
     * the area being represented by the LPM. The LPM itself, aside from a rotation contains exactly the
     * same information.
     *
     * \param    newReferenceFrame       New reference frame for the LPM
     */
    void changeReferenceFrame(const pose_t& newReferenceFrame);

    /**
     * isPoseInGrid checks if the specified pose exists in the grid.
     */
    bool isPoseInGrid(const pose_t& pose) const { return getBoundary().contains(pose.toPoint()); }

private:
    int64_t timestamp;   // Time at which the grid was last updated
    int32_t mapId;
    int32_t frameIndex;

    pose_t transformFromLastFrame;

    // Serialization support
    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& ::cereal::base_class<OccupancyGrid>(this);
        ar& timestamp;
        ar& mapId;
        ar& frameIndex;
        ar& transformFromLastFrame;
    }
};

}   // namespace hssh
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(hssh::LocalPerceptualMap, ("HSSH_LPM"))

#endif   // HSSH_LOCAL_METRIC_LPM_H
