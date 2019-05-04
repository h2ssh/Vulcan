/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_map.h
* \author   Collin Johnson
*
* Declaration of GlassMap.
*/

#ifndef HSSH_LOCAL_METRIC_GLASS_MAP_H
#define HSSH_LOCAL_METRIC_GLASS_MAP_H

#include <hssh/local_metric/lpm.h>
#include <math/angle_range.h>
#include <math/geometry/rectangle.h>
#include <utils/discretized_angle_grid.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{

using IntensityGrid = utils::CellGrid<uint16_t>;

/**
* GlassMap is a grid map that maintains both an occupancy estimate and an estimate of the visible range for each cell
* in the grid.
*/
class GlassMap
{
public:

    // Iterator for accessing the individual bins associated with a particular cell
    using BinConstIter = utils::DiscretizedAngleGrid::BinConstIter;


    explicit GlassMap(void);

    /**
    * Constructor to create a new GlassMap.
    *
    * \param    maxLaserRange           Maximum range the laser will be used for
    * \param    numAngleBins            Number of angle bins to use per cell
    * \param    flattenHitThreshold     Number of hits in a bin for it to be considered a hit when flattening
    * \param    flattenMissThreshold    Number of misses in a bin for it to be consider a miss when flattening
    * \param    gridWidth               Width of the grid in cells
    * \param    gridHeight              Height of the grid in cells
    * \param    gridScale               Meters per grid cell
    * \param    globalCenter            Global center of the map
    */
    GlassMap(float                     maxLaserRange,
             int                       numAngleBins,
             int8_t                    flattenHitThreshold,
             int8_t                    flattenMissThreshold,
             std::size_t               gridWidth,
             std::size_t               gridHeight,
             float                     gridScale,
             const Point<float>& globalCenter);

    GlassMap(const GlassMap& toCopy) = default;
    GlassMap(GlassMap&& toMove)      = default;
    GlassMap& operator=(const GlassMap& rhs) = default;
    GlassMap& operator=(GlassMap&& rhs)      = default;

    // Methods for accessing grid parameters
    int64_t getTimestamp(void) const { return timestamp_; }
    int32_t getId(void)        const { return id_; }

    std::size_t getWidthInCells(void) const  { return hitGrid_.getWidthInCells(); }
    float       getWidthInMeters(void) const { return hitGrid_.getWidthInMeters(); }

    std::size_t getHeightInCells(void) const  { return hitGrid_.getHeightInCells(); }
    float       getHeightInMeters(void) const { return hitGrid_.getHeightInMeters(); }

    int numAngleBins(void) const { return numAngleBins_; }

    float metersPerCell(void) const { return hitGrid_.metersPerCell();}
    float cellsPerMeter(void) const { return hitGrid_.cellsPerMeter(); }

    Point<float> getGlobalCenter(void) const { return hitGrid_.getGlobalCenter(); }
    Point<float> getBottomLeft(void) const { return hitGrid_.getBottomLeft(); }

    // Methods for mutating the grid

    void setTimestamp      (int64_t newTime) { timestamp_ = newTime; }
    void setId             (int32_t mapId)   { id_ = mapId; }
    void setGridSizeInCells(std::size_t width, std::size_t height);
    void setMetersPerCell      (float gridScale);
    void setBottomLeft     (const Point<float>& bottomLeft);

    // Methods for modifying position of the grid
    /**
    * changeBoundary changes the shape and boundary of the GlassMap. The reshaped area is specified as a new metric
    * boundary for the GlassMap. The parts of the current grid that fall into the reshaped area will be copied
    * into the appropriate location, and the rest of the cells will be set to the default value.
    *
    * \param    newBoundary         Reshaped boundary for the GlassMap
    */
    void changeBoundary(const math::Rectangle<float>& newBoundary);

    /**
    * recenterActiveRegion centers the active region of the map around the provided global position.
    *
    * \param    position                New center position for the active region of the glass map (global coords)
    */
    void recenterActiveRegion(const Point<float>& position);

    /**
    * reset resets all cells into the grid to value
    */
    void reset(void);

    bool isCellInGrid(const Point<int>& cell) const { return hitGrid_.isCellInGrid(cell.x, cell.y); }
    bool isCellInGrid(int x, int y)                 const { return hitGrid_.isCellInGrid(x, y); }

    /**
    * isCellInActiveRegion checks if a cell is in the active region. If it isn't, then the active region can be
    * recentered via recenterActiveRegion.
    */
    bool isCellInActiveRegion(int x, int y) const { return hitGrid_.isCellInActiveRegion(x, y); }

    /**
    * activeRegionInCells retrieves the boundary of the active region in cells.
    */
    math::Rectangle<int> activeRegionInCells(void) const { return hitGrid_.getActiveRegionInCells(); }

    /**
    * activeRegionInMeters retrieves the boundary of the active region in the map in meters/global coordinates.
    */
    math::Rectangle<float> activeRegionInMeters(void) const { return hitGrid_.getActiveRegion(); }

    /**
    * beginBin retrieves the start iterator for a cell's bins in the GlassMap. If the cell isn't in the active region,
    * then beginBin == endBin.
    */
    BinConstIter beginBin(int x, int y) const { return hitGrid_.beginBin(x, y); }

    /**
    * endBin retrieves the end iterator for a cell's bins in the GlassMap. If the cell isn't in the active region,
    * then beginBin == endBin.
    */
    BinConstIter endBin(int x, int y) const { return hitGrid_.endBin(x, y); }

    /**
    * haveOverlappingAngles checks if two cells are adjacent in both position and angle. The adjacency check requires
    * that the cells are 8-way connected within the grid. Given these 8-way connected cells, then the two cells
    * are considered overlapping if the following holds:
    *
    *   There exists some n in [0, numAngleBins) s.t.
    *           (beginBin(x1, y1) + n) > hitThreshold && (beginBin(x2, y2) + n) > hitThreshold
    *
    * \param    x1      x-coordinate of first cell
    * \param    y1      y-coordinate of first cell
    * \param    x2      x-coordinate of second cell
    * \param    y2      y-coordinate of second cell
    * \return   True if the two cells have overlapping angle ranges.
    */
    bool haveOverlappingAngles(int x1, int y1, int x2, int y2) const;

    /**
    * angleToBin converts an angle to a bin. The angle must be in the range [0, 2pi). This bin should be used when
    * marking an observed or missed cell.
    */
    int angleToBin(double angle) const;

    /**
    * binToAngle converts a bin to an angle. The angle returned will be in the range [0, 2pi).
    *
    * \param    bin         Bin to convert
    * \pre  bin < numAngleBins && bin >= 0
    */
    double binToAngle(int bin) const;

    /**
    * observedCell indicates an angle from which a cell was observed.
    *
    * \param    x           x-coordinate of cell
    * \param    y           y-coordinate of cell
    * \param    angleBin    Angle in of the cell
    * \param    intensity   Measured intensity (optional, default = 0)
    * \return   True if the state of the particular cell switches from free to occupied.
    */
    bool observedCell(int x, int y, int angleBin, uint16_t intensity = 0);

    /**
    * missedCell indicates the angle from which a cell was seen through.
    *
    * \return   True if the state of the cell switches from occupied to free.
    */
    bool missedCell(int x, int y, int angleBin);

    /**
    * reflectedCell marks a cell as being identified due to a reflection. A reflected resets all knowledge of the
    * particular bin to 0.
    */
    void reflectedCell(int x, int y, int angleBin);

    /**
    * flattenActiveRegion flattens the angle bins in the active region into a single occupancy grid value -- occupied
    * or free.
    *
    * \param    minVisibleRange         A parameter controlling the number of angle bins in which a cell is visible
    *   in order for it to be considered occupied (optional, default = 1)
    */
    void flattenActiveRegion(int minVisibleRange = 1);

    /**
    * flattenFullMap flattens the entire glass map into an occupancy grid.
    *
    * \param    minVisibleRange         A parameter controlling the number of angle bins in which a cell is visible
    *   in order for it to be considered occupied (optional, default = 1)
    */
    void flattenFullMap(int minVisibleRange = 1);

    /**
    * filterDynamicObjects filters dynamic objects out of the flattened map. The filtering process starts from
    * highly-visible cells and performs a breadth-first search through (x, y, theta) space. Any occupied cell
    * not encountered by the search becomes a dynamic cell.
    *
    * The filter only deals with the flattened map, so if the flattened map is stale, i.e. flattenMap() hasn't been
    * called recently, then the filter may not work as desired.
    *
    * The filter for dynamic objects occurs only within the active region in the map, as it requires angle data
    * which may or may not be loaded at the time.
    *
    * \param    minHighlyVisibleRange           Minimum visible range for a highly-visible cell
    *   (optional, default = 20)
    * \param    minGlassIntensity               Minimum intensity for a cell to be considered high-confidence glass,
    *   (optional, default = 8000)
    *
    * NOTE: The intensity threshold is sensor dependent. Every model of lidar will need to have a different threshold.
    * The default threshold here is for a Hokuyo UTM-30LX
    */
    void filterDynamicObjectsFromActiveRegion(int minHighlyVisibleRange = 20, uint16_t minGlassIntensity = 8000);

    /**
    * filterDynamicObjectsFromFullMap filters dynamic objects from the entire map. It shifts the active region as
    * needed to pass the filter over the full map. The active region is move by half its width/height, so the final
    * results of the filter might not be exactly the same as the incremental filtering performed during SLAM.
    *
    * At the end of the filtering, the active region will be returned to its original configuration.
    *
    * \param    minHighlyVisibleRange           Minimum visible range for a highly-visible cell
    *   (optional, default = 20)
    * \param    minGlassIntensity               Minimum intensity for a cell to be considered high-confidence glass,
    *   (optional, default = 8000)
    *
    * NOTE: The intensity threshold is sensor dependent. Every model of lidar will need to have a different threshold.
    * The default threshold here is for a Hokuyo UTM-30LX
    */
    void filterDynamicObjectsFromFullMap(int minHighlyVisibleRange = 20, uint16_t minGlassIntensity = 8000);

    /**
    * flattenedMap retrieves the map in which dynamic objects have been filtered out and the angle bins have been
    * flattened into a single occupancy value -- yay or nay.
    */
    const LocalPerceptualMap& flattenedMap(void) const { return flattenedGrid_; }

    /**
    * intensityMap retrieves the map containing the maximum intensity measurement for each cell in the grid. This map
    * only contains valid data if intensity information was available in the laser scan.
    */
    const IntensityGrid& intensityMap(void) const { return intensityGrid_; }

    /**
    * glassToFlatMapOffset retrieves the cell offset between the glass map and the flattened map. The origin of the glass
    * map is located at the returned offset point in the flat map. This offset allows easy and exact conversion between
    * the glass and flattened maps.
    *
    *   glassCell + glassToFlatMapOffset = flatCell
    *   flatCell - glassToFlatMapOffset = glassCell
    */
    Point<int> glassToFlatMapOffset(void) const { return glassToFlatCellOffset_; }

private:

    int64_t timestamp_;
    int32_t id_;

    int numAngleBins_;
    int8_t hitThreshold_;
    int8_t missThreshold_;
    mutable utils::DiscretizedAngleGrid hitGrid_;
    LocalPerceptualMap flattenedGrid_;
    utils::CellGrid<int16_t> hitCount_;
    utils::CellGrid<int16_t> missCount_;
    IntensityGrid intensityGrid_;
    bool haveIntensity_ = false;        // flag indicating if there is valid, non-zero intensity data in the scan
    Point<int> glassToFlatCellOffset_;

    void setGridOffset(void);
    Point<int> glassToFlatMap(int x, int y);

    // boundary in cell is for hitCount_/flattenedGrid_
    void flattenRegion(math::Rectangle<int> boundaryInCells, int minVisibleRange);
    void flushCacheToDisk(void) const;

    bool isHit(int8_t value) const { return value >= hitThreshold_; }

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        ar( timestamp_,
            id_,
            numAngleBins_,
            hitThreshold_,
            missThreshold_,
            hitGrid_,
            flattenedGrid_,
            hitCount_,
            missCount_,
            intensityGrid_,
            haveIntensity_,
            glassToFlatCellOffset_
        );
    }

    template <class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        flushCacheToDisk();

        ar( timestamp_,
            id_,
            numAngleBins_,
            hitThreshold_,
            missThreshold_,
            hitGrid_,
            flattenedGrid_,
            hitCount_,
            missCount_,
            intensityGrid_,
            haveIntensity_,
            glassToFlatCellOffset_
        );
    }
};

} // namespace hssh
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(hssh::GlassMap, ("DEBUG_HSSH_GLASS_MAP"))

#endif // HSSH_LOCAL_METRIC_GLASS_MAP_H
