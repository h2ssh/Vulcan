/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     scan_likelihood_grid.h
 * \author   Collin Johnson
 *
 * Declaration of ScanLikelihoodGrid.
 */

#ifndef LASER_SCAN_LIKELIHOOD_GRID_H
#define LASER_SCAN_LIKELIHOOD_GRID_H

#include "core/laser_scan.h"
#include "utils/cell_grid.h"
#include <cstdint>
#include <vector>

namespace vulcan
{
namespace laser
{

/**
 * ScanLikelihoodGrid is a grid representation of a scan that is used by the CorrelativeScanMatcher
 * to determine the likelihood of a given transform. The likelihood grid represents each point in
 * the reference scan as a radially symmetric Gaussian.
 *
 * The likelihood of a scan is the sum of the likelihood of each grid cell in which a scan point falls.
 * This likelihood is not normalized, so the most likely scan will simply be the one with the highest
 * score. Summing the scores across an entire region of transforms will give a reasonable normalizer.
 *
 * To use the ScanLikelihoodGrid, there are two primary methods:
 *
 *   setReferenceScan(scan) - sets the reference scan to be used for all comparisons, erases the
 *                            prior reference scan. Multiple reference scans may be supported in
 *                            the future, but are not right now
 *
 *   calculateScanLikelihood(scan) - calculates the likelihood of a scan based on the reference scan
 *
 *
 * The other method of use for the CorrelativeScanMatcher is creating a coarser representation of
 * the likelihood grid via the createCoarseGrid() method. A coarse grid is created by taking the
 * maximum likelihood of all fine-resolution cells within a single coarse cell. Doing so creates
 * the following constraints: likelihood_coarse >= likelihood_fine for all scan points. In other
 * words, the coarse grid represents the upper bound on the likelihood of a scan point within a
 * region in the fine-resolution grid.
 *
 */
class ScanLikelihoodGrid
{
public:
    /**
     * Constructor for ScanLikelihoodGrid.
     */
    ScanLikelihoodGrid(uint16_t widthInCells, uint16_t heightInCells, float metersPerCell);

    /**
     * setLaserVariance sets the variance of the laser sensor being used. The variance determines
     * the size of the raster used in the grid. If the variance is less than the size of a grid cell,
     * then a scan point will be represented by only a single cell. This representation would likely
     * result in poor matching behavior.
     *
     * The variance must be set before the first reference scan is set.
     *
     * \param    variance        Variance of the sensor in m^2
     */
    void setLaserVariance(float variance);

    /**
     * setReferenceScan sets the reference scan for the grid. The reference scan is assumed to be
     * in robot-centered coordinates, and thus does not require a pose. As other scans are checking
     * for their position relative to this scan, the actual coordinate frame doesn't matter as long
     * as they are the same for the two scans.
     *
     * \param    scan            Scan to be used for the reference
     */
    void setReferenceScan(const std::vector<Point<float>>& scan);

    /**
     * setLikelihoodSearchBaseScan initializes the state for a new search through the likelihood grid. The
     * scan provided is used as the base for determining the likelihood of scan transforms calculated
     * in calculateTransformLikelihood().
     *
     * \param    scan            Scan to be used as the base scan for transform searches
     */
    void setLikelihoodSearchBaseScan(const std::vector<Point<float>>& baseScan);

    /**
     * calculateTransformLikelihood finds the likelihood of the provided transform for the base scan
     * as compared to the reference scan.
     * The likelihood is an unnormalized log-likelihood that only is valid in comparison to other
     * scans using the same reference.
     *
     * \param    deltaX          Change in the x-coordinate for the transform
     * \param    deltaY          Change in the y-coordinate for the transform
     * \return   The log-likelihood of the scan.
     */
    uint32_t calculateTransformLikelihood(float deltaX, float deltaY) const;

    /**
     * createCoarseGrid makes a coarser representation of the current grid. See the class description
     * for details on the representation. For the coarse grid to work well, the coarse resolution
     * should be a multiple of fine-resolution, as provided in the constructor argument metersPerCell.
     *
     * A ScanLikelihoodGrid is provided as a referenced argument to allow the same grid to be used
     * over-and-over, i.e. a simple optimization over allocating a new grid each time, as the
     * likelihood grids could be rather large.
     *
     * \pre      coarseMetersPerCell > metersPerCell
     *
     * \param    coarseMetersPerCell         Meters per cell in the coarse representation of the grid
     * \param    coarseGrid                  Grid in which to save the coarse grid (output)
     * \return   The actual coarseMetersPerCell value used when creating the grid.
     */
    float createCoarseGrid(float coarseMetersPerCell, ScanLikelihoodGrid& coarseGrid) const;

private:
    void setReferenceScanOrigin(const std::vector<Point<float>>& scan);

    void rasterizeScanPoint(const Point<float>& scanPoint);

    void createRaster(float variance);
    void setRasterSize(float variance);
    void fillRasterArray(float variance);

    static const uint8_t MAX_RASTER_VALUE = 255;

    // Faster to put the raster on the stack
    static const uint8_t MAX_RASTER_SIZE = 51;

    uint8_t scanPointRaster[MAX_RASTER_SIZE][MAX_RASTER_SIZE];   // bottom left of raster
                                                                 // regardless of size is always [0][0]
    uint8_t rasterSize;                                          // actual raster size given the current variance

    utils::CellGrid<uint8_t> grid;
    Point<int16_t> referenceScanOrigin;   // position of (0, 0) for reference scan relative to the grid

    // Cached info about the current search scan being used
    std::vector<Point<uint16_t>> baseLikelihoodScanPoints;
};

}   // namespace laser
}   // namespace vulcan

#endif   // LASER_SCAN_LIKELIHOOD_GRID_H
