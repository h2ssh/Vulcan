/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_extent.h
* \author   Collin Johnson
*
* Declaration of AreaExtent.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_EXTENT_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_EXTENT_H

#include <hssh/types.h>
#include <math/geometry/polygon.h>
#include <math/geometry/rectangle.h>
#include <math/coordinates.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <vector>

namespace vulcan
{
namespace hssh
{

class Gateway;
class VoronoiSkeletonGrid;

/**
* extent_compactness_t contains compactness measures of the cells contained within the extent. Five measures are used:
*
*   1) Normalized discretized compactness
*   2) P_min / P
*   3) P / P_max
*   4) A_min / A
*   5) A / A_max
*
* These measures are described in:
*
*   "State of the art of compactness and circularity measures" by Montero et al.
*/
struct extent_compactness_t
{
    double circularity;
    double ndc;
    double pMin;
    double pMax;
    double aMin;
    double aMax;
};

/**
* AreaExtent defines the properties of the extent of an area. The extent of an area is the portion
* of the LPM that the area occupies. The extent itself is a rectangular bounding box. The properties of the
* extent give the proportion of the boundary that is a gateway, occupied cell, or frontier cells. These values
* provide the explored status of the area.
*/
class AreaExtent
{
public:

    using PointIter = std::vector<Point<double>>::const_iterator;

    /**
    * Default constructor for AreaExtent.
    *
    * Attempts to use an extent that is default will result in pain, but some circumstances it to float around for a bit.
    */
    AreaExtent(void) { }

    /**
    * Constructor for AreaExtent.
    *
    * \param    gateways            Gateways bounding the area
    * \param    startCell           A cell known to exist in the area
    * \param    grid                Grid in which the area exists
    */
    AreaExtent(const std::vector<Gateway>& gateways, cell_t startCell, const VoronoiSkeletonGrid& grid);

    /**
    * Constructor for AreaExtent.
    *
    * \param    gateways            Gateways bounding the area
    * \param    skeletonCells       Skeleton cells in the area
    * \param    grid                Grid in which to extract the extent
    */
    AreaExtent(const std::vector<Gateway>& gateways, const CellVector& skeletonCells, const VoronoiSkeletonGrid& grid);

    /**
    * Constructor for AreaExtent.
    *
    * Merges many extents into a single extent
    *
    * \param    extents             Extents to be merged
    */
    AreaExtent(const std::vector<const AreaExtent*>& extents);

    // Statistics about the extent
    pose_t center       (void) const { return center_;        }
    double        area         (void) const { return area_;          }
    double        perimeter    (void) const { return perimeter_;     }
    double        frontierRatio(void) const { return frontierRatio_; }
    double        hullPerimeter(const VoronoiSkeletonGrid& grid) const;
    extent_compactness_t compactness(void) const;

    // Approximations of the area extent
    math::Polygon<double>   polygonBoundary  (math::ReferenceFrame frame = math::ReferenceFrame::LOCAL) const;
    math::Rectangle<double> rectangleBoundary(math::ReferenceFrame frame = math::ReferenceFrame::LOCAL) const;

    /**
    * changeReferenceFrame
    *
    * \param    transformToFrame            Transform to apply to every piece of the extent
    */
    void changeReferenceFrame(const pose_t& transformToFrame);

    /**
    * setOrientation changes the orientation of the center to be some new value. The cell units internally remain the
    * same though.
    */
    void setOrientation(double orientation);

    /**
    * contains checks to see if a particular point is contained in the boundaries of the area.
    */
    bool contains(const Point<double>& point, math::ReferenceFrame frame) const;

    /**
    * cellContains is a finer-grained check to see if something is contained in the extent. The check looks if
    * the provided point is actually inside one of its cells.
    */
    bool cellContains(const Point<double>& position) const;

    // Iterators for the cells contained in the area -- the cells are stored in global coordinates
    std::size_t size (void) const { return cells_.size();  }
    PointIter   begin(void) const { return cells_.begin(); }
    PointIter   end  (void) const { return cells_.end();   }

private:

    // INVARIANT: All points and boundaries are stored in the GLOBAL reference frame

    pose_t center_;
    mutable math::Polygon<double> polygon_;
    math::Rectangle<double> rectangle_;
    mutable bool havePolygon_;

    std::vector<Point<double>> cells_;
    float perimeter_;
    float area_;
    float frontierRatio_;
    double cellsPerMeter_;

    void growExtent(const std::vector<Gateway>& gateways,
                    const CellVector&           skeletonCells,
                    const VoronoiSkeletonGrid&  grid);
    void calculateCenter    (void);
    void calculateBoundaries(void);
    void calculateCompactness(void);

    // Serialization support
    friend class ::cereal::access;

    template <class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        ar( center_,
            polygon_,
            rectangle_,
            cells_,
            perimeter_,
            area_,
            frontierRatio_);
    }

    template <class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        ar( center_,
            polygon_,
            rectangle_,
            cells_,
            perimeter_,
            area_,
            frontierRatio_);

        havePolygon_ = polygon_.size() > 0;
    }
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_EXTENT_H
