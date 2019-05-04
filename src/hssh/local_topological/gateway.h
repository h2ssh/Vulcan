/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway.h
* \author   Collin Johnson
*
* Declaration of Gateway and gateway_normal_t and SimilarGatewayComp.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAY_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAY_H

#include <hssh/types.h>
#include <core/line.h>
#include <math/coordinates.h>
#include <cereal/access.hpp>
#include <cstdint>

namespace vulcan
{
struct pose_t;
namespace hssh
{

class VoronoiIsovistField;
class VoronoiSkeletonGrid;

/**
* Gateway defines the location and properties of a gateway in the world. A gateway is the boundary between two areas in metric space, usually a
* path and a place. The gateway forms a bridge between the metric and topological descriptions of space, as topological places are based on symbols
* derived from gateways in the place.
*
* Being that the gateway abstraction separates two areas in small-scale space, the gateway representation fundamentally consists of a boundary, a
* direction, and two normals emanating from the center of the gateway. In addition to the metric coordinates of the boundary and center, the
* gateway contains the cells it occupies in the map it was built from. The cell representation is useful when comparing a gateway to the Voronoi
* skeleton, which is a cell-based representation.
*
* The boundary for a gateway is such that the .a of the boundary line is sorted topographically, so boundary.a < boundary.b
*
* Gateways are immutable.
*
* The gateway description contains the following:
*
*   - id            : id of the gateway in the place description
*   - center        : location of the gateway in the LPM
*   - boundary      : line segment defining the gateway in the LPM
*   - direction     : direction of the interior of the place from the center of the gateway -- the normal of the gateway pointing into the place
*   - cellBoundary  : cells the gateway boundary occupies
*   - skeletonCell  : cell in the Voronoi skeleton through which the gateway passes
*/
class Gateway
{
public:

    /**
    * Default constructor for Gateway.
    *
    * Creates a gateway with in invalid id to indicate it needs to be assigned before being used.
    */
    Gateway(void);

    /**
    * Constructor for Gateway.
    *
    * Creates a Gateway using the associated cells in the Voronoi skeleton. The metric representation of the gateway is found
    * using the provided grid. The direction and normals are calculated.
    *
    * \param    timestamp       Timestamp associated with the gateway
    * \param    id              Unique id to assign to the gateway
    * \param    cellBoundary    Location of the boundary cells defining the endpoints of the gateway
    * \param    skeletonCell    Cell in the rEVG through which the gateway passes
    * \param    grid            Grid in which the gateway was found
    *
    * \pre  All cells for the gateway are contained in the provided grid, i.e.
    *       grid.isCellInGrid(cellBoundary.a) && grid.isCellInGrid(cellBoundary.b) && grid.isCellInGrid(skeletonCell)
    */
    Gateway(int64_t                    timestamp,
            int32_t                    id,
            const Line<int>&     cellBoundary,
            const Point<int>&    skeletonCell,
            const VoronoiSkeletonGrid& grid);

    /**
    * Constructor for Gateway.
    *
    * Creates a Gateway using the associated cells in the Voronoi skeleton. The metric representation of the gateway is found
    * using the provided grid. The direction and normals are calculated.
    *
    * \param    timestamp       Timestamp associated with the gateway
    * \param    id              Unique id to assign to the gateway
    * \param    cellBoundary    Location of the boundary cells defining the endpoints of the gateway
    * \param    skeletonCell    Cell in the rEVG through which the gateway passes
    * \param    isovists        Isovist field of the current LPM
    * \param    grid            Grid in which the gateway was found
    *
    * \pre  All cells for the gateway are contained in the provided grid, i.e.
    *       grid.isCellInGrid(cellBoundary.a) && grid.isCellInGrid(cellBoundary.b) && grid.isCellInGrid(skeletonCell)
    * \pre  An isovist exists for the skeletonCell
    */
    Gateway(int64_t                    timestamp,
            int32_t                    id,
            const Line<int>&     cellBoundary,
            const Point<int>&    skeletonCell,
            const VoronoiIsovistField& isovists,
            const VoronoiSkeletonGrid& grid);

//     /**
//     * Constructor for Gateway.
//     *
//     * Create a Gateway with no cell-based information. Such Gateways are associated with areas, where the
//     * intervening skeleton information has been removed.
//     *
//     * \param    id              Unique id for the gateway
//     * \param    boundary        Metric boundary of the gateway in small-scale space
//     * \param    center          Center of the gateway
//     * \param    direction       Direction that space flows through the gateway
//     */
//     Gateway(int32_t                    id,
//             const Line<double>&  boundary,
//             const Point<double>& center,
//             float                      direction);


    // Observers -- self-explanatory
    int64_t             timestamp(void)      const { return timestamp_; }
    int32_t             id(void)             const { return id_; }
    double              probability(void)    const { return probability_; }
    Line<double>  boundary(void)       const { return boundary_; }
    Point<double> center(void)         const { return center_; }
    float               direction(void)      const { return leftDirection_; }
    double              length(void)         const { return length_; }
    float               leftDirection(void)  const { return leftDirection_; }
    float               rightDirection(void) const { return rightDirection_; }
    Line<int>     cellBoundary(void)   const { return boundaryCells_; }
    Point<int>    skeletonCell(void)   const { return skeletonCell_; }

    // Iterate over cells along the boundary
    std::size_t sizeCells(void) const { return cellsAlongBoundary_.size(); }
    std::vector<cell_t>::const_iterator beginCells(void) const { return cellsAlongBoundary_.begin(); }
    std::vector<cell_t>::const_iterator endCells(void) const { return cellsAlongBoundary_.end(); }

    // Producers
    /**
    * changeReferenceFrame converts the gateway into a new reference frame. The transform is applied to the boundary
    * and the grid is used to ensure the cell representation remains valid.
    *
    * \param    transform       Transform to apply to the gateway
    * \param    grid            Grid in which the gateway exists -- used to determine the cells of the transformed gateway
    * \return   A new Gateway with the boundary and center transformed.
    */
    Gateway changeReferenceFrame(const pose_t& transform, const VoronoiSkeletonGrid& grid) const;

    /**
    * setProbability sets the probability for this gateway. By default, all gateways have the same probability unless
    * changed by this method.
    *
    * \param    probability     Probability of this gateway per some classifier (use 1 if using uniform prior)
    */
    void setProbability(double probability) { probability_ = probability; }

    /**
    * reverseDirections switches the left and right directions for the gateway (which are arbitrary anyway).
    * This method is useful if some outside information is able to provide insight into what left and right means. And
    * in particular if methods will rely on the direction() method, which always is associated with the leftDirection().
    */
    void reverseDirections(void);

    // Operations
    /**
    * isCellToLeft determines if the entirety of the provided cell is to the left, right, or on the gateway cell boundary.
    * The boundary is treated as a line splitting the plane in two. The cell representation is discretized, so a cell is
    * to the left only if all four corners of the cell are to the left of the cell boundary. If only some corners are
    * left of the boundary, then the cell is on the boundary. If no corners are left of the boundary, the cell is right of
    * the boundary.
    *
    * \param    cell        Cell whose position will be checked relative to the gateway
    * \return   1 if left of the gateway. 0 if on the gateway. -1 if right of the gateway.
    */
    int isCellToLeft(const Point<int>& cell) const;

    /**
    * isPointToLeft determines if a metric point is to the left, right, or on the gateway boundary. The boundary is treated as a line
    * splitting the plane of the world into two. The point is assumed to be infinitesimally small. Thus, the point must fall exactly
    * on the boundary line in order to be considered on the line.
    *
    * \param    point       Point whose position will be checked relative to the gateway
    * \return   1 if left of the gateway. 0 if on the gateway. -1 if right of the gateway.
    */
    int isPointToLeft(const Point<float>& point) const;

    /**
    * intersectsWithCellBoundary finds the intersection between a line and the gateway if one exists. If it does exist, the result is
    * stored in intersection.
    *
    * \param        line            Line to check for intersection
    * \param[out]   intersection    Intersection of the line with the cell boundary
    * \return       True if the line intersects the boundary. The result is stored in intersection.
    */
    bool intersectsWithCellBoundary(const Line<int>& line, Point<int>& intersection) const;

    /**
    * intersectsWithBoundary finds the intersection between a line and the gateway if one exists. If it does exist, the result is
    * stored in intersection.
    *
    * \param        line            Line to check for intersection
    * \param[out]   intersection    Intersection of the line with the boundary
    * \return       True if the line intersects the boundary. The result is stored in intersection.
    */
    bool intersectsWithBoundary(const Line<double>& line, Point<double>& intersection) const;

	/**
    * isSimilarTo checks if this gateway is similar to another gateway. Two gateways are considered to be similiar if:
    *
    *   - Their centers are close:   dist(center, rhs.center)       < this.length/2
    *   - Their endpoints are close: dist(endpoints, rhs.endpoints) < 0.2m
    *
    * \param    rhs         Gateway to compare to
    * \return   True if the similarity checks are valid as described above.
    */
    bool isSimilarTo(const Gateway& rhs) const;

private:

    int64_t timestamp_;             ///< Time at which the gateway was observed
    int32_t id_;                    ///< Id for the gateway for the particular place
    double  probability_ = 1.0;     ///< Probability of this gateway existing per some classifier or other means

    Line<double>  boundary_;        ///< Location of the gateway in reference frame of current map
    Point<double> center_;          ///< Metric center of the gateway
    double              length_;         ///< Length of the gateway -- boundary to center to boundary
    float               leftDirection_;   ///< Direction from this gateway to the left
    float               rightDirection_;  ///< Direction from this gateway to the right

    // Fields specific to the map in which the gateway was found
    Point<int> skeletonCell_;      ///< Skeleton cell through which the gateway passes
    Line<int>  boundaryCells_;     ///< Location of the gateway in the place grid cells
    CellVector       cellsAlongBoundary_;   ///< Store the cells on the boundary for more accurate isCellLeft checks

    // Find the direction that is closer to the angle formed by the cell boundary using the skeleton (or isovists if
    // available)
    void calculateDirections(const VoronoiSkeletonGrid& grid, const VoronoiIsovistField* isovists);

    // Serialization support
    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( timestamp_,
            id_,
            probability_,
            boundary_,
            center_,
            length_,
            leftDirection_,
            rightDirection_,
            skeletonCell_,
            boundaryCells_,
            cellsAlongBoundary_);
    }
};

// Operator overloads for Gateway
/**
* operator< is a lexical comparison that looks at the center of the gateway. The operator considers the
* x-coordinate and then the y-coordinate.
*/
bool operator<(const Gateway& lhs, const Gateway& rhs);

/**
* operator== checks the equality of the Gateway boundaries. If the gateways have the same boundary, then
* they are the same gateway.
*/
bool operator==(const Gateway& lhs, const Gateway& rhs);

/**
* operator!= is the inverse of the gateway equality. Are the boundaries different?
*/
bool operator!=(const Gateway& lhs, const Gateway& rhs);


inline std::ostream& operator<<(std::ostream& out, const Gateway& rhs)
{
    out << rhs.boundary();
    return out;
}

/**
* SimilarGatewayComp is a functor that can be used as the Comparator for a set, find or other algorithm. It compares
* two gateways and returns true if isSimilarTo is true.
*/
struct SimilarGatewayComp
{
    const Gateway& lhs;

    SimilarGatewayComp(const Gateway& lhs) : lhs(lhs) { }

    bool operator()(const Gateway& rhs)
    {
        return lhs.isSimilarTo(rhs);
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAY_H
