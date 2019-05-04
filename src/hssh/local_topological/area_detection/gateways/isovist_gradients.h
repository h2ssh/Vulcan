/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist_gradients.h
* \author   Collin Johnson
*
* Declaration of VoronoiIsovistGradients and position_value_t.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_GRADIENTS_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_GRADIENTS_H

#include <hssh/types.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_edges.h>
#include <utils/isovist.h>
#include <memory>

namespace vulcan
{
namespace hssh
{

class VoronoiSkeletonGrid;
class VoronoiIsovistField;

/**
* position_value_t is a pair, mating a position with some numeric value.
*/
struct position_value_t
{
    cell_t position;
    double value;
    double data;

    explicit position_value_t(cell_t position = cell_t(), double value = 0.0, double data = 0.0)
    : position(position)
    , value(value)
    , data(data)
    {
    }
};

inline bool operator<(const position_value_t& lhs, const position_value_t& rhs) { return lhs.value < rhs.value; }
inline bool operator>(const position_value_t& lhs, const position_value_t& rhs) { return lhs.value > rhs.value; }
inline bool operator==(const position_value_t& lhs, const position_value_t& rhs) { return lhs.position == rhs.position; }
inline bool operator!=(const position_value_t& lhs, const position_value_t& rhs) { return lhs.position != rhs.position; }

/**
* VoronoiIsovistGradients contains gradient information for a VoronoiIsovistField. The gradient for each isovist doesn't
* contain direction information, just the magnitude, which suffices to get a feel for how things are changing in the map.
*
* Three types of information are available with the gradient:
*
*   1) The maximum gradient at each isovist location for the given scalar.
*/
class VoronoiIsovistGradients
{
public:

    using GradientConstIter = std::vector<position_value_t>::const_iterator;
    
    /**
    * Constructor for VoronoiIsovistGradients.
    *
    * \param    edges           Edges in the skeleton
    */
    explicit VoronoiIsovistGradients(const VoronoiEdges& edges);
    
    /**
    * calculateGradients finds the gradients along the edges in the Voronoi skeleton. The provided edges should be the
    * same edges as were set in the constructor.
    * 
    * \param    scalar          Scalar for which to compute the gradient
    * \param    field           Isovist field containing all isovists found so far
    */
    void calculateGradients(utils::Isovist::Scalar scalar, const VoronoiIsovistField& field);
    

    // Iterate over the gradient values for the isovist scalars
    std::size_t             sizeGradients(void)       const { return gradients_.size();  }
    GradientConstIter       beginGradients(void)      const { return gradients_.begin(); }
    GradientConstIter       endGradients(void)        const { return gradients_.end();   }
    const position_value_t& gradientAt(std::size_t n) const { return gradients_[n]; }
    const position_value_t& gradientAt(cell_t cell)   const { return gradients_[cellToIndex_.find(cell)->second]; }
    
    GradientConstIter begin(void) const { return gradients_.begin(); }
    GradientConstIter end(void) const { return gradients_.end();   }

private:

    using indices_t = std::vector<std::size_t>;

    VoronoiEdges edges_;
    CellVector isovistCells_;
    CellToIntMap cellToIndex_;
    
    std::vector<position_value_t> gradients_;
    std::vector<double> edgeDerivs_;
    std::vector<double> edgeScalars_;

    void initializeCellToIndex(void);
    void calculateGradients(utils::Isovist::Scalar scalar, 
                            const VoronoiIsovistField& field, 
                            const VoronoiSkeletonGrid& grid);
    void calculateGradientsForEdge(const CellVector& cells, 
                                   utils::Isovist::Scalar scalar, 
                                   const VoronoiIsovistField& field);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_GRADIENTS_H
