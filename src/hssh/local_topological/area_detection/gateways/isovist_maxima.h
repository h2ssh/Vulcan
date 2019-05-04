/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist_maxima.h
* \author   Collin Johnson
*
* Declaration of VoronoiIsovistMaxima and isovist_local_maxima_t.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_MAXIMA_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_MAXIMA_H

#include <hssh/types.h>
#include <hssh/local_topological/area_detection/gateways/isovist_gradients.h>
#include <unordered_set>

namespace vulcan
{
namespace hssh
{

/**
* isovist_local_maxima_t represents a local maxima of isovist values within a scalar isovist field.
*/
struct isovist_local_maximum_t
{
    position_value_t maximum;                       ///< The position of the maxima
    std::vector<position_value_t> skeletonCells;    ///< All skeleton cells associated with one of the maxima cells along the
                                                    ///< edge in which the maxima was found
                                                    ///< Sorted from in descending order of gradient value
    double totalChange;                             ///< Cumulative change across the entire gradient
};

// inline bool operator<(const isovist_local_maximum_t& lhs, const isovist_local_maximum_t& rhs) { return lhs.totalChange < rhs.totalChange; };
inline bool operator<(const isovist_local_maximum_t& lhs, const isovist_local_maximum_t& rhs) { return lhs.maximum < rhs.maximum; };
inline bool operator>(const isovist_local_maximum_t& lhs, const isovist_local_maximum_t& rhs) { return lhs.maximum > rhs.maximum; };

/**
* VoronoiIsovistMaxima finds and provides access to the local maxima amongst the isovist gradients.
*
* The maxima are stored from largest to smallest.
*/
class VoronoiIsovistMaxima
{
public:

    using ConstMaximaIter = std::vector<isovist_local_maximum_t>::const_iterator;

    /**
    * Constructor for VoronoiIsovistMaxima.
    *
    * \param    gradients               Isovist gradients from which to extract the maxima
    * \param    edges                   Edges in the grid
    * \param    grid                    Grid in which the isovists were calculated
    * \param    numAboveMean            Retrieve the number of gradient values that must be above the mean value for a maxima to be valid
    * \param    saveData                Flag indicating if data should be saved for debugging (optional, default = false)
    */
    VoronoiIsovistMaxima(const VoronoiIsovistGradients& gradients, 
                         const VoronoiEdges& edges,
                         const VoronoiSkeletonGrid& grid,
                         int numAboveMean = 2,
                         bool saveData = false);

    // Iterator for accessing the maxima
    std::size_t     size(void)  const { return maxima_.size();  }
    ConstMaximaIter begin(void) const { return maxima_.begin(); }
    ConstMaximaIter end(void)   const { return maxima_.end();   }
    const isovist_local_maximum_t& operator[](std::size_t n) const { return maxima_[n]; }

private:

    std::vector<isovist_local_maximum_t> maxima_;
    std::vector<double> edgeGradients_;
    std::vector<double> edgeDerivs_;
    int numAboveMean_;
    bool saveData_;

    const VoronoiIsovistGradients* gradients_;
    const VoronoiSkeletonGrid*     grid_;

    void calculateEdgeDerivatives(CellConstIter edgeBegin, CellConstIter edgeEnd);
    void findMaximaAlongEdge(CellConstIter begin, CellConstIter end, int descendingCutoff);
};

} // namespace hssh 
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_MAXIMA_H
