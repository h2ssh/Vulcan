/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist_maxima.cpp
* \author   Collin Johnson
*
* Definition of VoronoiIsovistMaxima.
*/

#include <hssh/local_topological/area_detection/gateways/isovist_maxima.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_edges.h>
#include <math/savitzky_golay.h>
#include <math/statistics.h>
#include <utils/algorithm_ext.h>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

// #define DEBUG_VISIBLE_UNITS

namespace vulcan
{
namespace hssh
{

isovist_local_maximum_t create_local_maximum(CellConstIter begin,
                                             CellConstIter end,
                                             CellConstIter beginEdgeCells,
                                             CellConstIter endEdgeCells,
                                             const VoronoiIsovistGradients& gradients,
                                             const VoronoiSkeletonGrid& skeleton);


VoronoiIsovistMaxima::VoronoiIsovistMaxima(const VoronoiIsovistGradients& gradients,
                                           const VoronoiEdges& edges,
                                           const VoronoiSkeletonGrid& grid,
                                           int numAboveMean,
                                           bool saveData)
: numAboveMean_(numAboveMean)
, saveData_(saveData)
, gradients_(&gradients)
, grid_(&grid)
{
    for(auto& e : edges)
    {
        if(e.size() >= 5)
        {
            calculateEdgeDerivatives(e.begin(), e.end());
            findMaximaAlongEdge(e.begin(), e.end(), 2);
        }
    }
}


void VoronoiIsovistMaxima::calculateEdgeDerivatives(CellConstIter edgeBegin, CellConstIter edgeEnd)
{
    edgeGradients_.clear();
    edgeDerivs_.clear();

    std::transform(edgeBegin, edgeEnd, std::back_inserter(edgeGradients_), [this](cell_t cell) {
        return gradients_->gradientAt(cell).value;
    });

    edgeDerivs_.resize(edgeGradients_.size());

    if(edgeGradients_.size() >= 7)
    {
        math::savitzky_golay_first_deriv(edgeGradients_.begin(),
                                         edgeGradients_.end(),
                                         edgeDerivs_.begin(),
                                         math::SGWindowRadius::three);
    }
    else if(edgeGradients_.size() >= 5)
    {
        math::savitzky_golay_first_deriv(edgeGradients_.begin(),
                                         edgeGradients_.end(),
                                         edgeDerivs_.begin(),
                                         math::SGWindowRadius::two);
    }
    else
    {
        std::adjacent_difference(edgeGradients_.begin(), edgeGradients_.end(), edgeDerivs_.begin());
        edgeDerivs_[0] = 0.0;
    }

    // Save the original data
    if(saveData_)
    {
        std::ostringstream file;
        file << "values_" << edgeBegin->x << '_' << (edgeEnd-1)->x << ".dat";
        std::ofstream out(file.str());

        int index = 0;
        for(auto cell : boost::make_iterator_range(edgeBegin, edgeEnd))
        {
            out << index << ' ' << gradients_->gradientAt(cell).data << '\n';
            ++index;
        }
    }

    // Save the gradient
    if(saveData_)
    {
        std::ostringstream file;
        file << "gradients_" << edgeBegin->x << '_' << (edgeEnd-1)->x << ".dat";
        std::ofstream out(file.str());

        int index = 0;
        for(auto& diff : edgeGradients_)
        {
            out << index << ' ' << diff << '\n';
            ++index;
        }
    }

    // Save the derivate of the gradient
    if(saveData_)
    {
        std::ostringstream file;
        file << "deriv_" << edgeBegin->x << '_' << (edgeEnd-1)->x << ".dat";
        std::ofstream out(file.str());

        int index = 0;
        for(auto& diff : edgeDerivs_)
        {
            out << index << ' ' << diff << '\n';
            ++index;
        }
    }
}


void VoronoiIsovistMaxima::findMaximaAlongEdge(CellConstIter edgeBegin, CellConstIter edgeEnd, int descendingCutoff)
{
    // Need the start index for saving the data later, as only want to incorporate maxima added on this update
    std::size_t maximaStartIndex = maxima_.size();

    std::vector<double> absGradients(std::distance(edgeBegin, edgeEnd));
    std::transform(edgeGradients_.begin(), edgeGradients_.end(), absGradients.begin(), [](double val) {
        return std::abs(val);
    });

    const double meanGradient = math::mean(absGradients.begin(), absGradients.end());

    int maximaStart      = 0;
    int maximaEnd        = 0;
    bool haveOppositeFlip = false;
    bool haveSameFlip = false;

    // The gradients are always positive as calculated. Thus, the start of a change will in the derivative will always
    // be positive to start.
    for(std::size_t n = 0; n < edgeGradients_.size(); ++n)
    {
        if(std::signbit(edgeDerivs_[n]) != std::signbit(edgeDerivs_[n-1]))
        {
            if(std::signbit(edgeDerivs_[n]) == std::signbit(edgeGradients_[n]))
            {
                haveSameFlip = true;
            }
            else // signs are different
            {
                haveOppositeFlip = true;
                haveSameFlip = false;
            }
        }

        // If increasing again and was already increasing and decreasing, then end of change region has been found
        bool isGradientEnd = std::signbit(edgeGradients_[n]) != std::signbit(edgeGradients_[n-1])
            || (haveSameFlip && haveOppositeFlip);
        bool isEndOfEdge   = n+1 == edgeGradients_.size();
        if((isGradientEnd || isEndOfEdge))
        {
            maximaEnd = isEndOfEdge ? n+1 : n;

            int outlierCount = std::count_if(absGradients.begin() + maximaStart,
                                             absGradients.begin() + maximaEnd,
                                             [&meanGradient](double gradient) {
                return gradient > meanGradient;
            });

            if(outlierCount >= numAboveMean_)
            {
                maxima_.push_back(create_local_maximum(edgeBegin+maximaStart,
                                                       edgeBegin+maximaEnd,
                                                       edgeBegin,
                                                       edgeEnd,
                                                       *gradients_,
                                                       *grid_));
            }

            maximaStart = maximaEnd;
            haveSameFlip = false;
            haveOppositeFlip = false;
        }
    }

    if(saveData_)
    {
        std::ostringstream file;
        file << "maxima_" << edgeBegin->x << '_' << (edgeEnd-1)->x << ".dat";
        std::ofstream out(file.str());

        for(auto& maximum : boost::make_iterator_range(maxima_.begin() + maximaStartIndex, maxima_.end()))
        {
            int index = std::distance(edgeBegin, std::find(edgeBegin, edgeEnd, maximum.maximum.position));
            out << index << ' ' << maximum.totalChange << '\n';
        }
    }
}


isovist_local_maximum_t create_local_maximum(CellConstIter begin,
                                             CellConstIter end,
                                             CellConstIter beginEdgeCells,
                                             CellConstIter endEdgeCells,
                                             const VoronoiIsovistGradients& gradients,
                                             const VoronoiSkeletonGrid& skeleton)
{
    assert(std::distance(begin, end) > 1);

    // Find all source cells for the maxima skeleton cells and count the gradient
    isovist_local_maximum_t maximum;
    maximum.totalChange = 0.0;

    CellSet maximaSources;
    for(auto cell : boost::make_iterator_range(begin, end))
    {
        maximaSources.insert(skeleton.beginSourceCells(cell), skeleton.endSourceCells(cell));

        maximum.skeletonCells.emplace_back(gradients.gradientAt(cell));
        maximum.totalChange += std::abs(maximum.skeletonCells.back().value);
    }

    std::sort(maximum.skeletonCells.begin(), maximum.skeletonCells.end(), [](position_value_t lhs, position_value_t rhs) {
        return std::abs(lhs.value) > std::abs(rhs.value);
    });

    maximum.maximum = maximum.skeletonCells.front();

    // For each cell along the edge, if it shares all of the source cells, then it should be added to the cells for
    // the maxima, but not counted towards the total value

    for(auto edgeCell : boost::make_iterator_range(beginEdgeCells + 1, endEdgeCells - 1))
    {
        // Only add cells that are not along the edge. Both edgeCells and maxima cells are sorted so they iterate in
        // the same order
        if(edgeCell == *begin)
        {
            ++begin;
            continue;
        }

        bool hasMaximaSource = true;

        for(auto source : boost::make_iterator_range(skeleton.beginSourceCells(edgeCell), skeleton.endSourceCells(edgeCell)))
        {
            hasMaximaSource &= maximaSources.find(source) != maximaSources.end();
        }

        if(hasMaximaSource)
        {
            maximum.skeletonCells.emplace_back(gradients.gradientAt(edgeCell));
        }
    }

    return maximum;
}

} // namespace hssh
} // namespace vulcan
