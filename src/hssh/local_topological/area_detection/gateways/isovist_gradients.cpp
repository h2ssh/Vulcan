/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist_gradients.cpp
* \author   Collin Johnson
*
* Definition of VoronoiIsovistGradients.
*/

#include "hssh/local_topological/area_detection/gateways/isovist_gradients.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "math/savitzky_golay.h"
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <numeric>
#include <cmath>

namespace vulcan
{
namespace hssh
{

VoronoiIsovistGradients::VoronoiIsovistGradients(const VoronoiEdges& edges)
: edges_(edges)
{
    for(const auto& e : edges_)
    {
        boost::push_back(isovistCells_, boost::as_array(e));
    }

    gradients_.resize(isovistCells_.size());
    initializeCellToIndex();
}


void VoronoiIsovistGradients::calculateGradients(utils::Isovist::Scalar scalar, const VoronoiIsovistField& field)
{
    for(const auto& edgeCells : edges_)
    {
        calculateGradientsForEdge(edgeCells, scalar, field);
    }
}


void VoronoiIsovistGradients::initializeCellToIndex(void)
{
    cellToIndex_.reserve(isovistCells_.size());
    int index = 0;
    for(auto& cell : isovistCells_)
    {
        cellToIndex_[cell] = index++;
    }
}


void VoronoiIsovistGradients::calculateGradientsForEdge(const CellVector& cells,
                                                        utils::Isovist::Scalar scalar,
                                                        const VoronoiIsovistField& field)
{
    edgeScalars_.clear();

    std::transform(cells.begin(), cells.end(), std::back_inserter(edgeScalars_), [&](cell_t cell) {
        return field.at(cell).scalar(scalar);
    });

    edgeDerivs_.resize(edgeScalars_.size());

    if(edgeDerivs_.size() >= 7)
    {
        math::savitzky_golay_first_deriv(edgeScalars_.begin(),
                                         edgeScalars_.end(),
                                         edgeDerivs_.begin(),
                                         math::SGWindowRadius::three);
    }
    else if(edgeDerivs_.size() >= 5)
    {
        math::savitzky_golay_first_deriv(edgeScalars_.begin(),
                                         edgeScalars_.end(),
                                         edgeDerivs_.begin(),
                                         math::SGWindowRadius::two);
    }
    else
    {
        std::adjacent_difference(edgeScalars_.begin(), edgeScalars_.end(), edgeDerivs_.begin());
        edgeDerivs_[0] = 0.0;
    }

    for(std::size_t n = 0; n < cells.size(); ++n)
    {
        int cellIndex = cellToIndex_[cells[n]];

        // Take the absolute value of the derivative to account for it being a function of the direction
        // of iteration along the edge. If we iterated the other direction, then it would have the opposite
        // sign -- thus causing some otherwise obvious gateways to "hide"
        gradients_[cellIndex] = position_value_t(cells[n], std::abs(edgeDerivs_[n]), edgeScalars_[n]);

        if((scalar == utils::Isovist::kWeightedOrientation) || (scalar == utils::Isovist::kOrientation))
        {
            gradients_[cellIndex].value = std::abs(wrap_to_pi_2(edgeDerivs_[n]));
        }
    }
}

} // namespace hssh
} // namespace vulcan
