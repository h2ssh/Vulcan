/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     isovist_orientation_gateway_generator.h
 * \author   Collin Johnson
 *
 * Declaration of IsovistOrientationGatewayGenerator.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_ISOVIST_ORIENTATION_GATEWAY_GENERATOR_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_ISOVIST_ORIENTATION_GATEWAY_GENERATOR_H

#include "hssh/local_topological/area_detection/gateways/generator.h"
#include "hssh/local_topological/area_detection/gateways/isovist_gradients.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/params.h"

namespace vulcan
{
namespace hssh
{

struct isovist_local_maximum_t;

const std::string kIsovistOrientationGatewayGeneratorType("orientation");

/**
 * IsovistOrientationGatewayGenerator generates gateways by finding the orientation associated with an isovist maximum
 * and using that to determine where the gateway should go.
 */
class IsovistOrientationGatewayGenerator : public GatewayGenerator
{
public:
    /**
     * Constructor for IsovistOrientationGatewayGenerator.
     *
     * \param    params          Parameters for generating the gateways
     */
    IsovistOrientationGatewayGenerator(const isovist_orientation_gateway_generator_params_t& params);

    // GatewayGenerator interface
    std::vector<WeightedGateway> generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                                  const VoronoiIsovistField& isovists,
                                                  const VoronoiSkeletonGrid& grid,
                                                  const EndpointValidator& validator) override;

private:
    int32_t nextGatewayId_;
    const VoronoiIsovistGradients* gradients_;
    const VoronoiIsovistField* isovists_;
    const VoronoiSkeletonGrid* grid_;
    const EndpointValidator* validator_;

    SourceToCellsMap sourceToSkeleton_;
    CellVector edgeCells_;
    std::deque<position_value_t> maximumCells_;
    std::vector<WeightedGateway> gatewayHypotheses_;

    isovist_orientation_gateway_generator_params_t params_;

    void createGatewaysForMaximum(const isovist_local_maximum_t& maximum);
    void extractEdgeCellsForMaximum(const isovist_local_maximum_t& maximum);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_ISOVIST_ORIENTATION_GATEWAY_GENERATOR_H
