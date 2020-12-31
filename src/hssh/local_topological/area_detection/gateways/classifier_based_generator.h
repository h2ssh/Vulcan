/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     classifier_based_generator.h
 * \author   Collin Johnson
 *
 * Declaration of ClassifierBasedGenerator.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_CLASSIFIER_BASED_GENERATOR_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_CLASSIFIER_BASED_GENERATOR_H

#include "hssh/local_topological/area_detection/gateways/gateway_classifier.h"
#include "hssh/local_topological/area_detection/gateways/generator.h"
#include <memory>

namespace vulcan
{
namespace hssh
{

struct classifier_based_generator_params_t;

const std::string kClassifierBasedGeneratorType("classifier");

/**
 * ClassifierBasedGenerator is a gateway generator that creates gateways based on their probability.
 */
class ClassifierBasedGenerator : public GatewayGenerator
{
public:
    /**
     * Constructor for ClassifierBasedGenerator.
     *
     * \param    params          Create a generator using the provided parameters
     * \param    mapName         Name of the map in which the robot is operating
     */
    explicit ClassifierBasedGenerator(const classifier_based_generator_params_t& params, const std::string& mapName);

    /**
     * Constructor for ClassifierBasedGenerator.
     *
     * \param    radius              Radius of the window for gateway features
     * \param    classifierFilename  Name of classifier to be loaded
     */
    ClassifierBasedGenerator(int radius, const std::string& classifierFilename);

    /**
     * Destructor for ClassifierBasedGenerator.
     */
    virtual ~ClassifierBasedGenerator(void);

    // GatewayGenerator interface
    std::vector<WeightedGateway> generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                                  const VoronoiIsovistField& isovists,
                                                  const VoronoiSkeletonGrid& skeleton,
                                                  const EndpointValidator& validator) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_CLASSIFIER_BASED_GENERATOR_H
