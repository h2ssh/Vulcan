/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_builder.h
* \author   Collin Johnson
*
* Declaration of LPMBuilder abstract base class for various methods of building the LocalPerceptualMap.
*
* Declaration of create_lpm_mapper() factory function.
*/

#ifndef HSSH_LOCAL_METRIC_MAPPING_LPM_MAPPER_H
#define HSSH_LOCAL_METRIC_MAPPING_LPM_MAPPER_H

#include "hssh/metrical/mapping/map_builder.h"
#include "hssh/local_metric/lpm.h"

namespace vulcan
{
namespace hssh
{

/**
* LPMBuilder builds a LocalPerceptualMap by incorporating the latest scan raster into the LocalPerceptualMap.
*/
class LPMBuilder : public MapBuilder<LocalPerceptualMap>
{
public:

    /**
    * Constructor for LPMBuilder.
    */
    LPMBuilder(const lpm_params_t& params, const pose_t& currentPose);

private:

    // MapBuilder interface
    void reset(void) override;
    void boundaryChanged(const math::Rectangle<float>& boundary) override;
    void update(const map_update_data_t& data) override;
    void rotate(float radians) override;
    
    LocalPerceptualMap&       getMapInstance(void) override      { return map; }
    const LocalPerceptualMap& getMapInstance(void) const override { return map; }
    
    LocalPerceptualMap map;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_METRIC_MAPPING_LocalPerceptualMap_MAPPER_H
