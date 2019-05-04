/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     map_optimizer.h
* \author   Collin Johnson
*
* Declaration of MapOptimizer interface and create_map_optimizer factory.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_MAP_OPTIMIZER_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_MAP_OPTIMIZER_H

#include <memory>

namespace vulcan
{
namespace hssh
{

class  Chi;
class  MapOptimizer;
class  TopologicalMap;
struct map_optimizer_params_t;

/**
* create_map_optimizer is a factory for creating new instances of the MapOptimizer interface.
*
* NOTE: Expect a crash if you pass an unknown type
*
* \param    type            Type of optimizer to create
* \param    params          Parameters for the optimizer
* \return   Pointer to the desired instance of MapOptimizer.
*/
std::unique_ptr<MapOptimizer> create_map_optimizer(const std::string& type, const map_optimizer_params_t& params);

/**
* MapOptimizer defines an interface for algorithms that optimize the location of places within the global
* topological map. The optimization should determine a global layout for the places, specifying (x,y,theta).
* The result of optimization is a Chi instance, which gives the optimized lambda values for each
* PathSegment in the map hypothesis.
*
* The interface for MapOptimizer is a single method:
*
*   Chi optimizeMap(const TopologicalMap& map);
*
*/
class MapOptimizer
{
public:

    virtual ~MapOptimizer(void) {}

    /**
    * optimizeMap calculates an optimized layout for the places in the topological map. The optimization
    * finds a position for each place that minimizes some error with respect to the measured lambda values
    * for the path segments in the map.
    *
    * \param    map         Map to be optimized
    * \return   Chi value giving the position of the places in a global layout.
    */
    virtual Chi optimizeMap(const TopologicalMap& map) = 0;
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_MAP_OPTIMIZER_H
