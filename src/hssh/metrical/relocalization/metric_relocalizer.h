/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_relocalizer.h
* \author   Collin Johnson
* 
* Declaration of MetricRelocalizer.
*/

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_METRIC_RELOCALIZER_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_METRIC_RELOCALIZER_H

#include <hssh/metrical/relocalization/params.h>
#include <hssh/metrical/relocalization/types.h>
#include <hssh/metrical/occupancy_grid.h>
#include <vector>

namespace vulcan
{
namespace robot { struct velocity_t; }
namespace hssh
{

struct metric_relocalization_debug_info_t;
struct metric_slam_data_t;
struct particle_t;
class  Mapper;
class  MonteCarloLocalization;
class  OccupancyGrid;
class  ParticleFilter;
class  FilterInitializer;

/**
* MetricRelocalizer relocalizes the robot within a stored metric gridmap using current sensor data.
* The relocalizer runs the same particle filter as the local_metric_hssh localizer. The relocalizer
* is provided a relocalization request that contains the stored map and the estimated region in which
* the robot might be located.
* 
* The initial estimate for the region can be one of the following, as specified via relocalization_initialization_mode_t:
* 
*   - A Gaussian distribution from which the actual pose can be sampled.
*   - A rectangular region from which the samples should be drawn uniformly.
*   - Use a scan matcher to determine the initial Gaussian from which to sample.
* 
* After initialization, the relocalizer is run using the same sensor inputs as the normal local SLAM process. After
* each update, the progress of the relocalization is given. Once completed, then the relocalization action should
* be performed via the performAction() method, followed by sending out the message indicating the success or
* failure of the relocalization.
* 
* On each update, debugging info for the relocalization can be obtained. The information includes the initial samples,
* the current samples, the estimated pose, and the map. Combining these will allow for seeing how progress is moving
* along with regards to figuring out the robot position in the map.
*/
class MetricRelocalizer
{
public:
    
    /**
    * Constructor for MetricRelocalizer.
    * 
    * \param    params          Parameters for the relocalizer
    */
    MetricRelocalizer(const metric_relocalizer_params_t& params);
    
    /**
    * Destructor for MetricRelocalizer.
    */
    ~MetricRelocalizer(void);
    
    /**
    * processRequest processes a new localization request. The request will initialize a new relocalization
    * process. If a current process was underway, it will be eliminated in favor of the new request.
    * 
    * \param    data                Data to use in the initialization
    * \param    map                 Map in which the relocalization will happen
    * \param    initializer         Initializer to use for generating the initial set of particles
    */
    void startRelocalization(const metric_slam_data_t& data,
                             const OccupancyGrid&      map,
                             const FilterInitializer&  initializer);
    
    /**
    * updateRelocalization provides new sensor data to the relocalizer for updating the estimated pose within
    * the requested map. The current progress of the relocalization is returned, allowing the caller to
    * determine the next action to take with regards relocalization.
    * 
    * The progress of the task is:
    *   - NO_TASK     : nothing to do, why are you updating at all?
    *   - IN_PROGRESS : still need more time/data to localize in the map
    *   - COMPLETED   : successfully localized!
    *   - FAILED      : the robot must not be in this map because the sensor data doesn't agree
    * 
    * \param    data        Data to use for the relocalization update
    * \param    debug       
    * \return   The progress of the relocalization task.
    */
    relocalization_progress_t updateRelocalization(const metric_slam_data_t& data,
                                                   metric_relocalization_debug_info_t* debug = 0);

    /**
    * isRelocalizing checks if the relocalizer is currently relocalizing within a new map.
    */
    bool isRelocalizing(void) const { return status_ == RelocalizationStatus::InPrograss; }

    /**
    * relocalizedMap retrieves the map in which the relocalizer has been relocalizing.
    */
    const OccupancyGrid& relocalizedMap(void) const { return map_; }

private:
    
    RelocalizationStatus            status_;
    std::unique_ptr<ParticleFilter> filter_;
    int                             numRelocalizationAttempts_;
    std::vector<particle_t>         initialSamples_;
    OccupancyGrid map_;         // Map in which to relocalize
    
    const int   maxRelocalizationAttempts_;
    const float maxPositionStdDev_;
    const float maxOrientationStdDev_;
    
    void initializeParticleFilter(const metric_slam_data_t& data,
                                  const OccupancyGrid&      map,
                                  const FilterInitializer&  initializer);
    bool isRelocalized(const pose_distribution_t& pose);
};

}
}

#endif // HSSH_UTILS_METRICAL_RELOCALIZATION_METRIC_RELOCALIZER_H
