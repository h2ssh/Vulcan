/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_detector.h
* \author   Collin Johnson
*
* Declaration of AreaDetector.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTOR_H

#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/error.h"
#include "hssh/local_topological/event.h"
#include <boost/optional.hpp>
#include <memory>
#include <cassert>

namespace vulcan
{
namespace system { class DebugCommunicator; }
namespace utils { class CommandLine; }
namespace utils { class ConfigFile; }
namespace hssh
{

// From local_metric
class LocalPose;
class LocalPerceptualMap;

// From local_topological
class AreaClassifier;
class GatewayLocator;
class VoronoiSkeletonBuilder;

/**
* AreaDetectorResult defines the result of the area detection process. The process is either sucessful, in which
* case map != boost::none. Otherwise, result contains the error code for the problem that occurred during the
* search for area labels.
*/
struct AreaDetectorResult
{
    boost::optional<LocalTopoMap> map;
    LabelingError result;

    // For success, simply pass the proposals
    explicit AreaDetectorResult(const LocalTopoMap& map)
    : map(map)
    , result(LabelingError::success)
{
    }

    // For errors, pass the error code. Error can't be success because there must be proposals!
    explicit AreaDetectorResult(LabelingError error)
    : map(boost::none)
    , result(error)
    {
        assert(result != LabelingError::success);
    }
};

/**
* AreaDetector handles the task of detecting and tracking areas in the LPM. The detector uses a visibility-based
* gateway detector to create the initial parsing of the environment into areas. These proposed area are evaluted for
* a couple conditions:
*
*   1) Does it sit at the intersection of paths?
*   2) Is it more path-like or place-like?
*
* After the ambiguous regions are classified via a CSP and maximum likelihood classifier, the final areas are passed to
* a tracker that builds consistent models of the areas for the duration of time in which some portion of the area
* remains in the LPM. This tracking is needed to ensure that path segments are properly constructed, even in the event
* that the start of the path segment scrolls out of the LPM before reaching the end.
*
* Because the areas in the map are tracked over time, calls to detectAreas() with new maps will build the representation
* over time. As such, if the LPM changes in some dramatic way, say a log is restarted, then the areas should be reset
* via resetAreas() to ensure that the newly detected areas are invalidated by pre-existing state information.
*/
class AreaDetector
{
public:

    /**
    * Constructor for AreaDetector.
    *
    * \param    config          ConfigFile containing the parameters to use for initializing the AreaDetector.
    * \param    cmdLine         CommandLine containing any provided command-line arguments
    * \param    mapName         Name of the map in which the robot is operating
    */
    AreaDetector(const utils::ConfigFile& config,
                 const utils::CommandLine& cmdLine,
                 const std::string& mapName);

    /**
    * Destructor for AreaDetector.
    */
    ~AreaDetector(void);

    /**
    * detectAreas parses an LPM into a set of distinct areas. The detected areas are integrated with the previous model
    * of the local topology to form a better model with the updated information. If previously detected areas no
    * longer have any boundaries within the LPM, then they are erased from the map.
    *
    * \param    pose        Pose of the robot within the LPM
    * \param    map         LPM of local surround
    * \return   LocalTopoMap representation of the areas in the LPM.
    */
    AreaDetectorResult detectAreas(const LocalPose& pose, const LocalPerceptualMap& map);

    /**
    * processAreaEvents provides feedback to the area detector on the set of local area events that occurred within the
    * LocalTopoMap created by the latest call to detect areas.
    *
    * \param    events          LocalAreaEvents that occurred within the current LocalTopoMap
    */
    void processAreaEvents(const LocalAreaEventVec& events);

    /**
    * resetAreas resets the state of areas being tracked internally. The reset should happen if the map or environment
    * changes in some way such that any previously existing areas are invalidated.
    */
    void resetAreas(void);

    /**
    * sendDebug has the detector send any internally generated debugging information.
    *
    * \param    communicator        Communicator for sending out the data
    */
    void sendDebug(system::DebugCommunicator& communicator);

private:

    std::unique_ptr<VoronoiSkeletonBuilder> skeletonBuilder_;
    std::unique_ptr<GatewayLocator>         gatewayLocator_;
    std::unique_ptr<AreaClassifier>         areaClassifier_;

    int32_t nextMapId_;

    bool shouldBuildSkeleton_;
    bool shouldComputeIsovists_;
    bool shouldFindGateways_;

    float maxIsovistRange_;
    int numIsovistRays_;

    int gwyFailCount_ = 0;
    std::vector<Point<float>> pointsOfInterest_;

    int64_t voronoiTotalTime_ = 0;
    int64_t gatewayTotalTime_ = 0;
    int64_t parsingTotalTime_ = 0;
    int64_t numUpdates_ = 0;

    // TODO: Create DurationStats class that keeps track of stats about elapsed time for compuation
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTOR_H
