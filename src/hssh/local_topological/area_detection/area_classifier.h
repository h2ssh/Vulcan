/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_classifier.h
 * \author   Collin Johnson
 *
 * Declaration of AreaClassifier interface and create_area_classifier factory function.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_AREA_CLASSIFIER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_AREA_CLASSIFIER_H

#include "hssh/local_topological/error.h"
#include "hssh/local_topological/event.h"
#include <memory>
#include <string>
#include <vector>

namespace vulcan
{
namespace system
{
class DebugCommunicator;
}
namespace hssh
{

class AreaClassifier;
class Gateway;
class LocalArea;
class LocalPerceptualMap;
class VoronoiIsovistField;
class VoronoiSkeletonGrid;
struct area_classifier_params_t;


/**
 * create_area_classifier is a factory function for creating an AreaClassifier instance based on the provided type
 * identifier and parameters.
 *
 * \pre type is a valid AreaClassifier
 * \param    type            Type of classifier to make
 * \param    mapName         Name of the map in which the robot is operating (basename for all classifier files)
 * \param    params          Parameters for the classifier
 * \return   An instance of the specified subclass of AreaClassifier.
 */
std::unique_ptr<AreaClassifier>
  create_area_classifier(const std::string& type, const std::string& mapName, const area_classifier_params_t& params);


/**
 * AreaClassifier is an interface defining the classification Strategy being used by an AreaDetector. The classifier
 * takes the gateways, skeleton, and LPM and then segments and labels them as LocalAreas.
 */
class AreaClassifier
{
public:
    using Areas = std::pair<std::vector<std::shared_ptr<LocalArea>>, double>;   // areas, log-probability

    /**
     * classifyAreas identifies and classifies the LocalAreas in the current map.
     *
     * \param    gateways            Gateways found in the map
     * \param    skeleton            Voronoi skeleton of the map
     * \param    isovistField        Isovist field for the current LPM
     * \param    lpm                 Current LPM -- needed for the metric maps associated with places
     * \return   LabelingError describing the result of the classification process
     */
    virtual LabelingError classifyAreas(const std::vector<Gateway>& gateways,
                                        const VoronoiSkeletonGrid& skeleton,
                                        const VoronoiIsovistField& isovistField,
                                        const LocalPerceptualMap& lpm) = 0;

    /**
     * processAreaEvents provides feedback on any LocalTopoEvents that occurred within the most recently created
     * LocalTopoMap resultings from the areas created on the most recent call to classifyAreas.
     *
     * \param    events          Events that occurred in the most recently detected areas
     */
    virtual void processAreaEvents(const LocalAreaEventVec& events) = 0;

    /**
     * currentAreas retrieves the confirmed areas in the robot's current surround.
     */
    virtual Areas currentAreas(void) const = 0;

    /**
     * sendDebug uses the provided communicator to send out any relevant debug information.
     */
    virtual void sendDebug(system::DebugCommunicator& communicator) const = 0;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_AREA_CLASSIFIER_H
