/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_area_editor.h
* \author   Collin Johnson
*
* Definition of LocalTopoAreaEditor.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_AREA_EDITOR_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_AREA_EDITOR_H

#include <hssh/local_topological/area_detection/labeling/hypothesis.h>
#include <hssh/local_topological/area_detection/gateways/feature_extraction.h>
#include <hssh/local_topological/gateway.h>
#include <hssh/local_topological/local_topo_map.h>
#include <boost/optional.hpp>
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class  AreaGraph;
class  HypothesisGraph;
class  VoronoiSkeletonBuilder;
class  VoronoiSkeletonGrid;
class  GatewayGenerator;
class  GatewayLocator;
class  LocalPerceptualMap;
struct local_topology_params_t;

/**
* LocalTopoAreaEditor is an editor that allows hand-labeling and merging of area hypotheses in an LPM.
* The editor is able to build the local topological representation up to the point where AreaHypotheses are created.
*
* The basic flow for using LocalTopoAreaEditor is:
*
*   - buildSkeleton : create the VoronoiSkeletonGrid underlying all local topo representations
*   - findGateways : find gateways in the current grid
*   - constructHypotheses : create the area hypotheses based on a set of gateways
*
* At this point, the AreaHypotheses found can be iterated over. Two operations can be performed on the hypotheses:
*
*   - label : assign a label to an area
*   - merge : merge a set of hypotheses into a single hypothesis
*
* An integer is used as the handle for changing the area hypotheses. Whenever a new LPM is processed, the handles,
* along with the hypothesis iterators are invalidated.
*/
class LocalTopoAreaEditor
{
public:

    using HypothesisConstIter = std::vector<AreaHypothesis*>::const_iterator;

    /**
    * Constructor for LocalTopoAreaEditor.
    *
    * \param    params          Parameters to use for building the representation of the local topology
    */
    LocalTopoAreaEditor(const local_topology_params_t& params);

    /**
    * Destructor for LocalTopoAreaEditor.
    */
    ~LocalTopoAreaEditor(void);

    /**
    * setGatewayGenerator sets the gateway classifier to use with the GatewayLocator.
    */
    void setGatewayGenerator(std::unique_ptr<GatewayGenerator> generator);

    /**
    * buildSkeleton constructs the VoronoiSkeletonGrid from the provided LPM. The constructed skeleton will subsequently
    * be used for any editing operations.
    *
    * This method invalidates all iterators that exist
    *
    * \param    lpm         LPM for which local topo information is being added
    * \return   A copy of the skeleton that was built.
    */
    VoronoiSkeletonGrid buildSkeleton(const LocalPerceptualMap& lpm);

    /**
    * findGateways creates gateways within the current skeleton.
    *
    * \return   Gateways created by the internal gateway generator.
    */
    std::vector<Gateway> findGateways(void) const;

    /**
    * findMoreGateways adds gateways within the current skeleton on top of ground-truth gateways that already exist.
    *
    * \param    initialGateways     The initial gateways to start from for generation
    * \return   Gateways created by the internal gateway generator.
    */
    std::vector<Gateway> findMoreGateways(const std::vector<Gateway>& initialGateways) const;

    /**
    * computeGatewayFeatures computes the gateway-related features for the entire map.
    */
    SkeletonFeatures computeGatewayFeatures(void) const;

    /**
    * isovistField retrieves the isovist field asociated with the current map.
    */
    const VoronoiIsovistField& isovistField(void) const;

    /**
    * constructHypotheses constructs AreaHypotheses to be edited.
    *
    * Note that using gateways that existing from a different LPM will have extremely undesirable effects, so don't
    * do it.
    *
    * This method invalidates all iterators that exist.
    *
    * \param    gateways            Gateways associated with the current skeleton to use for creating new areas
    */
    void constructHypotheses(const std::vector<Gateway>& gateways);

    /**
    * labelArea assigns a new label to an area.
    *
    * \param    area        Area to be labeled
    * \param    label       Label to be assigned
    * \return   True if an area with the given index exists and the label was set successfully.
    */
    bool labelArea(AreaHypothesis* area, HypothesisType label);

    /**
    * labelRemainingAreas assigns the provided label to all remaining areas that have not been labeled yet, i.e.
    * those whose type is HypothesisType::kArea.
    *
    * \param    label           Label to apply to all generic areas
    * \return   The number of areas labeled.
    */
    std::size_t labelRemainingAreas(HypothesisType label);

    /**
    * simplifyAreas runs the HypothesisGraph simplification algorithm. This algorithm is only useful to run
    * after the areas have been labeled. Once labeled, the simplification process will dramatically reduce the number of
    * hypotheses that might need further attention.
    */
    void simplifyAreas(void);

    /**
    * mergeAreas merges a collection of areas into a single area. Note that merged areas can not be merged further. Therefore,
    * take care that the merge contains all the desired areas from the get-go. Furthermore, merged areas can not be further
    * simplified. Consequently, merging should only take place after labeling and simplification of the areas.
    *
    * \param    hypothesesToMerge           Hypotheses to be merged into a single area
    * \return   True if all indices were valid and a single hypothesis has been created.
    */
    bool mergeAreas(const std::vector<AreaHypothesis*>& hypothesesToMerge);

    /**
    * resetAreas resets all merges and labeling operations that have been performed, returning the editor to its original state.
    */
    void resetAreas(void);

    /**
    * saveToLocalTopoMap saves the current set of hypotheses to a LocalTopoMap. The hypotheses need to be valid or the
    * program will currently crash.
    *
    * \param    filename            Filename to save the map to
    */
    void saveToLocalTopoMap(const std::string& filename) const;

    /**
    * constructLocalTopoMap creates a LocalTopoMap from the hand-labeled areas.
    */
    boost::optional<LocalTopoMap> constructLocalTopoMap(void) const;

    // Iterators
    HypothesisConstIter begin(void) const { return hypotheses_.begin(); }
    HypothesisConstIter end(void)   const { return hypotheses_.end(); }
    std::size_t         numHypotheses(void) const { return hypotheses_.size(); }
    const AreaHypothesis& operator[](std::size_t n) { return *hypotheses_[n]; }

    HypothesisConstIter beginInitialHypotheses(void) const { return initialHypotheses_.begin(); }
    HypothesisConstIter endInitialHypotheses(void)   const { return initialHypotheses_.end(); }

private:

    std::vector<AreaHypothesis*> hypotheses_;
    std::vector<AreaHypothesis*> initialHypotheses_;
    std::vector<std::unique_ptr<AreaHypothesis>> mergedHypotheses_;       // the hypotheses that have been merged and are owned by the editor, not the HypGraph

    std::unique_ptr<AreaGraph>       areaGraph_;
    std::unique_ptr<HypothesisGraph> hypothesisGraph_;
    std::unique_ptr<HypothesisGraph> initialHypothesisGraph_;

    std::unique_ptr<VoronoiSkeletonBuilder> voronoiBuilder_;
    std::unique_ptr<GatewayLocator>         gatewayLocator_;
    mutable std::unique_ptr<VoronoiIsovistField> isovistField_;
    std::shared_ptr<SmallScaleStarBuilder>  starBuilder_;
    std::unique_ptr<hssh::LocalPerceptualMap> lpm_;

    float maxIsovistRange_;
    int numIsovistRays_;
    int gatewayFeatureRadius_;

    // INVARIANT: hypotheses_.size() <= mergeHypotheses_.capacity() --- allows us to know that mergedHypotheses_ will never be reallocated
    //            so pointers to mergedHypotheses_ elements are safe to store in hypotheses_

    void createIsovistsIfNeeded(void) const;
    void resetStoredHypotheses(void);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_AREA_EDITOR_H
