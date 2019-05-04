/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_labels.h
* \author   Collin Johnson
*
* Declaration of functions for saving and loading ground-truth labeled areas and features:
*
*   - save_map_labels
*   - load_map_labels
*
* Saving and loading the labeled areas should use the following steps:
*
* Saving:
*
*   When saving a map (when the Store Labels or Save Labels button is pressed),
*
*       1) Call save_map_labels
*       2) Use the returned LabeledAreaData for classification
*
* Loading:
*
*   When loading a map,
*
*       1) Call load_map_labels
*       2) Use the returned LabeledAreaData for classification
*
*
* Each map has its data stored in multiple files all associated with the map's filename. The naming scheme for maps
* must follow this format:
*
*   'building'('floor')(_'incremental number')
*
*   - The building name must not contain numbers.
*   - The floor must be a number.
*   - The incremental number must be separated by an underscore and be a number.
*
* For example, the third floor of the EECS building is eecs3. The thirtieth map in an incremental sequence of
* maps of the third floor of the EECS building is eecs3_30.
*
* The load_labels_file takes a file with a .lbl extension, which has the following format:
*
*       map_name1 map_directory1
*       map_name2 map_directory2
*               .
*               .
*               .
*       map_nameN map_directoryN
*
* The map_name is assumed to end in .lpm and thus the file extension should be left off. For simplicity, don't put any
* underscores in the 'building' name.
*
* The map_directory should be the absolute directory of the map file.
*
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_TRAINING_AREA_LABELS_H
#define HSSH_LOCAL_TOPOLOGICAL_TRAINING_AREA_LABELS_H

#include <hssh/local_topological/training/labeled_area_data.h>
#include <hssh/local_topological/training/labeled_boundary_data.h>
#include <hssh/local_topological/training/labeled_gateway_data.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/area_detection/gateways/feature_extraction.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_edges.h>
#include <string>
#include <vector>
#include <utility>

namespace vulcan
{
namespace hssh
{

class LocalPerceptualMap;
class LocalTopoAreaEditor;
class LocalTopoMap;
struct local_topology_params_t;

/**
* MapName maintains name information for a map. It consists of:
*
*   - building
*   - floor
*   - incremental number (if part of an incremental series of maps)
*/
struct MapName
{
    std::string building;                   ///< Name of the building from which the data was gathered
    std::string floor;                      ///< Floor of the building
    std::string incrementalNum;             ///< Index of the incremental map (if it was an incremental map)

    /**
    * toMapName converts the MapLabels name information back into a single map name suitable for saving to file.
    */
    std::string toMapName(void) const;
};


/**
* MapLabels stores information about the labels in a map along with information about the map's location in the
* real world and on disk.
*/
struct MapLabels
{
    MapName name;                           ///< Name of the map associated with these labels
    std::string directory;                  ///< Directory in which the map labels exist

    LabeledAreaData initialData;            ///< Data from the initial set of hypotheses
    LabeledAreaData simplifiedData;         ///< Data from the simplified and complete hypotheses
    LabeledGatewayData gatewayData;         ///< Data from the hand-labeled gateways
    LabeledBoundaryData boundaryData;       ///< Data from the boundary types in the ground-truth maps
    std::shared_ptr<LocalPerceptualMap> lpm;                ///< LPM for the map
    std::shared_ptr<VoronoiSkeletonGrid> skeleton;          ///< Skeleton of the map
    std::shared_ptr<VoronoiIsovistField> isovists;          ///< Isovists of the map
    std::shared_ptr<VoronoiEdges> edges;                    ///< Edges in the Voronoi skeleton

    std::shared_ptr<LocalTopoAreaEditor> editor;        ///< Editor associated with the state for these map labels
};

/**
* LabelsFile contains the organized data loaded from a .lbl file, whose format is described above. The contents of
* the labels file are split into full map data and incremental map data. All map data is stored per building to make
* it easy to only grab labels from different buildings, which will therefore have different structures to avoid
* polluting training data.
*/
struct LabelsFile
{
    using LabelsVec = std::vector<MapLabels>;
    using BuildingLabels = std::map<std::string, LabelsVec>;    // maps building name -> LabelsVec

    BuildingLabels fullMaps;
    BuildingLabels incrementalMaps;
};


/**
* save_map_labels saves labels in the LPM using the associated labels existing in the editor.
*
* Saving a map creates six files:
*
*   - dir/mapName.feat          -- LabeledAreaData for the map
*   - dir/mapName_initial.feat  -- LabeledAreaData for the unsimplified hypotheses
*   - dir/mapName.cell          -- Mapping of cell->HypothesisType
*   - dir/mapName.gwy           -- Location of gateways in the map
*   - dir/mapName_gwy.feat      -- Gateway features -- both positive and negative
*   - dir/mapName_truth.ltm     -- Ground-truth LocalTopoMap for the simplified map
*
* The LabeledAreaData stored in the .feat file is returned. The .cell file is used to associated the ground-truth labels
* with the underlying map, whose representation isn't going to change. The local topo representation isn't completely
* solidified yet, and thus isn't as suitable for storing long-term data.
*
* \param    lpm             Map for which the labels were created
* \param    editor          Editor containing the labeled hypotheses
* \param    gateways        Hand-labeled gateways for the map
* \param    mapName         Name of the labeled map
* \param    mapDirectory    Directory in which the map is saved
* \return   Map labels created for the current LPM.
*/
MapLabels save_map_labels(const LocalPerceptualMap& lpm,
                          const LocalTopoAreaEditor& editor,
                          const std::vector<Gateway>& gateways,
                          const std::string& mapName,
                          const std::string& mapDirectory);

/**
* load_map_labels loads labels associated with a map and creates the appropriate LabeledAreaData.
*
* The following steps are performed when loading a map:
*
*   1) If a .feat file exists, then load it. If the saved features are the same version as the current feature version,
*      then skip the remaining steps and just return those features
*
*   2) If a .cell file exists, then load it. Use the editor to create AreaHypotheses for the LPM. These hypotheses will
*      have the appropriate type assigned using a simple voting system. The most common type for the cells in the area
*      will be the type assigned to the hypothesis. After assigning types, the hypotheses are simplified. Finally, the
*      labeled features will be created and returned.
*
*   3) If a _gwy.feat file exists, ground-truth gateway information will be created.
*
* \param    mapName         Name of the map to load
* \param    mapDirectory    Directory where the labels exist
* \param    editor          Editor to use for creating the area hypotheses
* \return   Labels created for the map.
*/
MapLabels load_map_labels(const std::string& mapName,
                          const std::string& mapDirectory,
                          LocalTopoAreaEditor& editor);

/**
* compute_gateway_features loads ground-truth gateways associated with a particular map. It creates the appropriate
* LabeledGatewayData that is used for training a gateway classifier.
*
* \param    map             Map for which to load gateway information
* \return   True if gateways were found. False if no .gwy file is associated with the provided map.
*/
bool compute_gateway_features(MapLabels& map);

/**
* compute_area_and_boundary_features loads ground-truth cell information for the map. It then creates gateways using
* the area editor to create the initial set of areas. Following that, it creates the simplified areas. The features for
* both of these can then be used to learn a classifier for the area hypotheses.
*
* Once the simplified areas are created, boundary data is computed for these areas.
*
* \param    map             Map for which to compute area features
* \return   True if a .cell file was found so all desired information could be computed.
*/
bool compute_area_and_boundary_features(MapLabels& map);

/**
* load_labels_file loads all MapLabels from a .lbl file whose format is described above and returns the data as
* a LabelsFile.
*
* \param    labelsFile          Filename for the labels file
* \param    localTopoParams     Parameters to use for creating the necessary data structures for loading map labels
* \return   All MapLabels loaded from the labels file.
*/
LabelsFile load_labels_file(const std::string& labelsFile, const local_topology_params_t& localTopoParams);

/**
* load_labels_directory loads all MapLabels associated with LPMs contained in the specified directory.
*
* \param    directory           Directory with the map labels
* \param    localTopoParams     Parameters to use for creating the necessary data structures for loading map labels
* \return   All MapLabels loaded from the labels file.
*/
LabelsFile load_labels_directory(const std::string& directory, const local_topology_params_t& localTopoParams);

/**
* create_labeled_gateway_data creates a gateway data from the skeleton features in a map and the corresponding
* gateways.
*
* \param    mapName         Name of the map in which the gateways were found
* \param    gateways        Gateways to create features for
* \param    editor          Editor for computing the appropriate features
* \return   LabeledGatewayData corresponding to the features and gateways in the provided map.
*/
LabeledGatewayData create_labeled_gateway_data(const std::string& mapName,
                                               const std::vector<Gateway>& gateways,
                                               const LocalTopoAreaEditor& editor);

/**
* create_labeled_boundary_data creates boundary data from a LocalTopoMap. It notes each type of boundary relation
* that occurs in the provided map.
*
* \param    mapName     Name of the map to associate with the data
* \param    editor      Editor containing the hypotheses
* \return   Boundary data associated with the LocalTopoMap.
*/
LabeledBoundaryData create_labeled_boundary_data(const std::string& mapName, const LocalTopoAreaEditor& editor);

/**
* reconstruct_ltm_from_labels takes the map information from the map labels, loads the gateways, reassigns the types,
* then saves the final result as the ground-truth map. No map simplification occurs so the output is exactly the input
* here.
*
* This function is useful whenever the LTM format changes or an error in the LTM information is found.
*
* \param    map         Map to reconstruct
* \return   True if the map is successfully re-built.
*/
bool reconstruct_ltm_from_labels(MapLabels& map);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_TRAINING_AREA_LABELS_H
