/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_labels.cpp
* \author   Collin Johnson
*
* Definition of save_map_labels and load_map_labels.
*/

#include <hssh/local_topological/training/area_labels.h>
#include <hssh/local_topological/training/local_topo_area_editor.h>
#include <hssh/local_topological/training/labeled_area_data.h>
#include <hssh/local_topological/area.h>
#include <hssh/local_topological/area_extent.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/labeling/boundary.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/lpm_io.h>
#include <hssh/types.h>
#include <utils/algorithm_ext.h>
#include <utils/serialized_file_io.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <fstream>
#include <sstream>
#include <cstring>

namespace vulcan
{
namespace hssh
{

using CellTypes = CellToTypeMap<HypothesisType>;
using HypIter = LocalTopoAreaEditor::HypothesisConstIter;


const std::string kFeatureExtension(".feat");
const std::string kInitialExtension("_initial");
const std::string kCellExtension(".cell");
const std::string kGatewayExtension(".gwy");
const std::string kGatewayFeatExtension("_gwy");
const std::string kBoundaryFeatExtension("_bnd");
const std::string kGroundTruthExtension("_truth.ltm");
const std::string kGeneratedExtension("_gen.ltm");


std::string full_filename(const std::string& mapName, const std::string& mapDirectory, const std::string& extension);
MapName parse_map_name(const std::string& mapName);
LabeledAreaData create_labeled_data(const std::string& mapName, HypIter begin, HypIter end);
CellTypes create_cell_types(const LocalPerceptualMap& lpm, HypIter begin, HypIter end);
void save_cell_types(std::ofstream& out, const CellTypes& cells);
CellTypes load_cell_types(std::ifstream& in);
HypothesisType assign_hypothesis_label(const AreaHypothesis& hyp,
                                       const LocalPerceptualMap& map,
                                       const CellTypes& cellToType);
HypothesisType hyp_type_from_area_type(const LocalArea& area, const LocalArea& adjArea);

////////////////// MapName implementation ///////////////////////////////////

std::string MapName::toMapName(void) const
{
    std::string mapName = building + floor;

    if(!incrementalNum.empty())
    {
        mapName += "_";
        mapName += incrementalNum;
    }

    return mapName;
}

////////////////// I/O functions implementation ///////////////////////////////

MapLabels save_map_labels(const LocalPerceptualMap& lpm,
                          const LocalTopoAreaEditor& editor,
                          const std::vector<Gateway>& gateways,
                          const std::string& mapName,
                          const std::string& mapDirectory)
{
    MapLabels labels;
    labels.initialData = create_labeled_data(mapName, editor.beginInitialHypotheses(), editor.endInitialHypotheses());
    labels.simplifiedData = create_labeled_data(mapName, editor.begin(), editor.end());

    {
        std::ofstream initialOut(full_filename(mapName, mapDirectory, kInitialExtension + kFeatureExtension));
        assert(initialOut.is_open());
        initialOut << labels.initialData;
    }

    {
        std::ofstream labelOut(full_filename(mapName, mapDirectory, kFeatureExtension));
        assert(labelOut.is_open());
        labelOut << labels.simplifiedData;
    }

    {
        auto cellTypes = create_cell_types(lpm, editor.begin(), editor.end());
        std::ofstream cellOut(full_filename(mapName, mapDirectory, kCellExtension));
        assert(cellOut.is_open());
        save_cell_types(cellOut, cellTypes);
    }

    if(!gateways.empty())
    {
        utils::save_serializable_to_file(full_filename(mapName, mapDirectory, kGatewayExtension), gateways);
        auto features = create_labeled_gateway_data(mapName, gateways, editor);
        std::ofstream out(full_filename(mapName, mapDirectory, kGatewayFeatExtension + kFeatureExtension));
        assert(out.is_open());
        out << features;
    }

    // Save the generated ground truth map
    editor.saveToLocalTopoMap(full_filename(mapName, mapDirectory, kGroundTruthExtension));

    return labels;
}


MapLabels load_map_labels(const std::string& mapName,
                          const std::string& mapDirectory,
                          LocalTopoAreaEditor& editor)
{
    MapLabels labels;
    labels.name = parse_map_name(mapName);
    labels.directory = mapDirectory;

    // The skeleton and isovists always needs to be stored with the gateway data
    labels.lpm = std::make_shared<LocalPerceptualMap>();
    load_lpm_1_0(full_filename(mapName, mapDirectory, kLPMExtension), *labels.lpm);
    labels.skeleton = std::make_shared<VoronoiSkeletonGrid>(editor.buildSkeleton(*labels.lpm));
    labels.isovists = std::make_shared<VoronoiIsovistField>(editor.isovistField());
    labels.edges = std::make_shared<VoronoiEdges>(*labels.skeleton, SKELETON_CELL_REDUCED_SKELETON);

    std::ifstream initialIn(full_filename(mapName, mapDirectory, kInitialExtension + kFeatureExtension));
    std::ifstream simplifiedIn(full_filename(mapName, mapDirectory, kFeatureExtension));
    std::ifstream gwyFeatIn(full_filename(mapName, mapDirectory, kGatewayFeatExtension + kFeatureExtension));
    std::ifstream bndFeatIn(full_filename(mapName, mapDirectory, kBoundaryFeatExtension + kFeatureExtension));

    // If the feature files exist:
    if(simplifiedIn.is_open() && initialIn.is_open() && gwyFeatIn.is_open() && bndFeatIn.is_open())
    {
        std::cout << "INFO: load_map_labels: Found stored features for " << mapName << '\n';

        // Attempt to load the data
        initialIn >> labels.initialData;
        simplifiedIn >> labels.simplifiedData;
        gwyFeatIn >> labels.gatewayData;
        bndFeatIn >> labels.boundaryData;

        bool haveInitial = !labels.initialData.empty()
            && (labels.initialData.version() == current_hypothesis_features_version());
        bool haveSimple = !labels.simplifiedData.empty()
            && (labels.simplifiedData.version() == current_hypothesis_features_version());
        bool haveGateway = !labels.gatewayData.empty()
            && (labels.gatewayData.version() == current_gateway_features_version());
        bool haveBoundary = !labels.boundaryData.empty();

        // If the features saved are still valid, then data will exist and is thus valid
        if(haveInitial && haveSimple && haveGateway && haveBoundary)
        {
            std::cout << "Stored features were still valid for initial, simplified, gateways, and boundaries.\n";
            return labels;
        }
        else
        {
            std::cout << "Didn't find up-to-date feature data for all learned classifiers.\n";
        }
    }
    else
    {
        std::cout << "INFO: load_map_labels: Failed to find all needed saved features for " << mapName << '\n';
    }

    return labels;
}


bool compute_gateway_features(MapLabels& map)
{
    auto mapName = map.name.toMapName();

    // Need to first get the gateways for the map.
    std::vector<Gateway> gateways;
    // Always attempt to load gateways first. Don't want to compute them unless absolutely necessary.
    utils::load_serializable_from_file(full_filename(mapName, map.directory, kGatewayExtension), gateways);

    // If there weren't gateways, then they'll need to be computed
    if(gateways.empty())
    {
        std::cerr << "WARNING: compute_gateway_features: Didn't find saved gateways for " << mapName << '\n';
        return false;
    }

    map.gatewayData = create_labeled_gateway_data(mapName, gateways, *map.editor);

    std::ofstream gatewayOut(full_filename(mapName, map.directory, kGatewayFeatExtension + kFeatureExtension));
    assert(gatewayOut.is_open());
    gatewayOut << map.gatewayData;

    return true;
}


bool compute_area_and_boundary_features(MapLabels& map)
{
    auto mapName = map.name.toMapName();

    std::ifstream cellIn(full_filename(mapName, map.directory, kCellExtension));

    if(!cellIn.is_open())
    {
        std::cerr << "WARNING: compute_area_and_boundary_features: Didn't find saved cell information for "
            << mapName << " Checked in " << full_filename(mapName, map.directory, kCellExtension) << std::endl;
        assert(false);
        return false;
    }

    // If there are cells and some bit of data is missing, then need to process everything
    // Need to first get the gateways for the map.
    std::vector<Gateway> gateways;
    // Always attempt to load gateways first. Don't want to compute them unless absolutely necessary.
    utils::load_serializable_from_file(full_filename(mapName, map.directory, kGatewayExtension), gateways);

    // Always start with the initial set of ground-truth gateways to ensure false negatives in the gateway classifier
    // don't skew the classification results
    gateways = map.editor->findMoreGateways(gateways);
    map.editor->constructHypotheses(gateways);

    std::cout << "INFO: load_map_labels: Found saved cells labels for " << mapName << '\n';
    CellTypes cellToType = load_cell_types(cellIn);

    for(auto& hyp : boost::make_iterator_range(map.editor->beginInitialHypotheses(), map.editor->endInitialHypotheses()))
    {
        auto type = assign_hypothesis_label(*hyp, *map.lpm, cellToType);
        hyp->setType(type);
    }

    for(auto& hyp : *map.editor)
    {
        auto type = assign_hypothesis_label(*hyp, *map.lpm, cellToType);
        hyp->setType(type);
    }

    map.editor->simplifyAreas();

    int numInitial = std::distance(map.editor->beginInitialHypotheses(), map.editor->endInitialHypotheses());
    int numSimplified = std::distance(map.editor->begin(), map.editor->end());
    assert(numInitial >= numSimplified);

    map.initialData = create_labeled_data(mapName,
                                          map.editor->beginInitialHypotheses(),
                                          map.editor->endInitialHypotheses());
    map.simplifiedData = create_labeled_data(mapName, map.editor->begin(), map.editor->end());
    map.boundaryData = create_labeled_boundary_data(mapName, *map.editor);

    assert(map.initialData.size() >= map.simplifiedData.size());

    // Create the map labels and save over the previous labels so this calculation doesn't need to happen again
    std::ofstream initialOut(full_filename(mapName, map.directory, kInitialExtension + kFeatureExtension));
    assert(initialOut.is_open());
    initialOut << map.initialData;

    std::ofstream simplifiedOut(full_filename(mapName, map.directory, kFeatureExtension));
    assert(simplifiedOut.is_open());
    simplifiedOut << map.simplifiedData;

    std::ofstream boundaryOut(full_filename(mapName, map.directory, kBoundaryFeatExtension + kFeatureExtension));
    assert(boundaryOut.is_open());
    boundaryOut << map.boundaryData;

    // Save the generated ground truth map
    map.editor->saveToLocalTopoMap(full_filename(mapName, map.directory, kGeneratedExtension));

    return true;
}


LabelsFile load_labels_file(const std::string& labelsFile, const local_topology_params_t& localTopoParams)
{
    LabelsFile labels;

    std::string name;
    std::string directory;
    std::ifstream in(labelsFile);

    while(!in.eof())
    {
        in >> name >> directory;

        if(in.good())
        {
            std::cout << "INFO: LocalTopoEditor: Loading data for " << name << " in " << directory << '\n';
            auto editor = std::make_shared<LocalTopoAreaEditor>(localTopoParams);
            auto mapLabels = load_map_labels(name, directory, *editor);
            mapLabels.editor = editor;

            // If there's no incremental number, then it is a full map
            if(mapLabels.name.incrementalNum.empty())
            {
                labels.fullMaps[mapLabels.name.building].push_back(mapLabels);
            }
            // Otherwise add it to the incremental maps
            else
            {
                labels.incrementalMaps[mapLabels.name.building].push_back(mapLabels);
            }
        }
    }

    return labels;
}


LabelsFile load_labels_directory(const std::string& directory, const local_topology_params_t& localTopoParams)
{
    namespace bfs = boost::filesystem;

    LabelsFile labels;

    if(!bfs::is_directory(directory))
    {
        std::cerr << "ERROR: load_labels_directory: " << directory << " is not a directory.\n";
        return labels;
    }

    for(bfs::directory_entry& file : bfs::directory_iterator(directory))
    {
        auto path = file.path();

        if(path.extension() == ".lpm")
        {
            std::cout << "INFO: load_labels_directory: Loading data for " << path.filename() << " in "
                << path.parent_path() << '\n';
            auto editor = std::make_shared<LocalTopoAreaEditor>(localTopoParams);
            auto mapLabels = load_map_labels(path.stem().string(), directory, *editor);
            mapLabels.editor = editor;

            // If there's no incremental number, then it is a full map
            if(mapLabels.name.incrementalNum.empty())
            {
                labels.fullMaps[mapLabels.name.building].push_back(mapLabels);
            }
            // Otherwise add it to the incremental maps
            else
            {
                labels.incrementalMaps[mapLabels.name.building].push_back(mapLabels);
            }
        }
    }

    return labels;
}


std::string full_filename(const std::string& mapName, const std::string& mapDirectory, const std::string& extension)
{
    std::ostringstream out;
    out << mapDirectory << '/' << mapName << extension;
    return out.str();
}


MapName parse_map_name(const std::string& mapName)
{
    assert(mapName.length() > 0);

    // Convert the map name into the building, floor, and optional incremental index.
    // The format is: 'building''floor'(_'incremental number')

    // The building contains everything up to the first numeric character
    std::size_t start = 0;
    std::size_t end = mapName.length();
    for(std::size_t n = start; n < mapName.length(); ++n)
    {
        if(isdigit(mapName[n]))
        {
            end = n;
            break;
        }
    }

    MapName name;

    if(end == 1 && mapName[0] == '_')
    {
        std::cerr << "ERROR: load_map_labels: Invalid map name. Can't end with underscore.\n";
        return name;
    }

    // If the last character before the digit was an underscore, then there's no floor number
    if(end == 0)
    {
        name.building = mapName;
    }
    else if(mapName[end - 1] == '_')
    {
        --end;
        name.building = mapName.substr(start, end - start);
        start = end;
    }
    else
    {
        name.building = mapName.substr(start, end - start);
        start = end;
        end = mapName.length();
        // The floor contains everything after the building until an underscore is encountered
        for(std::size_t n = start; n < mapName.length(); ++n)
        {
            if(mapName[n] == '_')
            {
                end = n;
                break;
            }
        }
    }

    // If the end has changed, then there was a floor number
    if(start != end)
    {
        name.floor = mapName.substr(start, end - start);
    }

    // If start isn't the end of the map name and the search stopped at an underscore, then we found an incremental
    // map number
    start = end;
    if(start != mapName.length())
    {
        name.incrementalNum = mapName.substr(start + 1); // skip the _
    }

    std::cout << "Building:" << name.building << " Floor:" << name.floor << " Incr:" << name.incrementalNum << '\n';
    return name;
}


LabeledAreaData create_labeled_data(const std::string& mapName, HypIter begin, HypIter end)
{
    LabeledAreaData data;

    for(auto& hyp : boost::make_iterator_range(begin, end))
    {
        data.addExample(mapName, { hyp->getType(), hyp->features() });
    }

    return data;
}


CellTypes create_cell_types(const LocalPerceptualMap& lpm, HypIter begin, HypIter end)
{
    CellTypes cells;

    for(auto& hyp : boost::make_iterator_range(begin, end))
    {
        // Iterate through every cell in the hypothesis extent
        for(auto& cell : hyp->extent())
        {
            // Round it to the nearest grid coordinate to avoid subtle truncation errors
            auto gridCoords = utils::global_point_to_grid_cell_round(cell, lpm);

            // If the cell is already stored, that means it is on the boundary of two areas. These cells should be
            // removed so they don't unduly influence the area on either side
            auto cellIt = cells.find(gridCoords);
            if(cellIt != cells.end())
            {
                cells.erase(cellIt);
            }

            // Add the cell->type association
            cells.insert(std::make_pair(gridCoords, hyp->getType()));
        }
    }

    return cells;
}


void save_cell_types(std::ofstream& out, const CellTypes& cells)
{
    for(auto c : cells)
    {
        out << c.first.x << ' ' << c.first.y << ' ' << c.second << '\n';
    }
}


CellTypes load_cell_types(std::ifstream& in)
{
    CellTypes cells;

    while(in.good())
    {
        cell_t cell;
        HypothesisType type;
        in >> cell.x >> cell.y >> type;

        if(in.good())
        {
            cells.insert(std::make_pair(cell, type));
        }
    }

    return cells;
}


HypothesisType assign_hypothesis_label(const AreaHypothesis& hyp,
                                             const LocalPerceptualMap& map,
                                             const CellTypes& cellToType)
{
    int totalCells = 0;
    int numDecision = 0;
    int numDest = 0;
    int numPath = 0;

    for(auto cell : hyp.extent())
    {
        auto gridCell = utils::global_point_to_grid_cell_round(cell, map);
        auto typeIt = cellToType.find(gridCell);

        if(typeIt != cellToType.end())
        {
            ++totalCells;

            switch(typeIt->second)
            {
                case HypothesisType::kDecision:
                    ++numDecision;
                    break;
                case HypothesisType::kDest:
                    ++numDest;
                    break;
                case HypothesisType::kPath:
                    ++numPath;
                    break;
                default:
                    std::cout << "Invalid label for cell: " << gridCell << '\n';
            }
        }
    }

    if(totalCells == 0)
    {
        std::cerr << "ERROR: No cells found for hypothesis at "
            << hyp.extent().rectangleBoundary(math::ReferenceFrame::GLOBAL) << '\n';
        return HypothesisType::kArea;
    }

    std::cout << "INFO: Distribution for hypothesis at " << hyp.extent().rectangleBoundary(math::ReferenceFrame::GLOBAL)
        << ":\nDecision:   " << (static_cast<float>(numDecision) / totalCells)
        << "\nDestination: " << (static_cast<float>(numDest) / totalCells)
        << "\nPath:        " << (static_cast<float>(numPath) / totalCells) << '\n';

    if(numPath >= numDecision && numPath >= numDest)
    {
        return HypothesisType::kPath;
    }
    else if(numDecision >= numDest)
    {
        return HypothesisType::kDecision;
    }
    else
    {
        return HypothesisType::kDest;
    }
}


LabeledGatewayData create_labeled_gateway_data(const std::string& mapName,
                                               const std::vector<Gateway>& gateways,
                                               const LocalTopoAreaEditor& editor)
{
    // Create a LabeledGatewayFeatures instance for each of the features in skeleton features.
    // If the associated cell is a skeleton cell in gateways, then it should be marked as a gateway

    const int kGatewayNeighborDist = 5;     // how many cells away to consider a potentially similar gateway?
    const double kMaxNormalDiff = 0.02;     // how much difference in angle normals is allowed?

    auto features = editor.computeGatewayFeatures();
    const auto& isovists = editor.isovistField();

    LabeledGatewayData data;
    for(auto& f : features)
    {
        LabeledGatewayFeatures example;
        example.features = f.second;
        example.cell = f.first;
        example.isGateway = utils::contains_if(gateways, [&f](const Gateway& g) {
            return g.skeletonCell() == f.first;
        });

        auto neighborIt = std::find_if(gateways.begin(), gateways.end(), [&](const Gateway& g) {
            return (std::abs(f.first.x - g.skeletonCell().x) < kGatewayNeighborDist)
                && (std::abs(f.first.y - g.skeletonCell().y) < kGatewayNeighborDist);
        });

        // See if the neighbor gateway and this cell have a similar MinNormDiff because that indicates if
        // in the direction of opening or moving down the hall. Since boundaries are flexible, allow some
        // of this slop in the ground-truth to create a more defined difference
        if(neighborIt != gateways.end() && isovists.contains(f.first) && isovists.contains(neighborIt->skeletonCell()))
        {
            example.isGateway = std::abs(isovists.at(f.first).scalar(utils::Isovist::kMinNormalDiff)
                - isovists.at(neighborIt->skeletonCell()).scalar(utils::Isovist::kMinNormalDiff)) < kMaxNormalDiff;
        }

        data.addExample(mapName, example);
    }

    return data;
}


LabeledBoundaryData create_labeled_boundary_data(const std::string& mapName, const LocalTopoAreaEditor& editor)
{
    LabeledBoundaryData data;

    // Store the boundaries that are found. For nodes that are gateways, but aren't boundaries, then those are
    // the is-off gateway examples, which are also counted to get a proper prior distribution.
    std::unordered_set<const AreaHypothesisBoundary*> allBoundaries;

    for(auto& hyp : editor)
    {
        // For each boundary, find the neighbor areas and mark the types in the data
        for(auto& bnd : boost::make_iterator_range(hyp->beginBoundary(), hyp->endBoundary()))
        {
            if(allBoundaries.find(bnd) != allBoundaries.end())
            {
                continue;
            }

            auto otherHyp = bnd->getOtherHypothesis(*hyp);

            LabeledBoundaryFeatures feat;
            feat.types[0] = bnd->getBoundaryType(hyp);
            feat.types[1] = bnd->getBoundaryType(otherHyp);
            feat.isOn = true;
            data.addExample(mapName, feat);

            allBoundaries.insert(bnd);
        }
    }

    // Now go through the initial hypotheses. All boundaries that aren't still on had a merge happen and
    // thus were false positives that were turned off

    for(auto& hyp : boost::make_iterator_range(editor.beginInitialHypotheses(), editor.endInitialHypotheses()))
    {
        // For each boundary, find the neighbor areas and mark the types in the data
        for(auto& bnd : boost::make_iterator_range(hyp->beginBoundary(), hyp->endBoundary()))
        {
            // If the boundary has been processed, then ignore it
            if(allBoundaries.find(bnd) != allBoundaries.end())
            {
                continue;
            }

            auto otherHyp = bnd->getOtherHypothesis(*hyp);

            // We're only trying to figure out what the internal off-gateway distributions are
            // if something is strange in the initial hypotheses and the boundary is in a strange
            // state, just ignore it.
            if(hyp->getType() == otherHyp->getType())
            {
                LabeledBoundaryFeatures feat;
                feat.types[0] = bnd->getBoundaryType(hyp);
                feat.types[1] = bnd->getBoundaryType(otherHyp);
                feat.isOn = false;
                data.addExample(mapName, feat);
            }

            allBoundaries.insert(bnd);
        }
    }

    return data;
}


bool reconstruct_ltm_from_labels(MapLabels& map)
{
    auto mapName = map.name.toMapName();
    std::ifstream cellIn(full_filename(mapName, map.directory, kCellExtension));

    if(!cellIn.is_open())
    {
        std::cerr << "ERROR: reconstruct_ltm_from_labels: Didn't find saved cell information for "
            << mapName << " Checked in " << full_filename(mapName, map.directory, kCellExtension) << std::endl;
        return false;
    }

    // If there are cells and some bit of data is missing, then need to process everything
    // Need to first get the gateways for the map.
    std::vector<Gateway> gateways;
    // Always attempt to load gateways first. Don't want to compute them unless absolutely necessary.
    if(!utils::load_serializable_from_file(full_filename(mapName, map.directory, kGatewayExtension), gateways))
    {
        std::cerr << "ERROR: reconstruct_ltm_from_labels: Failed to find gateways for " << mapName
            << " at " << full_filename(mapName, map.directory, kGatewayExtension) << '\n';
        return false;
    }

    map.editor->constructHypotheses(gateways);

    std::cout << "INFO: load_map_labels: Found saved cells labels for " << mapName << '\n';
    CellTypes cellToType = load_cell_types(cellIn);

    for(auto& hyp : boost::make_iterator_range(map.editor->beginInitialHypotheses(), map.editor->endInitialHypotheses()))
    {
        auto type = assign_hypothesis_label(*hyp, *map.lpm, cellToType);
        hyp->setType(type);
    }

    for(auto& hyp : *map.editor)
    {
        auto type = assign_hypothesis_label(*hyp, *map.lpm, cellToType);
        hyp->setType(type);
    }

    // Save the reconstructed ground-truth map
    map.editor->saveToLocalTopoMap(full_filename(mapName, map.directory, kGroundTruthExtension));

    return true;
}


HypothesisType hyp_type_from_area_type(const LocalArea& area, const LocalArea& adjArea)
{
    switch(area.type())
    {
    case AreaType::decision_point:
        return HypothesisType::kDecision;
    case AreaType::destination:
        return HypothesisType::kDest;
    case AreaType::path_segment:
        return area.isEndpoint(adjArea) ? HypothesisType::kPathEndpoint : HypothesisType::kPathDest;
    case AreaType::place:       // intentional fall-through
    case AreaType::area:        // intentional fall-through
    case AreaType::path:        // intentional fall-through
    case AreaType::dead_end:    // intentional fall-through
    case AreaType::frontier:    // intentional fall-through
    default:
        assert(!"Invalid area type for boundary data!");
    }

    return HypothesisType::kArea;
}

} // namespace ui
} // nmaespace vulcan
