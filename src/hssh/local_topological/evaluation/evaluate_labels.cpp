/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     evaluate_labels.cpp
* \author   Collin Johnson
*
* evaluate_labels is a simple program that generates a heat map of a ground-truth and labeled LocalTopoMap. Simple
* statistics are calculated for the heat map and saved to a file.
*/

#include "hssh/local_topological/evaluation/heat_map.h"
#include "hssh/local_topological/evaluation/topological_edit_distance.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/areas/decision_point.h"
#include "hssh/local_topological/areas/destination.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/lpm_io.h"
#include "core/float_comparison.h"
#include "utils/command_line.h"
#include "utils/serialized_file_io.h"
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cassert>


using namespace vulcan;
using namespace vulcan::hssh;
using namespace boost::accumulators;

using StatsAcc = accumulator_set<double, stats<tag::mean, tag::max, tag::min, tag::variance>>;
using LabelGrid = utils::CellGrid<int8_t>;

enum LabelType : int8_t
{
    dest,
    decision,
    path,
    other,  // types that don't matter -- skeleton, unknown, frontier
};


void calculate_and_save_heatmap_stats(const LocalTopoHeatMap& heatMap, std::ostream& out);
void calculate_and_save_topological_edit_distance(const LocalTopoHeatMap& heatMap, std::ostream& out);
void calculate_and_save_cell_by_cell_stats(const LocalTopoMap& truth, const LocalTopoMap& labeled, std::ostream& out);
LabelGrid create_label_grid(const LocalTopoMap& topoMap);

/**
* evaluate_labels takes four arguments: 'grouth-truth LTM' 'labeled LTM' 'num paths' 'output file'
*
* Each map is loaded from file and then a heat map with the desired number of paths is generated. Once the heat map is
* created the following statistics are generated:
*
*   - conciseness : mean path length
*   - mean conciseness-diff : mean difference between path lengths in ground-truth vs labeled map
*   - std conciseness-diff : std of path length difference
*
* These statistics are saved to a file -- 'labeled LTM'.stats
*
* The file output is appended to the provided file. The results are saved as:
*
*   map_name num_paths mean_normalized_edit_dist var_normalized_edit_dist mean_raw_edit_dist var_raw_edit_dist \
*       cell_success_rate num_cells num_correct num_dec_as_path num_dec_as_dest num_dest_as_dec num_dest_as_path \
*       num_path_as_dest num_path_as_dec
*/
int main(int argc, char** argv)
{
    if(argc < 5)
    {
        std::cout << "evaluate_labels: Expected command line: 'grouth-truth LTM' 'labeled LTM' 'num paths' 'output file'\n";
        return -1;
    }

    LocalTopoMap groundTruth;
    utils::load_serializable_from_file(argv[1], groundTruth);

    LocalTopoMap labeled;
    utils::load_serializable_from_file(argv[2], labeled);

    int numPaths = atoi(argv[3]);

    LocalTopoHeatMap heatMap(groundTruth, labeled);
    heatMap.generatePaths(numPaths);

    std::string statsFilename = argv[4];
    std::ofstream statsOut(statsFilename, std::ios::app);

    if(!statsOut.is_open())
    {
        std::cerr << "ERROR: evaluate_labels: Failed to open file to save statistics:" << statsFilename << '\n';
        return -1;
    }

    std::cout << "=============== Evaluating " << argv[2] << "===================\n";

    statsOut << argv[2] << ' ';

//     calculate_and_save_heatmap_stats(heatMap, statsOut);
    calculate_and_save_topological_edit_distance(heatMap, statsOut);
    calculate_and_save_cell_by_cell_stats(groundTruth, labeled, statsOut);
    statsOut << '\n';

    return 0;
}


void calculate_and_save_heatmap_stats(const LocalTopoHeatMap& heatMap, std::ostream& out)
{
    StatsAcc truthAcc;
    StatsAcc labelAcc;
    StatsAcc diffAcc;

    const auto& truthStats = heatMap.groundTruthStats();
    const auto& labelStats = heatMap.labeledStats();

    for(std::size_t n = 0, end = truthStats.paths.size(); n < end; ++n)
    {
        int truthPathLength = truthStats.paths[n].size();
        int labelPathLength = labelStats.paths[n].size();

        truthAcc(truthPathLength);
        labelAcc(labelPathLength);
        diffAcc(std::abs(truthPathLength - labelPathLength));
    }

//     out << "============ Heat Map Stats ==================\n"
//         << "Truth Mean: " << mean(truthAcc) << '\n'
//         << "Label Mean: " << mean(labelAcc) << '\n'
//         << "Diff Mean: " << mean(diffAcc) << '\n'
//         << "Diff Std: " << std::sqrt(variance(diffAcc)) << '\n'
//         << "Diff Min: " << min(diffAcc) << '\n'
//         << "Diff Max: " << max(diffAcc) << "\n\n";


    std::cout << "============ Heat Map Stats ==================\n"
        << "Truth Mean: " << mean(truthAcc) << '\n'
        << "Label Mean: " << mean(labelAcc) << '\n'
        << "Diff Mean: " << mean(diffAcc) << '\n'
        << "Diff Std: " << std::sqrt(variance(diffAcc)) << '\n'
        << "Diff Min: " << min(diffAcc) << '\n'
        << "Diff Max: " << max(diffAcc) << "\n\n";
}


void calculate_and_save_topological_edit_distance(const LocalTopoHeatMap& heatMap, std::ostream& out)
{
    // It looks like the unit in the VRF paper is 0.4m based on a comparison of the nodes on the image in the Intel map
    // with my version of the map that I'm using. It is a strange unit, but based on looking at pixels in the
    // figure and comparing against the known map, the unit appears to be 0.4m sampling along the skeleton.
//     const double kRouteDistUnit = 0.25;
    const double kRouteDistUnit = 0.4;

    StatsAcc topoAcc;
    StatsAcc routeAcc;

    const auto& truthStats = heatMap.groundTruthStats();
    const auto& labelStats = heatMap.labeledStats();
    int zeroRouteErrorCount = 0;
    int totalRouteCount = 0;

    for(std::size_t n = 0, end = truthStats.paths.size(); n < end; ++n)
    {
        if(std::abs(labelStats.paths[n].length() - truthStats.paths[n].length()) > 2.0)
        {
            std::cerr << "Ignoring path with vastly different lengths. They took entirely different routes, which "
                << "isn't what the topo edit distance is trying to evaluate:\n"
                << "Truth:" << truthStats.paths[n] << '\n'
                << "Label:" << labelStats.paths[n] << '\n';
        }
        else
        {
            auto editDistance = topological_edit_distance(labelStats.paths[n], truthStats.paths[n], kRouteDistUnit);
            topoAcc(editDistance.topological);
            routeAcc(editDistance.route);

            if(absolute_fuzzy_equal(editDistance.route, 0.0))
            {
                ++zeroRouteErrorCount;
            }

            ++totalRouteCount;
        }
    }

    double zeroErrorPercent = static_cast<double>(zeroRouteErrorCount) / totalRouteCount;

    out << truthStats.paths.size() << ' ' << mean(topoAcc) << ' ' << variance(topoAcc) << ' '
        << mean(routeAcc) << ' ' << variance(routeAcc) << ' ' << zeroErrorPercent << ' ';

    std::cout << "============ Edit Distance Stats ==================\n"
        << "Edit Topo Mean: " << mean(topoAcc) << '\n'
        << "Edit Topo Std: " << std::sqrt(variance(topoAcc)) << '\n'
        << "Edit Route Mean: " << mean(routeAcc) << '\n'
        << "Edit Route Std: " << std::sqrt(variance(routeAcc)) << '\n'
        << "Edit Route Zero Error: " << zeroErrorPercent << "\n\n";

    for(std::size_t n = 0, end = truthStats.paths.size(); n < end; ++n)
    {
        auto editDistance = topological_edit_distance(labelStats.paths[n], truthStats.paths[n], kRouteDistUnit);

        if(editDistance.topological == max(topoAcc))
        {
            std::cout << "Worst path:\nTruth: " << truthStats.paths[n] << " Length:" << truthStats.paths[n].length()
                << "\nLabel: " << labelStats.paths[n] << " Length:" << labelStats.paths[n].length() << '\n';
        }
    }

    std::cout << "\n";
}


void calculate_and_save_cell_by_cell_stats(const LocalTopoMap& truth, const LocalTopoMap& labeled, std::ostream& out)
{
    auto truthGrid = create_label_grid(truth);
    auto labelsGrid = create_label_grid(labeled);

    assert(truthGrid.getWidthInCells() == labelsGrid.getWidthInCells());
    assert(truthGrid.getHeightInCells() == labelsGrid.getHeightInCells());

    // Overall counts
    int numTests = 0;
    int numCorrectTests = 0;

    // Individual types of errors -- expected as actual
    int numDecisionAsPath = 0;
    int numDecisionAsDest = 0;
    int numDestAsPath = 0;
    int numDestAsDecision = 0;
    int numPathAsDecision = 0;
    int numPathAsDest = 0;

    for(std::size_t y = 0; y < truthGrid.getHeightInCells(); ++y)
    {
        for(std::size_t x = 0; x < truthGrid.getWidthInCells(); ++x)
        {
            // Skip any cells not valid in the ground-truth
            if(truthGrid(x, y) == LabelType::other)
            {
                continue;
            }

            ++numTests;
            if(truthGrid(x, y) == labelsGrid(x, y))
            {
                ++numCorrectTests;
            }
            else if((truthGrid(x, y) == LabelType::decision) && (labelsGrid(x, y) == LabelType::dest))
            {
                ++numDecisionAsDest;
            }
            else if((truthGrid(x, y) == LabelType::decision) && (labelsGrid(x, y) == LabelType::path))
            {
                ++numDecisionAsPath;
            }
            else if((truthGrid(x, y) == LabelType::dest) && (labelsGrid(x, y) == LabelType::path))
            {
                ++numDestAsPath;
            }
            else if((truthGrid(x, y) == LabelType::dest) && (labelsGrid(x, y) == LabelType::decision))
            {
                ++numDestAsDecision;
            }
            else if((truthGrid(x, y) == LabelType::path) && (labelsGrid(x, y) == LabelType::dest))
            {
                ++numPathAsDest;
            }
            else if((truthGrid(x, y) == LabelType::path) && (labelsGrid(x, y) == LabelType::decision))
            {
                ++numPathAsDecision;
            }
        }
    }

    double successRate = static_cast<double>(numCorrectTests) / numTests;
    std::cout << "============ Cell-by-Cell Stats ==================\n"
        << "Cell success rate: " << successRate << '\n'
        << "Total cells: " << numTests << '\n'
        << "Num correct: " << numCorrectTests << '\n'
        << "Decision as path:" << numDecisionAsPath << '\n'
        << "Decision as dest:" << numDecisionAsDest << '\n'
        << "Dest as path:" << numDestAsPath << '\n'
        << "Dest as decision:" << numDestAsDecision << '\n'
        << "Path as decision:" << numPathAsDecision << '\n'
        << "Path as dest:" << numPathAsDest << "\n\n";

    out << successRate << ' '
        << numTests << ' '
        << numCorrectTests << ' '
        << numDecisionAsPath << ' '
        << numDecisionAsDest << ' '
        << numDestAsPath << ' '
        << numDestAsDecision << ' '
        << numPathAsDecision << ' '
        << numPathAsDest << ' ';
}


LabelGrid create_label_grid(const LocalTopoMap& topoMap)
{
    LabelGrid grid(
        topoMap.voronoiSkeleton().getWidthInCells(),
        topoMap.voronoiSkeleton().getHeightInCells(),
        0.05,
        topoMap.voronoiSkeleton().getGlobalCenter()
    );

    grid.reset(LabelType::other);

    for(auto& area : topoMap)
    {
        LabelType type = LabelType::other;
        switch(area->type())
        {
        case AreaType::decision_point:
            type = LabelType::decision;
            break;
        case AreaType::destination:
            type = LabelType::dest;
            break;
        case AreaType::path_segment:
            type = LabelType::path;
            break;
        default:
            type = LabelType::other;
            std::cerr << "WARNING: create_label_grid: Unknown assigned area type. Not one of three allowed.\n";
        }

        for(auto pos : area->extent())
        {
            auto cell = utils::global_point_to_grid_cell_round(pos, topoMap.voronoiSkeleton());
            grid(cell.x, cell.y) = type;
        }
    }

    return grid;
}
