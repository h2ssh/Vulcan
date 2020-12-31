/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file
 * \author   Collin Johnson
 *
 * A simple program to load the results of the human-human and human-robot labeling comparisons and generate a
 * scatterplot of the results for route edit distance vs. cell-by-cell accuracy.
 */

#include <boost/filesystem.hpp>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <gnuplot-iostream.h>
#include <iostream>
#include <sstream>
#include <string>


const int kRobotId = -1;

enum class ResultType
{
    human,
    robot,
};

struct map_results_t
{
    ResultType type;
    std::pair<int, int> mapIds;
    std::string name;
    int numPaths = 0;
    double meanTopoEdit = 0.0;
    double varTopoEdit = 0.0;
    double meanRouteEdit = 0.0;
    double varRouteEdit = 0.0;
    double zeroErrorRoutePercent = 0.0;
    double cellSuccessRate = 0.0;
    int64_t totalCells = 0;
    int64_t correctCells = 0;
    int numDecisionAsPath = 0;
    int numDecisionAsDest = 0;
    int numDestAsPath = 0;
    int numDestAsDecision = 0;
    int numPathAsDecision = 0;
    int numPathAsDest = 0;
};

std::pair<int, int> extract_map_ids(const std::string& resultPath, const std::string& compareMap);
map_results_t operator+(const map_results_t& lhs, const map_results_t& rhs);
bool operator==(const map_results_t& lhs, const map_results_t& rhs);
std::istream& operator>>(std::istream& in, map_results_t& results);
void create_scatterplot(const std::vector<map_results_t>& results, const std::string& mapName);
void create_human_only_scatterplot(const std::vector<map_results_t>& results, const std::string& mapName);

/**
 * summarize_label_results takes the name of the results file as its only argument.
 */
int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "INFO: Expected command-line: create_comparison_plots 'results directory' 'map name'\n";
        return -1;
    }

    // Iterate through the directory and find all maps containing "_results.txt"
    std::vector<map_results_t> pairwiseResults;

    using namespace boost::filesystem;

    for (directory_iterator dirIt(argv[1]), endIt; dirIt != endIt; ++dirIt) {
        auto path = dirIt->path();
        if ((path.string().find("ltm_compare_results.txt") != std::string::npos)
            || (path.string().find("robot_results.txt") != std::string::npos)) {
            ResultType type =
              (path.string().find("robot") != std::string::npos) ? ResultType::robot : ResultType::human;

            std::ifstream in(path.string());
            std::string resultsLine;

            std::cout << "Loading results for " << path.string() << '\n';

            while (std::getline(in, resultsLine)) {
                // Break as soon as finished reading file
                if (resultsLine.length() < 10) {
                    break;
                }

                map_results_t results;
                std::istringstream inStr(resultsLine);
                inStr >> results;
                results.type = type;

                std::cout << results.name << '\n';

                try {
                    results.mapIds = extract_map_ids(path.string(), results.name);

                    auto resIt = std::find(pairwiseResults.begin(), pairwiseResults.end(), results);
                    if (resIt != pairwiseResults.end()) {
                        std::cout << "Found duplicate: " << resIt->mapIds.first << ',' << resIt->mapIds.second << '\n';
                        *resIt = *resIt + results;
                    } else {
                        std::cout << "Adding new: " << results.mapIds.first << ',' << results.mapIds.second << '\n';
                        pairwiseResults.push_back(results);
                    }
                } catch (std::invalid_argument& e) {
                    std::cerr << "Ignored map " << results.name << ":: " << e.what() << '\n';
                }
            }
        }
    }

    create_scatterplot(pairwiseResults, argv[2]);
    create_human_only_scatterplot(pairwiseResults, argv[2]);

    return 0;
}


std::pair<int, int> extract_map_ids(const std::string& resultPath, const std::string& compareMap)
{
    // In the results format, both the path of the results file and the comparison maps have exactly one number:
    // their id.

    const char* kDigits = "0123456789";

    std::pair<int, int> ids;

    auto resultStart = resultPath.find_first_of(kDigits);
    if (resultStart != std::string::npos) {
        ids.first =
          std::strtol(resultPath.substr(resultStart, resultPath.find_first_not_of(kDigits, resultStart)).c_str(),
                      0,
                      10);
    } else {
        ids.first = kRobotId;
    }

    auto compStart = compareMap.find_first_of(kDigits);
    if (compStart != std::string::npos) {
        ids.second =
          std::strtol(compareMap.substr(compStart, compareMap.find_first_not_of(kDigits, compStart)).c_str(), 0, 10);
    } else {
        ids.second = kRobotId;
    }

    // Sort so the small id is always first
    if (ids.second < ids.first) {
        std::swap(ids.first, ids.second);
    }

    if ((ids.second == kRobotId) || (ids.first == ids.second)) {
        throw std::invalid_argument("Ids should be different and robot ids are always lowest");
    }

    return ids;
}


map_results_t operator+(const map_results_t& lhs, const map_results_t& rhs)
{
    map_results_t sumResults = lhs;

    // Accumulate the edit distance values
    sumResults.numPaths += rhs.numPaths;
    sumResults.meanTopoEdit =
      ((lhs.numPaths * lhs.meanTopoEdit) + (rhs.numPaths * rhs.meanTopoEdit)) / sumResults.numPaths;
    sumResults.varTopoEdit += rhs.varTopoEdit;
    sumResults.meanRouteEdit =
      ((lhs.numPaths * lhs.meanRouteEdit) + (rhs.numPaths * rhs.meanRouteEdit)) / sumResults.numPaths;
    sumResults.varRouteEdit += rhs.varRouteEdit;
    sumResults.zeroErrorRoutePercent =
      ((lhs.zeroErrorRoutePercent * lhs.numPaths) + (rhs.zeroErrorRoutePercent * rhs.numPaths)) / sumResults.numPaths;

    // Accumulate the cell-by-cell values
    sumResults.totalCells += rhs.totalCells;
    sumResults.correctCells += rhs.correctCells;
    sumResults.cellSuccessRate = static_cast<double>(sumResults.correctCells) / sumResults.totalCells;
    sumResults.numDecisionAsPath += rhs.numDecisionAsPath;
    sumResults.numDecisionAsDest += rhs.numDecisionAsDest;
    sumResults.numDestAsPath += rhs.numDestAsPath;
    sumResults.numDestAsDecision += rhs.numDestAsDecision;
    sumResults.numPathAsDecision += rhs.numPathAsDecision;
    sumResults.numPathAsDest += rhs.numPathAsDest;

    return sumResults;
}


bool operator==(const map_results_t& lhs, const map_results_t& rhs)
{
    return lhs.mapIds == rhs.mapIds;
}


std::istream& operator>>(std::istream& in, map_results_t& results)
{
    in >> results.name >> results.numPaths >> results.meanTopoEdit >> results.varTopoEdit >> results.meanRouteEdit
      >> results.varRouteEdit >> results.zeroErrorRoutePercent >> results.cellSuccessRate >> results.totalCells
      >> results.correctCells >> results.numDecisionAsPath >> results.numDecisionAsDest >> results.numDestAsPath
      >> results.numDestAsDecision >> results.numPathAsDecision >> results.numPathAsDest;
    return in;
}


void create_scatterplot(const std::vector<map_results_t>& results, const std::string& mapName)
{
    Gnuplot plot;
    plot << "set terminal qt font 'Verdana,12'\n";
    plot << "set style fill solid border lc black\n";
    plot << "set title 'Pairwise Comparision of MCMC vs. Human Labeling - " << mapName << "'\n";
    plot << "set ylabel 'Accuracy (% correct cells)'\n";
    plot << "set xlabel 'Route Edit Distance'\n";
    plot << "set yrange [50:100]\n";
    plot << "set xrange [0:8]\n";

    plot << "plot '-' using 1:2 ps 2 lw 2 lc rgb \"#87ceeb\" title 'Human-vs-Human',"
         << "'-' using 1:2 ps 2 lw 2 lc rgb \"#ff6347\" title 'MCMC-vs-Human'";
    plot << std::endl;

    // Data is col 1 = value, col 2 = tic value
    std::vector<double> humanCell;
    std::vector<double> humanRoute;
    std::vector<double> robotCell;
    std::vector<double> robotRoute;

    for (auto& r : results) {
        auto& cell = (r.type == ResultType::human) ? humanCell : robotCell;
        auto& route = (r.type == ResultType::human) ? humanRoute : robotRoute;

        cell.push_back(r.cellSuccessRate * 100);
        route.push_back(r.meanRouteEdit);
    }

    plot.send1d(boost::make_tuple(humanRoute, humanCell));
    plot.send1d(boost::make_tuple(robotRoute, robotCell));
    sleep(1);
}


void create_human_only_scatterplot(const std::vector<map_results_t>& results, const std::string& mapName)
{
    Gnuplot plot;
    plot << "set terminal qt font 'Verdana,12'\n";
    plot << "set style fill solid border lc black\n";
    plot << "set title 'Pairwise Comparision of Human Map Labels - " << mapName << "'\n";
    plot << "set ylabel 'Accuracy (% correct cells)'\n";
    plot << "set xlabel 'Route Edit Distance'\n";
    plot << "set yrange [50:100]\n";
    plot << "set xrange [0:8]\n";

    plot << "plot '-' using 1:2 ps 2 lw 2 lc rgb \"#87ceeb\" title 'Human-vs-Human'";
    plot << std::endl;

    // Data is col 1 = value, col 2 = tic value
    std::vector<double> humanCell;
    std::vector<double> humanRoute;

    for (auto& r : results) {
        if (r.type == ResultType::human) {
            humanCell.push_back(r.cellSuccessRate * 100);
            humanRoute.push_back(r.meanRouteEdit);
        }
    }

    plot.send1d(boost::make_tuple(humanRoute, humanCell));
    sleep(1);
}
