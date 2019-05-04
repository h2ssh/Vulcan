/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     summarize_label_results.cpp
* \author   Collin Johnson
*
* A simple program to load the results of running evaluate_labels on a variety of maps. It accumulates the data from
* each map to produce overall means and stds for the edit distance and cell-by-cell comparisons.
*/

#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/weighted_mean.hpp>
#include <boost/accumulators/statistics/weighted_variance.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/weighted_median.hpp>
#include <boost/tuple/tuple.hpp>
#include <gnuplot-iostream.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>


struct map_results_t
{
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

map_results_t operator+(const map_results_t& lhs, const map_results_t& rhs);
std::istream& operator>>(std::istream& in, map_results_t& results);
std::ostream& operator<<(std::ostream& out, map_results_t& results);

void plot_cell_by_cell(std::vector<map_results_t>& results);
void plot_topological_edit_distance(std::vector<map_results_t>& results);
void plot_route_edit_distance(std::vector<map_results_t>& results);


/**
* summarize_label_results takes the name of the results file as its only argument.
*/
int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << "INFO: Expected command-line: summarize_label_results 'results filename' 'plots/no_plots (default)'\n";
        return -1;
    }

    bool showPlots = false;
    if(argc > 2)
    {
        showPlots = std::string(argv[2]) == std::string("plots");
    }

    using namespace boost::accumulators;
    using StatsAcc = accumulator_set<double, stats<tag::min,
                                                   tag::max,
                                                   tag::weighted_mean,
                                                   tag::weighted_variance,
                                                   tag::weighted_median>, int>;
    StatsAcc cellAcc;
    StatsAcc topoAcc;
    StatsAcc routeAcc;
    StatsAcc errorAcc;

    std::ifstream in(argv[1]);
    std::string resultsLine;

    map_results_t total;
    map_results_t individual;
    std::vector<map_results_t> allResults;

    while(true)
    {
        std::getline(in, resultsLine);

        // Break as soon as finished reading file
        if(resultsLine.length() < 10)
        {
            break;
        }

        std::istringstream inStr(resultsLine);
        inStr >> individual;
        total = total + individual;
        std::cout << individual << '\n';
        allResults.push_back(individual);

        cellAcc(individual.cellSuccessRate, weight = individual.totalCells);
        topoAcc(individual.meanTopoEdit, weight = 1);
        routeAcc(individual.meanRouteEdit, weight = 1);
        errorAcc(individual.zeroErrorRoutePercent, weight = 1);
    }

    std::cout << "\n================== Overall ====================\n"
        << "Topo edit (min, max, mean, std, median): " << min(topoAcc) << ", " << max(topoAcc) << ", " << weighted_mean(topoAcc) << ", " << std::sqrt(weighted_variance(topoAcc)) << ", " << weighted_median(topoAcc) << '\n'
        << "Route edit (min, max, mean, std, median): " << min(routeAcc) << ", " << max(routeAcc) << ", " << weighted_mean(routeAcc) << ", " << std::sqrt(weighted_variance(routeAcc)) << ", " << weighted_median(routeAcc) << '\n'
        << "Zero error (min, max, mean, std, median): " << min(errorAcc) << ", " << max(errorAcc) << ", " << weighted_mean(errorAcc) << ", " << std::sqrt(weighted_variance(errorAcc)) << ", " << weighted_median(errorAcc) << '\n'
        << "Cell % (min, max, mean, std, median): " << min(cellAcc) << ", " << max(cellAcc) << ", " << weighted_mean(cellAcc) << ", " << std::sqrt(weighted_variance(cellAcc)) << ", " << weighted_median(cellAcc) << '\n';

    std::cout << "\n================== Latex ====================\n";
    printf("Norm edit: %.3f & %.3f & %.3f & %.3f & %.3f & \\\\ \n", min(topoAcc), max(topoAcc), weighted_mean(topoAcc), std::sqrt(weighted_variance(topoAcc)), weighted_median(topoAcc));
    printf("Raw edit:  %.3f & %.3f & %.3f & %.3f & %.3f & \\\\ \n", min(routeAcc), max(routeAcc), weighted_mean(routeAcc), std::sqrt(weighted_variance(routeAcc)), weighted_median(routeAcc));
    printf("Cell:      %.3f & %.3f & %.3f & %.3f & %.3f & \\\\ \n", min(cellAcc), max(cellAcc), weighted_mean(cellAcc), std::sqrt(weighted_variance(cellAcc)), weighted_median(cellAcc));

    if(showPlots)
    {
        plot_cell_by_cell(allResults);
        plot_topological_edit_distance(allResults);
        plot_route_edit_distance(allResults);
    }

    return 0;
}


map_results_t operator+(const map_results_t& lhs, const map_results_t& rhs)
{
    map_results_t sumResults;
    sumResults.name = "Total:";

    // Accumulate the edit distance values
    sumResults.numPaths = lhs.numPaths + rhs.numPaths;
    sumResults.meanTopoEdit = ((lhs.numPaths * lhs.meanTopoEdit) + (rhs.numPaths * rhs.meanTopoEdit)) / sumResults.numPaths;
    sumResults.varTopoEdit = lhs.varTopoEdit + rhs.varTopoEdit;
    sumResults.meanRouteEdit = ((lhs.numPaths * lhs.meanRouteEdit) + (rhs.numPaths * rhs.meanRouteEdit)) / sumResults.numPaths;
    sumResults.varRouteEdit = lhs.varRouteEdit + rhs.varRouteEdit;
    sumResults.zeroErrorRoutePercent = ((lhs.zeroErrorRoutePercent * lhs.numPaths)
        + (rhs.zeroErrorRoutePercent * rhs.numPaths)) / (lhs.numPaths + rhs.numPaths);

    // Accumulate the cell-by-cell values
    sumResults.totalCells = lhs.totalCells + rhs.totalCells;
    sumResults.correctCells = lhs.correctCells + rhs.correctCells;
    sumResults.cellSuccessRate = static_cast<double>(sumResults.correctCells) / sumResults.totalCells;
    sumResults.numDecisionAsPath = lhs.numDecisionAsPath + rhs.numDecisionAsPath;
    sumResults.numDecisionAsDest = lhs.numDecisionAsDest + rhs.numDecisionAsDest;
    sumResults.numDestAsPath = lhs.numDestAsPath + rhs.numDestAsPath;
    sumResults.numDestAsDecision = lhs.numDestAsDecision + rhs.numDestAsDecision;
    sumResults.numPathAsDecision = lhs.numPathAsDecision + rhs.numPathAsDecision;
    sumResults.numPathAsDest = lhs.numPathAsDest + rhs.numPathAsDest;

    return sumResults;
}


std::istream& operator>>(std::istream& in, map_results_t& results)
{
    in >> results.name >> results.numPaths >> results.meanTopoEdit >> results.varTopoEdit >> results.meanRouteEdit
        >> results.varRouteEdit >> results.zeroErrorRoutePercent >> results.cellSuccessRate >> results.totalCells
        >> results.correctCells >> results.numDecisionAsPath >> results.numDecisionAsDest >> results.numDestAsPath
        >> results.numDestAsDecision >> results.numPathAsDecision >> results.numPathAsDest;
    return in;
}


std::ostream& operator<<(std::ostream& out, map_results_t& results)
{
    out << "================ " << results.name << "================\n"
        << "Norm edit (mean, std):" << results.meanTopoEdit << ", " << std::sqrt(results.varTopoEdit) << '\n'
        << "Raw edit (mean, std):" << results.meanRouteEdit << ", " << std::sqrt(results.varRouteEdit) << '\n'
        << "Zero error percent: " << (results.zeroErrorRoutePercent * 100) << '\n'
        << "Cell-by-cell (overall %):" << results.cellSuccessRate << '\n';
    return out;
}


void plot_cell_by_cell(std::vector<map_results_t>& results)
{
    std::sort(results.begin(), results.end(), [](auto& lhs, auto& rhs) {
        return lhs.cellSuccessRate > rhs.cellSuccessRate;
    });

    Gnuplot plot;
    plot << "set yrange [0:100]\n";
    plot << "set style fill solid border lc black\n";
    plot << "set title 'Cell-by-Cell Results for MCMC Place Classification'\n";
    plot << "set ylabel 'Accuracy (% correct cells)'\n";
    plot << "set xtics rotate by -60\n";
    plot << "set xtics format \"%s\"\n";
    plot << "set xtics noenhanced\n";   // display underscores
    plot << "set style data histograms\n";
    plot << "set style histogram clustered gap 1\n";
    plot << "set style fill solid 1.0\n";

    plot << "plot '-' using 1:xticlabels(2) fillcolor \"skyblue\" notitle\n";
    plot << std::endl;

    // Data is col 1 = value, col 2 = tic value
    std::vector<double> accuracy;
    std::vector<char*> xtics;

    for(auto& r : results)
    {
        accuracy.push_back(r.cellSuccessRate * 100);
        std::string name = r.name;  // print a short version of the name
        auto end = name.find("_result");
        if(end == std::string::npos)
        {
            end = name.find("_best");

            if(end == std::string::npos)
            {
                end = name.find_first_of("_.");
            }
        }
        name.erase(end, std::string::npos);
        char *strname = strdup(name.c_str());
        xtics.push_back(strname);
    }

    plot.send1d(boost::make_tuple(accuracy, xtics));
    sleep(1);

    for(auto& str : xtics)
    {
        free(str);
    }
}


void plot_topological_edit_distance(std::vector<map_results_t>& results)
{
    Gnuplot plot;
    plot << "set yrange [0:60]\n";
    plot << "set style fill solid border lc black\n";
    plot << "set title 'Mean Normalized Edit Distance for MCMC Place Classification'\n";
    plot << "set ylabel 'Normalized Edit Distance'\n";
    plot << "set xtics rotate by -60\n";
    plot << "set xtics format \"%s\"\n";
    plot << "set xtics noenhanced\n";   // display underscores
    plot << "set style data histograms\n";
    plot << "set style histogram clustered gap 1\n";
    plot << "set style fill solid 1.0\n";

    plot << "plot '-' using 1:xticlabels(2) fillcolor \"skyblue\" notitle\n";
    plot << std::endl;

    // Data is col 1 = value, col 2 = tic value
    std::vector<double> mean;
    std::vector<char*> xtics;

    for(auto& r : results)
    {
        mean.push_back(r.meanTopoEdit);
        std::string name = r.name;  // print a short version of the name
        auto end = name.find("_result");
        if(end == std::string::npos)
        {
            end = name.find("_best");

            if(end == std::string::npos)
            {
                end = name.find_first_of("_.");
            }
        }
        name.erase(end, std::string::npos);
        char *strname = strdup(name.c_str());
        xtics.push_back(strname);
    }

    plot.send1d(boost::make_tuple(mean, xtics));
    sleep(1);

    for(auto& str : xtics)
    {
        free(str);
    }
}


void plot_route_edit_distance(std::vector<map_results_t>& results)
{
    std::sort(results.begin(), results.end(), [](auto& lhs, auto& rhs) {
        return lhs.meanRouteEdit < rhs.meanRouteEdit;
    });

    Gnuplot plot;
    plot << "set yrange [0:5]\n";
    plot << "set style fill solid border lc black\n";
    plot << "set title 'Mean Route Edit Distance for MCMC Place Classification'\n";
    plot << "set ylabel 'Route Edit Distance'\n";
    plot << "set xtics rotate by -60\n";
    plot << "set xtics format \"%s\"\n";
    plot << "set xtics noenhanced\n";   // display underscores
    plot << "set style data histograms\n";
    plot << "set style histogram clustered gap 1\n";
    plot << "set style fill solid 1.0\n";

    plot << "plot '-' using 1:xticlabels(2) fillcolor \"skyblue\" notitle\n";
    plot << std::endl;

    // Data is col 1 = value, col 2 = tic value
    std::vector<double> mean;
    std::vector<char*> xtics;

    for(auto& r : results)
    {
        mean.push_back(r.meanRouteEdit);
        std::string name = r.name;  // print a short version of the name
        auto end = name.find("_result");
        if(end == std::string::npos)
        {
            end = name.find("_best");

            if(end == std::string::npos)
            {
                end = name.find_first_of("_.");
            }
        }
        name.erase(end, std::string::npos);
        char *strname = strdup(name.c_str());
        xtics.push_back(strname);
    }

    plot.send1d(boost::make_tuple(mean, xtics));
    sleep(1);

    for(auto& str : xtics)
    {
        free(str);
    }
}
