/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     search_test.cpp
* \author   Collin Johnson
*
* This file tests and exercises the AStarSearch.
*/

#include "planner/goal/search.h"
#include "hssh/global_topological/graph.h"
#include <string>
#include <sstream>
#include <iostream>

using namespace vulcan;

typedef graph::Path<hssh::TopologicalVertex> Path;

struct test_result_t
{
    std::string testName;
    bool        passed;

    test_result_t(std::string name = "", bool passed = false)
        : testName(name)
        , passed(passed)
    {
    }
};

test_result_t test_start_and_goal_same(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);
test_result_t test_start_not_in_graph(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);
test_result_t test_goal_not_in_graph(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);
test_result_t test_start_and_goal_different_subgraphs(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);
test_result_t test_shortest_path_more_nodes(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);
test_result_t test_simple_shortest_path(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);
test_result_t test_two_paths_same_length(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);
test_result_t test_shortest_path_after_initial_goal_expansion(const hssh::TopologicalGraph& graph, planner::AStarSearch& search);

std::string print_path(const Path& path);
void print_test_results(const std::vector<test_result_t>& results);

// usually, I would put this in order when called in the file, but it is long and ugly, so hiding at the end rather than obscuring the test functions
void construct_test_graph(hssh::TopologicalGraph& graph);

int main(int argc, char** argv)
{
    hssh::TopologicalGraph     graph;
    std::vector<test_result_t> results;

    construct_test_graph(graph);

    planner::AStarSearch search(graph);

    results.push_back(test_start_and_goal_same(graph, search));
    results.push_back(test_start_not_in_graph(graph, search));
    results.push_back(test_goal_not_in_graph(graph, search));
    results.push_back(test_start_and_goal_different_subgraphs(graph, search));
    results.push_back(test_shortest_path_more_nodes(graph, search));
    results.push_back(test_simple_shortest_path(graph, search));
    results.push_back(test_two_paths_same_length(graph, search));
    results.push_back(test_shortest_path_after_initial_goal_expansion(graph, search));

    print_test_results(results);

    return 0;
}


test_result_t test_start_and_goal_same(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    // For the start and goal same test, the finished path should be a single vertex, the start/goal vertex
    std::string testName("test_start_and_goal_same");
    bool        passed = false;

    auto startVertex = graph.getVertex(0);
    auto goalVertex  = graph.getVertex(0);

    if(search.search(startVertex, goalVertex))
    {
        Path path = search.getPath();

        auto vertices = path.getPath();

        std::cout<<testName<<": "<<print_path(path)<<'\n';

        if((vertices.size() == 1) && (vertices[0] == goalVertex))
        {
            passed = true;
        }
    }

    return test_result_t(testName, passed);
}


test_result_t test_start_not_in_graph(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    // If the start is not in the graph, then no path should be found
    std::string testName("test_start_not_in_graph");
    bool        passed = false;

    hssh::TopologicalVertex start(graph.numVertices() + 5, Point<float>(10.0f, 10.0f), hssh::NodeData(graph.numVertices() + 5),  0.0);
    auto goal = graph.getVertex(0);

    if(!search.search(start, goal))
    {
        passed = true;
    }
    else
    {
        std::cout<<"ERROR:"<<testName<<": Found path."<<print_path(search.getPath())<<'\n';
    }

    return test_result_t(testName, passed);
}


test_result_t test_goal_not_in_graph(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    // If the goal is not in the graph, then no path should be found
    std::string testName("test_goal_not_in_graph");
    bool        passed = false;

    auto start = graph.getVertex(0);
    hssh::TopologicalVertex goal(graph.numVertices() + 5, Point<float>(10.0f, 10.0f), hssh::NodeData(graph.numVertices() + 5),  0.0);

    if(!search.search(start, goal))
    {
        passed = true;
    }
    else
    {
        std::cout<<"ERROR:"<<testName<<": Found path."<<print_path(search.getPath())<<'\n';
    }

    return test_result_t(testName, passed);
}


test_result_t test_start_and_goal_different_subgraphs(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    // If the start and goal are in different subgraphs (not quite the right term), there should exist no path
    std::string testName("test_start_and_goal_different_subgraphs");
    bool        passed = false;

    auto start = graph.getVertex(0);
    auto goal  = graph.getVertex(2);

    if(!search.search(start, goal))
    {
        passed = true;
    }
    else
    {
        std::cout<<"ERROR:"<<testName<<": Found path."<<print_path(search.getPath())<<'\n';
    }

    return test_result_t(testName, passed);
}


test_result_t test_shortest_path_more_nodes(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    // In this test, fewer nodes to go via 8, but the last segment has high cost, so the lowest cost plan goes via 10
    std::string testName("test_shortest_path_more_nodes");
    bool        passed = false;

    auto start = graph.getVertex(0);
    auto goal  = graph.getVertex(9);

    if(search.search(start, goal))
    {
        Path path = search.getPath();
        auto vertices = path.getPath();

        if(vertices.size() == 11)
        {
            passed = true;

            passed &= vertices[0].getId() == 0;
            passed &= (vertices[2].getId() == 1) || (vertices[2].getId() == 4);
            passed &= vertices[4].getId() == 5;
            passed &= vertices[6].getId() == 6;
            passed &= vertices[8].getId() == 10;
            passed &= vertices[10].getId() == 9;

            if(!passed)
            {
                std::cout<<"ERROR:"<<testName<<":Incorrect path. Should be 0 1/4 5 6 10 9. Found "<<print_path(path)<<'\n';
            }
        }
        else
        {
            passed = false;

            std::cout<<"ERROR:"<<testName<<":Path wrong length:"<<print_path(path)<<'\n';
        }
    }

    return test_result_t(testName, passed);
}


test_result_t test_simple_shortest_path(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    // In this test, see that a simple shortest path attempt actually works
    std::string testName("test_simple_shortest_path");
    bool        passed = false;

    auto start = graph.getVertex(0);
    auto goal  = graph.getVertex(8);

    if(search.search(start, goal))
    {
        Path path = search.getPath();
        auto vertices = path.getPath();

        if(vertices.size() == 7)
        {
            passed = true;

            passed &= vertices[0].getId() == 0;
            passed &= vertices[2].getId() == 4;
            passed &= vertices[4].getId() == 7;
            passed &= vertices[6].getId() == 8;

            if(!passed)
            {
                std::cout<<"ERROR:"<<testName<<":Incorrect path. Should be 0 4 7 8. Found "<<print_path(path)<<'\n';
            }
        }
        else
        {
            passed = false;

            std::cout<<"ERROR:"<<testName<<":Path wrong length:"<<print_path(path)<<'\n';
        }
    }

    return test_result_t(testName, passed);
}


test_result_t test_two_paths_same_length(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    // In this test, see that a path is found even if two choices are the same length
    std::string testName("test_two_paths_same_length");
    bool        passed = false;

    auto start = graph.getVertex(0);
    auto goal  = graph.getVertex(5);

    if(search.search(start, goal))
    {
        Path path = search.getPath();
        auto vertices = path.getPath();

        if(vertices.size() == 5)
        {
            passed = true;

            passed &= vertices[0].getId() == 0;
            passed &= (vertices[2].getId() == 4) || (vertices[1].getId() == 1);
            passed &= vertices[4].getId() == 5;

            if(!passed)
            {
                std::cout<<"ERROR:"<<testName<<":Incorrect path. Should be 0 1/4 5. Found "<<print_path(path)<<'\n';
            }
        }
        else
        {
            passed = false;

            std::cout<<"ERROR:"<<testName<<":Path wrong length:"<<print_path(path)<<'\n';
        }
    }

    return test_result_t(testName, passed);
}


test_result_t test_shortest_path_after_initial_goal_expansion(const hssh::TopologicalGraph& graph, planner::AStarSearch& search)
{
    std::string testName("test_shortest_path_after_initial_goal_expansion");
    bool        passed = false;
    return test_result_t(testName, passed);
}


std::string print_path(const Path& path)
{
    std::ostringstream out;

    const std::vector<hssh::TopologicalVertex>& vertices = path.getPath();

    for(auto vertexIt = vertices.begin(), vertexEnd = vertices.end(); vertexIt != vertexEnd; ++vertexIt)
    {
        out<<' '<<vertexIt->getId();
    }

    return out.str();
}


void print_test_results(const std::vector<test_result_t>& results)
{
    int totalTests  = results.size();
    int testsPassed = 0;

    for(auto resultIt = results.begin(), resultEnd = results.end(); resultIt != resultEnd; ++resultIt)
    {
        std::cout<<resultIt->testName<<" : "<<(resultIt->passed ? "PASSED" : "FAILED")<<'\n';

        if(resultIt->passed)
        {
            ++testsPassed;
        }
    }

    std::cout<<"Overall results: Passed "<<testsPassed<<'/'<<totalTests<<" tests!\n";
}


void construct_test_graph(hssh::TopologicalGraph& graph)
{
    hssh::TopologicalVertex place0 (0,  Point<float>(0.0f, 0.0f), hssh::NodeData(0),  0.0);
    hssh::TopologicalVertex place1 (1,  Point<float>(5.0f, 0.0f), hssh::NodeData(1),  0.0);
    hssh::TopologicalVertex place2 (2,  Point<float>(8.0f, 0.0f), hssh::NodeData(2),  0.0);
    hssh::TopologicalVertex place3 (3,  Point<float>(9.0f, 0.0f), hssh::NodeData(3),  0.0);
    hssh::TopologicalVertex place4 (4,  Point<float>(0.0f, 2.0f), hssh::NodeData(4),  0.0);
    hssh::TopologicalVertex place5 (5,  Point<float>(5.0f, 2.0f), hssh::NodeData(5),  0.0);
    hssh::TopologicalVertex place6 (6,  Point<float>(7.0f, 2.0f), hssh::NodeData(6),  0.0);
    hssh::TopologicalVertex place7 (7,  Point<float>(0.0f, 3.0f), hssh::NodeData(7),  0.0);
    hssh::TopologicalVertex place8 (8,  Point<float>(5.0f, 3.0f), hssh::NodeData(8),  0.0);
    hssh::TopologicalVertex place9 (9,  Point<float>(6.0f, 3.0f), hssh::NodeData(9),  0.0);
    hssh::TopologicalVertex place10(10, Point<float>(7.0f, 3.0f), hssh::NodeData(10), 0.0);

    // Just treat the segments as places because no conversion back to path segments will occur
    hssh::TopologicalVertex segment0 (11, Point<float>(2.5f, 0.0f), hssh::NodeData(11), 5.0);
    hssh::TopologicalVertex segment1 (12, Point<float>(8.5f, 0.0f), hssh::NodeData(12), 1.0);
    hssh::TopologicalVertex segment2 (13, Point<float>(0.0f, 1.0f), hssh::NodeData(13), 2.0);
    hssh::TopologicalVertex segment3 (14, Point<float>(5.0f, 1.0f), hssh::NodeData(14), 2.0);
    hssh::TopologicalVertex segment4 (15, Point<float>(2.5f, 2.0f), hssh::NodeData(15), 5.0);
    hssh::TopologicalVertex segment5 (16, Point<float>(6.0f, 2.0f), hssh::NodeData(16), 2.0);
    hssh::TopologicalVertex segment6 (17, Point<float>(0.0f, 2.5f), hssh::NodeData(17), 1.0);
    hssh::TopologicalVertex segment7 (18, Point<float>(5.0f, 2.5f), hssh::NodeData(18), 2.0);
    hssh::TopologicalVertex segment8 (19, Point<float>(7.0f, 2.5f), hssh::NodeData(19), 1.0);
    hssh::TopologicalVertex segment9 (20, Point<float>(2.5f, 3.0f), hssh::NodeData(20), 5.0);
    hssh::TopologicalVertex segment10(21, Point<float>(5.5f, 3.0f), hssh::NodeData(21), 10.0);
    hssh::TopologicalVertex segment11(22, Point<float>(6.5f, 3.0f), hssh::NodeData(22), 1.0);

    std::vector<hssh::TopologicalEdge> edges;
    edges.push_back(hssh::TopologicalEdge(0, place0, segment0, 0.0));
    edges.push_back(hssh::TopologicalEdge(1, place1, segment0, 0.0));
    edges.push_back(hssh::TopologicalEdge(2, place2, segment1, 0.0));
    edges.push_back(hssh::TopologicalEdge(3, place3, segment1, 0.0));
    edges.push_back(hssh::TopologicalEdge(4, place0, segment2, 0.0));
    edges.push_back(hssh::TopologicalEdge(5, place4, segment2, 0.0));
    edges.push_back(hssh::TopologicalEdge(6, place1, segment3, 0.0));
    edges.push_back(hssh::TopologicalEdge(7, place5, segment3, 0.0));
    edges.push_back(hssh::TopologicalEdge(8, place4, segment4, 0.0));
    edges.push_back(hssh::TopologicalEdge(9, place5, segment4, 0.0));
    edges.push_back(hssh::TopologicalEdge(10, place5, segment5, 0.0));
    edges.push_back(hssh::TopologicalEdge(11, place6, segment5, 0.0));
    edges.push_back(hssh::TopologicalEdge(12, place4, segment6, 0.0));
    edges.push_back(hssh::TopologicalEdge(13, place7, segment6, 0.0));
    edges.push_back(hssh::TopologicalEdge(14, place5, segment7, 0.0));
    edges.push_back(hssh::TopologicalEdge(15, place8, segment7, 0.0));
    edges.push_back(hssh::TopologicalEdge(16, place6, segment8, 0.0));
    edges.push_back(hssh::TopologicalEdge(17, place10, segment8, 0.0));
    edges.push_back(hssh::TopologicalEdge(18, place7, segment9, 0.0));
    edges.push_back(hssh::TopologicalEdge(19, place8, segment9, 0.0));
    edges.push_back(hssh::TopologicalEdge(20, place8, segment10, 0.0));
    edges.push_back(hssh::TopologicalEdge(21, place9, segment10, 0.0));
    edges.push_back(hssh::TopologicalEdge(22, place9, segment11, 0.0));
    edges.push_back(hssh::TopologicalEdge(23, place10, segment11, 0.0));

    for(auto edgeIt = edges.begin(), edgeEnd = edges.end(); edgeIt != edgeEnd; ++edgeIt)
    {
        graph.addEdge(*edgeIt);
    }
}
