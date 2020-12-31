/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "utils/visibility_graph.h"
#include <gtest/gtest.h>
#include <iostream>

using namespace vulcan;


const int kGridSize = 3;


inline int grid_index(int x, int y)
{
    return y * kGridSize + x;
}


class VisGraphTest : public ::testing::Test
{
public:
    VisGraphTest(void)
    {
        using Vertex = utils::VisGraphVertex;
        using Edge = std::pair<int, int>;

        std::vector<Vertex> completeVertices = {Vertex{0, 0}, Vertex{0, 1}, Vertex{1, 1}, Vertex{1, 0}};
        std::vector<Edge> completeEdges = {Edge(0, 1), Edge(0, 2), Edge(0, 3), Edge(1, 2), Edge(1, 3), Edge(2, 3)};

        completeGraph_.reset(new utils::VisibilityGraph(completeVertices, completeEdges));

        // Arrange the grid points in row-major order
        std::vector<Vertex> gridVertices;
        gridVertices.reserve(kGridSize * kGridSize);
        for (int y = 0; y < kGridSize; ++y) {
            for (int x = 0; x < kGridSize; ++x) {
                gridVertices.emplace_back(x, y);
            }
        }

        std::vector<Edge> gridEdges;
        for (int x = 0; x < kGridSize; ++x) {
            for (int y = 0; y < kGridSize; ++y) {
                if (x + 1 < kGridSize) {
                    gridEdges.emplace_back(grid_index(x, y), grid_index(x + 1, y));
                }
                if (y + 1 < kGridSize) {
                    gridEdges.emplace_back(grid_index(x, y), grid_index(x, y + 1));
                }
            }
        }

        gridGraph_.reset(new utils::VisibilityGraph(gridVertices, gridEdges));

        std::vector<Edge> emptyEdges;
        unconnectedGraph_.reset(new utils::VisibilityGraph(completeVertices, emptyEdges));

        std::vector<Vertex> emptyVertices;
        emptyGraph_.reset(new utils::VisibilityGraph(emptyVertices, emptyEdges));
    }

protected:
    std::unique_ptr<utils::VisibilityGraph> completeGraph_;
    std::unique_ptr<utils::VisibilityGraph> gridGraph_;
    std::unique_ptr<utils::VisibilityGraph> unconnectedGraph_;
    std::unique_ptr<utils::VisibilityGraph> emptyGraph_;
};


TEST_F(VisGraphTest, MeanPathEdgeCount)
{
}
