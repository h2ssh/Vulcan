/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     edit_distance.cpp
* \author   Collin Johnson
*
* Definition of edit_distance function.
*/

#include "utils/edit_distance.h"
#include "utils/cell_grid.h"
#include <iostream>
#include <sstream>

namespace vulcan
{
namespace utils
{

void output_edits(const std::string& x, const std::string& y, const CellGrid<double>& changes);


double edit_distance(const std::string& x,
                     const std::string& y,
                     edit_distance_weights_t weights,
                     bool outputChanges)
{
    const double kMax = std::numeric_limits<double>::max();

    // If x is empty, then every character in y must be inserted.
    if(x.empty())
    {
        return weights.insertion * y.length();
    }
    // If y is empty, then every character in x must be deleted.
    else if(y.empty())
    {
        return weights.deletion * x.length();
    }
    /*
    * Otherwise, we need to perform the full search:
    *
    * Possible changes:
    *
    *   - increment : the value in the string is the same, so just move diagonally
    *   - substitute : replace the value in x by that in y, again move diagonally
    *   - delete : delete a character in x, move right
    *   - insert : add a character to x, move up
    */

    // Use a cell grid, as we just need a grid to hold the scores. The coordinate system values are irrelevant.
    CellGrid<double> scores(x.length() + 1, y.length() + 1, 0.5, Point<float>(0.0, 0.0));
    scores(0, 0) = 0.0;
    std::array<double, 4> costs;

    for(std::size_t m = 0; m < y.length() + 1; ++m)
    {
        for(std::size_t n = 0; n < x.length() + 1; ++n)
        {
            if(m == 0 && n == 0)
            {
                continue;
            }

            costs[0] = ((n > 0) && (m > 0) && (m <= y.length()) && (n <= x.length()) && (x[n - 1] == y[m - 1])) ?
                scores(n - 1, m - 1) : kMax;
            costs[1] = (n > 0 && m > 0) ? scores(n - 1, m - 1) + weights.substitution : kMax;
            costs[2] = (m > 0) ? scores(n, m - 1) + weights.insertion : kMax;
            costs[3] = (n > 0) ? scores(n - 1, m) + weights.deletion : kMax;

            scores(n, m) = *std::min_element(costs.begin(), costs.end());
        }
    }

    if(outputChanges)
    {
        output_edits(x, y, scores);
    }

    return scores(x.length(), y.length());
}


void output_edits(const std::string& x, const std::string& y, const CellGrid<double>& changes)
{
    const double kMax = std::numeric_limits<double>::max();

    std::vector<std::string> edits;

    int xIndex = x.length();
    int yIndex = y.length();
    std::string goalString = y;
    int goalIndex = goalString.length()-1;

    std::cout << "DEBUG: edit_distance: Start: " << x << " Goal:" << y << '\n';

    while((xIndex > 0) || (yIndex > 0))
    {
        // Calculate the cost for making each possible operation
        double substituteCost = (xIndex > 0 && yIndex > 0) ? changes(xIndex - 1, yIndex - 1) : kMax;
        double insertCost = (yIndex > 0) ? changes(xIndex, yIndex - 1) : kMax;
        double deleteCost = (xIndex > 0) ? changes(xIndex - 1, yIndex) : kMax;

        std::ostringstream out;

        double cost = changes(xIndex, yIndex);
        out << "cost " << cost << " : ";

        // Moving diagonally in the grid
        if((substituteCost < insertCost) && (substituteCost < deleteCost))
        {
            // Can't move diagonally if either cursor is at the end of the string
            assert(xIndex > 0 && yIndex > 0);
            --xIndex;
            --yIndex;

            if(x[xIndex] == y[yIndex])
            {
                out << "right : " << goalString << '\n';
            }
            else
            {
                out << "substitute " << x[xIndex] << " -> " << y[yIndex] << " : " << goalString << '\n';
                goalString[goalIndex] = x[xIndex];
            }

            --goalIndex;
        }
        // Moving up in the grid
        else if(insertCost < deleteCost)
        {
            // Can't insert if cursor is at the end of the y string
            assert(yIndex > 0);
            --yIndex;
            out << "inserting " << y[yIndex] << " : " << goalString << '\n';
            goalString.erase(goalIndex, 1);
            --goalIndex;

        }
        // Moving horizontally in the grid
        else
        {
            // Can't delete if the cursor is at the end of the x string
            assert(xIndex > 0);
            --xIndex;
            out << "deleting " << x[xIndex] << " : " << goalString << '\n';
            goalString.insert(goalIndex + 1, x.substr(xIndex, 1));
        }

        edits.push_back(out.str());
    }

    std::reverse(edits.begin(), edits.end());
    std::copy(edits.begin(), edits.end(), std::ostream_iterator<std::string>(std::cout));
}

} // namespace utils
} // namespace vulcan
