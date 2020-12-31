/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_topological/area.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/areas/serialization.h"
#include "utils/serialized_file_io.h"
#include "core/image.h"
#include "vision/image_utils.h"
#include <boost/filesystem.hpp>

using namespace vulcan;
using namespace vulcan::hssh;

int main(int argc, char** argv)
{
    using namespace boost::filesystem;

    if(argc < 3)
    {
        std::cout << "Expected usage: generate_label_heat_maps 'directory_w_maps' 'base img name'\n";
        return -1;
    }

    // Iterate through the directory and find all maps containing "_truth.ltm"
    std::vector<LocalTopoMap> maps;

    for(directory_iterator dirIt(argv[1]), endIt; dirIt != endIt; ++dirIt)
    {
        auto path = dirIt->path();
        if(path.string().find("_truth.ltm") != std::string::npos)
        {
            LocalTopoMap newMap;
            if(utils::load_serializable_from_file(path.string(), newMap))
            {
                maps.push_back(std::move(newMap));
            }
        }
    }

    if(maps.empty())
    {
        std::cerr << "ERROR: Failed to find any LTMs in " << argv[1] << '\n';
        return -1;
    }

    // Create separate vote grids for dest, decision, and path segment. All maps will place their votes in the
    // appropriate cell
    const auto& skeleton = maps.back().voronoiSkeleton();
    int width = skeleton.getWidthInCells();
    int height = skeleton.getHeightInCells();
    float scale = skeleton.metersPerCell();
    auto center = skeleton.getGlobalCenter();

    utils::CellGrid<int8_t> pathVotes(width, height, scale, center, 0);
    utils::CellGrid<int8_t> destVotes(width, height, scale, center, 0);
    utils::CellGrid<int8_t> decisionVotes(width, height, scale, center, 0);

    // For each map, tally the votes from each area
    for(auto& map : maps)
    {
        for(auto& area : map)
        {
            for(auto& cell : area->extent())
            {
                auto gridCell = utils::global_point_to_grid_cell_round(cell, pathVotes);

                switch(area->type())
                {
                case AreaType::path_segment:
                    ++pathVotes(gridCell.x, gridCell.y);
                    break;

                case AreaType::decision_point:
                    ++decisionVotes(gridCell.x, gridCell.y);
                    break;

                case AreaType::destination:
                    ++destVotes(gridCell.x, gridCell.y);
                    break;
                default:
                    // Ignore any others -- they shouldn't exit anyway
                    break;
                }
            }
        }
    }

    double colorPerVote = 255.0 / maps.size();  // amount to add to color channel for a vote
    // Create two images from these maps.
    // The consensus image shows how much agreement there is about the label for a cell.
    // White cells have 100% agreement, gray cells have no agreement
    // Black means no votes, i.e. obstacle or unknown

    Image conImg(width, height, Colorspace::MONO);

    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            // Find the max vote, which determines the consensus
            int maxVote = 0;
            if((pathVotes(x, y) > destVotes(x, y)) && pathVotes(x, y) > decisionVotes(x, y))
            {
                maxVote = pathVotes(x, y);
            }
            else if(destVotes(x, y) > decisionVotes(x, y))
            {
                maxVote = destVotes(x, y);
            }
            else
            {
                maxVote = decisionVotes(x, y);
            }
            // No vote = gray

            int consensus = maps.size() - maxVote;
            int color = std::min(255, static_cast<int>(consensus * colorPerVote));

            // Draw the boundary of the map as a black pixel
            if(skeleton.getClassification(x, y) & (SKELETON_CELL_OCCUPIED | SKELETON_CELL_FRONTIER))
            {
                color = 255;
            }
            else if(skeleton.getClassification(x, y) & SKELETON_CELL_UNKNOWN)
            {
                color = 127;
            }

            conImg.setPixel(x, height - y, 255 - color, 0, 0);
        }
    }

    std::string basename = argv[2];
    std::string conName = basename + "_consensus.pgm";
    vision::save_image_to_file(conImg, conName);

    Image voteImg(width, height, Colorspace::RGB);

    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            // Find the max vote, which determines the consensus
            int pathColor = std::min(255, static_cast<int>(pathVotes(x, y) * colorPerVote));
            int destColor = std::min(255, static_cast<int>(destVotes(x, y) * colorPerVote));
            int decisionColor = std::min(255, static_cast<int>(decisionVotes(x, y) * colorPerVote));

            // If we have votes, then change the color
            if(pathColor + destColor + decisionColor > 0)
            {
                voteImg.setPixel(x, height - y, destColor, pathColor, decisionColor);
            }
            // Draw the boundary in black
            else if(skeleton.getClassification(x, y) & (SKELETON_CELL_OCCUPIED | SKELETON_CELL_FRONTIER))
            {
                voteImg.setPixel(x, height - y, 0, 0, 0);
            }
            // Draw the rest in gray
            else
            {
                voteImg.setPixel(x, height - y, 127, 127, 127);
            }
        }
    }

    std::string voteName = basename + "_votes.ppm";
    vision::save_image_to_file(voteImg, voteName);

    return 0;
}
