/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/areas/serialization.h>
#include <utils/cell_grid_utils.h>
#include <utils/serialized_file_io.h>
#include <core/image.h>
#include <vision/image_utils.h>
#include <iostream>

using namespace vulcan;
using namespace vulcan::hssh;


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << "Expected usage: ltm_to_ppm 'ltm_file'\n";
        return -1;
    }

    // Iterate through the directory and find all maps containing "_truth.ltm"
    LocalTopoMap map;

    if(!utils::load_serializable_from_file(argv[1], map))
    {
        std::cerr << "ERROR: Failed to load requested LTM: " << argv[1] << '\n';
    }

    // Create separate vote grids for dest, decision, and path segment. All maps will place their votes in the
    // appropriate cell
    const VoronoiSkeletonGrid& grid = map.voronoiSkeleton();
    int width = grid.getWidthInCells();
    int height = grid.getHeightInCells();

    // Create two images from these maps.
    // One image will show just the occupancy grid that is used
    // The other image overlays the label on top of the the free space cells

    Image mapImg(width, height, Colorspace::MONO);
    Image ltmImg(width, height, Colorspace::RGB);

    // Fill the images with the underlying map first
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            if(grid.getClassification(x, y) & SKELETON_CELL_FREE)
            {
                mapImg.setPixel(x, height - y, 255, 255, 255);
                ltmImg.setPixel(x, height - y, 255, 255, 255);
            }
            else if(grid.getClassification(x, y) & SKELETON_CELL_OCCUPIED)
            {
                mapImg.setPixel(x, height - y, 0, 0, 0);
                ltmImg.setPixel(x, height - y, 0, 0, 0);
            }
            else
            {
                mapImg.setPixel(x, height - y, 127, 127, 127);
                ltmImg.setPixel(x, height - y, 127, 127, 127);
            }
        }
    }

    // For each area cell, now mark the RGB value for the associated type
    for(auto& area : map)
    {
        for(auto& cell : area->extent())
        {
            auto gridCell = utils::global_point_to_grid_cell_round(cell, grid);

            switch(area->type())
            {
                case AreaType::path_segment:
                    ltmImg.setPixel(gridCell.x, height - gridCell.y, 24, 145, 0);
                    break;

                case AreaType::decision_point:
                    ltmImg.setPixel(gridCell.x, height - gridCell.y, 7, 55, 237);
                    break;

                case AreaType::destination:
                    ltmImg.setPixel(gridCell.x, height - gridCell.y, 224, 6, 6);
                    break;
                default:
                    // Ignore any others -- they shouldn't exit anyway
                    break;
            }
        }
    }

    std::string basename = argv[1];
    auto pos = basename.rfind(".ltm");
    basename.erase(pos, std::string::npos);
    std::string mapName = basename + "_map.pgm";
    std::string ltmName = basename + ".ppm";
    vision::save_image_to_file(mapImg, mapName);
    vision::save_image_to_file(ltmImg, ltmName);

    return 0;
}
