/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     lpm_io.cpp
 * \author   Collin Johnson
 *
 * Definition of functions for loading/saving LPMs.
 */

#include "hssh/local_metric/lpm_io.h"
#include "hssh/local_metric/lpm.h"
#include "vision/image_utils.h"
#include <cassert>
#include <cereal/archives/binary.hpp>
#include <fstream>
#include <iostream>

namespace vulcan
{
namespace hssh
{

bool import_lpm_from_image(const std::string& filename,
                           const image_import_properties_t& properties,
                           LocalPerceptualMap& lpm)
{
    Image image;

    if (!vision::load_image_from_file(filename, image)) {
        std::cerr << "ERROR::import_lpm_from_image: Failed to load image: " << filename << '\n';
        return false;
    }

    if (image.getColorspace() != MONO) {
        std::cerr << "ERROR::import_lpm_from_image: Loaded image was not MONO: " << filename << '\n';
        return false;
    }

    lpm.setMetersPerCell(properties.scale);
    lpm.setGridSizeInCells(image.getWidth(), image.getHeight());
    lpm.reset();

    unsigned char pixelValue = 0;

    for (std::size_t y = 0; y < lpm.getHeightInCells(); ++y) {
        for (std::size_t x = 0; x < lpm.getWidthInCells(); ++x) {
            // Need to flip the y-axis of the image because the pgm files are being saved in left-handed coordinates
            assert(image.getPixel(x, image.getHeight() - y - 1, pixelValue, pixelValue, pixelValue));

            if (pixelValue < properties.occupiedThreshold) {
                lpm.setCostNoCheck(Point<int>(x, y), lpm.getMaxCellCost());
                lpm.setTypeNoCheck(Point<int>(x, y), kOccupiedOccGridCell);
            } else if (pixelValue > properties.freeThreshold) {
                lpm.setCostNoCheck(Point<int>(x, y), 0);
                lpm.setTypeNoCheck(Point<int>(x, y), kFreeOccGridCell);
            }
        }
    }

    return true;
}


/*
 * The version 1.0 format for the LPM:
 *
 * Binary file.
 *
 * width height scale centerX centerY mapId maxCost
 * occupancy grid
 * classification grid
 */

void save_lpm_1_0(const LocalPerceptualMap& lpm, const std::string& filename)
{
    std::ofstream out(filename.c_str(), std::ios_base::binary);
    assert(out.is_open());

    try {
        cereal::BinaryOutputArchive arOut(out);
        arOut << lpm;
    } catch (std::exception& e) {
        std::cerr << "EXCEPTION: Failed to save LPM to " << filename << " Reason:" << e.what() << '\n';
    }
}


void load_lpm_1_0(const std::string& filename, LocalPerceptualMap& lpm)
{
    std::ifstream in(filename.c_str(), std::ios_base::binary);

    if (!in.is_open()) {
        std::cerr << "ERROR:load_lpm: Failed to load LPM:" << filename << '\n';
        return;
    }

    try {
        cereal::BinaryInputArchive arIn(in);
        arIn >> lpm;
    } catch (std::exception& e) {
        std::cerr << "EXCEPTION: Failed to load LPM from " << filename << " Reason:" << e.what() << '\n';
    }
}


void save_lpm_occupancy_text(const LocalPerceptualMap& lpm, std::ofstream& stream)
{
    stream.seekp(0);

    for (size_t y = 0; y < lpm.getHeightInCells(); ++y) {
        for (size_t x = 0; x < lpm.getWidthInCells(); ++x) {
            int cell = (lpm.getCellTypeNoCheck(x, y) & kOccupiedOccGridCell) ? 1 : 0;
            stream << cell << ' ';
        }

        stream << '\n';
    }
}

}   // namespace hssh
}   // namespace vulcan
