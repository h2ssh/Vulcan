/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     lpm_io_test.cpp
 * \author   Collin Johnson
 *
 * A test program for the LPM IO functions. An LPM is randomly generated, saved, and then loaded to see
 * that all values are correctly loaded.
 */

#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/lpm_io.h"
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iostream>

using namespace vulcan::hssh;


LocalPerceptualMap generate_lpm(uint16_t width, uint16_t height);
void compare_lpms(const LocalPerceptualMap& generated, const LocalPerceptualMap& loaded);


int main(int argc, char** argv)
{
    srand48(time(0));

    LocalPerceptualMap generated = generate_lpm(250, 250);
    LocalPerceptualMap loaded;

    save_lpm_1_0(generated, argv[1]);
    //     load_lpm_1_0("lpm_io_test.lpm", loaded);
    //     load_lpm_1_0(argv[1], loaded);

    //     compare_lpms(generated, loaded);

    return 0;
}


LocalPerceptualMap generate_lpm(uint16_t width, uint16_t height)
{
    LocalPerceptualMap generated(width, height, 0.05, vulcan::Point<float>(drand48(), drand48()), 200, 0);

    generated.setId(513);

    vulcan::Point<uint16_t> cell;

    for (cell.y = 0; cell.y < height; ++cell.y) {
        for (cell.x = 0; cell.x < width; ++cell.x) {
            generated.setCostNoCheck(cell, drand48() * 255);
        }
    }

    for (cell.y = 0; cell.y < height; ++cell.y) {
        for (cell.x = 0; cell.x < width; ++cell.x) {
            generated.setTypeNoCheck(cell, drand48() * 255);
        }
    }

    return generated;
}


void compare_lpms(const LocalPerceptualMap& generated, const LocalPerceptualMap& loaded)
{
    assert(generated.getWidthInCells() == loaded.getWidthInCells());
    assert(generated.getHeightInCells() == loaded.getHeightInCells());
    assert(generated.metersPerCell() == loaded.metersPerCell());
    assert(generated.getGlobalCenter() == loaded.getGlobalCenter());
    assert(generated.getMaxCellCost() == loaded.getMaxCellCost());
    assert(generated.getId() == loaded.getId());

    for (uint16_t y = 0, height = generated.getHeightInCells(); y < height; ++y) {
        for (uint16_t x = 0, width = generated.getWidthInCells(); x < width; ++x) {
            assert(generated.getCostNoCheck(x, y) == loaded.getCostNoCheck(x, y));
            assert(generated.getCellTypeNoCheck(x, y) == loaded.getCellTypeNoCheck(x, y));
        }
    }
}
