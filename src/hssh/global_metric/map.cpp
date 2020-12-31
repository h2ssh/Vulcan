/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     map.cpp
 * \author   Collin Johnson
 *
 * Definition of GlobalMetricMap.
 */

#include "hssh/global_metric/map.h"
#include <cereal/archives/portable_binary.hpp>
#include <fstream>

namespace vulcan
{
namespace hssh
{

int32_t GlobalMetricMap::nextId_ = 0;


GlobalMetricMap::GlobalMetricMap(void) : id_(-1), name_("default constructed")
{
}


GlobalMetricMap::GlobalMetricMap(const std::string& name, const OccupancyGrid& map)
: OccupancyGrid(map)
, id_(nextId_++)
, name_(name)
{
}


bool GlobalMetricMap::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename, std::ios_base::binary);

    if (out.good()) {
        cereal::PortableBinaryOutputArchive ar(out);
        ar(*this);
    } else {
        std::cerr << "ERROR: GlobalMetricMap::saveToFile(" << filename << ")\n";
    }

    return out.good();
}


bool GlobalMetricMap::loadFromFile(const std::string& filename)
{
    std::ifstream in(filename, std::ios_base::binary);

    if (in.good()) {
        cereal::PortableBinaryInputArchive ar(in);
        ar(*this);
    } else {
        std::cerr << "ERROR: GlobalMetricMap::loadFromFile(" << filename << ")\n";
    }

    return in.good();
}

}   // namespace hssh
}   // namespace vulcan
