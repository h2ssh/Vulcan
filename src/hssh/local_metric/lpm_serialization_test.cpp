/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_metric/lpm.h"
#include "system/message_traits.h"
#include "system/serialized_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <cereal/archives/binary.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/stream.hpp>
#include <cassert>
#include <iostream>

using namespace vulcan;


int main(int argc, char** argv)
{
    hssh::LocalPerceptualMap lpm(100, 100, 0.05, Point<float>(0, 0), 200, 0);
    hssh::LocalPerceptualMap incoming;
    
    lpm.setTimestamp(1000);
    lpm.setId(10);
    
    system::serialized_t msg;
    
    msg.data.clear();
    
    // Put the archiving stream in a scope so the destructors will be called and automatically flush the buffered data into outMsg_.data
    {
        auto outDevice = boost::iostreams::back_inserter(msg.data);
        boost::iostreams::stream<decltype(outDevice)> outStream(outDevice);
        cereal::BinaryOutputArchive out(outStream);
        out << lpm;
    }
    
    std::cout << "Sending message of size:" << msg.data.size() << '\n';
    
    msg.size = msg.data.size();
    
    {
        boost::iostreams::array_source inputBuf(msg.data.data(), msg.data.size());
        boost::iostreams::stream<decltype(inputBuf)> input(inputBuf);
        cereal::BinaryInputArchive in(input);
        
        std::cout << "Received message of size:" << msg.data.size() << '\n';
        
        in >> incoming;
    }
    
    assert(lpm.getTimestamp() == incoming.getTimestamp());
    assert(lpm.getId() == incoming.getId());
    assert(lpm.cellsPerMeter() == incoming.cellsPerMeter());
    assert(lpm.metersPerCell() == incoming.metersPerCell());
    assert(lpm.getWidthInCells() == incoming.getWidthInCells());
    assert(lpm.getHeightInCells() == incoming.getHeightInCells());
    assert(lpm.getWidthInMeters() == incoming.getWidthInMeters());
    assert(lpm.getHeightInMeters() == incoming.getHeightInMeters());
    assert(lpm.getMaxCellCost() == incoming.getMaxCellCost());
    assert(lpm.getBoundary() == incoming.getBoundary());
    assert(lpm.getBottomLeft() == incoming.getBottomLeft());
    
    for(std::size_t y = 0; y < lpm.getHeightInCells(); ++y)
    {
        for(std::size_t x = 0; x < lpm.getWidthInCells(); ++x)
        {
            assert(lpm.getCost(Point<int>(x, y)) == incoming.getCost(Point<int>(x, y)));
            assert(lpm.getCellType(Point<int>(x, y)) == incoming.getCellType(Point<int>(x, y)));
        }
    }
    
    return 0;
}
