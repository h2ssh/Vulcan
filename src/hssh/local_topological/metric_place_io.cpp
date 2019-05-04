/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_place_io.cpp
* \author   Collin Johnson
*
* Definition of create_metric_place_io factory function.
*/

#include <cassert>
#include <iostream>
#include <hssh/local_topological/metric_place_io.h>

namespace vulcan
{
namespace hssh
{

boost::shared_ptr<MetricPlaceIO> create_metric_place_io(const std::string& type,
                                                        const std::string& directory,
                                                        const std::string& basename)
{
    std::cerr<<"ERROR: create_metric_place_io: Unknown type: "<<type<<std::endl;
    assert(false);
    
    return boost::shared_ptr<MetricPlaceIO>();
}
    
}
}
