/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     localizer.cpp
* \author   Collin Johnson
*
* Definition of create_localizer factory.
*/

#include <cassert>
#include <iostream>
#include "hssh/metrical/localization/localizer.h"

namespace vulcan
{
namespace hssh
{

boost::shared_ptr<Localizer> create_localizer(const std::string& localizerName, const localization_params_t& params)
{
//     if(localizerName == "mcl")
//     {
//         return boost::shared_ptr<Localizer>(new MCLLocalizer(params));
//     }
//     else
//     {
//         std::cerr<<"ERROR: localizerName matched no known localizers: "<<localizerName<<std::endl;
//         assert(false);
//     }
    
    return boost::shared_ptr<Localizer>();
}

}
}
