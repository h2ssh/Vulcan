/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     stub.h
* \author   Collin Johnson
*
* Definition of macro for writing simple information about a stubbed method:
*
*   - PRINT_STUB : STUB! <method name>: file:line
*/

#ifndef UTILS_STUB_H
#define UTILS_STUB_H

#include <iostream>

#define PRINT_STUB(MethodName) \
    std::cout << "STUB! " << MethodName << ": " << __FILE__ << ':' << __LINE__ << std::endl;

#define PRINT_PRETTY_STUB() \
    std::cout << "STUB! " << __FUNCTION__ << ": " << __FILE__ << ':' << __LINE__ << std::endl;
    
#endif // UTILS_STUB_H
