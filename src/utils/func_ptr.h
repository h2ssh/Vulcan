/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     func_ptr.h
 * \author   Collin Johnson
 *
 * Definition of some helpful macros for dealing with function pointers and some examples of
 * the syntax, which I often forget.
 */

#ifndef UTILS_FUNC_PTR_H
#define UTILS_FUNC_PTR_H

#define CALL_MEMBER_FN(object, ptrToMember) ((object).*(ptrToMember))

#endif   // UTILS_FUNC_PTR_H
