/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     strided_sequence.h
* \author   Collin Johnson
* 
* Definition of utility functions for dealing iterating through sequences using a stride:
*   
*   - strided_sequence_length : how many elements are going to be in the strided sequence?
*/

#ifndef UTILS_STRIDED_SEQUENCE_H
#define UTILS_STRIDED_SEQUENCE_H

namespace vulcan
{
namespace utils
{

/**
* strided_sequence_length calculates the number of elements in the strided sequence given some
* initial sequence and a stride through the sequence.
* 
* The logic is:
* 
*   - if sequenceLength % stride == 0, stridedLength = sequenceLength / stride
*   - else stridedLength = sequenceLength / stride + 1
* 
* Example:
*   Consider sequences of length 4, 5, and 6 with a stride of 3:
*   
*   (0 1 2 3)     -> strided: (0 3)
*   (0 1 2 3 4)   -> strided: (0 3)
*   (0 1 2 3 4 5) -> strided: (0 3)
* 
* \param    sequenceLength      Length of the sequence to be iterated through with a stride
* \param    stride              Stride to use for the iteration
* \pre  stride > 0
* \return   Number of elements in the strided sequence.
*/
inline int strided_sequence_length(int sequenceLength, int stride)
{
    int stridedLength = sequenceLength / stride;
    // if integer truncation occurred, the product of quotient * divisor won't equal the dividend
    return (stridedLength * stride != sequenceLength) ? stridedLength + 1 : stridedLength;
}

}
}

#endif // UTILS_STRIDED_SEQUENCE_H
