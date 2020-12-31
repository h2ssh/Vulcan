/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// /**
// * \file     lpm_utils.cpp
// * \author   Collin Johnson
// *
// * Definition of the following functions:
// *
// *   - change_lpm_reference_frame
// */
//
// #include "hssh/utils/lpm_utils.h"
// #include "hssh/local_metric/lpm.h"
// #include "core/pose.h"
//
// #define DEBUG_CHANGE_FRAME
//
// #ifdef DEBUG_CHANGE_FRAME
// #include <iostream>
// #endif
//
// namespace vulcan
// {
// namespace hssh
// {
//
// LocalPerceptualMap change_lpm_reference_frame(const LocalPerceptualMap& lpm, const pose_t& newReference)
// {
//     /*
//     * Need to find the new width and height of the LPM after rotation. Find the radius of the LPM.
//     * Take the angle of the radius and rotate it by rotationAngle to find where the radius cell lies in the
//     * rotated LPM. These will be the maxima to determine the width and height of the new LPM.
//     */
//
//     float rotationAngle = -newReference.theta;
//
//     float halfWidth  = lpm.getWidthInCells() / 2.0f;
//     float halfHeight = lpm.getHeightInCells() / 2.0f;
//
//     float radiusAngle = atan2(halfHeight, halfWidth);
//     float radius      = ceil(sqrt(halfWidth*halfWidth + halfHeight*halfHeight));
//
//     int newHalfWidth  = ceil(radius * std::max(std::abs(cos(radiusAngle + rotationAngle)), std::abs(cos(-radiusAngle
//     + rotationAngle)))); int newHalfHeight = ceil(radius * std::max(std::abs(sin(radiusAngle + rotationAngle)),
//     std::abs(sin(-radiusAngle + rotationAngle))));
//
// #ifdef DEBUG_CHANGE_FRAME
//     std::cout<<"INFO:change_lpm_reference_frame:Old dim:("<<halfWidth<<','<<halfHeight<<") New
//     dim:("<<newHalfWidth<<','<<newHalfHeight<<")\n";
// #endif
//
//     LPM rotated(lpm);
//     rotated.setGridSizeInCells(2*newHalfWidth, 2*newHalfHeight);
//     rotated.reset();
//
//     Point<float> originalCell;
//     Point<float> rotatedCell;
//
//     int centerX = lpm.getWidthInCells() / 2;
//     int centerY = lpm.getHeightInCells() / 2;
//
//     // Go through each cell in the rotated grid to ensure no holes crop up in the new image rather than only
//     converting
//     // the old lpm cell-by-cell into the new one.
//     for(uint16_t y = 0; y < rotated.getHeightInCells(); ++y)
//     {
//         rotatedCell.y = y - newHalfHeight;
//
//         for(uint16_t x = 0; x < rotated.getWidthInCells(); ++x)
//         {
//             rotatedCell.x  = x - newHalfWidth;
//             originalCell   = rotate(rotatedCell, -rotationAngle);
//             originalCell.x += centerX;
//             originalCell.y += centerY;
//
//             if(lpm.isCellInGrid(originalCell))
//             {
//                 rotated.setCost(Point<int>(x, y), lpm.getCost(originalCell));
//                 rotated.setType(Point<int>(x, y), lpm.getCellType(originalCell));
//             }
//         }
//     }
//
//     // After copying the cells, need to adjust the bottom left corner to be in the new reference frame
//     Point<float> oldCenter = lpm.getGlobalCenter();
//
//     // Use the cell dimensions/2 in order to make sure the bottom left is an integral number of cells from the
//     center, which it needs to be rotated.setBottomLeft(Point<float>(oldCenter.x -
//     newHalfWidth*rotated.metersPerCell()  - newReference.x,
//                                              oldCenter.y - newHalfHeight*rotated.metersPerCell() - newReference.y));
//
// #ifdef DEBUG_CHANGE_FRAME
//     std::cout<<"INFO:change_lpm_reference_frame:Old BL:"<<lpm.getBottomLeft()<<" New
//     BL:"<<rotated.getBottomLeft()<<'\n';
// #endif
//
//     return rotated;
// }
//
// }
// }
