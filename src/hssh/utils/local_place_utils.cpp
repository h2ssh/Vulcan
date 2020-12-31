/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_place_utils.cpp
 * \author   Collin Johnson
 *
 * Definition of utility functions for local places:
 *
 *   LocalPlace rotate_local_place(LocalPlace, theta)
 */

#include "hssh/utils/local_place_utils.h"
#include "hssh/local_topological/local_place.h"

namespace vulcan
{
namespace hssh
{

LPM rotate_lpm(const LocalPerceptualMap& lpm, float angle);
SmallScaleStar rotate_star(const SmallScaleStar& star, float angle);
std::vector<Gateway> rotate_gateways(const std::vector<Gateway>& gateways, float angle);
place_extent_t rotate_extent(const place_extent_t& extent, float angle);
Gateway rotate_gateway(const Gateway& gateway, float angle);

LocalPlace transform_local_place(const LocalPlace& place, uint32_t newId, const pose_t& transform)
{
    return LocalPlace(newId,
                      rotate_gateways(place.getGateways(), transform.theta),
                      rotate_star(place.getStar(), transform.theta),
                      rotate_lpm(place.getLPM(), transform.theta),
                      rotate_extent(place.extent(), transform.theta),
                      pose_t(transform.x, transform.y, 0.0f));
}


LPM rotate_lpm(const LocalPerceptualMap& lpm, float angle)
{
    /*
     *  Need to find the new width and height of the LPM after rotation. Find the radius of the LPM.
     * Take the angle of the radius and rotate it by rotation angle to find where the radius cell lies in the
     * rotated LPM. These will be the maxima to determine the width and height of the new LPM.
     */


    float halfWidth = lpm.getWidthInCells() / 2.0f;
    float halfHeight = lpm.getHeightInCells() / 2.0f;

    float radiusAngle = atan2(halfHeight, halfWidth);
    float radius = ceil(sqrt(halfWidth * halfWidth + halfHeight * halfHeight));

    float newHalfWidth =
      ceil(radius * std::max(std::abs(cos(radiusAngle + angle)), std::abs(cos(-radiusAngle + angle))));
    float newHalfHeight =
      ceil(radius * std::max(std::abs(sin(radiusAngle + angle)), std::abs(sin(-radiusAngle + angle))));

    LPM rotated(lpm);
    rotated.setGridSizeInCells(2 * newHalfWidth, 2 * newHalfHeight);

    Point<int> originalCell;
    Point<int> rotatedCell;

    for (uint16_t y = 0; y < lpm.getHeightInCells(); ++y) {
        originalCell.y = y - halfHeight;

        for (x = 0; x < lpm.getWidthInCells(); ++x) {
            originalCell.x = x - halfWidth;
            rotatedCell = rotate(originalCell, angle);
            rotatedCell.x += newHalfWidth;
            rotatedCell.y += newHalfHeight;

            Point<uint16_t> shifted(rotatedCell.x + newHalfWidth, rotatedCell.y + newHalfHeight);

            rotated.setCost(shifted, lpm.getCostNoCheck(x, y));
            rotated.setType(shifted, lpm.getCellTypeNoCheck(x, y));
        }
    }

    return rotated;
}


SmallScaleStar rotate_star(const SmallScaleStar& star, float angle)
{
    std::vector<local_path_fragment_t> rotated = star.getAllFragments();

    std::for_each(rotated.begin(), rotated.end(), [angle](local_path_fragment_t& fragment) {
        fragment.gateway = rotate_gateway(fragment.gateway, angle);
    });

    return SmallScaleStar(rotated);
}


std::vector<Gateway> rotate_gateways(const std::vector<Gateway>& gateways, float angle)
{
    std::vector<Gateway> rotated(gateways);

    std::for_each(rotated.begin(), rotated.end(), [angle](Gateway& gateway) {
        gateway = rotate_gateway(gateway, angle);
    });

    return rotated;
}


place_extent_t rotate_extent(const place_extent_t& extent, float angle)
{
    place_extent_t rotated(extent);

    rotated.boundary.bottomLeft = rotate(extent.boundary.bottomLeft, angle);
    rotated.boundary.bottomRight = rotate(extent.boundary.bottomRight, angle);
    rotated.boundary.topLeft = rotate(extent.boundary.topLeft, angle);
    rotated.boundary.topRight = rotate(extent.boundary.topRight, angle);

    return rotated;
}


Gateway rotate_gateway(const Gateway& gateway, float angle)
{
    return Gateway(gateway.id(),
                   Line<double>(rotate(gateway.boundary.a, angle), rotate(gateway.boundary.b, angle)),
                   rotate(gateway.center(), angle),
                   angle_sum(gateway.direction, angle));
}

}   // namespace hssh
}   // namespace vulcan
