/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <istream>
#include <ostream>
#include "vision/homography.h"


using namespace vulcan;


std::istream& vulcan::vision::operator>>(std::istream& in, homography_matrix_t& matrix)
{
    in>>matrix.wi00>>matrix.wi01>>matrix.wi02
      >>matrix.wi10>>matrix.wi11>>matrix.wi12
      >>matrix.wi20>>matrix.wi21>>matrix.wi22
      >>matrix.iw00>>matrix.iw01>>matrix.iw02
      >>matrix.iw10>>matrix.iw11>>matrix.iw12
      >>matrix.iw20>>matrix.iw21>>matrix.iw22;
      
    return in;
}


std::ostream& vulcan::vision::operator<<(std::ostream& out, const homography_matrix_t& matrix)
{
    out<<matrix.wi00<<' '<<matrix.wi01<<' '<<matrix.wi02<<'\n'
       <<matrix.wi10<<' '<<matrix.wi11<<' '<<matrix.wi12<<'\n'
       <<matrix.wi20<<' '<<matrix.wi21<<' '<<matrix.wi22<<'\n'
       <<matrix.iw00<<' '<<matrix.iw01<<' '<<matrix.iw02<<'\n'
       <<matrix.iw10<<' '<<matrix.iw11<<' '<<matrix.iw12<<'\n'
       <<matrix.iw20<<' '<<matrix.iw21<<' '<<matrix.iw22<<'\n';
       
    return out;
}


Point<float> vulcan::vision::world_to_image_coordinates(const Point<float>& world, const homography_matrix_t& homography)
{
    double xTrans = (world.x * homography.wi00) + (world.y * homography.wi01) + homography.wi02;
    double yTrans = (world.x * homography.wi10) + (world.y * homography.wi11) + homography.wi12;
    double zTrans = (world.x * homography.wi20) + (world.y * homography.wi21) + homography.wi22;
    
    return Point<float>(xTrans / zTrans, yTrans / zTrans);
}


Point<float> vulcan::vision::image_to_world_coordinates(const Point<int16_t>& image, const homography_matrix_t& homography)
{
    double xTrans = (image.x * homography.iw00) + (image.y * homography.iw01) + homography.iw02;
    double yTrans = (image.x * homography.iw10) + (image.y * homography.iw11) + homography.iw12;
    double zTrans = (image.x * homography.iw20) + (image.y * homography.iw21) + homography.iw22;
    
    return Point<float>(xTrans / zTrans, yTrans / zTrans);
}
