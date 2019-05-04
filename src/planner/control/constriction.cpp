/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// /**
// * \file     constriction.cpp
// * \author   Collin Johnson
// * 
// * Definition of Constriction, find_route_constrictions, and find_map_constrictions.
// */
// 
// #include <planner/control/constriction.h>
// #include <hssh/local_topological/voronoi_skeleton_grid.h>
// #include <hssh/types.h>
// 
// namespace vulcan 
// {
// namespace planner 
// {
// 
// Constrictions find_map_constrictions(const hssh::VoronoiSkeletonGrid& skeleton, double maxConstrictionWidth)
// {
//     
// }
// 
// 
// Constrictions find_route_constrictions(position_t start, 
//                                        position_t goal, 
//                                        const hssh::VoronoiSkeletonGrid& skeleton, 
//                                        const Constrictions& constrictions,
//                                        double minTraversableWidth)
// {
//     hssh::CellToTypeMap<const Constriction*> cellToConstriction;
//     cellToConstriction.reserve(constrictions.size());
//     for(auto& c : constrictions)
//     {
//         cellToConstriction[c.cell()] = &c;
//     }
// }
// 
// 
// Constriction::Constriction(Cell minimum, std::pair<Cell, Cell> boundary, hssh::VoronoiSkeletonGrid& skeleton)
// : position_(utils::grid_point_to_global_point(minimum, skeleton))
// , cell_(minimum)
// , boundary_(utils::grid_point_to_global_point(boundary.first, skeleton), 
//             utils::grid_point_to_global_point(boundary.second, skeleton))
// , width_(skeleton.getMetricDistance(cell_.x, cell_.y) * 2.0)
// {
//     auto sources = skeleton.getSourceCells(cell_);
//     normal_ = angle_sum(angle_to_point(sources[0], sources[1]), M_PI_2);
// }
// 
// } // namespace planner
// } // namespace vulcan
