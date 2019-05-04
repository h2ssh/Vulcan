/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     voronoi_isovist_gradients_renderer.cpp
* \author   Collin Johnson
*
* Definition of VoronoiIsovistGradientsRenderer.
*/

#include <ui/components/voronoi_isovist_gradients_renderer.h>
#include <ui/common/gl_shapes.h>
#include <hssh/local_topological/area_detection/gateways/isovist_gradients.h>
#include <hssh/local_topological/area_detection/gateways/isovist_maxima.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>

namespace vulcan
{
namespace ui
{
    
using PositionIter = hssh::VoronoiIsovistGradients::GradientConstIter;
using MaximumIter  = hssh::VoronoiIsovistMaxima::ConstMaximaIter;
    
void convert_position_values_to_vertices(PositionIter                     begin,
                                         PositionIter                     end,
                                         const hssh::VoronoiSkeletonGrid& grid,
                                         const LinearColorInterpolator&         interpolator,
                                         std::vector<GLfloat>&            vertices,
                                         std::vector<GLfloat>&            colors);

void convert_local_maxima_to_vertices(MaximumIter                      begin,
                                      MaximumIter                      end,
                                      const hssh::VoronoiSkeletonGrid& grid,
                                      const LinearColorInterpolator&         interpolator,
                                      std::vector<GLfloat>&            vertices,
                                      std::vector<GLfloat>&            colors);
void convert_probabilities_to_vertices(const hssh::CellToTypeMap<double>& probabilities,
                                       const hssh::VoronoiSkeletonGrid& grid,
                                      const LinearColorInterpolator& interpolator,
                                      std::vector<GLfloat>& vertices,
                                      std::vector<GLfloat>& colors);

void add_point_to_vertices(const Point<float>& point, float size, const GLColor& color, std::vector<GLfloat>& vertices, std::vector<GLfloat>& colors);

void draw_gl_quads(const std::vector<GLfloat>& vertices, const std::vector<GLfloat>& colors);


void VoronoiIsovistGradientsRenderer::setRenderColors(const GLColor& localMaximaColor, const std::vector<GLColor>& gradientColors)
{
    maximaColor_ = localMaximaColor;
    interpolator_.setColors(gradientColors);
}


void VoronoiIsovistGradientsRenderer::setGradients(const hssh::VoronoiIsovistGradients& gradients,
                                                   const hssh::VoronoiIsovistMaxima&    maxima,
                                                   const hssh::VoronoiSkeletonGrid&     grid)
{
    convert_position_values_to_vertices(gradients.beginGradients(),
                                        gradients.endGradients(),
                                        grid,
                                        interpolator_,
                                        cellGradientVertices_,
                                        cellGradientColors_);
    
    convert_local_maxima_to_vertices(maxima.begin(),
                                     maxima.end(),
                                     grid,
                                     interpolator_,
                                     localMaximaVertices_,
                                     localMaximaColors_);
}


void VoronoiIsovistGradientsRenderer::setProbabilities(const hssh::CellToTypeMap<double>& probabilities,
                                                       const hssh::VoronoiSkeletonGrid& grid)
{
    convert_probabilities_to_vertices(probabilities,
                                      grid,
                                      interpolator_,
                                      probabilitiesVertices_,
                                      probabilitiesColors_);
}


void VoronoiIsovistGradientsRenderer::renderCellGradients(void) const
{
    draw_gl_quads(cellGradientVertices_, cellGradientColors_);
}


void VoronoiIsovistGradientsRenderer::renderLocalMaxima(void) const
{
    draw_gl_quads(localMaximaVertices_, localMaximaColors_);
}


void VoronoiIsovistGradientsRenderer::renderProbabilities(void) const
{
    draw_gl_quads(probabilitiesVertices_, probabilitiesColors_);
}


void convert_position_values_to_vertices(PositionIter                     begin,
                                         PositionIter                     end,
                                         const hssh::VoronoiSkeletonGrid& grid,
                                         const LinearColorInterpolator&   interpolator,
                                         std::vector<GLfloat>&            vertices,
                                         std::vector<GLfloat>&            colors)
{
    double minValue    = std::min_element(begin, end)->value;
    double rangeValues = std::max_element(begin, end)->value - minValue;
    
    if(rangeValues == 0.0)
    {
        rangeValues = 1.0;
    }
    
    vertices.clear();
    colors.clear();
    vertices.reserve(std::distance(begin, end) * 8);        // two floats per vertex, four vertices per cell
    colors.reserve(std::distance(begin, end) * 16);         // RGBA
    
    for(auto pvIt = begin; pvIt != end; ++pvIt)
    {
        auto color    = interpolator.calculateColor((pvIt->value - minValue) / rangeValues);
        auto position = utils::grid_point_to_global_point(pvIt->position, grid);
        
        add_point_to_vertices(position, grid.metersPerCell(), color, vertices, colors);
    }
}


void convert_local_maxima_to_vertices(MaximumIter                      begin,
                                      MaximumIter                      end,
                                      const hssh::VoronoiSkeletonGrid& grid,
                                      const LinearColorInterpolator&   interpolator,
                                      std::vector<GLfloat>&            vertices,
                                      std::vector<GLfloat>&            colors)
{
    float minValue    = std::min_element(begin, end)->maximum.value;
    float rangeValues = std::sqrt(std::max_element(begin, end)->maximum.value - minValue);

    if(rangeValues == 0.0)
    {
        rangeValues = 1.0;
    }

    vertices.clear();
    colors.clear();
    vertices.reserve(std::distance(begin, end) * 8);        // two floats per vertex, four vertices per cell
    colors.reserve(std::distance(begin, end) * 16);         // RGBA

    for(auto maxIt = begin; maxIt != end; ++maxIt)
    {
        auto color = interpolator.calculateColor(std::sqrt((maxIt->maximum.value-minValue)) / rangeValues);
        
        for(auto cell : maxIt->skeletonCells)
        {
            auto position = utils::grid_point_to_global_point(cell.position, grid);

            add_point_to_vertices(position, grid.metersPerCell(), color, vertices, colors);
        }
    }
}


void convert_probabilities_to_vertices(const hssh::CellToTypeMap<double>& probabilities,
                                       const hssh::VoronoiSkeletonGrid& grid,
                                       const LinearColorInterpolator& interpolator,
                                       std::vector<GLfloat>& vertices,
                                       std::vector<GLfloat>& colors)
{
    vertices.clear();
    colors.clear();
    vertices.reserve(probabilities.size() * 8);        // two floats per vertex, four vertices per cell
    colors.reserve(probabilities.size() * 16);         // RGBA
    
    for(auto& p : probabilities)
    {
        auto color = interpolator.calculateColor(p.second);
        add_point_to_vertices(utils::grid_point_to_global_point(p.first, grid),
                              grid.metersPerCell(),
                              color,
                              vertices,
                              colors);
    }
}


void add_point_to_vertices(const Point<float>& point, float size, const GLColor& color, std::vector<GLfloat>& vertices, std::vector<GLfloat>& colors)
{
    // bottom left corner
    vertices.push_back(point.x);
    vertices.push_back(point.y);
    // top left corner
    vertices.push_back(point.x);
    vertices.push_back(point.y + size);
    // top right corner
    vertices.push_back(point.x + size);
    vertices.push_back(point.y + size);
    // bottom right corner
    vertices.push_back(point.x + size);
    vertices.push_back(point.y);

    // RGBA
    colors.push_back(color.red());
    colors.push_back(color.green());
    colors.push_back(color.blue());
    colors.push_back(color.alpha());

    colors.push_back(color.red());
    colors.push_back(color.green());
    colors.push_back(color.blue());
    colors.push_back(color.alpha());

    colors.push_back(color.red());
    colors.push_back(color.green());
    colors.push_back(color.blue());
    colors.push_back(color.alpha());

    colors.push_back(color.red());
    colors.push_back(color.green());
    colors.push_back(color.blue());
    colors.push_back(color.alpha());
}


void draw_gl_quads(const std::vector<GLfloat>& vertices, const std::vector<GLfloat>& colors)
{
    glDisableClientState(GL_EDGE_FLAG_ARRAY);  // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    
    glVertexPointer(2, GL_FLOAT, 0, vertices.data());
    glColorPointer(4, GL_FLOAT, 0, colors.data());
    
    glDrawArrays(GL_QUADS, 0, vertices.size() / 2);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    
    glEnd();
}

} // namespace hssh
} // namespace vulcan
