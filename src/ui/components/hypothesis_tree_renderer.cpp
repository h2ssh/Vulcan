/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_tree_renderer.cpp
* \author   Collin Johnson
*
* Definition of HypothesisTreeRenderer.
*/

#include "ui/components/hypothesis_tree_renderer.h"
#include "hssh/global_topological/debug/hypothesis_tree.h"
#include <iostream>
#include <GL/gl.h>

// #define DEBUG_SLOTS

namespace vulcan
{
namespace ui
{

HypothesisTreeRenderer::HypothesisTreeRenderer(void)
: tree_(std::make_unique<hssh::HypothesisTree>())
{
}


HypothesisTreeRenderer::~HypothesisTreeRenderer(void)
{
    // For std::unique_ptr
}


void HypothesisTreeRenderer::setRenderColors(const GLColor& nodeColor, const GLColor& edgeColor)
{
    nodeColor_ = nodeColor;
    edgeColor_ = edgeColor;
}


void HypothesisTreeRenderer::setHypothesisTree(const hssh::HypothesisTree& tree)
{
    *tree_ = tree;
    extractNodesFromTree();

    if(!nodes_.empty())
    {
        assignVerticesToNodes();
        allocateGLBuffers();
        assignNodeColors();
        assignEdges();
        findTreeBoundary();
    }
}


hssh::Id HypothesisTreeRenderer::nodeClosestToPoint(const Point<float>& point) const
{
    if(nodes_.empty())
    {
        return 0;
    }

    hssh::Id closest = 0;
    float    minDistance = distance_between_points(nodes_[0]->pixelCenter, point);

    for(auto nodeIt = nodes_.begin(), nodeEnd = nodes_.end(); nodeIt != nodeEnd; ++nodeIt)
    {
        float nodeDistance = distance_between_points((*nodeIt)->pixelCenter, point);

        if(nodeDistance < minDistance)
        {
            minDistance = nodeDistance;
            closest     = (*nodeIt)->id;
        }
    }

    return closest;
}


void HypothesisTreeRenderer::highlightHypothesis(uint32_t index, const GLColor& color)
{
    hypothesisColors_[index] = color;
    changeNodeColor(index, color);
}


void HypothesisTreeRenderer::cancelHighlight(uint32_t index)
{
    auto colorIt = hypothesisColors_.find(index);

    if(colorIt != hypothesisColors_.end())
    {
        hypothesisColors_.erase(colorIt);
        changeNodeColor(index, nodeColor_);
    }
}


void HypothesisTreeRenderer::renderTree(void)
{
    // Need a tree to actually render
    if(nodes_.empty())
    {
        return;
    }

    glDisableClientState(GL_EDGE_FLAG_ARRAY);  // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnableClientState(GL_VERTEX_ARRAY);

    edgeColor_.set();
    glLineWidth(2.0f);
    glVertexPointer(2, GL_FLOAT, 0, edges_.data());
    glDrawArrays(GL_LINES, 0, (nodes_.size()-1)*2);
    glEnd();

    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, vertices_.data());
    glColorPointer(4, GL_FLOAT, 0, vertexColors_.data());

    // Transform is to rotate the points and then translate to appropriate global position

    glPointSize(4.0f);
    glDrawArrays(GL_POINTS, 0, nodes_.size());

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnd();
}


void HypothesisTreeRenderer::extractNodesFromTree(void)
{
    nodePool_.reset();
    nodes_.clear();

    const hssh::hypothesis_tree_node_t* root = tree_->getRootNode();
    if(root)
    {
        tree_node_t* rootNode = nodePool_.newObject();

        rootNode->parent = 0;
        expandHypothesisNode(*root, rootNode);
    }
}


void HypothesisTreeRenderer::expandHypothesisNode(const hssh::hypothesis_tree_node_t& hypNode, tree_node_t* treeNode)
{
    // If no children, then a leaf node and thus breadth is 1 because this is the base case of the recursion
    nodes_.push_back(treeNode);
    treeNode->breadth      = hypNode.children.empty() ? 1 : 0;
    treeNode->id = hypNode.id;
    treeNode->children.resize(hypNode.children.size());

    for(size_t n = 0; n < hypNode.children.size(); ++n)
    {
        tree_node_t* childNode = nodePool_.newObject();
        childNode->parent      = treeNode;
        expandHypothesisNode(*tree_->getNode(hypNode.children[n]), childNode);

        treeNode->breadth     += childNode->breadth;
        treeNode->children[n]  = childNode;
    }

#ifdef DEBUG_SLOTS
    std::cout<<"DEBUG:HypothesisTree:Node "<<treeNode->id<<" Breadth:"<<treeNode->breadth<<" Children:"<<hypNode.children.size()<<'\n';
#endif
}


void HypothesisTreeRenderer::assignVerticesToNodes(void)
{
    // Each node in the tree gets a slot based on the breadth of the tree below it.
    treeBreadth_ = nodes_[0]->breadth;
    treeDepth_   = 0;

    assignTreeSlots(nodes_[0], 0, treeBreadth_, treeDepth_);

    // Once all nodes are assigned a vertex, adjust the x-value to make the tree more balanced when rendering
    // If too fat, the x-values will stretch it, if too skinny, they'll move in a bit

    float depthRatio = (treeDepth_ == 0) ? 1.0f : std::max(1.0f, static_cast<float>(treeBreadth_) / treeDepth_);

    for(size_t n = 0; n < nodes_.size(); ++n)
    {
        nodes_[n]->pixelCenter.x *= depthRatio;
    }
}


void HypothesisTreeRenderer::assignTreeSlots(tree_node_t* node, int slotStart, int slotEnd, int depth)
{
    if(depth > treeDepth_)
    {
        treeDepth_ = depth;
    }

    // Center each node within its slot
    node->pixelCenter.x = depth;
    node->pixelCenter.y = slotStart + (slotEnd-slotStart-1) / 2;

#ifdef DEBUG_SLOTS
    std::cout<<"DEBUG:HypothesisTree:Node "<<node->id<<" Breadth:"<<node->breadth<<" Slot:"<<node->pixelCenter<<" Children:"<<node->children.size()<<'\n';
#endif

    for(size_t n = 0; n < node->children.size(); ++n)
    {
        assignTreeSlots(node->children[n], slotStart, slotStart + node->children[n]->breadth, depth+1);

        slotStart += node->children[n]->breadth;
    }
}


void HypothesisTreeRenderer::allocateGLBuffers(void)
{
    // If more nodes than currently allocated vertices_, reallocate all buffers to fit the larger tree
    std::size_t numVertices = nodes_.size() * 2;
    if(vertices_.size() < numVertices)
    {
        vertices_.resize(numVertices);
        edges_.resize(numVertices * 2);
        vertexColors_.resize(numVertices * 2);
        edgeColors_.resize(numVertices * 4);
    }
}


void HypothesisTreeRenderer::assignNodeColors(void)
{
    GLColor color;

    for(size_t n = 0; n < nodes_.size(); ++n)
    {
        color = findNodeColor(nodes_[n]);

        vertices_[2*n]     = nodes_[n]->pixelCenter.x;
        vertices_[2*n + 1] = nodes_[n]->pixelCenter.y;

        vertexColors_[4*n]     = color.red();
        vertexColors_[4*n + 1] = color.green();
        vertexColors_[4*n + 2] = color.blue();
        vertexColors_[4*n + 3] = color.alpha();
    }
}


void HypothesisTreeRenderer::assignEdges(void)
{
    size_t nextEdge = 0;

    for(size_t n = 0; n < nodes_.size(); ++n)
    {
        for(size_t i = 0; i < nodes_[n]->children.size(); ++i)
        {
            edges_[4*nextEdge]     = nodes_[n]->pixelCenter.x;
            edges_[4*nextEdge + 1] = nodes_[n]->pixelCenter.y;
            edges_[4*nextEdge + 2] = nodes_[n]->children[i]->pixelCenter.x;
            edges_[4*nextEdge + 3] = nodes_[n]->children[i]->pixelCenter.y;

            ++nextEdge;
        }
    }
}


void HypothesisTreeRenderer::findTreeBoundary(void)
{
    if(!nodes_.empty())
    {
        Point<float> bottomLeft = nodes_.front()->pixelCenter;
        Point<float> topRight   = nodes_.front()->pixelCenter;

        for(size_t n = 0; n < nodes_.size(); ++n)
        {
            if(nodes_[n]->pixelCenter.x < bottomLeft.x)
            {
                bottomLeft.x = nodes_[n]->pixelCenter.x;
            }

            if(nodes_[n]->pixelCenter.y < bottomLeft.y)
            {
                bottomLeft.y = nodes_[n]->pixelCenter.y;
            }

            if(nodes_[n]->pixelCenter.x > topRight.x)
            {
                topRight.x = nodes_[n]->pixelCenter.x;
            }

            if(nodes_[n]->pixelCenter.y > topRight.y)
            {
                topRight.y = nodes_[n]->pixelCenter.y;
            }
        }

        treeBoundary_ = math::Rectangle<float>(bottomLeft, topRight);
    }
}


GLColor HypothesisTreeRenderer::findNodeColor(tree_node_t* node)
{
    auto colorIt = hypothesisColors_.find(node->id);

    if(colorIt != hypothesisColors_.end())
    {
        return colorIt->second;
    }
    else
    {
        return nodeColor_;
    }
}


void HypothesisTreeRenderer::changeNodeColor(uint32_t id, const GLColor& color)
{
    for(size_t n = 0; n < nodes_.size(); ++n)
    {
        if(nodes_[n]->id == id)
        {
            vertexColors_[4*n]     = color.red();
            vertexColors_[4*n + 1] = color.green();
            vertexColors_[4*n + 2] = color.blue();
            vertexColors_[4*n + 3] = color.alpha();
        }
    }
}

} // namespace ui
} // namespace vulcan
