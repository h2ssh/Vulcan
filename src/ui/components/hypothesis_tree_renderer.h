/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_tree_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of HypothesisTreeRenderer.
 */

#ifndef UI_COMPONENTS_HYPOTHESIS_TREE_RENDERER_H
#define UI_COMPONENTS_HYPOTHESIS_TREE_RENDERER_H

#include "core/point.h"
#include "hssh/utils/id.h"
#include "math/geometry/rectangle.h"
#include "ui/common/ui_color.h"
#include "utils/object_pool.h"
#include <map>

namespace vulcan
{
namespace hssh
{
class HypothesisTree;
}
namespace hssh
{
struct hypothesis_tree_node_t;
}
namespace ui
{

/**
 * HypothesisTreeRenderer is a renderer for the hssh::HypothesisTree that contains information
 * on how the tree is evolving over time, both in terms of hypotheses generated and their probabilities.
 *
 * Each node in the tree is rendered as a circle with line segments connecting it to its children. The
 * color of a node, along with its alpha, is used to indicate the probability of the nodes. Each node
 * in the hypothesis tree contains multiple probability values. The value to use for coloring a node can
 * be changed, which allows for considering a number of different probability values.
 */
class HypothesisTreeRenderer
{
public:
    /**
     * Constructor for HypothesisTreeRenderer.
     */
    HypothesisTreeRenderer(void);

    ~HypothesisTreeRenderer(void);

    /**
     * setRenderColors sets the colors with which to render the nodes and edges of the hypothesis tree.
     *
     * \param    nodeColor       Color to assign to the nodes
     * \param    edgeColor       Color to assign to the edges from parent to child
     */
    void setRenderColors(const GLColor& nodeColor, const GLColor& edgeColor);

    /**
     * setHypothesisTree sets the HypothesisTree to be rendered.
     *
     * \param    tree            Tree to be rendered
     */
    void setHypothesisTree(const hssh::HypothesisTree& tree);

    /**
     * setProbabilityIndex sets the index of the probability to be displayed. If the index is greater than the
     * probabilities from the hypothesis tree, no change will occur.
     *
     * \param    index           Index of the probability value to use for the node coloring
     */
    void setProbabilityIndex(size_t index);

    /**
     * nodeClosestToPoint retrieves the node in the rendered tree closest to the provide (x,y) point. This method
     * is for selecting a map in the tree to determine its properties.
     *
     * \param    point           Point to be selected
     * \return   Index of closest point.
     */
    hssh::Id nodeClosestToPoint(const Point<float>& point) const;

    /**
     * highlightHypothesis specifies the color to draw a particular hypothesis in the tree to distinguish
     * it from the rest of the hypotheses.
     *
     * \param    index           Index of the hypothesis to be highlighted
     * \param    color           Color to draw the hypothesis
     */
    void highlightHypothesis(uint32_t index, const GLColor& color);

    /**
     * cancelHighlight erases the highlight for a particular hypothesis.
     *
     * \param    index           Index of the highlight to cancel
     */
    void cancelHighlight(uint32_t index);

    /**
     * resetHypothesisHighlights erases all information about highlighting particular hypotheses, so all hypotheses
     * will be rendered in their normal color again.
     */
    void resetHypothesisHighlights(void) { hypothesisColors_.clear(); }

    /**
     * getRenderedBoundary retrieves the boundary of the rendered hypothesis tree, allowing the camera to be
     * repositioned correctly ahead of time.
     */
    math::Rectangle<float> getRenderedBoundary(void) const { return treeBoundary_; }

    /**
     * renderTree renders the tree. The units for rendering are abstract and don't correspond to any physical value.
     */
    void renderTree(void);

private:
    HypothesisTreeRenderer(const HypothesisTreeRenderer& toCopy) = delete;
    HypothesisTreeRenderer& operator=(const HypothesisTreeRenderer& rhs) = delete;

    struct tree_node_t
    {
        Point<float> pixelCenter;
        hssh::Id id;
        uint32_t breadth;

        tree_node_t* parent;
        std::vector<tree_node_t*> children;
    };

    std::unique_ptr<hssh::HypothesisTree> tree_;
    math::Rectangle<float> treeBoundary_;
    std::vector<tree_node_t*> nodes_;
    utils::ObjectPool<tree_node_t> nodePool_;

    int treeBreadth_;
    int treeDepth_;

    /*
     * Sizes:
     *
     * vertices     = nodes.size() * 2
     * edges        = nodes.size() * 4
     * vertexColors = nodes.size() * 4
     * edgeColors   = nodes.size() * 8
     */
    std::vector<float> vertices_;
    std::vector<float> edges_;

    std::vector<float> vertexColors_;
    std::vector<float> edgeColors_;

    GLColor nodeColor_;
    GLColor edgeColor_;

    std::map<uint32_t, GLColor> hypothesisColors_;

    void extractNodesFromTree(void);
    void expandHypothesisNode(const hssh::hypothesis_tree_node_t& hypNode, tree_node_t* treeNode);
    void assignVerticesToNodes(void);
    void assignTreeSlots(tree_node_t* node, int slotStart, int slotEnd, int depth);
    void allocateGLBuffers(void);
    void assignNodeColors(void);
    void assignEdges(void);
    void findTreeBoundary(void);

    GLColor findNodeColor(tree_node_t* node);
    void changeNodeColor(uint32_t id,
                         const GLColor& color);   // for setting a color outside of when a tree is being built
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_HYPOTHESIS_TREE_RENDERER_H
