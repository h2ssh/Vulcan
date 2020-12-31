/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     labeling_csp_playback.h
 * \author   Collin Johnson
 *
 * Declaration of LabelingCSPPlayer.
 */

#ifndef UI_COMPONENTS_LABELING_CSP_PLAYER_H
#define UI_COMPONENTS_LABELING_CSP_PLAYER_H

#include <memory>

namespace vulcan
{
namespace hssh
{
class CSPDebugInfo;
}
namespace hssh
{
class CSPArea;
}
namespace hssh
{
class CSPIteration;
}
namespace ui
{

class AreaExtentRenderer;

/**
 * LabelingCSPPlayer provides a movie-like player for the search performed by the labeling CSP. Each iteration is
 * displayed for a configurable number of frames. The failed areas are shown with a thick border and the area that was
 * changed is filled in. The left half of the changed boundary contains the old type, the right half contains the new
 * type.
 *
 * The player has the usual play/pause/stop as well as stepping forward and backward or jumping to the front or end.
 * When stop is hit, nothing is displayed. Pause will halt playback, but keep displaying information. Hitting step while
 * playing will go into pause mode.
 */
class LabelingCSPPlayer
{
public:
    /**
     * Constructor for LabelingCSPPlayer.
     */
    LabelingCSPPlayer(void);

    ~LabelingCSPPlayer(void);

    /**
     * update updates the visible CSP iteration.
     */
    void update(void);

    /**
     * setCSPInfo sets the information to be played back. When new information is set, the iteration is reset to the
     * start and playback is paused (if playing), or remains stopped.
     *
     * \param    info                New set of debug info to be played
     * \param    metersPerCell       Meters per cell for the extents in the debug info
     */
    void setCSPInfo(const hssh::CSPDebugInfo& info, double metersPerCell);

    /**
     * numIterations retrieves the total number of iterations in the currently displayed CSP solution.
     */
    int numIterations(void) const;

    /**
     * currentIteration shows the currently displayed iteration.
     */
    int currentIteration(void) const { return iteration_; }

    /**
     * setFramesPerIteration sets the number of frames to display an iteration before moving on to the next.
     *
     * \pre  framesPerIteration > 0
     * \param    framesPerIteration          Number of frames to display an iteration
     */
    void setFramesPerIteration(int framesPerIteration);

    /**
     * play starts playing the search.
     */
    void play(void);

    /**
     * pause pauses playing, so the state can be investigated.
     */
    void pause(void);

    /**
     * stop stops playing and stops displaying the current iteration.
     */
    void stop(void);

    /**
     * stepForward moves to the next iteration.
     */
    void stepForward(void);

    /**
     * stepBackward moves to the previous iteration.
     */
    void stepBackward(void);

    /**
     * jumpToStart jumps back to the first iteration and pauses playback.
     */
    void jumpToStart(void);

    /**
     * jumpToEnd jumps to the last iteration and pauses playback.
     */
    void jumpToEnd(void);

    /**
     * jumpToIteration jumps to the specified iteration.
     *
     * If iteration < 0 || iteration > numIterations, no change is made.
     *
     * \param    iteration           Jumps to the provided iteration
     */
    void jumpToIteration(int iteration);

private:
    enum State
    {
        playing,
        paused,
        stopped,
    };

    enum AreaStatus
    {
        failed,
        assigned,
    };

    std::unique_ptr<hssh::CSPDebugInfo> debugInfo_;   // Current information being re-played
    int iteration_;                                   // Current visible iteration

    State state_;              // Current state of the playback
    int numFramesShown_;       // Number of frames the current iteration has been shown
    int framesPerIteration_;   // Number of frames to show an iteration

    double metersPerCell_;   // Meters per cell of the extents
    std::unique_ptr<AreaExtentRenderer>
      extentRenderer_;   // Render to use for drawing the underlying assignment of labels

    void stepIteration(int direction);
    void renderIteration(const hssh::CSPIteration& iteration);
    void renderArea(const hssh::CSPArea& area, AreaStatus status);
    void renderEndpoints(const hssh::CSPIteration& iteration);
    void renderGateways(const hssh::CSPIteration& iteration);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_LABELING_CSP_PLAYER_H
