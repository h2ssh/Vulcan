/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     large_scale_star.h
 * \author   Collin Johnson
 *
 * Definition of LargeScaleStar and global_path_fragment_t.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_LARGE_SCALE_STAR_H
#define HSSH_GLOBAL_TOPOLOGICAL_LARGE_SCALE_STAR_H

#include "hssh/global_topological/global_path.h"
#include <iosfwd>
#include <vector>

namespace vulcan
{
namespace hssh
{

class SmallScaleStar;
struct local_path_fragment_t;

/**
 * path_fragment_exploration_t defines the exploration status of a global_path_fragment_t. A
 * fragment can be either a frontier or explored.
 */
enum path_fragment_exploration_t
{
    PATH_FRAGMENT_EXPLORED,
    PATH_FRAGMENT_FRONTIER
};

/**
 * path_fragment_type_t specifies where the fragment leads. That is, what sort of place will the
 * robot arrive at if it moves through this fragment.
 */
enum path_fragment_type_t
{
    PATH_FRAGMENT_PATH,         ///< The normal fragment, which goes from the decision point onto a path
    PATH_FRAGMENT_DECISION,     ///< The fragment leads from this place to a decision point
    PATH_FRAGMENT_DESTINATION   ///< The fragmnet leads from this place into a destination
};

const int INVALID_FRAGMENT_ID = -1;

/**
 * global_path_fragment_t identifies the path associated with a transition in the large-scale star. A
 * path fragment is defined by:
 *
 *   - a fragment id indicating the unique fragment within the large-scale star
 *   - a path id indicating the path to which the fragment connects/corresponds
 *   - a direction indicating the side of the path on which the fragment lies. Each path has a direction. If you draw a
 * line through the place from + to - along the path, the star fragment first intersected would be + and the second
 * would be -
 *   - a flag indicating if the fragment is navigable, i.e. is this a path endpoint?
 *   - a flag indicating if the fragment belongs to a frontier along its path
 */
struct global_path_fragment_t
{
    global_path_fragment_t(int fragmentId = INVALID_FRAGMENT_ID, int pathId = -1, bool navigable = false)
    : fragmentId(fragmentId)
    , pathId(pathId)
    , direction(PATH_NONE)
    , navigable(navigable)
    , exploration(PATH_FRAGMENT_FRONTIER)
    , type(PATH_FRAGMENT_PATH)
    {
    }

    int fragmentId;
    int pathId;
    path_direction_t direction;
    bool navigable;
    path_fragment_exploration_t exploration;
    path_fragment_type_t type;
};

/**
 * LargeScaleStar is the symbolic representation of the topology of a Place in the
 * topological map. A large-scale star represents the sequential ordering of the path
 * fragments arriving at a place. A bijective mapping exists between the small-scale
 * of a place and the large-scale star.
 *
 * The ordering of global_path_fragment_ts in the LargeScaleStar is identical to the ordering
 * of local_path_fragment_ts in the SmallScaleStar from which it was created. Thus, a
 * LargeScaleStar can be converted back into a SmallScaleStar with the direction of the
 * local fragments being the only potential difference -- though the direction is arbitrary
 * and can thus be reassigned without changing the symbolic value of the SmallScaleStar.
 */
class LargeScaleStar
{
public:
    /**
     * Default constructor for LargeScaleStar.
     */
    LargeScaleStar(void);

    /**
     * Constructor for LargeScaleStar.
     *
     * Creates a LargeScaleStar with all associated path fragments being frontiers.
     *
     * \param    smallStar       SmallScaleStar from which to craft the LargeScaleStar
     */
    LargeScaleStar(const SmallScaleStar& smallStar);

    /**
     * Constructor for LargeScaleStar.
     *
     * Creates a LargeScaleStar with all associated path fragments being frontiers.
     *
     * \param    smallStar       SmallScaleStar from which to craft the LargeScaleStar
     * \param    entryFragment   Path fragment through which the star was entered
     */
    LargeScaleStar(const SmallScaleStar& smallStar, const local_path_fragment_t& entryFragment);

    /**
     * Constructor for LargeScaleStar.
     *
     * Creates a LargeScaleStar from a collection of existing global_path_fragment_t.
     *
     * \param    fragments        Fragments which will comprise the star
     */
    LargeScaleStar(const std::vector<global_path_fragment_t>& fragments);

    // Operator overloads
    bool operator==(const LargeScaleStar& rhs) const;
    bool operator!=(const LargeScaleStar& rhs) const;

    // Methods

    /**
     * getPathFragments retrieves the global_path_fragment_t that comprise the star. The
     * path fragments are stored in clockwise order as they appear around the star.
     */
    std::vector<global_path_fragment_t> getPathFragments(void) const { return fragments; }

    /**
     * getPathFragment retrieves the path fragment associated with the given path and direction.
     */
    global_path_fragment_t getPathFragment(int pathId, path_direction_t direction) const;

    /**
     * getEntryPathFragment retrieves the path fragment representing the transition through which the
     * robot entered the place represented by the star.
     */
    global_path_fragment_t getEntryPathFragment(void) const { return entryFragment; }

    /**
     * getFragmentWithId retrieves the star fragment with the specified id.
     */
    global_path_fragment_t getFragmentWithId(int8_t fragmentId) const;

    /**
     * getOtherFragmentOnPath retrieves the star fragment that shares a path with the provided fragment.
     */
    global_path_fragment_t getOtherFragmentOnPath(int8_t fragmentId) const;

    /**
     * setEntryPathFragment sets the fragment through which the star was most recently entered.
     */
    void setEntryPathFragment(int8_t fragmentId);

    /**
     * toSmallScaleStar creates a SmallScaleStar with the equivalent topology and fragment indices to the
     * LargeScaleStar. In the created small-scale star, the local_path_fragment_t ids will be the same as the associated
     * global_path_fragment_t ids.
     */
    SmallScaleStar toSmallScaleStar(void) const;

    /**
     * findExitFragment determines the exit star fragment based the small-scale star with the same topology
     * along with the small-scale star's entry and exit fragments. If the entry star fragment is known,
     * then the exit fragment can be found in straightforward fashion.
     *
     * \param    smallStar           Star of the local topology
     * \param    entryFragment       Fragment through which the star was entered
     * \param    exitFragment        Fragment through which the star was exited
     * \return   global_path_fragment_t corresponding to the exit fragment in the small-scale star.
     */
    global_path_fragment_t findExitFragment(const SmallScaleStar& smallStar,
                                            const local_path_fragment_t& entryFragment,
                                            const local_path_fragment_t& exitFragment) const;

    /**
     * assignPath assigns a path to a given global path fragment. The navigability of the
     * fragment is not altered. An unnavigable path is the end of the path to which it is assigned.
     *
     * \param    fragmentId              Id of the star fragment to assign a path to
     * \param    pathId                  Path id to assign to the fragment
     * \param    direction               Direction of the fragment along the path,
     *                                   moving to this fragment from inside the place is moving in direction along the
     * path
     */
    void assignPath(int8_t fragmentId, int pathId, path_direction_t direction);

    /**
     * changePath changes the id of a path passing through the star. If the direction of the path also changes,
     * the reverseDirection flag should be set.
     *
     * \param    currentId               Currently stored id for the path
     * \param    newId                   New id for the path
     * \param    reverseDirection        Flag indicating if the direction should be reversed
     */
    void changePath(int currentId, int newId, bool reverseDirection);

    /**
     * setFragmentExploration updates the exploration status of a star fragment.
     *
     * \param    fragmentId      Id of the fragment to update
     * \param    exploration     New exploration status of the fragment
     */
    void setFragmentExploration(int8_t fragmentId, path_fragment_exploration_t exploration);

    /**
     * enteredOnKnownPath compares the layout of the provided LargeScaleStar and the entry fragment to
     * determine if the robot entered the place along a known path.
     *
     * The robot will have entered along a known path only if the path fragment through which the robot
     * entered exists in the LargeScaleStar. If the robot entered along a known path, the global_path_fragment_t
     * associated with the path will be assigned to the fragment argument.
     *
     * \param    largeStar       LargeScaleStar associated with current place
     * \param    fragment        Fragment by which the robot entered the place (output -- valid if return value is true)
     * \return   True if the entry path fragment unambiguously maps to an existing path fragment in the place.
     */
    bool enteredOnKnownPath(const LargeScaleStar& largeStar, global_path_fragment_t& fragment) const;

    /**
     * potentialEntryFragments compares the provided LargeScaleStar and entry path with this LargeScaleStar
     * to determine the potential entry paths. Each potential entry path suggests a different topology of
     * the map. An entry path is a potential entry only if it is topologically the same and is a frontier.
     * If a fragment isn't a frontier, then the entry path would be known.
     *
     * \param    largeStar       LargeScaleStar associated with the current place
     * \param    entryFragment   Path from which the largeStar was entered
     * \return   Set of path fragments that could be the same as entry fragment in this star.
     */
    std::vector<global_path_fragment_t> potentialEntryFragments(const LargeScaleStar& largeStar,
                                                                const global_path_fragment_t& entryFragment) const;

    /**
     * areStarsCompatible checks to see if the star passed in as an argument is compatible with this star. A compatible
     * star will having a matching topology and a compatible entry fragment. For the entry fragments to be compatible,
     * they need to be equivalent using only clockwise rotations of the star.
     *
     * \param    star            Star to be checked for compatibility. The entry fragment is assumed set.
     * \return   True if the stars are compatible as defined above. False otherwise.
     */
    bool areStarsCompatible(const LargeScaleStar& star) const;

private:
    // INVARIANT: The fragmentId == the path fragments's index in the fragments vector

    std::vector<global_path_fragment_t> fragments;
    global_path_fragment_t entryFragment;
};

std::ostream& operator<<(std::ostream& out, const LargeScaleStar& star);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_LARGE_SCALE_STAR_H
