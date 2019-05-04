/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist.h
* \author   Collin Johnson
*
* Definition of classes:
*
*   - Isovist
*   - IsovistField
*/

#ifndef UTILS_ISOVIST_H
#define UTILS_ISOVIST_H

#include <core/point.h>
#include <utils/algorithm_ext.h>
#include <utils/cell_grid.h>
#include <utils/ray_tracing.h>
#include <array>
#include <functional>
#include <future>

namespace vulcan
{
namespace utils
{

/**
* isovist_options_t specifies options for the calculation and storage of the isovists.
*/
struct isovist_options_t
{
    std::size_t numRays       = 500;          ///< Number of rays to use for each isovist
    float       maxDistance   = 15.0f;        ///< Maximum distance to trace the isovist
    bool        saveEndpoints = false;        ///< Flag indicating if the endpoints for the isovists should be saved
    std::vector<Point<float>> endpointStorage_;  ///< Temp memory for dealing with the endpoints
};

/**
* Isovist is a representation of an isovist as defined by Michael Benedikt and Larry Davis in:
*   Computational Models of Space: Isovists and Isovist Fields (1979)
*
* An Isovist can be created from a CellGrid by specifying a start position and a termination function that takes a
* grid and a cell and returns whether or not it terminates a ray. The Isovist is simply the collection of endpoints
* of rays cast from this starting cell.
*
* The endpoints stored in the isovist are in global coordinates, not grid coordinates.
*
* Each Isovist has a number of scalar properties:
*
*
* Calculations for the shape-based scalars come from  progmat.uw.hu/oktseg/kepelemzes/lec13_shape_4.pdf
*/
class Isovist
{
public:

    using Iter = std::vector<Point<float>>::const_iterator;

    /// Enum defining the scalar values that can be calculated about the Isovist
    enum Scalar : int
    {
        // Shape-based scalars
        kArea       = 0,
        kPerimeter,
        kCircularity,
        kOrientation,
        kWeightedOrientation,
        kShapeEccentricity,
        kShapeCompactness,
        kShapeWaviness,
        kMinDistOrientation,
        kMaxThroughDist,                // max dist when considering line of rays in both dirs
        kMaxThroughDistOrientation,     // orientation of max through dist
        kMinLineDist,                   // distance of minimum length line running through center
        kMinLineNormal,                 // normal of the min dist line
        kMinNormalDiff,                 // difference between min line normal and min dist orientation

        // Ray-based scalars
        kDmax,
        kDmin,
        kDavg,
        kDstd,
        kDVariation,
        kDskewness,
        kShapeDistAvg,
        kShapeDistStd,
        kShapeDistVariation,
        kShapeDistCompactness,
        kDeltaAvg,
        kDeltaStd,
        kDeltaVariation,
        kDistRelationAvg,
        kDistRelationStd,
        kRayCompactness,
        kAngleBetweenMinDists,  // angle between shortest rays

        // Deriv-based features
        kMinHalfArea,       // minimum area on either side of dividing boundary
        kAreaBalance,       // ratio of min half area to max half area -- evenly divided is 1.0, super-skewed ~ 0.0
        kNumScalars
    };

    /**
    * Constructor for Isovist.
    *
    * Creates an Isovist from an arbitrary CellGrid.
    *
    * The TerminationFunc must have the signature: bool(const Grid&, Point<int>)
    *
    * \param    position                Global position of the isovist in the grid
    * \param    grid                    Grid in which to find the isovist
    * \param    rayTerminationFunc      Function that specifies when a cell in the grid should terminate a ray in the isovist
    * \param    options                 Options for the calculation of the isovists (optional, default = default constructed options)
    */
    template <class Grid, class TerminationFunc>
    Isovist(Point<int16_t> position,
            const Grid&          grid,
            TerminationFunc      rayTerminationFunc,
            isovist_options_t&   options)
    : position_(grid_point_to_global_point(position, grid))
    , cell_(position)
    {

        const float kIsovistAngleStep  = 2.0 * M_PI / options.numRays;

        // Need to do numRays increments, but need to include the first value as well, thus, numRays + 1 isovists.
        options.endpointStorage_.resize(options.numRays + 1);   // make sure we have enough memory available
        ray_trace_range_t isovistRays(0.0f, options.numRays + 1, kIsovistAngleStep, true);
        trace_range_until_condition(position, isovistRays, options.maxDistance, grid, rayTerminationFunc, options.endpointStorage_.begin());

        // In rare cases, i.e. on the edge of the map or edge of visible space, many of the rays will terminate at the
        // isovist position. In these cases, we want to only keep one of these values around because the isovist is
        // actually a polygon and we are calculating an approximation of it. By including the adjacent duplicate values,
        // then the isovist endpoints no longer are a good approximation for the bounding polygon. Using erase_unique
        // will keep around a single copy of the duplicated endpoint, as desired
        utils::erase_unique(options.endpointStorage_);

        for(auto& cell : options.endpointStorage_)
        {
            cell = grid_point_to_global_point(cell, grid);
        }

        // Make sure that everything wraps around
        options.endpointStorage_.back() = options.endpointStorage_.front();

        calculateScalars(options.endpointStorage_.begin(), options.endpointStorage_.end(), scalars_);

        if(options.saveEndpoints)
        {
            endpoints_ = options.endpointStorage_;
        }

        calculateDerivs(options.endpointStorage_);

        scalars_[kMinNormalDiff] = angle_diff_abs_pi_2(scalars_[kMinLineNormal],
                                                             scalars_[kMinDistOrientation]);
    }

    /**
    * Constructor for Isovist.
    *
    * Creates an Isovist from a position and a set of endpoints.
    *
    * \pre      endpoints are circularly sorted around the position
    * \param    position            Global position of the isovist
    * \param    endpoints           Endpoints of the isovist
    */
    Isovist(Point<double> position, const std::vector<Point<float>>& endpoints)
    : position_(position)
    , endpoints_(endpoints)
    {
        calculateScalars(endpoints_.begin(), endpoints_.end(), scalars_);
        calculateDerivs(endpoints_);
    }

    /**
    * Default constructor for Isovist.
    */
    Isovist(void) { }

    // Use default move and copy constructors
    Isovist(const Isovist& rhs)            = default;
    Isovist(Isovist&& rhs)                 = default;
    Isovist& operator=(const Isovist& rhs) = default;
    Isovist& operator=(Isovist&& rhs)      = default;

    // Properties of the Isovist
    Point<double> position(void)          const { return position_;        }
    Point<int>    cell    (void)          const { return cell_;            }
    double              scalar  (Scalar scalar) const { return scalars_[scalar]; }
    double              scalarDeriv(Scalar scalar) const { return scalarDerivs_[scalar]; }
    int                 numScalars(void) const { return scalars_.size(); }
    static std::string scalarName(Scalar scalar);

    // Iterators
    std::size_t size(void)  const { return endpoints_.size();  }
    Iter        begin(void) const { return endpoints_.begin(); }
    Iter        end(void)   const { return endpoints_.end();   }

private:

    using PointIter = std::vector<Point<float>>::const_iterator;

    Point<double>             position_;
    Point<int>                cell_;
    std::vector<Point<float>> endpoints_;
    std::vector<double>             scalars_;
    std::vector<double>             scalarDerivs_;

    void calculateScalars        (PointIter begin, PointIter end, std::vector<double>& scalars);
    void calculateDistanceScalars(PointIter begin, PointIter end, std::vector<double>& scalars);
    void calculateDeltaScalars   (PointIter begin, PointIter end, std::vector<double>& scalars);
    void calculatePolygonScalars (PointIter begin, PointIter end, std::vector<double>& scalars);
    void calculateMomentsScalars (PointIter begin, PointIter end, std::vector<double>& scalars);
    void calculateDerivs(std::vector<Point<float>>& endpoints);
};

/**
* IsovistField is a field of Isovists constructed from a grid and a collection of positions for the isovists.
*
* The IsovistField is a scalar field defined by a single value at each isovist position. With this implementation, the
* isovists in the field can be iterated over and a particular field can be generated by calling the appropriate property observer
* for the individual Isovists.
*/
class IsovistField
{
public:

    using Iter = std::vector<Isovist>::const_iterator;

    /**
    * Constructor for IsovistField.
    *
    * Create an IsovistField for a collection of positions.
    *
    * The isovists are saved in the same order as the positions are input. Thus, a mapping can be maintained from the
    * position to the isovist, if desired.
    *
    * \param    positions               Global positions of the isovists in the grid
    * \param    grid                    Grid for the isovists
    * \param    rayTerminationFunc      Condition to check when an isovist ray should terminate
    * \param    options                 Options for the calculation of the isovists (optional, default = default constructed options)
    */
    template <class Grid, class TerminationFunc>
    IsovistField(const std::vector<Point<int16_t>>& positions,
                 const Grid&                              grid,
                 TerminationFunc                          rayTerminationFunc,
                 isovist_options_t                        options = isovist_options_t())
    {
        using IsovistVec = std::vector<Isovist>;

        isovists.reserve(positions.size());

        if(!positions.empty())
        {
            // Create up to four threads for finding the isovists. Assign 1/4th of the work to each thread.
            std::size_t kMaxThreads = 4;
            const int kNumThreads = std::min(kMaxThreads, positions.size()/kMaxThreads+1);

            std::size_t positionsPerThread = positions.size() / kNumThreads + 1; // Add one to the positions/thread to account for truncating due to integer division
                                                                                 // The last thread will take up to 3 fewer isovist calculations

            // Launch the threads
            std::vector<std::future<IsovistVec>> asyncIsovists;
            for(int n = 0; n < kNumThreads; ++n)
            {
                std::size_t start = positionsPerThread*n;
                std::size_t end   = std::min(positionsPerThread*(n+1), positions.size());

                asyncIsovists.push_back(std::async(std::launch::async,
                                                   IsovistThreadFunc<Grid, TerminationFunc>(),
                                                   positions.begin()+start,
                                                   positions.begin()+end,
                                                   std::ref(grid),
                                                   rayTerminationFunc,
                                                   options));
            }

            for(auto& f : asyncIsovists)
            {
                auto&& fIsovists = f.get();
                isovists.insert(isovists.end(),
                                std::make_move_iterator(fIsovists.begin()),
                                std::make_move_iterator(fIsovists.end()));
            }
        }
    }

    /**
    * Constructor for IsovistField.
    *
    * Creates an IsovistField from an existing collection of isovists.
    */
    IsovistField(const std::vector<Isovist>& isovists)
    : isovists(isovists)
    {
    }

    // Iterators
    Iter        begin(void) const { return isovists.begin(); }
    Iter        end(void)   const { return isovists.end(); }
    std::size_t size(void)  const { return isovists.size(); }

    const Isovist& operator[](std::size_t index) const { return isovists[index]; }

private:

    using PointIter = std::vector<Point<int16_t>>::const_iterator;

    template <class Grid, class TerminationFunc>
    struct IsovistThreadFunc
    {
        // Need to take options by copy so the storage remains thread-local
        std::vector<Isovist> operator()(PointIter begin, PointIter end, const Grid& grid, TerminationFunc rayTerminationFunc, isovist_options_t options)
        {
            std::vector<Isovist> isovists;
            options.endpointStorage_.resize(options.numRays);
            isovists.reserve(std::distance(begin, end));
            for(; begin != end; ++begin)
            {
                isovists.push_back(Isovist(*begin, grid, rayTerminationFunc, options));
            }

            return isovists;
        }
    };

    std::vector<Isovist> isovists;
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_ISOVIST_H
