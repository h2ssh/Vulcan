/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_object.h
* \author   Collin Johnson
* 
* Declaration of LaserObject.
*/

#ifndef TRACKER_LASER_OBJECT_H
#define TRACKER_LASER_OBJECT_H

#include "tracker/object_boundary.h"
#include "tracker/types.h"
#include "core/multivariate_gaussian.h"
#include "core/point.h"
#include <boost/optional.hpp>
#include <cereal/access.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace tracker
{

/**
* LaserObject contains the data about an object detected in a single laser scan. The laser object identifies the
* center, points, and uncertainty about the object location. Each object has a variety of possible boundary shape
* that are estimated from it. Each shape is a best-fit to the points contained in the laser.
*
* The points in a LaserObject are radially ordered in the sequence they were acquired by the lidar.
*/
class LaserObject
{
public:
    
    /**
    * Default constructor for serialization.
    */
    LaserObject(void);
    
    /**
    * Constructor for LaserObject.
    * 
    * The points are assumed to be sorted radially, in order they were measured by the laser.
    * 
    * \pre      points.size() >= 3
    * \param    timestamp       Time at which the measurement was made
    * \param    pointsBegin     Start of the points in the object
    * \param    pointsEnd       End of the points in the object
    * \param    laserPosition   Position of the laser that measured the points
    * \param    variance        Variance for the individual laser rays that measured the object
    */
    LaserObject(int64_t         timestamp, 
                ConstPointIter  pointsBegin, 
                ConstPointIter  pointsEnd, 
                const Position& laserPosition,
                double          variance = 0.01);
    
    /**
    * Constructor for LaserObject.
    * 
    * Merges multiple LaserObjects into a single LaserObject.
    * 
    * \param    objects     Objects to be merged together
    */
    LaserObject(const std::vector<const LaserObject*>& objects);
    
    /**
    * timestamp retrieves the timestamp of the scan in which this object was detected.
    */
    int64_t timestamp(void) const { return timestamp_; }
    
    /**
    * setBoundaryType sets the boundary to be used for this object. By default, the boundary is the best, which is
    * defined as the minimum error boundary based on the fit. However, all possible shapes are fit to the object and
    * prior information can inform the current type of the boundary. Thus, this method allows setting which boundary
    * to use for the computation of the centerWithUncertainty() and center().
    */
    void setBoundaryType(BoundaryType type);

    /**
    * centerWithUncertainty retrieves the estimated center for the object, taking into account
    * the uncertainty of the point measurements.
    */
    const MultivariateGaussian& centerWithUncertainty(void) const { return centerDistribution_; }
    
    /**
    * center retrieves the estimated center of the object.
    */
    Position center(void) const { return Position(centerDistribution_[0], centerDistribution_[1]); }
    
    /**
    * generateShadowedCircle creates a new LaserObject from this LaserObject with a second circle created in the
    * shadow of the actually detected circle. The shadowed circle is intended to be used for the helping fill in the
    * blanks when a leg blocks the view of the other leg for a person.
    */
    LaserObject generateShadowedCircle(void) const;
    
    /**
    * estimatedRectangle retrieves the rectangle boundary estimated for the laser points.
    */
    EstimatedShape<Rectangle> estimatedRectangle(void) const { return fitRect_; }

    /**
    * estimatedArc retrieves the estimated arc for the laser points.
    */
    EstimatedShape<Arc> estimatedArc(void) const { return fitArc_; }

    /**
    * estimatedTwoArcs retrieves the two-arc estimate for the laser points. Not every shape has two valid arcs that
    * can fit the data.
    */
    boost::optional<EstimatedShape<TwoArcs>> estimatedTwoArcs(void) const { return fitTwoArcs_; }

    /**
    * estimatedTwoRects retreives the estimated two rectangles for the laser points. Not every shape has two valid
    * rectangles that can fit the data.
    */
    boost::optional<EstimatedShape<TwoRects>> estimatedTwoRects(void) const { return fitTwoRects_; }

    /**
    * minErrorBoundary retrieves the boundary for the laser object with the smallest error.
    */
    ObjectBoundary minErrorBoundary(void) const;
    
    /**
    * boundaryWithType retrieves the estimated boundary with the given type.
    * 
    * \param    type            Type of boundary to retrieve for this object
    * \return   ObjectBoundary of the specified type.
    */
    ObjectBoundary boundaryWithType(BoundaryType type) const;
    
    /**
    * circleApproximation retrieves the circle approximation of the boundary.
    */
    Circle circleApproximation(void) const { return circleApproximation_; }
    
    /**
    * laserPosition position of the laser that detected this object.
    */
    Position laserPosition(void) const { return laserPosition_; }
    
    // Methods for iterating over the points contained in the object
    std::size_t    size(void)  const { return points_.size(); }
    ConstPointIter begin(void) const { return points_.begin(); }
    ConstPointIter end(void)   const { return points_.end(); }
    
private:
    
    int64_t timestamp_;
    MultivariateGaussian centerDistribution_;
    std::vector<Position> points_;
    double laserVariance_;
    EstimatedShape<Rectangle> fitRect_;
    EstimatedShape<Arc> fitArc_;
    boost::optional<EstimatedShape<TwoArcs>> fitTwoArcs_;
    boost::optional<EstimatedShape<TwoRects>> fitTwoRects_;
    Position laserPosition_;
    Circle circleApproximation_;

    void fitShapesToPoints(bool searchForOutliers);
    void calculateUncertainty(BoundaryType type);
    void calculateCircleApprox(void);
    
    std::pair<ObjectBoundary, Matrix> minErrorBoundaryAndUncertainty(void) const;
    
    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void save(Archive& ar) const
    {
        ar (timestamp_,
            centerDistribution_,
            points_,
            fitRect_,
            fitArc_,
            laserPosition_,
            circleApproximation_);

        bool haveTwoArcs = fitTwoArcs_;
        ar (haveTwoArcs);
        if(haveTwoArcs)
        {
            ar (*fitTwoArcs_);
        }
        
        bool haveTwoRects = fitTwoRects_;
        ar (haveTwoRects);
        if(haveTwoRects)
        {
            ar (*fitTwoRects_);
        }
    }

    template <class Archive>
    void load(Archive& ar)
    {
        ar (timestamp_,
            centerDistribution_,
            points_,
            fitRect_,
            fitArc_,
            laserPosition_,
            circleApproximation_);

        bool haveTwoArcs = false;
        ar (haveTwoArcs);
        if(haveTwoArcs)
        {
            EstimatedShape<TwoArcs> twoArcs;
            ar (twoArcs);
            fitTwoArcs_ = twoArcs;
        }
        else
        {
            fitTwoArcs_ = boost::none;
        }

        bool haveTwoRects = false;
        ar (haveTwoRects);
        if(haveTwoRects)
        {
            EstimatedShape<TwoRects> twoRects;
            ar (twoRects);
            fitTwoRects_ = twoRects;
        }
        else
        {
            fitTwoRects_ = boost::none;
        }
    }
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_LASER_OBJECT_H
