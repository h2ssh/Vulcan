/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_boundary.h
* \author   Collin Johnson
*
* Declaration of ObjectBounday.
*/

#ifndef TRACKER_OBJECT_BOUNDARY_H
#define TRACKER_OBJECT_BOUNDARY_H

#include "tracker/types.h"
#include "tracker/boundaries/shapes.h"
#include "core/point.h"
#include <cereal/access.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/boost_variant.hpp>
#include <iosfwd>

namespace vulcan
{
namespace tracker
{

using BoundaryShape = boost::variant<Rectangle, Circle, TwoCircles, CircleRect, TwoRects>;

/**
* ObjectBoundary
*/
class ObjectBoundary
{
public:

    /**
    * Default constructor for ObjectBoundary/
    */
    ObjectBoundary(void);

    /**
    * Constructor for ObjectBoundary.
    *
    * Create a rectangle boundary
    *
    * \param    rectangle           Rectangle boundary for the object
    */
    ObjectBoundary(const Rectangle& rectangle);

    /**
    * Constructor for ObjectBoundary.
    *
    * Create a one circle boundary
    *
    * \param    circle              One circle boundary for the object
    */
    ObjectBoundary(const Circle& circle);

    /**
    * Constructor for ObjectBoundary.
    *
    * Create a two circle boundary.
    *
    * \param    circles             Circles bounding the object
    */
    ObjectBoundary(const TwoCircles& circles);

    /**
    * Constructor for ObjectBoundary.
    *
    * Create a two rectangle boundary.
    *
    * \param    rects               Rectangles bounding the object
    */
    ObjectBoundary(const TwoRects& rects);

    /**
    * Constructor for ObjectBoundary.
    *
    * Create a circle-rectangle boundary.
    *
    * \param    circleRect          Circle and rect bounding the object
    */
    ObjectBoundary(const CircleRect& circleRect);

    /**
    * assign assigns a new shape to the boundary. The assigned shape must be a valid BoundaryType.
    */
    template <class BoundaryShape>
    void assign(BoundaryShape&& shape, BoundaryType type)
    {
        type_ = type;
        shape_ = shape;
        center_ = center(shape);
    }

    /**
    * assignApproximation assigns a new circle approximation to the boundary.
    */
    void assignApproximation(Circle approximation) { circleApproximation_ = approximation; }

    /**
    * position retrieves the current position of the object.
    */
    Point<float> position(void) const { return center_; }

    /**
    * type retrieves the shape that best describes the boundary.
    */
    BoundaryType type(void) const { return type_; }

    /**
    * circleApproximation retrieves the circle approximation of the actual underlying boundary type. It can be used as
    * an easy approximation of the boundary for collision checking and such.
    */
    Circle circleApproximation(void) const { return circleApproximation_; }

    /**
    * visitShape accepts a visitor for the BoundaryShape variant and visits it.
    *
    * \param    visitor         Visitor for the variant
    * \return   Result type specified by the Visitor class.
    */
    template <typename Visitor>
    typename Visitor::result_type visitShape(Visitor&& visitor) const
    {
        return boost::apply_visitor(visitor, shape_);
    }

    /**
    * updateBoundary updates the estimate of the boundary with a new measurement.
    *
    * \param    boundary            Measured boundary in integrate into this boundary estimate
    */
    void updateBoundary(const ObjectBoundary& boundary);

    /**
    * updateApproximation updates the circle approximation of the boundary using the latest measurement.
    */
    void updateApproximation(Circle approx);

    /**
    * distanceFromBoundary finds the cumulative distance of a collection of points from the boundary.
    *
    * The distance here is the absolute distance to the boundary. If a point is inside the boundary, it still has
    * some distance. For a distance of zero if an object is inside another object, use distanceToObject.
    *
    * \param    begin           Beginning of measured points
    * \param    end             One-past-end of measured points
    * \return   Min distance and average distance to the boundary.
    */
    std::tuple<double, double> distanceFromBoundary(ConstPointIter begin, ConstPointIter end) const;

    /**
    * distanceToObject finds the cumulative distance of a collection of point to the object represented by the
    * boundary.
    *
    * The distance to the object means that a point inside the boundary has a zero distance -- it must be a part of
    * the object.
    *
    * \param    begin           Beginning of measured points
    * \param    end             One-past-end of measured points
    * \return   Min distance and average distance to the object.
    */
    std::tuple<double, double> distanceToObject(ConstPointIter begin, ConstPointIter end) const;

    /**
    * boundaryAtPosition retrieves the model of this boundary if it was shifted to be centered at the provided
    * position.
    *
    * \param    position            Position at which to center the object
    * \return   The ObjectBoundary shifted to be centerd at position.
    */
    ObjectBoundary boundaryAtPosition(Position position) const;

private:

    BoundaryType       type_ = BoundaryType::unknown;
    Point<float> center_;
    BoundaryShape      shape_;
    Circle             circleApproximation_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( type_,
            center_,
            shape_,
            circleApproximation_);
    }
};

// Output operator for BoundaryType
std::ostream& operator<<(std::ostream& out, BoundaryType type);

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_OBJECT_BOUNDARY_H
