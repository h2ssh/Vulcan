/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     circle.h
* \author   Collin Johnson
*
* Definition of Circle subclass of the Shape interface.
*/

#ifndef MATH_GEOMETRY_CIRCLE_H
#define MATH_GEOMETRY_CIRCLE_H

#include <math/geometry/shape.h>
#include <core/line.h>
#include <core/point.h>
#include <cereal/access.hpp>
#include <cassert>
#include <cmath>
#include <ostream>

namespace vulcan
{
namespace math
{

/**
* Circle is an implementation of the Shape interface for a circle. Much simpler than the other shapes!
*/
template <typename T>
class Circle : public Shape<T>
{
public:

    /**
    * Constructor for Circle.
    *
    * \param    radius      Radius of the circle  (default = 1.0)
    * \param    center      Center of the circle  (default = 0,0)
    */
    explicit Circle(double radius = 1.0, Point<T> center = Point<T>(0, 0))
    : radius_(radius)
    , center_(center)
    {
        assert(radius_ > 0.0);
    }

    template <typename U>
    Circle(const Circle<U>& toCopy)
    : radius_(toCopy.radius())
    {
    }

    template <typename U>
    Circle& operator=(const Circle<U>& rhs)
    {
        radius_ = rhs.radius();
        center_ = rhs.center();

        return *this;
    }

    /** Destructor. */
    virtual ~Circle() { }

    /**
    * center retrieves the center of the shape.
    */
    virtual Point<T> center(void) const
    {
        return center_;
    }

    /**
    * radius retrieves the radius of the circle.
    */
    T radius(void) const
    {
        return radius_;
    }

    /**
    * area calculates the area of the shape.
    */
    virtual double area(void) const
    {
        return M_PI * radius_ * radius_;
    }

    /**
    * perimeter calculates the perimeter of the shape.
    */
    virtual double perimeter(void) const
    {
        return M_PI * radius_ * 2.0;
    }

    /**
    * contains determines if the provided Point lies within the enclosed area of the Shape.
    *
    * \param    point           Point to check
    * \return   True if the point is contained in the interior of the Shape.
    */
    virtual bool contains(const Point<T>& point) const
    {
        return contains(point.x, point.y);
    }

    /**
    * contains determines if the provided (x, y) coordinates pair lies within the enclosed area of the Shape
    *
    * \param    x       x-position of the coordinate
    * \param    y       y-position of the coordinate
    * \return   True if the coordinate pair is contained in the interior of the Shape.
    */
    virtual bool contains(double x, double y) const
    {
        return distance_between_points(x, y, center_.x, center_.y) <= radius_;
    }

    /**
    * intersects checks to see if a given line segment intersects the shape.
    *
    * \param    line    Line to check for intersection
    * \return   True if the line intersects the shape.
    */
    virtual bool intersects(const Line<T>& line) const
    {
        return distance_to_line_segment(center_, line) <= radius_;
    }

    /**
    * intersections finds all points at which the provided line intersects with the shape.
    *
    * \param    line            Line for which to find the intersections
    * \param    intersections   Intersections between the line and the shape.
    * \return   True if at least one intersection was found. The intersections are ordered by increasing distance
    *   from the starting point of the line segment. (intersections[0] is closer than intersections[1])
    */
    virtual bool intersections(const Line<T>& line, std::vector<Point<T>>& intersections) const
    {
        /*
        * Using algorithm taken from: http://stackoverflow.com/questions/1073336/circle-line-collision-detection
        * Basic idea is a parametric equation using vectors emanating from the start of the line segment, line.a.
        *
        * Find the two positions, t1 and t2, where the collisions occur. If they are in the range from 0->1, then
        * those are intersections to consider.
        */

        auto aToBVec    = line.b - line.a;
        auto aToCircVec = line.a - center_;

        double a = (aToBVec.x*aToBVec.x + aToBVec.y*aToBVec.y);
        double b = 2.0 * (aToCircVec.x*aToBVec.x + aToCircVec.y*aToBVec.y);
        double c = (aToCircVec.x*aToCircVec.x + aToCircVec.y*aToCircVec.y) - radius_*radius_;

        double discriminant = b*b - 4.0*a*c;

        if(discriminant < 0)
        {
            // no intersection
            return false;
        }
        else // potential intersection
        {
            // calculate the two t-values from the quadratic equation -- either value being in the [0,1] range means it intersects
            // if the t-values are the same, then we have degenerate case where the line is tangent to the circle, which means a single intersection

            double root = std::sqrt(discriminant);
            double t1   = (-b - root) / (2.0*a);
            double t2   = (-b + root) / (2.0*a);

            if(0.0 <= t1 && t1 <= 1.0)
            {
                intersections.push_back(Point<T>(line.a.x + t1*aToBVec.x, line.a.y + t1*aToBVec.y));
            }

            if(0.0 <= t2 && t2 <= 1.0 && t1 != t2)
            {
                intersections.push_back(Point<T>(line.a.x + t2*aToBVec.x, line.a.y + t2*aToBVec.y));

                // Put the closest intersection first
                if(t2 < t1)
                {
                    std::swap(intersections[0], intersections[1]);
                }
            }
        }

        return !intersections.empty();
    }

    /**
    * distanceToPoint calculates the smallest distance between the shape boundary and the
    * provided point. If the point is inside the shape, the distance is 0.
    *
    * \param    point           Point for which the distance will be calculated
    * \return   Minimum distance between the point and the shape.
    */
    virtual double distanceToPoint(const Point<T>& point) const
    {
        return std::max(0.0, distance_between_points(point, center_) - radius_);
    }

    /**
    * distanceFrom calculates the smallest distance between the shape boundary and the
    * provided point.
    *
    * \param    point           Point for which the distance will be calculated
    * \return   Minimum distance between the point and the shape.
    */
    double distanceFromBoundary(const Point<T>& point) const
    {
        return std::abs(distance_between_points(point, center_) - radius_);
    }

    // Mutators

    /**
    * translate moves the shape so it is recentered at center.x + deltaX, center.y + deltaY.
    *
    * \param    deltaX          Amount to translate the x-coordinate
    * \param    deltaY          Amount to translate the y-coordinate
    */
    virtual void translate(T deltaX, T deltaY)
    {
        center_.x += deltaX;
        center_.y += deltaY;
    }

    /**
    * rotate rotates the shape the specified number of radians around its center.
    *
    * \param    radians         Number of radians to rotate the shape
    */
    virtual void rotate(double radians)
    {
        // Nothing to do for a circle
    }

private:

    double         radius_;
    Point<T> center_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( radius_,
            center_);
    }
};


template <typename T>
std::ostream& operator<<(std::ostream& out, const Circle<T>& circle)
{
    out << '{' << circle.center() << ',' << circle.radius() << '}';
    return out;
}

}
}

#endif // MATH_GEOMETRY_CIRCLE_H
