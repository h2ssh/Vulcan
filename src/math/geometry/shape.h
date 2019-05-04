/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     shapes.h
* \author   Collin Johnson
*
* Definition of the Shape interface.
*/

#ifndef MATH_GEOMETRY_SHAPE_H
#define MATH_GEOMETRY_SHAPE_H

#include <core/line.h>
#include <core/point.h>
#include <vector>

namespace vulcan
{
namespace math
{
    
/**
* Shape is an interface defining basic attributes for a class that is meant to represent some sort
* of geometric shape.
* 
* Each shape has the following properties:
* 
*   - center
*   - area
*   - perimeter
* 
* The following operations are supported for a shape:
* 
*   - contains          : check if a point is contained in the Shape
*   - intersects        : check if a line intersects the shape
*   - intersections     : finds all intersections between the line and the shape
*   - distanceToPoint   : finds the distance from the boundary of the shape to a point outside the shape
*/
template <typename T>
class Shape
{
public:

    /** Destructor. */
    virtual ~Shape() { }
    
    /**
    * center retrieves the center of the shape.
    */
    virtual Point<T> center(void) const = 0;
    
    /**
    * area calculates the area of the shape.
    */
    virtual double area(void) const = 0;
    
    /**
    * perimeter calculates the perimeter of the shape.
    */
    virtual double perimeter(void) const = 0;
    
    /**
    * contains determines if the provided Point lies within the enclosed area of the Shape.
    *
    * \param    point           Point to check
    * \return   True if the point is contained in the interior of the Shape.
    */
    virtual bool contains(const Point<T>& point) const = 0;

    /**
    * contains determines if the provided (x, y) coordinates pair lies within the enclosed area of the Shape
    *
    * \param    x       x-position of the coordinate
    * \param    y       y-position of the coordinate
    * \return   True if the coordinate pair is contained in the interior of the Shape.
    */
    virtual bool contains(double x, double y) const = 0;

    /**
    * intersects checks to see if a given line segment intersects the shape.
    *
    * \param    line    Line to check for intersection
    * \return   True if the line intersects the shape.
    */
    virtual bool intersects(const Line<T>& line) const = 0;

    /**
    * intersections finds all points at which the provided line intersects with the shape.
    *
    * \param    line        Line for which to find the intersections
    * \param    inters      Intersections between the line and the shape.
    * \return   True if at least one intersection was found.
    */
    virtual bool intersections(const Line<T>& line, std::vector<Point<T>>& inters) const = 0;
    
    /**
    * distanceToPoint calculates the smallest distance between the shape boundary and the
    * provided point. If the point is inside the shape, the distance is 0.
    *
    * \param    point           Point for which the distance will be calculated
    * \return   Minimum distance between the point and the shape.
    */
    virtual double distanceToPoint(const Point<T>& point) const = 0;
    
    // Mutators
    
    /**
    * translate moves the shape so it is recentered at center.x + deltaX, center.y + deltaY.
    * 
    * \param    deltaX          Amount to translate the x-coordinate
    * \param    deltaY          Amount to translate the y-coordinate
    */
    virtual void translate(T deltaX, T deltaY) = 0;
    
    /**
    * rotate rotates the shape the specified number of radians around its center.
    * 
    * \param    radians         Number of radians to rotate the shape
    */
    virtual void rotate(double radians) = 0;
};

} // namespace math
} // namespace vulcan

#endif // MATH_GEOMETRY_SHAPE_H
