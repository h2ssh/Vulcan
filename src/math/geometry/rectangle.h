/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rectangle.h
* \author   Collin Johnson
*
* Implementation of a simple Rectangle type. Subclass of the Shape type.
*/

#ifndef MATH_GEOMETRY_RECTANGLE_H
#define MATH_GEOMETRY_RECTANGLE_H

#include <math/geometry/shape.h>
#include <core/line.h>
#include <cereal/access.hpp>

namespace vulcan
{
namespace math
{

enum class RectSide : int
{
    left = 0,
    right,
    front,
    back,
    inside,
};

/**
* Rectangle represents a rectangle, which is a quadrilateral with two pairs of sides with equal lengths.
* A Rectangle is defined by four vertices.
*
* Rectangle implements the Shape interface, providing getVertices and contains methods.
*/
template <typename T>
class Rectangle : public Shape<T>
{
public:

    Point<T> topLeft;               ///< Upper-left vertex of the Rectangle
    Point<T> topRight;              ///< Upper-right vertex of the Rectangle
    Point<T> bottomLeft;            ///< Lower-left vertex of the Rectangle
    Point<T> bottomRight;           ///< Lower-right vertex of the Rectangle

    /** Default constructor for Rectangle. */
    Rectangle(void)
    : topLeft(0, 1)
    , topRight(1, 1)
    , bottomLeft(0, 0)
    , bottomRight(1, 0)
    {
    }

    /** Constructor for Rectangle. */
    Rectangle(const Point<T>& lowerLeft, const Point<T>& upperRight)
    : topRight(upperRight)
    , bottomLeft(lowerLeft)
    {
        // Fill in the other two positions
        bottomRight.x = topRight.x;
        bottomRight.y = bottomLeft.y;

        topLeft.x = bottomLeft.x;
        topLeft.y = topRight.y;
    }

    /** Constructor for Rectangle. */
    Rectangle(const Point<T>& upperLeft, const Point<T>& upperRight, const Point<T>& lowerRight, const Point<T>& lowerLeft)
    : topLeft(upperLeft)
    , topRight(upperRight)
    , bottomLeft(lowerLeft)
    , bottomRight(lowerRight)
    {
    }

    template <typename U>
    Rectangle(const Rectangle<U>& toCopy)
    : topLeft(toCopy.topLeft)
    , topRight(toCopy.topRight)
    , bottomLeft(toCopy.bottomLeft)
    , bottomRight(toCopy.bottomRight)
    {
    }

    /** Overloaded assignment operator. */
    template <typename U>
    Rectangle& operator=(const Rectangle<U>& rhs)
    {
        topLeft     = rhs.topLeft;
        topRight    = rhs.topRight;
        bottomLeft  = rhs.bottomLeft;
        bottomRight = rhs.bottomRight;

        return *this;
    }

    /** Destructor. */
    virtual ~Rectangle(void) { }

    /**
    * area calculates the area of the rectangle.
    */
    virtual double area(void) const
    {
        return distance_between_points(topLeft, bottomLeft) * distance_between_points(topLeft, topRight);
    }

    /**
    * perimeter calculates the perimeter of the rectangle.
    */
    virtual double perimeter(void) const
    {
        return distance_between_points(topLeft, bottomLeft) + distance_between_points(topLeft, topRight) +
                distance_between_points(topRight, bottomRight) + distance_between_points(bottomRight, bottomLeft);
    }

    /**
    * center calculates the center of the rectangle.
    */
    virtual Point<T> center(void) const
    {
        return Point<T>((bottomLeft.x + bottomRight.x + topLeft.x + topRight.x) / 4,
                              (bottomLeft.y + bottomRight.y + topLeft.y + topRight.y) / 4);
    }

    /** contains determines if the provided point is within the bounds of the Rectangle. */
    virtual bool contains(double x, double y) const
    {
        return contains(Point<double>(x, y));
    }

    /**
    * contains determines if the provided Point lies within the enclosed area of the Rectangle.
    *
    * \param    point           Point to check
    * \return   True if the point is contained in the interior of the Rectangle.
    */
    virtual bool contains(const Point<T>& point) const
    {
        /*
         * As described above, just check if the position of the point is to the left of all the lines
         * using the loeft_of_line function with the endpoints moving clockwise around the perimeter
         * of the Rectangle.
         */

        return left_of_line(bottomLeft, bottomRight, point)
            && left_of_line(bottomRight, topRight, point)
            && left_of_line(topRight, topLeft, point)
            && left_of_line(topLeft, bottomLeft, point);
    }

    /**
    * intersects checks to see if the given line intersects the boundary of the Rectangle.
    *
    * \param    line        Line to check for intersection
    * \return   True if the line intersects the rectangle.
    */
    virtual bool intersects(const Line<T>& line) const
    {
        // There are four lines to check for intersection, so create each and check to see if it intersects the given line
        return line_segments_intersect(line, Line<T>(topRight, topLeft))        ||
                line_segments_intersect(line, Line<T>(topLeft, bottomLeft))     ||
                line_segments_intersect(line, Line<T>(bottomLeft, bottomRight)) ||
                line_segments_intersect(line, Line<T>(bottomRight, topRight));
    }

    /**
    * intersections finds all points at which the provided line intersects with the shape.
    *
    * \param    line    Line for which to find the intersections
    * \param    inters  Intersections between the line and the shape (output)
    * \return   True if an intersection was found, false otherwise.
    */
    virtual bool intersections(const Line<T>& line, std::vector<Point<T>>& inters) const
    {
        // Create each of the four lines and check them to see if there is an intersection, if so, snag the point and carry on with life
        Point<T> temp;

        if(line_segment_intersection_point(line, Line<T>(topRight, topLeft), temp))
        {
            inters.push_back(temp);
        }

        if(line_segment_intersection_point(line, Line<T>(topLeft, bottomLeft), temp))
        {
            inters.push_back(temp);
        }

        if(line_segment_intersection_point(line, Line<T>(bottomLeft, bottomRight), temp))
        {
            inters.push_back(temp);
        }

        if(line_segment_intersection_point(line, Line<T>(bottomRight, topRight), temp))
        {
            inters.push_back(temp);
        }

        return !inters.empty();
    }

    /**
    * distanceToPoint calculates the smallest distance between the rectangle boundary and the
    * provided point. If the point is inside the rectangle, the distance is 0.
    *
    * \param    point           Point for which the distance will be calculated
    * \return   Minimum distance between the point and any side of the rectangle.
    */
    virtual double distanceToPoint(const Point<T>& point) const
    {
        if(contains(point))
        {
            return 0.0;
        }

        // Make the points go around the edge counter-clockwise. Whichever is the closest line,
        // the point needs to be to the right of the line or else it must be inside the rectangle
        Line<T> edges[4];
        edges[0] = Line<float>(bottomLeft,  bottomRight);
        edges[1] = Line<float>(topLeft, bottomLeft);
        edges[2] = Line<float>(bottomRight, topRight);
        edges[3] = Line<float>(topRight, topLeft);

        double minDistance = HUGE_VAL;

        for(int n = 0; n < 4; ++n)
        {
            auto closest = closest_point_on_line_segment(point, edges[n]);
            double distance = (point.x - closest.x)*(point.x - closest.x)
                + (point.y - closest.y)*(point.y - closest.y);

            if(distance < minDistance)
            {
                minDistance = distance;
            }
        }

        return std::sqrt(minDistance);
    }

    /**
    * translate moves the shape so it is recentered at center.x + deltaX, center.y + deltaY.
    *
    * \param    deltaX          Amount to translate the x-coordinate
    * \param    deltaY          Amount to translate the y-coordinate
    */
    virtual void translate(T deltaX, T deltaY)
    {
        bottomLeft.x  += deltaX;
        bottomLeft.y  += deltaY;
        bottomRight.x += deltaX;
        bottomRight.y += deltaY;
        topLeft.x     += deltaX;
        topLeft.y     += deltaY;
        topRight.x    += deltaX;
        topRight.y    += deltaY;

    }

    /**
    * rotate rotates the shape the specified number of radians around its center.
    *
    * \param    radians         Number of radians to rotate the shape
    */
    virtual void rotate(double radians)
    {
        auto c = center();

        bottomLeft  = vulcan::rotate(bottomLeft  - c, radians) + c;
        bottomRight = vulcan::rotate(bottomRight - c, radians) + c;
        topLeft     = vulcan::rotate(topLeft     - c, radians) + c;
        topRight    = vulcan::rotate(topRight    - c, radians) + c;
    }

    // Rectangle-specific methods
    /**
    * width calculates the width of the rectangle, as determined by the distance between the bottom left and bottom right points.
    */
    float width(void) const
    {
        return distance_between_points(bottomLeft, bottomRight);
    }

    /**
    * height calculates the height of the rectangle, as determined by the distance between the bottom left and top left points.
    */
    float height(void) const
    {
        return distance_between_points(bottomLeft, topLeft);
    }

    /**
    * aspectRatio calculates the aspect ratio of the bounding box. 0 is a line, 1 is a square.
    */
    float aspectRatio(void) const
    {
        float leftSideLength   = distance_between_points(bottomLeft, topLeft);
        float bottomSideLength = distance_between_points(bottomLeft, bottomRight);

        if(leftSideLength == 0 || bottomSideLength == 0)
        {
            return 0;
        }
        else
        {
            return (leftSideLength < bottomSideLength) ? leftSideLength/bottomSideLength : bottomSideLength/leftSideLength;
        }
    }

    /**
    * overlap calculates the amount of overlap between the rectangles. The overlap is the maximum of the sharedarea/rect area
    * for each of the rectangles.
    *
    * \param    rect        Rectangle with which to determine the overlap
    * \return   The overlap as a ratio of max(sharedarea/rectarea).
    */
    float overlap(const Rectangle<T>& rect) const
    {
        float overlapArea = intersection(rect).area();

        return std::max(overlapArea / area(), overlapArea / rect.area());
    }

    /**
    * intersection finds the intersection between this rectangle and the provided rectangle. The two rectangles
    * should be axis-aligned.
    *
    * \param    rect            Rectangle to find the intersection with
    * \return   Intersecting region of the rectangles. A rectangle of zero area is returned if there is no intersection.
    */
    Rectangle<T> intersection(const Rectangle<T>& rect) const
    {
        Point<float> newTop(std::min(topRight.x, rect.topRight.x), std::min(topRight.y, rect.topRight.y));
        Point<float> newBottom(std::max(bottomLeft.x, rect.bottomLeft.x), std::max(bottomLeft.y, rect.bottomLeft.y));

        // If top right is to the left of new bottom or below, then there's no overlap!
        if(newTop.y <= newBottom.y || newTop.x <= newBottom.x)
        {
            return Rectangle<T>(newTop, newTop);
        }
        else
        {
            return Rectangle<T>(newBottom, newTop);
        }
    }

    /**
    * distanceFromBoundary calculates the distance from the boundary to the point. This is similar to
    * distanceToPoint, except the distance is non-zero if the point is inside the rectangle.
    */
    double distanceFromBoundary(const Point<T>& point) const
    {
        Line<T> edges[4];
        edges[0] = Line<float>(bottomLeft,  bottomRight);
        edges[1] = Line<float>(bottomLeft,  topLeft);
        edges[2] = Line<float>(bottomRight, topRight);
        edges[3] = Line<float>(topLeft,     topRight);

        double minDistance = HUGE_VAL;

        for(int n = 0; n < 4; ++n)
        {
            double distance = distance_to_line_segment(point, edges[n]);

            if(distance < minDistance)
            {
                minDistance = distance;
            }
        }

        return minDistance;
    }

    /**
    * closestPointOnBoundary finds the closest point on the boundary of the rectangle to the provided point.
    *
    * \param    point           Point for which to find closest boundary point
    * \return   Pair: .first = Closest point on the rectangle boundary to point, .second = side
    */
    std::pair<Point<double>, RectSide> closestPointOnBoundary(const Point<T>& point) const
    {
        // Make the points go around the edge counter-clockwise. Whichever is the closest line,
        // the point needs to be to the right of the line or else it must be inside the rectangle
        Line<T> edges[4];
        edges[0] = Line<float>(topLeft, bottomLeft);
        edges[1] = Line<float>(bottomRight, topRight);
        edges[2] = Line<float>(topRight, topLeft);
        edges[3] = Line<float>(bottomLeft,  bottomRight);

        double minDistance = HUGE_VAL;
        Point<double> minDistPoint;
        RectSide side = RectSide::left;

        for(int n = 0; n < 4; ++n)
        {
            auto pointOnBoundary = closest_point_on_line_segment(point, edges[n]);
            double distance = (point.x - pointOnBoundary.x)*(point.x - pointOnBoundary.x)
                + (point.y - pointOnBoundary.y)*(point.y - pointOnBoundary.y);

            if(distance < minDistance)
            {
                minDistance = distance;
                minDistPoint = pointOnBoundary;
                side = static_cast<RectSide>(n);
            }
        }

        return std::make_pair(minDistPoint, side);
    }

private:

    friend class ::cereal::access;

    // Serialization support
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & bottomLeft;
        ar & bottomRight;
        ar & topLeft;
        ar & topRight;
    }
};

/**
* make_major_axis_along_bottom creates a new Rectangle that permutes the endpoints of input rectangle such that
* the boundary from bottomLeft to bottomRight. The Rectangle shape is unchanged, just the order of the endpoints
* changes.
*/
template <typename T>
Rectangle<T> make_major_axis_along_bottom(const Rectangle<T>& rect)
{
    // Is the distance greater along the bottom already?
    if(distance_between_points(rect.bottomLeft, rect.bottomRight) >=
        distance_between_points(rect.bottomLeft, rect.topLeft))
    {
        return rect;
    }
    // Otherwise shift all the endpoints by one place along the border
    {
        return Rectangle<T>(rect.bottomLeft, rect.topLeft, rect.topRight, rect.bottomRight);
    }
}

/** Overloaded '==' operator for Rectangle. */
template <typename T> bool operator==(const Rectangle<T>& lhs, const Rectangle<T>& rhs)
{
    return (lhs.bottomLeft  == rhs.bottomLeft)   &&
            (lhs.bottomRight == rhs.bottomRight) &&
            (lhs.topLeft     == rhs.topLeft)     &&
            (lhs.topRight    == rhs.topRight);
}

template <typename T> bool operator!=(const Rectangle<T>& lhs, const Rectangle<T>& rhs)
{
    return !(lhs == rhs);
}


/** Overloaded output operator for Rectangle. */
template <typename T, class ostream>
ostream& operator<<(ostream& out, const Rectangle<T>& rect)
{
    out<<rect.bottomLeft<<"->"<<rect.topRight;
    return out;
}

} // namespace math
} // namespace vulcan

#endif // MATH_GEOMETRY_RECTANGLE_H
