/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     polygon.h
 * \author   Collin Johnson
 *
 * Definition of a Polygon class that implements the Shape interface.
 */

#ifndef MATH_GEOMETRY_POLYGON_H
#define MATH_GEOMETRY_POLYGON_H

#include "core/line.h"
#include "core/point.h"
#include "math/geometry/shape.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <ostream>

// Traits support for use of Polygon with Boost.Geometry modeled as a Ring -- see bottom of file
#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/point_order.hpp>
#include <boost/geometry/core/tag.hpp>
#include <boost/geometry/core/tags.hpp>

namespace vulcan
{
namespace math
{

/**
 * Polygon is a convex shape that consists of a connected set of lines. A Polygon is represented by
 * a series of vertices arranged in counter-clockwise order.
 *
 * Polygons can be generated from a series of points using the convex_hull() function.
 *
 * For simplicity in implementation, the first vertex is also the last vertex to make iterating
 * through the vertices easier.
 */
template <typename T>
class Polygon : public Shape<T>
{
public:
    using ConstIter =
      typename std::vector<Point<T>>::const_iterator;        ///< Iterator type for the vertices of the polygon
    using Iter = typename std::vector<Point<T>>::iterator;   ///< Iterator type for the vertices of the polygon

    /** Default constructor for Polygon. */
    Polygon(void) { }

    /**
     * Constructor for Polygon.
     *
     * This constructor exists for the sake of convex_hull() which uses a deque for storing
     * the points rather than a vector.
     *
     * NOTE: The provided points should define the boundary of the Polygon running counter-clockwise order
     *       around the outside of the polygon.
     *
     * NOTE: The constructor does not currently check the validity of the provided points.
     *
     * \param    begin           Start of the range of vertices
     * \param    end             End of the range of vertices
     */
    template <class PointIterator>
    Polygon(PointIterator begin, PointIterator end) : vertices_(begin, end)
    {
        if (!vertices_.empty() && (vertices_.front() != vertices_.back())) {
            vertices_.push_back(vertices_.front());
        }
    }

    /**
     * Constructor for Polygon.
     *
     * NOTE: The provided points should define the boundary of the Polygon running counter-clockwise order
     *       around the outside of the polygon.
     *
     * NOTE: The constructor does not currently check the validity of the provided points.
     *
     * \param    points         Vertices that comprise the Polygon
     */
    template <typename U>
    explicit Polygon(const std::vector<Point<U>>& points) : Polygon(points.begin(), points.end())
    {
    }

    /**
     * Copy constructor for Polygon.
     *
     * NOTE: The provided points should define the boundary of the Polygon running counter-clockwise order
     *       around the outside of the polygon.
     *
     * NOTE: The constructor does not currently check the validity of the provided points.
     *
     * \param    rhs         Polygon to copy
     */
    template <typename U>
    explicit Polygon(const Polygon<U>& rhs) : Polygon(rhs.begin(), rhs.end())
    {
    }

    /**
     * Assignment operator for Polygon.
     *
     * NOTE: The provided points should define the boundary of the Polygon running counter-clockwise order
     *       around the outside of the polygon.
     *
     * NOTE: The constructor does not currently check the validity of the provided points.
     *
     * \param    rhs         Polygon to copy
     */
    template <typename U>
    Polygon<T>& operator=(const Polygon<U>& rhs)
    {
        vertices_.clear();
        std::copy(rhs.begin(), rhs.end(), std::back_inserter(vertices_));
        return *this;
    }

    // Changes the vertices in the polygon -- for optimization sake
    void vertices(const std::vector<Point<T>>& v) { vertices_ = v; }
    void vertices(std::vector<Point<T>>&& v) { vertices_ = v; }

    const std::vector<Point<T>>& vertices(void) const { return vertices_; }

    // Iterators for accessing the vertices of the polygon
    std::size_t size(void) const { return vertices_.size(); }
    ConstIter begin(void) const { return vertices_.begin(); }
    ConstIter end(void) const { return vertices_.end(); }
    Iter begin(void) { return vertices_.begin(); }
    Iter end(void) { return vertices_.end(); }

    // Shape interface
    /**
     * center calculates the centroid of the Polygon. No center exists for non-regular polygons.
     */
    virtual Point<T> center(void) const
    {
        T xSum = 0;
        T ySum = 0;
        T areaSum = 0;

        for (std::size_t n = 1; n < vertices_.size(); ++n) {
            const Point<T>& pointA = vertices_[n - 1];
            const Point<T>& pointB = vertices_[n];

            T abProduct = (pointA.x * pointB.y) - (pointB.x * pointA.y);
            xSum += (pointA.x + pointB.x) * abProduct;
            ySum += (pointA.y + pointB.y) * abProduct;
            areaSum += abProduct;
        }

        if (areaSum != 0) {
            return Point<T>(xSum / (3 * areaSum), ySum / (3 * areaSum));
        } else {
            return Point<T>();
        }
    }

    /**
     * area calculates the area of the polygon.
     *
     * \return   Area of the polygon.
     */
    virtual double area(void) const
    {
        /*
         * Equation for area of a polygon is:
         *
         *   abs((x1y2 - y1x2) + (x2y3 - y2x3) + ... + (xny1 - ynx1)) / 2
         *
         * Taken from: http://www.mathopenref.com/coordpolygonarea.html
         */

        Point<double> pointA;
        Point<double> pointB;

        double sum = 0.0;

        for (std::size_t n = 1; n < vertices_.size(); ++n) {
            pointA = vertices_[n - 1];
            pointB = vertices_[n];

            sum += (vertices_[n - 1].x * vertices_[n].y) - (vertices_[n - 1].y * vertices_[n].x);
        }

        return std::abs(sum / 2.0);
    }


    /**
     * perimeter calculates the perimeter of the shape.
     */
    virtual double perimeter(void) const
    {
        double total = 0.0;

        for (std::size_t n = 1; n < vertices_.size(); ++n) {
            total += distance_between_points(vertices_[n - 1], vertices_[n]);
        }

        return total;
    }

    /**
     * contains determines if the provided Point lies within the enclosed area of the Polygon.
     *
     * \param    point           Point to check
     * \return   True if the point is contained in the interior of the Polygon.
     */
    bool contains(const Point<T>& point) const { return contains(point.x, point.y); }

    /**
     * contains checks to see if the provided location exists inside the polygon.
     *
     * \param    x               x-position to check
     * \param    y               y-position to check
     * \return   True if the point is contained in the polygon.
     */
    bool contains(double x, double y) const
    {
        // This code is interpreted from: http://www.visibone.com/inpoly/
        if (vertices_.empty()) {
            return false;
        }

        bool yflag0 = (vertices_[vertices_.size() - 1].y >= y);
        bool yflag1 = false;
        bool inside = false;

        for (int i = 0, j = vertices_.size() - 1, end = vertices_.size(); i < end; ++i) {
            yflag1 = (vertices_[i].y >= y);

            if (yflag0 != yflag1) {
                if (((vertices_[i].y - y) * (vertices_[j].x - vertices_[i].x)
                     >= (vertices_[i].x - x) * (vertices_[j].y - vertices_[i].y))
                    == yflag1) {
                    inside = !inside;
                }
            }

            yflag0 = yflag1;
            j = i;
        }

        return inside;
    }

    /**
     * intersects checks to see if the given line segments intersects the polygon at any point.
     *
     * \param    line            Line to check for intersection
     * \return   True if there is an intersection.
     */
    bool intersects(const Line<T>& line) const
    {
        // Check each line segment on the perimeter to see if there is an intersection
        for (std::size_t n = 1; n < vertices_.size(); ++n) {
            if (line_segments_intersect(line, Line<T>(vertices_[n - 1], vertices_[n]))) {
                return true;
            }
        }

        return false;
    }

    /**
     * intersections finds all points at which the provided line intersects with the shape.
     *
     * \param    line    Line for which to find the intersections
     * \param    inters  Intersections between the line and the shape.
     * \return   True if an intersection was found, false otherwise.
     */
    virtual bool intersections(const Line<T>& line, std::vector<Point<T>>& inters) const
    {
        Point<T> temp;

        for (std::size_t n = 1; n < vertices_.size(); ++n) {
            if (line_segment_intersection_point(line, Line<T>(vertices_[n - 1], vertices_[n]), temp)) {
                inters.push_back(temp);
            }
        }

        return !inters.empty();
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
        //         if(contains(point))
        //         {
        //             return 0.0;
        //         }

        double minDistance = HUGE_VAL;
        int minIndex = 0;

        for (std::size_t n = 1; n < vertices_.size(); ++n) {
            Line<T> edge(vertices_[n - 1], vertices_[n]);

            double distance = distance_to_line_segment(point, edge);

            if (distance < minDistance) {
                minDistance = distance;
                minIndex = n;
            }
        }

        return left_of_line(vertices_[minIndex], vertices_[minIndex - 1], point) ? 0.0 : minDistance;
    }

    /**
     * translate moves the shape so it is recentered at center.x + deltaX, center.y + deltaY.
     *
     * \param    deltaX          Amount to translate the x-coordinate
     * \param    deltaY          Amount to translate the y-coordinate
     */
    virtual void translate(T deltaX, T deltaY)
    {
        for (auto& vertex : vertices_) {
            vertex.x += deltaX;
            vertex.y += deltaY;
        }
    }

    /**
     * rotate rotates the shape the specified number of radians around its center.
     *
     * \param    radians         Number of radians to rotate the shape
     */
    virtual void rotate(double radians)
    {
        auto c = center();
        for (auto& vertex : vertices_) {
            vertex = vulcan::rotate(vertex - c, radians) + c;
        }
    }

    virtual void rotate(double radians, Point<T> center)
    {
        for (auto& vertex : vertices_) {
            vertex = vulcan::rotate(vertex - center, radians) + center;
        }
    }

private:
    // INVARIANT: Vertices are stored such that the shape is traced out in counter-clockwise order around the polygons
    // perimeter
    std::vector<Point<T>> vertices_;   ///< Vertices that make-up the polygon

    // Serialization support
    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(vertices_);
    }
};


/** Overloaded output operator for Polygon. */
template <typename T>
std::ostream& operator<<(std::ostream& out, const Polygon<T>& poly)
{
    out << "Polygon: " << poly.size() << " vertices_. ";

    for (auto& vertex : poly) {
        out << "(" << vertex.x << ", " << vertex.y << "), ";
    }

    return out;
}

}   // namespace math
}   // namespace vulcan

namespace boost
{
namespace geometry
{
namespace traits
{

template <typename T>
struct tag<vulcan::math::Polygon<T>>
{
    typedef ring_tag type;
};

template <typename T>
struct closure<vulcan::math::Polygon<T>>
{
    static const closure_selector value = closed;
};


template <typename T>
struct point_order<vulcan::math::Polygon<T>>
{
    static const order_selector value = counterclockwise;
};

}   // namespace traits
}   // namespace geometry
}   // namespace boost

#endif   // MATH_GEOMETRY_POLYGON_H
