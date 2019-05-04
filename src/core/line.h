/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef CORE_LINE_H
#define CORE_LINE_H

#include <core/point.h>
#include <core/angle_functions.h>

namespace vulcan
{

/**
* Line is a simple definition of a line as represented by two endpoints.
*/
template <typename T>
struct Line
{
    Line(void)
    {
    }

    Line(const Point<T>& pointA, const Point<T>& pointB)
    : a(pointA)
    , b(pointB)
    {
    }

    Line(T pointAX, T pointAY, T pointBX, T pointBY)
    : a(pointAX, pointAY)
    , b(pointBX, pointBY)
    {
    }

    // Constructor for creating a line from a line using a different type
    template <typename U>
    Line(const Line<U>& copy)
    : a(copy.a)
    , b(copy.b)
    {
    }

    Point<T> a;
    Point<T> b;

    // Serialization support for transmission
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (a,
            b);
    }
};


template <class ostream, typename T>
ostream& operator<<(ostream& out, const Line<T>& line)
{
    out<<line.a<<"->"<<line.b;
    return out;
}

template <typename T, typename U>
bool operator==(const Line<T>& lhs, const Line<U>& rhs)
{
    return (lhs.a == rhs.a) && (lhs.b == rhs.b);
}


template <typename T, typename U>
bool operator!=(const Line<T>& lhs, const Line<U>& rhs)
{
    return !(lhs == rhs);
}


/**
* center computes the center of the line
*
* \return   The length of the Line segment.
*/
template <typename T>
Point<T> center(const Line<T>& line)
{
    return Point<T>((line.a.x + line.b.x) / 2, (line.a.y + line.b.y) / 2);
}


/**
* length determines the length of the Line using our good friend Pythagoras' theorem.
*
* \return   The length of the Line segment.
*/
template <typename T>
double length(const Line<T>& line)
{
    return distance_between_points(line.a, line.b);
}

/**
* slope finds the slope of the line.
*
* \return   The slope of the line. If the line is vertical, then HUGE_VAL is returned.
*/
template <typename T>
double slope(const Line<T>& line)
{
    if(line.a.x == line.b.x)
    {
        return HUGE_VAL;
    }
    else
    {
        return static_cast<double>(line.b.y - line.a.y) / (line.b.x - line.a.x);
    }
}

/**
* direction determines the direction the line points as if it were a vector with tail at .a and head at .b.
*
* \return   atan2(b.y-a.y, b.x-a.x)
*/
template <typename T>
double direction(const Line<T>& line)
{
    return angle_to_point(line.a, line.b);
}


/**
* normal finds the line normal for the input line.
*
* \return   Normal to the line calculated by rotating line.b by pi/2.
*/
template <typename T>
Line<T> normal(const Line<T>& line)
{
    Point<T> center((line.a.x + line.b.x) / 2, (line.a.y + line.b.y) / 2);

    return Line<T>(center, Point<T>(center.x - (line.b.y - line.a.y),
                                    center.y + (line.b.x - line.a.x)));
}

/**
 * left_of_line checks to see if a point is left of the ray starting at endA and running through endB.
 *
 * \param    endA                Starting point of the ray
 * \param    endB                Point on the ray
 * \param    p                   Point to be checked
 * \return   True if left of line. False if on line or right of line.
 */
template <typename T, typename U>
bool left_of_line(const Point<T>& endA, const Point<T>& endB, const Point<U>& p)
{
    /*
     * For the check, consider the case where p is connected via a line to p. There are
     * two lines: (endA, endB) and (endB, p). Determining whether p is left of the ray
     * reduces to determining if (endA, endB, p) turns left or right. From CLRS 33.1,
     * this check is simply whether the cross-product of (endA, endB) and (endA, p) is
     * > 0 (left) = 0 (on line) < 0 (right).
     *
     * Define p1 = (p - endA), p2 = (endB - endA)
     *
     * The cross-product is: p2 x p1 = x2y1 - x1y2
     */

    if(std::is_floating_point<T>::value || std::is_floating_point<U>::value)
    {
        Point<double> p1(static_cast<double>(p.x)    - endA.x, static_cast<double>(p.y)    - endA.y);
        Point<double> p2(static_cast<double>(endB.x) - endA.x, static_cast<double>(endB.y) - endA.y);

        double cross = p2.x*p1.y - p1.x*p2.y;

        if(fabs(cross) < 0.0000001)
        {
            return false;
        }
        return cross > 0.0;
    }
    else
    {
        Point<T> p1(p.x - endA.x, p.y - endA.y);
        Point<T> p2(endB.x - endA.x, endB.y - endA.y);

        T cross = p2.x*p1.y - p1.x*p2.y;
        return cross > 0;
    }
}


/**
* left_of_line checks to see if the provided point is to the left of the line.
*/
template <typename T, typename U>
bool left_of_line(const Line<T>& line, const Point<U>& p)
{
    return left_of_line(line.a, line.b, p);
}

/**
* right_of_line checks to see if the provided point is to the right of the line.
*/
template <typename T, typename U>
bool right_of_line(const Line<T>& line, const Point<U>& p)
{
    if(line.a.x == line.b.x)
    {
        return (p.x > line.b.x);  // if x is right of line
    }
    else if(line.a.y == line.b.y)
    {
        return p.y < line.b.y;
    }

    double m = (line.b.y - line.a.y) / (line.b.x - line.a.x);

    return (m < 0) ? (p.y - line.a.y) > m * (p.x - line.a.x) :
                     (p.y - line.a.y) < m * (p.x - line.a.x);
}

/**
* closet_point_on_line finds the point on the line closest to the provided point.
*
* Essentially, closet_point_on_line() returns the orthogonal projection of the point onto the line.
*
* \return   Projection of p onto the space of the line.
*/
template <typename T, typename U>
Point<T> closest_point_on_line(const Point<T>& p, const Line<U>& line)
{
    // The projection of the point onto the line is    P = (a' * p * a) / (a' * a)

    double ax = line.b.x - line.a.x;
    double ay = line.b.y - line.a.y;
    double aa = ax*ax + ay*ay;

    // Must subtract out line.a from p because the vectors need to share a common origin here, which is line.a
    double aDotp = (ax*(p.x-line.a.x) + ay*(p.y-line.a.y))/aa;

    // Have to then transform back into the original coordinate frame
    return Point<T>(aDotp * ax + line.a.x, aDotp * ay + line.a.y);
}


/**
* closet_point_on_line_segment finds the point on the line closest to the provided point.
*
* closet_point_on_line_segment() returns the orthogonal projection of the point onto the line if the projection falls
* between the endpoints. If the project is beyond an endpoint, then the closest endpoint is returned.
*
* \param    p               Point to project onto line
* \param    line            Line segment for projection
* \return   Projection of p onto the space of the line segment, as described above.
*/
template <typename T, typename U>
Point<double> closest_point_on_line_segment(const Point<T>& p, const Line<U>& line)
{
    // The projection of the point onto the line is    P = (a' * p * a) / (a' * a)
    double ax = line.b.x - line.a.x;
    double ay = line.b.y - line.a.y;
    double aa = ax*ax + ay*ay;

    // Must subtract out line.a from p because the vectors need to share a common origin here, which is line.a
    double aDotP = (ax*(p.x-line.a.x) + ay*(p.y-line.a.y))/aa;

    if(aDotP < 0)
    {
        return line.a;
    }
    else if(aDotP > 1)
    {
        return line.b;
    }
    else
    {
        // Have to then transform back into the original coordinate frame
        return Point<double>(aDotP * ax + line.a.x, aDotP * ay + line.a.y);
    }
}

/**
* is_projection_on_line_segment checks to see if the projection of the given point onto the line defined by
* the line segment is between the endpoints of the line segment.
*
* \param    point           Point to check if it is on the line
* \param    line            Line segment on which to project the point
* \return   True if the projection of the point onto the line is between the endpoints of the line segment.
*/
template <typename T, typename U>
bool is_projection_on_line_segment(const Point<T>& point, const Line<U>& line)
{
    // The projection of the point onto the line is    P = (a' * p * a) / (a' * a)
    double ax = line.b.x - line.a.x;
    double ay = line.b.y - line.a.y;
    double aa = ax*ax + ay*ay;

    // Must subtract out line.a from p because the vectors need to share a common origin here, which is line.a
    double aDotP = (ax*(point.x-line.a.x) + ay*(point.y-line.a.y))/aa;
    return (aDotP >= 0.0) && (aDotP <= 1.0);
}

/**
* distance_to_line calculates the orthogonal distance from the line to the provided point.
*
* \return   Orthogonal distance from point to line.
*/
template <typename T, typename U>
double distance_to_line(const Point<T>& p, const Line<U>& line)
{
    return distance_between_points(closest_point_on_line(p, line), p);
}

/**
* distance_to_line_segment calculates the distance from a point to a line segment. If the projection of the point
* onto the line of which the segment is a part is not contained within the segment, then the distance to the
* closest endpoint is returned.
*
* \param    p           Point for which to get the distance
* \param    line        Line segment to get the distance to
* \return   Distance from the point to the line segment, as described above.
*/
template <typename T, typename U>
double distance_to_line_segment(const Point<T>& p, const Line<U>& line)
{
    return distance_between_points(p, closest_point_on_line_segment(p, line));
}

/**
* distance_to_point_from_line_segment calculates the distance from a point to a line segment.
* This should be a slightly faster version, with an option to return the closest point.
*
* \param    point           Point for which to get the distance
* \param    lineSegment     Line segment to get the distance to
* \return   closestPoint    The closest point on the line segment
* \return   Distance from the point to the line segment, as described above.
*/
template <typename T>
double distance_to_point_from_line_segment(const Point<T>& point, const Line<T>& lineSegment, Point<T>* closestPoint)
{
    double distanceToPoint;

    Point<T> OA = lineSegment.a - point; // pointA - pointO;
    Point<T> OB = lineSegment.b - point; // pointB - pointO;
    Point<T> AB = OB - OA; //
    Point<T> pointOnLine;

    if(inner_product_between_points(AB,OA) > 0) // A is the closest point to the point
    {
        pointOnLine     = lineSegment.a;
        distanceToPoint = point_norm_squared(OA);
    }
    else if(inner_product_between_points(AB,OB) < 0) // B is the closest point to the origin
    {
        pointOnLine     = lineSegment.b;
        distanceToPoint = point_norm_squared(OB);
    }
    else // closest point is on the line segment
    {
        T projectedLength = -(inner_product_between_points(OA,AB) / (AB.x*AB.x + AB.y*AB.y));
        pointOnLine       =  lineSegment.a + Point<T>(projectedLength*AB.x, projectedLength*AB.y);
        distanceToPoint   =  point_norm_squared(pointOnLine - point);
    }

    if(closestPoint)
    {
        *closestPoint = pointOnLine;
    }

    distanceToPoint = std::sqrt(static_cast<double>(distanceToPoint));

    return distanceToPoint;
};


/**
* angle_between_lines calculates the angle of separation between the two lines if they are treated as vectors
* point from line.a to line.b. The calculation is left - right.
*
* \return   Angle between the lines in the range [-pi/2, pi/2].
*/
template <typename T, typename U>
double angle_between_lines(const Line<T>& left, const Line<U>& right)
{
    double angle = angle_diff(atan2(left.b.y-left.a.y, left.b.x-left.a.x),
                              atan2(right.b.y-right.a.y, right.b.x-right.a.x));

    if(angle > M_PI/2.0)
    {
        angle -= M_PI;
    }
    else if(angle < -M_PI/2.0)
    {
        angle += M_PI;
    }

    return angle;
}

// Helper class that handles the common calculation between all of the line/line segment intersection algorithms
template <typename T>
struct line_intersection_data_t
{
    line_intersection_data_t(const Line<T>& lineA, const Line<T>& lineB)
    : lineA(lineA)
    , lineB(lineB)
    {
        /*
        * The following intersection algorithm is pulled from: http://www.faqs.org/faqs/graphics/algorithms-faq/ Section 1.03
        *
        * Let A,B,C,D be 2-space position vectors.  Then the directed line
        * segments AB & CD are given by:
        *
        * AB=A+r(B-A), r in [0,1]
        * CD=C+s(D-C), s in [0,1]
        *
        * If AB & CD intersect, then
        *
        *   A+r(B-A)=C+s(D-C), or
        *
        *   Ax+r(Bx-Ax)=Cx+s(Dx-Cx)
        *   Ay+r(By-Ay)=Cy+s(Dy-Cy)  for some r,s in [0,1]
        *
        * Solving the above for r and s yields
        *
        *        (Ay-Cy)(Dx-Cx)-(Ax-Cx)(Dy-Cy)
        *    r = -----------------------------  (eqn 1)
        *        (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)
        *
        *        (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
        *    s = -----------------------------  (eqn 2)
        *        (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)
        *
        * Let P be the position vector of the intersection point, then
        *
        *    P=A+r(B-A) or
        *
        *    Px=Ax+r(Bx-Ax)
        *    Py=Ay+r(By-Ay)
        *
        * By examining the values of r & s, you can also determine some
        * other limiting conditions:
        *
        *    If 0<=r<=1 & 0<=s<=1, intersection exists
        *        r<0 or r>1 or s<0 or s>1 line segments do not intersect
        *
        *    If the denominator in eqn 1 is zero, AB & CD are parallel
        *    If the numerator in eqn 1 is also zero, AB & CD are collinear.
        *
        * If they are collinear, then the segments may be projected to the x-
        * or y-axis, and overlap of the projected intervals checked.
        *
        * If the intersection point of the 2 lines are needed (lines in this
        * context mean infinite lines) regardless whether the two line
        * segments intersect, then
        *
        *    If r>1, P is located on extension of AB
        *    If r<0, P is located on extension of BA
        *    If s>1, P is located on extension of CD
        *    If s<0, P is located on extension of DC
        *
        * Also note that the denominators of eqn 1 & 2 are identical.
        */

        rNum = (lineA.a.y - lineB.a.y)*(lineB.b.x - lineB.a.x) - (lineA.a.x - lineB.a.x)*(lineB.b.y - lineB.a.y);
        sNum = (lineA.a.y - lineB.a.y)*(lineA.b.x - lineA.a.x) - (lineA.a.x - lineB.a.x)*(lineA.b.y - lineA.a.y);
        den  = (lineA.b.x - lineA.a.x)*(lineB.b.y - lineB.a.y) - (lineA.b.y - lineA.a.y)*(lineB.b.x - lineB.a.x);

//         if(fabs(den) < 0.000001)
//         {
//             den = 0;
//         }
    }

    bool linesIntersect(void) const
    {
        return den != 0;
    }

    bool segmentsIntersect(void) const
    {
        return linesIntersect()           &&
               ((rNum / den) >= -0.0000001) &&
               ((rNum / den) <= 1.0000001)  &&
               ((sNum / den) >= -0.0000001) &&
               ((sNum / den) <= 1.0000001);
    }

    Point<T> intersectionPoint(void) const
    {
        return Point<T>(lineA.a.x + (rNum / den)*(lineA.b.x - lineA.a.x),
                        lineA.a.y + (rNum / den)*(lineA.b.y - lineA.a.y));
    }

    Line<T> lineA;
    Line<T> lineB;

    double rNum;
    double sNum;
    double den;
};


/**
* lines_intersect checks to see if two lines intersect.
*/
template <typename T, typename U>
bool lines_intersect(const Line<T>& lineA, const Line<U>& lineB)
{
    line_intersection_data_t<T> data(lineA, lineB);

    return data.linesIntersect();
}


/**
* line_intersection_point calculates the intersection point between two lines. If
* the lines don't intersect, then intersectionPoint remains unchanged.
*
* \param    intersectionPoint           Point at which the lines intersect (output)
* \return   True if the lines intersect.
*/
template <typename T>
bool line_intersection_point(const Line<T>& lineA, const Line<T>& lineB, Point<T>& intersectionPoint)
{
    line_intersection_data_t<T> data(lineA, lineB);

    if(data.linesIntersect())
    {
        intersectionPoint = data.intersectionPoint();

        return true;
    }
    else
    {
        return false;
    }
}


/**
* line_segments_intersect checks to see if two line segments intersect.
*
* \return   True if the lines intersect.
*/
template <typename T, typename U>
bool line_segments_intersect(const Line<T>& lineA, const Line<U>& lineB)
{
    line_intersection_data_t<T> data(lineA, lineB);

    // If den != 0, then check the r-conditions as above
    // If den == 0, then see if the x-projection of the lines overlap at all, i.e. collinear lines -- yes, very henious looking equation
    return data.segmentsIntersect()                                                ||
           ((data.den == 0) && ((lineA.a.x >= lineB.a.x && lineA.a.x <= lineB.b.x) ||
                                (lineA.b.x >= lineB.a.x && lineA.b.x <= lineB.b.x) ||
                                (lineB.a.x >= lineA.a.x && lineB.a.x <= lineA.b.x) ||
                                (lineB.b.x >= lineA.a.x && lineB.b.x <= lineA.b.x)));
}


/**
* line_segment_intersection_point calculates the intersection point for two line segments.
*
* \return   True if the lines actually intersect, false otherwise.
*/
template <typename T>
bool line_segment_intersection_point(const Line<T>& lineA, const Line<T>& lineB, Point<T>& p)
{
    line_intersection_data_t<T> data(lineA, lineB);

    if(data.segmentsIntersect())
    {
        p = data.intersectionPoint();

        return true;
    }
    else
    {
        return false;
    }
}

} // namespace vulcan

#endif // CORE_LINE_H
