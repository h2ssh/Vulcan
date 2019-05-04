/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     line_intersections.cpp
* \author   Collin Johnson
*
* Definition of find_line_intersections() function using a sweep-line algorithm.
*/

#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <algorithm>
#include <core/float_comparison.h>
#include <math/geometry/line_intersections.h>

// #define DEBUG_INITIAL_EVENT_QUEUE
// #define DEBUG_EVENT_QUEUE
// #define DEBUG_CURRENT_EVENT
// #define DEBUG_SWEEP_LINE
// #define DEBUG_FINAL_SWEEP_LINE
// #define DEBUG_INTERSECTION
// #define DEBUG_NEW_EVENT

namespace vulcan
{
namespace math
{

const double FUZZY_EQUAL_TOLERANCE = 1e-12;

// Comparison function for two points provide the proper lexicographical sort
// The lines are arranged such that the top left point is the start. If a line is horizontal, the
// leftmost point is the start
inline bool lexicographic_less(const Point<double>& lhs, const Point<double>& rhs)
{
    bool almostEqualY = absolute_fuzzy_equal(lhs.y, rhs.y, FUZZY_EQUAL_TOLERANCE);
    return (!almostEqualY && lhs.y > rhs.y) ||
           (almostEqualY && !absolute_fuzzy_equal(lhs.x, rhs.x, FUZZY_EQUAL_TOLERANCE) && lhs.x < rhs.x);
}

inline bool point_fuzzy_equal(const Point<double>& lhs, const Point<double>& rhs)
{
    return absolute_fuzzy_equal(lhs.x, rhs.x, FUZZY_EQUAL_TOLERANCE) && absolute_fuzzy_equal(lhs.y, rhs.y, FUZZY_EQUAL_TOLERANCE);
}

// Quick function to distinguish start points and end points of lines
// Event comparison provides the proper sorting order
inline Point<double> start_point(const Line<double>& line)
{
    return lexicographic_less(line.a, line.b) ? line.a : line.b;
}

inline Point<double> end_point(const Line<double>& line)
{
    return lexicographic_less(line.a, line.b) ? line.b : line.a;
}

/*
* event_t defines the possible types of events that occur between the sweep line and a
* line in the set.
*/
enum event_t
{
    START_POINT,
    INTERSECTION_POINT,
    END_POINT
};

/*
* line_event_t defines an event between the sweep line and lines in the set. Three types
* of events can occur. Store each in a separate vector to allow for easy access.
*/
struct line_event_t
{
    // Use sets here because the algorithm can potentially find the same intersection between
    // two lines multiple times. Clearly, only a single instance of the line should exist though
    std::set<int> startLines;
    std::set<int> endLines;
    std::set<int> intersectLines;
};

/*
* sweep_line_value_t holds the information necessary for determining the ordering of elements along
* the sweep line.
*/
struct sweep_line_value_t
{
    static Point<double> eventPoint;
    static char          eventNumber;

    int          lineIndex;
    Line<double> line;

    sweep_line_value_t(void) :
                    lineIndex(0)
    {
    }

    sweep_line_value_t(int index, Line<double> line) :
                    lineIndex(index),
                    line(line)
    {
    }

    double distanceToEvent(void) const
    {
//         if(currentEventNumber == eventNumber)
//         {
//             return eventDistance;
//         }

        if(line.a.x == line.b.x)
        {
            eventDistance = eventPoint.x - line.a.x;
        }
        else if(line.a.y == line.b.y)
        {
            // For a horizontal line, the endpoint will fall either left of the start point or between the start point and the endpoint
            // When the line is between the start point and the end point, then the distance to the event point becomes 0 because the line
            // intersects all events that occur in this region
            eventDistance = (start_point(line).x >= eventPoint.x) ? (eventPoint.x - start_point(line).x) : 0.0;
        }
        else
        {
            eventDistance = eventPoint.x - (line.a.x + ((line.b.x-line.a.x) * (eventPoint.y-line.a.y) / (line.b.y-line.a.y)));
        }

        currentEventNumber = eventNumber;

        return eventDistance;
    }

private:

    mutable double eventDistance;
    mutable char   currentEventNumber;
};

Point<double> sweep_line_value_t::eventPoint;
char          sweep_line_value_t::eventNumber = 0;

// Use the lexicographic
struct event_queue_comparison
{
    bool operator()(const Point<double>& lhs, const Point<double>& rhs) const
    {
        return lexicographic_less(lhs, rhs);
    }
};

// The sweep line segments are ordered left-to-right along the x-axis
struct sweep_line_comparison
{
    bool operator()(const sweep_line_value_t& lhs, const sweep_line_value_t& rhs) const
    {
        // Lines left of event have a positive distance to event point and ordering is left-to-right
        double lhsDistance = lhs.distanceToEvent();
        double rhsDistance = rhs.distanceToEvent();

        // Order of the comparison matters here because the fuzzy equal check always needs to happen!
        return !absolute_fuzzy_equal(lhsDistance, rhsDistance, FUZZY_EQUAL_TOLERANCE) && (lhs.distanceToEvent() > rhs.distanceToEvent());
    }
};

/*
* Multimap is used for sweep_line_t because intersections necessarily involve multiple
* lines being at the same point. For C++11, the insertion order of elements with the same key
* in a multimap is preserved. This fact makes switching two points at the intersection trivial.
* Erase them both and then insert in the opposite order.
*/
typedef std::map<Point<double>, line_event_t, event_queue_comparison> event_queue_t;
typedef std::multiset<sweep_line_value_t, sweep_line_comparison>      sweep_line_t;

/*
* intersection_state_t contains the current state of the algorithm. The primary pieces of
* the algorithm are the event queue and the sweep line set. The event queue is a priority
* queue sorted in lexicographical order, y then x. The sweep line set contains the current
* lines intersecting the sweep line in left-to-right order.
*/
struct intersection_state_t
{
    intersection_state_t(const std::vector<Line<double>>& lines) :
                    lines(lines)
    {
    }

    event_queue_t eventQueue;
    sweep_line_t  sweepLineSet;

    std::vector<intersection_point_t> intersections;    // Intersections found between the lines
    const std::vector<Line<double>>& lines;            // Lines in which to find the intersections
    std::set<int>                     ended;            // Lines whose ends have been processed --- don't add these again!
};

// Helpers that handle the individual steps
void queue_initial_events(intersection_state_t& state);
void add_endpoint_event  (const Point<double>& point, int lineIndex, bool startPoint, event_queue_t& queue);

void handle_next_event            (intersection_state_t& state);
void create_new_intersection_point(const Point<double>& point, const line_event_t& event, intersection_state_t& state);
void adjust_sweep_line_order      (const sweep_line_value_t& point, line_event_t& event, intersection_state_t& state);
void add_lines_to_angle_map       (const Point<double>& origin, const std::vector<Line<double>>& lines, const std::set<int>& lineIndices, std::multimap<double, int>& angleMap);
void erase_points_from_sweep_line (const sweep_line_value_t& event, const std::set<int>& lines, sweep_line_t& sweepLine);

void find_new_events(const sweep_line_value_t& currentEvent, intersection_state_t& state);
void check_for_event(sweep_line_t::const_iterator leftLine, sweep_line_t::const_iterator rightLine, intersection_state_t& state);

void print_event_queue(const event_queue_t& queue);
void print_event      (const Point<double>&  eventPoint, const line_event_t&  event);
void print_sweep_line (const sweep_line_t&  line);


// Definition of the functions
std::vector<intersection_point_t> find_line_intersections(const std::vector<Line<double>>& lines)
{
    /*
    * The flow of the algorithm is to add all endpoints of the lines as the initial events. The events
    * are then processed sequentially until the queue is empty. For each event, the adjacent lines are
    * checked for intersection. If they intersect, a new event is created at the intersection point.
    */
    intersection_state_t state(lines);

    queue_initial_events(state);

    #ifdef DEBUG_INITIAL_EVENT_QUEUE
    std::cout<<"DEBUG: Initial event queue contents:\n";
    print_event_queue(state.eventQueue);
    #endif

    while(!state.eventQueue.empty())
    {
        handle_next_event(state);

        state.eventQueue.erase(state.eventQueue.begin());

        #ifdef DEBUG_EVENT_QUEUE
        print_event_queue(state.eventQueue);
        #endif

        #ifdef DEBUG_SWEEP_LINE
        print_sweep_line(state.sweepLineSet);
        #endif
    }

    #ifdef DEBUG_FINAL_SWEEP_LINE
    std::cout<<"DEBUG: Final sweep line contents:\n";
    print_sweep_line(state.sweepLineSet);
    #endif

    return state.intersections;
}


void queue_initial_events(intersection_state_t& state)
{
    bool pointAStart = false;

    // Only add lines whose start and endpoints aren't the same
    for(size_t n = 0; n < state.lines.size(); ++n)
    {
        if(state.lines[n].a.x != state.lines[n].b.x ||
           state.lines[n].a.y != state.lines[n].b.y)
        {
            pointAStart = lexicographic_less(state.lines[n].a, state.lines[n].b);

            add_endpoint_event(state.lines[n].a, n, pointAStart,  state.eventQueue);
            add_endpoint_event(state.lines[n].b, n, !pointAStart, state.eventQueue);
        }
    }
}


void add_endpoint_event(const Point<double>& point, int lineIndex, bool startPoint, event_queue_t& queue)
{
    auto eventIt = queue.find(point);

    // If the event exists, add to the correct set already there
    if(eventIt != queue.end())
    {
        if(startPoint)
        {
            eventIt->second.startLines.insert(lineIndex);
        }
        else
        {
            eventIt->second.endLines.insert(lineIndex);
        }
    }
    else    // no such event exists, so create a new event
    {
        line_event_t event;

        if(startPoint)
        {
            event.startLines.insert(lineIndex);
        }
        else
        {
            event.endLines.insert(lineIndex);
        }

        queue.insert(std::make_pair(point, event));
    }
}


void handle_next_event(intersection_state_t& state)
{
    auto eventIt = state.eventQueue.begin();

    #ifdef DEBUG_CURRENT_EVENT
    std::cout<<"DEBUG: Current event:";
    print_event(eventIt->first, eventIt->second);
    std::cout<<'\n';
    #endif

    line_event_t& event = eventIt->second;

    // Change to the new event point for the rest of the processing
    sweep_line_value_t::eventPoint = eventIt->first;
    ++sweep_line_value_t::eventNumber;

    sweep_line_value_t currentValue;    // current value can be any line in line_event_t, as they'll all have the same
                                        // distance from the event point (0!)
    if(!event.startLines.empty())
    {
        currentValue.lineIndex = *(event.startLines.begin());
        currentValue.line      = state.lines[currentValue.lineIndex];
    }
    else if(!event.endLines.empty())
    {
        currentValue.lineIndex = *(event.endLines.begin());
        currentValue.line      = state.lines[currentValue.lineIndex];
    }
    else if(!event.intersectLines.empty())
    {
        currentValue.lineIndex = *(event.intersectLines.begin());
        currentValue.line      = state.lines[currentValue.lineIndex];
    }

    // Need to check the current sweep line to see if there are any matches, if there are, then they
    // need to be added to the set of intersecting lines. From the deBerg book, this check is equivalent to
    // finding the lines containing the event point
    auto containsRange = state.sweepLineSet.equal_range(currentValue);
    for(auto pointIt = containsRange.first; pointIt != containsRange.second; ++pointIt)
    {
        // Don't add ending lines to the collection of intersect lines, as they would be reinserted into the sweep line -- bad!
        if(event.endLines.find(pointIt->lineIndex) == event.endLines.end() && event.startLines.find(pointIt->lineIndex) == event.startLines.end())
        {
            event.intersectLines.insert(pointIt->lineIndex);

            #ifdef DEBUG_CURRENT_EVENT
            std::cout<<"DEBUG: Added "<<pointIt->lineIndex<<" to event\n";
            #endif
        }
    }

    // If the event contains more than a single line, it must be some sort of intersection
    if(event.startLines.size() + event.endLines.size() + event.intersectLines.size() > 1)
    {
        create_new_intersection_point(eventIt->first, event, state);
    }

    // All of the endLines can be removed from the sweep line now because they can't intersect any new lines
    if(!event.endLines.empty())
    {
        state.ended.insert(event.endLines.begin(), event.endLines.end());

        erase_points_from_sweep_line(currentValue, event.endLines, state.sweepLineSet);
    }

    if(!event.startLines.empty() || !event.intersectLines.empty())
    {
        adjust_sweep_line_order(currentValue, event, state);
    }

    // At this point, currentValue is assigned to one of the line indexes involved in the event
    // so the search for new events can use it without assignment of lineIndex
    find_new_events(currentValue, state);
}


void create_new_intersection_point(const Point<double>& point, const line_event_t& event, intersection_state_t& state)
{
    intersection_point_t intersection;

    intersection.intersection = point;

    intersection.indices.insert(intersection.indices.end(), event.startLines.begin(),     event.startLines.end());
    intersection.indices.insert(intersection.indices.end(), event.endLines.begin(),       event.endLines.end());
    intersection.indices.insert(intersection.indices.end(), event.intersectLines.begin(), event.intersectLines.end());

    state.intersections.push_back(intersection);
}


void adjust_sweep_line_order(const sweep_line_value_t& point, line_event_t& event, intersection_state_t& state)
{
    /*
    * To adjust the sweep line ordering for the event point, the lines need to be ordered so the sweep line passes
    * through them in left-to-right order immediately after the point. This condition is the equivalent of the endpoints
    * being sorted radially from left-to-right in a coordinate frame rotated pi radians, s.t. negative x-axis becomes
    * the positive x-axis centered at the event point. All angles are in the range [0, pi) because points above the
    * sweep line have been considered.
    *
    * Using a map keyed on the angle allows the points to be sorted quickly and then reinserted into the sweep
    * line set.
    */

    #ifdef DEBUG_SWEEP_LINE
    std::cout<<"DEBUG:Adjusting sweep line order:";
    #endif

    std::multimap<double, int> orderedPoints;

    add_lines_to_angle_map(point.eventPoint, state.lines, event.intersectLines, orderedPoints);
    add_lines_to_angle_map(point.eventPoint, state.lines, event.startLines,     orderedPoints);

    // Erase the previous intersection points before inserting these new points
    if(!event.intersectLines.empty())
    {
        std::set<int> toErase;
        for(auto lineIt = event.intersectLines.begin(), lineEnd = event.intersectLines.end(); lineIt != lineEnd; ++lineIt)
        {
            if(state.ended.find(*lineIt) == state.ended.end())
            {
                toErase.insert(*lineIt);
            }
        }

        erase_points_from_sweep_line(point, toErase, state.sweepLineSet);
    }

    for(auto orderedIt = orderedPoints.begin(), orderedEnd = orderedPoints.end(); orderedIt != orderedEnd; ++orderedIt)
    {
        if(state.ended.find(orderedIt->second) == state.ended.end())
        {
            state.sweepLineSet.insert(sweep_line_value_t(orderedIt->second, state.lines[orderedIt->second]));
            #ifdef DEBUG_SWEEP_LINE
            std::cout<<orderedIt->second<<' ';
            #endif
        }
        #ifdef DEBUG_INTERSECTION
        else
        {
            std::cerr<<"WARNING:Ignored adding an intersection that had already ended!\n";
        }
        #endif
    }

    #ifdef DEBUG_SWEEP_LINE
    std::cout<<'\n';
    print_sweep_line(state.sweepLineSet);
    #endif
}


void add_lines_to_angle_map(const Point<double>&              origin,
                            const std::vector<Line<double>>& lines,
                            const std::set<int>&              lineIndices,
                            std::multimap<double, int>&       angleMap)
{
    double        lineAngle = 0.0f;
    Point<double> endpoint;

    for(auto indexIt = lineIndices.begin(), indexEnd = lineIndices.end(); indexIt != indexEnd; ++indexIt)
    {
        endpoint  = end_point(lines[*indexIt]);
        lineAngle = atan2(endpoint.y-origin.y, endpoint.x-origin.x) + M_PI;

        angleMap.insert(std::make_pair(lineAngle, *indexIt));
    }
}


void erase_points_from_sweep_line(const sweep_line_value_t& event, const std::set<int>& lines, sweep_line_t& sweepLine)
{
    auto pointsRange = sweepLine.equal_range(event);

    if(pointsRange.first == pointsRange.second && !lines.empty())
    {
        std::cerr<<"ERROR:Failed to find event! Point:"<<event.eventPoint<<" Dist:"<<event.distanceToEvent()<<" Index:"<<event.lineIndex<<'\n';
    }

    size_t pointsErased = 0;

    while(pointsRange.first != pointsRange.second)
    {
        if(lines.find(pointsRange.first->lineIndex) != lines.end())
        {
            sweepLine.erase(pointsRange.first++);
            ++pointsErased;
        }
        else
        {
            ++pointsRange.first;
        }
    }

    if(pointsErased != lines.size())
    {
        std::cout<<"WARNING:Wanted to erase "<<lines.size()<<" points, but only erased "<<pointsErased<<" points. Performing brute force search"<<std::endl;

        std::cout<<"Current sweep line:";
        print_sweep_line(sweepLine);
        std::cout<<"\nLines to erase:";

        for(auto lineIt = lines.begin(); lineIt != lines.end(); ++lineIt)
        {
            std::cout<<' '<<*lineIt;
        }
        std::cout<<"\nErased:";

        for(auto lineIt = lines.begin(); lineIt != lines.end(); ++lineIt)
        {
            for(auto pointIt = sweepLine.begin(), pointEnd = sweepLine.end(); pointIt != pointEnd; ++pointIt)
            {
                if(*lineIt == pointIt->lineIndex)
                {
                    std::cout<<pointIt->lineIndex<<' ';
                    sweepLine.erase(pointIt);
                    ++pointsErased;
                    break;
                }
            }
        }
        std::cout<<'\n';

//         assert(false);
    }
}


void find_new_events(const sweep_line_value_t& currentEvent, intersection_state_t& state)
{
    /*
    * To find new events, search for an intersection between the two outside segments for the current event
    * point and the adjacent points along the sweep line. For a multiset, the two segments to check are
    * equal_range().first-1 and equal_range().second. If .first == .second, then no segment was found in the
    * set equal to currentEvent. Therefore, only need to check for one intersection: .first-1 and .first
    */

    // there must be some segments intersecting the sweep line, otherwise nothing to do
    if(state.sweepLineSet.empty())
    {
        return;
    }

    auto eventRange = state.sweepLineSet.equal_range(currentEvent);

    // If the leftmost point is the beginning of the set, then there are clearly no points to the left to be checked
    if(eventRange.first != state.sweepLineSet.begin() && eventRange.first != state.sweepLineSet.end())
    {
        auto priorEvent = eventRange.first;
        --priorEvent;

        check_for_event(priorEvent, eventRange.first, state);
    }
    #ifdef DEBUG_NEW_EVENT
    else
    {
        std::cout<<"DEBUG: No line before "<<currentEvent.lineIndex<<" on sweep line\n";
    }
    #endif

    // If the rightmost point is not .end() then there is a segment to investigate to the right
    // Need to check the iterators aren't the same, otherwise same event would be checked for twice!
    if(eventRange.second != state.sweepLineSet.end() && eventRange.second != state.sweepLineSet.begin())
    {
        auto endEvent = eventRange.second;
        --endEvent;

        check_for_event(endEvent, eventRange.second, state);
    }
    #ifdef DEBUG_NEW_EVENT
    else
    {
        std::cout<<"DEBUG: No line after "<<currentEvent.lineIndex<<" on sweep line\n";
    }
    #endif
}


void check_for_event(sweep_line_t::const_iterator leftLine, sweep_line_t::const_iterator rightLine, intersection_state_t& state)
{
    Point<double> intersectionPoint;

    const Line<double>& left  = state.lines[leftLine->lineIndex];
    const Line<double>& right = state.lines[rightLine->lineIndex];

    // If the lines intersect and the intersection doesn't come before the current event point,
    // then there is a new event to be added with these lines being the intersection
    // It might also be the case that the event is occurring at the same place as this event.
    if(line_segment_intersection_point(left, right, intersectionPoint))
    {
        if(!lexicographic_less(intersectionPoint, sweep_line_value_t::eventPoint) ||
            point_fuzzy_equal(intersectionPoint, sweep_line_value_t::eventPoint))
        {
            // Check if the event has already been discovered by some other intersection,
            // If so, just add these indices to the lines to be considered. Otherwise,
            // create a new event and add it to the queue
            auto eventIt = state.eventQueue.find(intersectionPoint);

            if(eventIt != state.eventQueue.end())
            {
                line_event_t& eventToModify = eventIt->second;

                // IMPORTANT: Only add lines that do not end to the set of intersecting lines. If the ending lines
                //            are included in the intersectLines set, then they are added back into the sweepLine after
                //            the line end exists, which then breaks the sweepLine because the lines are never erased
                //            This issue could be handled in the code dealing with intersections, but easier to never
                //            add the points then deal with them again.
                if(eventToModify.endLines.find(leftLine->lineIndex) == eventToModify.endLines.end())
                {
                    eventToModify.intersectLines.insert(leftLine->lineIndex);
                }

                if(eventToModify.endLines.find(rightLine->lineIndex) == eventToModify.endLines.end())
                {
                    eventToModify.intersectLines.insert(rightLine->lineIndex);
                }

                #ifdef DEBUG_INTERSECTION
                std::cout<<"DEBUG: New intersection at existing event point: "<<intersectionPoint<<" Indices:"<<leftLine->lineIndex<<','<<rightLine->lineIndex<<'\n';
                #endif
            }
            else
            {
                line_event_t intersectionEvent;

                intersectionEvent.intersectLines.insert(leftLine->lineIndex);
                intersectionEvent.intersectLines.insert(rightLine->lineIndex);

                state.eventQueue.insert(std::make_pair(intersectionPoint, intersectionEvent));

                #ifdef DEBUG_INTERSECTION
                std::cout<<"DEBUG: New intersection at new event point: "<<intersectionPoint<<" Indices:"<<leftLine->lineIndex<<','<<rightLine->lineIndex<<'\n';
                #endif
            }
        }
        else if(absolute_fuzzy_equal(intersectionPoint.y, sweep_line_value_t::eventPoint.y, FUZZY_EQUAL_TOLERANCE) && intersectionPoint.y < sweep_line_value_t::eventPoint.y)
        {
            std::cerr<<"WARNING:Equal lines, but not really. Diff:"<<(sweep_line_value_t::eventPoint.y - intersectionPoint.y)<<'\n';
        }
    }

    #ifdef DEBUG_INTERSECTION
    if(line_segment_intersection_point(left, right, intersectionPoint))
    {
        if(lexicographic_less(intersectionPoint, sweep_line_value_t::eventPoint) && !point_fuzzy_equal(intersectionPoint, sweep_line_value_t::eventPoint))
//         if(!((absolute_fuzzy_equal(intersectionPoint.y, sweep_line_value_t::eventPoint.y) || intersectionPoint.y < sweep_line_value_t::eventPoint.y)))
        {
            std::cout<<"DEBUG: Intersection above sweep line: sweep:"<<sweep_line_value_t::eventPoint<<" intersection:"<<intersectionPoint<<" Indices:"<<leftLine->lineIndex<<','<<rightLine->lineIndex<<'\n';

            if(absolute_fuzzy_equal(intersectionPoint.y, sweep_line_value_t::eventPoint.y, FUZZY_EQUAL_TOLERANCE) && intersectionPoint.y < sweep_line_value_t::eventPoint.y)
            {
                std::cout<<"WARNING: Less than within the fuzzy boundary!\n";
            }
        }
    }
    else
    {
        std::cout<<"DEBUG: No intersection between "<<leftLine->lineIndex<<','<<rightLine->lineIndex<<'\n';
    }
    #endif
}


void print_event_queue(const event_queue_t& queue)
{
    std::cout<<"DEBUG: event points:";

    for(auto eventIt = queue.begin(), eventEnd = queue.end(); eventIt != eventEnd; ++eventIt)
    {
        std::cout<<'\n';
        print_event(eventIt->first, eventIt->second);
    }

    std::cout<<'\n';
}


void print_event(const Point<double>& eventPoint, const line_event_t& event)
{
    std::cout<<std::setprecision(12)<<eventPoint<<": start:";
    for(auto indexIt = event.startLines.begin(), indexEnd = event.startLines.end(); indexIt != indexEnd; ++indexIt)
    {
        std::cout<<' '<<*indexIt;
    }

    std::cout<<" end:";
    for(auto indexIt = event.endLines.begin(), indexEnd = event.endLines.end(); indexIt != indexEnd; ++indexIt)
    {
        std::cout<<' '<<*indexIt;
    }

    std::cout<<" intersect:";
    for(auto indexIt = event.intersectLines.begin(), indexEnd = event.intersectLines.end(); indexIt != indexEnd; ++indexIt)
    {
        std::cout<<' '<<*indexIt;
    }
}


void print_sweep_line(const sweep_line_t& line)
{
    std::cout<<"DEBUG: sweep line order:";

    double lastDistance = 10000000.0f;

    bool error = false;

    for(auto lineIt = line.begin(), lineEnd = line.end(); lineIt != lineEnd; ++lineIt)
    {
        error |= !(absolute_fuzzy_equal(lastDistance, lineIt->distanceToEvent(), FUZZY_EQUAL_TOLERANCE) || (lineIt->distanceToEvent() < lastDistance));

        std::cout<<' '<<lineIt->lineIndex<<':'<<lineIt->distanceToEvent();

        lastDistance = lineIt->distanceToEvent();
    }

    std::cout<<'\n';

    if(error)
    {
        std::cout<<"ERROR: Sweep line not in ascending order\n";
//         assert(false);
    }
}

} // namespace math
} // namespace vulcan
