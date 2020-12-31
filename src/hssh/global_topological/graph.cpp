/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     graph.cpp
 * \author   Collin Johnson
 *
 * Definition of convert_map_to_graph.
 */

#include "hssh/global_topological/graph.h"
#include "hssh/global_topological/topological_map.h"
#include <boost/range/adaptor/map.hpp>
#include <cassert>

namespace vulcan
{
namespace hssh
{

TopologicalVertex convert_path_segment_to_vertex(const GlobalPathSegment& segment,
                                                 const Point<float>& plusPosition,
                                                 const Point<float>& minusPosition);
TopologicalVertex convert_frontier_path_segment_to_vertex(const GlobalPathSegment& segment,
                                                          const Point<float>& position);

void add_explored_segment(const GlobalPathSegment& segment, const TopologicalMap& map, TopologicalGraph& graph);
void add_frontier_segment(const GlobalPathSegment& segment, const TopologicalMap& map, TopologicalGraph& graph);
GlobalPlace
  transition_place(const GlobalTransition& transition, const GlobalPathSegment& segment, const TopologicalMap& map);


TopologicalGraph convert_map_to_graph(const TopologicalMap& map)
{
    auto& segments = map.segments();

    TopologicalGraph graph;

    for (auto& s : boost::adaptors::values(segments)) {
        if (s->isFrontier()) {
            add_frontier_segment(*s, map, graph);
        } else {
            add_explored_segment(*s, map, graph);
        }
    }

    return graph;
}


TopologicalVertex convert_location_to_vertex(const GlobalLocation& location, const TopologicalMap& map)
{
    if (location.areaType == AreaType::path_segment) {
        assert(map.getPathSegment(location.areaId));

        GlobalPathSegment segment = *map.getPathSegment(location.areaId);

        if (!segment.isFrontier()) {
            const GlobalPlace plus = transition_place(segment.plusTransition(), segment, map);
            const GlobalPlace minus = transition_place(segment.minusTransition(), segment, map);

            return TopologicalVertex(convert_path_segment_to_vertex(segment,
                                                                    map.referenceFrame(plus.id()).toPoint(),
                                                                    map.referenceFrame(minus.id()).toPoint()));
        } else {
            auto transition =
              segment.plusTransition().isFrontier() ? segment.plusTransition() : segment.minusTransition();
            GlobalPlace place = transition_place(transition, segment, map);

            return TopologicalVertex(
              convert_frontier_path_segment_to_vertex(segment, map.referenceFrame(place.id()).toPoint()));
        }
    } else   // at a place
    {
        return convert_place_to_vertex(*map.getPlace(location.areaId), map);
    }
}


TopologicalVertex convert_place_to_vertex(const GlobalPlace& place, const TopologicalMap& map)
{
    Point<float> location = map.referenceFrame(place.id()).toPoint();
    return TopologicalVertex(place.id(), location, NodeData(place.id()));
}


TopologicalVertex convert_path_segment_to_vertex(const GlobalPathSegment& segment,
                                                 const Point<float>& plusPosition,
                                                 const Point<float>& minusPosition)
{
    Point<float> segmentLocation((plusPosition.x + minusPosition.x) / 2.0f, (plusPosition.y + minusPosition.y) / 2.0f);

    return TopologicalVertex(segment.id(), segmentLocation, NodeData(segment.id(), true), segment.lambda().magnitude());
}


TopologicalVertex convert_frontier_path_segment_to_vertex(const GlobalPathSegment& segment,
                                                          const Point<float>& position)
{
    // For frontier segments, put the location halfway down the explored section of the segment.
    Point<float> segmentLocation((position.x + segment.lambda().x) / 2.0f, (position.y + segment.lambda().y) / 2.0f);

    return TopologicalVertex(segment.id(), segmentLocation, NodeData(segment.id(), true), segment.lambda().magnitude());
}


void add_explored_segment(const GlobalPathSegment& segment, const TopologicalMap& map, TopologicalGraph& graph)
{
    // Ids for the edges in the graph are monotonically increasing. Can just use the current number of edges
    // as the benchmark for determining the next id

    auto plus = segment.plusTransition();
    auto minus = segment.minusTransition();

    TopologicalVertex plusVertex = convert_place_to_vertex(transition_place(plus, segment, map), map);
    TopologicalVertex minusVertex = convert_place_to_vertex(transition_place(minus, segment, map), map);
    TopologicalVertex segmentVertex =
      convert_path_segment_to_vertex(segment, plusVertex.getPosition(), minusVertex.getPosition());

    TopologicalEdge plusEdge(graph.numEdges() + 1, plusVertex, segmentVertex, 0.0);
    TopologicalEdge minusEdge(graph.numEdges() + 2, minusVertex, segmentVertex, 0.0);

    graph.addVertex(plusVertex);
    graph.addVertex(minusVertex);
    graph.addVertex(segmentVertex);
    graph.addEdge(plusEdge);
    graph.addEdge(minusEdge);
}


void add_frontier_segment(const GlobalPathSegment& segment, const TopologicalMap& map, TopologicalGraph& graph)
{
    auto transition = segment.plusTransition().isFrontier() ? segment.plusTransition() : segment.minusTransition();

    TopologicalVertex placeVertex = convert_place_to_vertex(transition_place(transition, segment, map), map);
    TopologicalVertex segmentVertex = convert_frontier_path_segment_to_vertex(segment, placeVertex.getPosition());

    TopologicalEdge edge(graph.numEdges() + 1, placeVertex, segmentVertex, 0.0);

    graph.addVertex(placeVertex);
    graph.addVertex(segmentVertex);
    graph.addEdge(edge);
}


GlobalPlace
  transition_place(const GlobalTransition& transition, const GlobalPathSegment& segment, const TopologicalMap& map)
{
    return *map.getPlace(transition.otherArea(segment.toArea()).id());
}

}   // namespace hssh
}   // namespace vulcan
