//
// Created by alberto on 21/10/17.
//

#ifndef OP_GRAPHTYPES_H
#define OP_GRAPHTYPES_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace op {
/**
     * Represents a vertex of the graph.
     */
    struct Vertex {
        /**
         * Unique, progressive, vertex id.
         */
        std::size_t id;

        /**
         * True iff the vertex is the depot for the instance.
         */
        bool depot;

        /**
         * True iff the vertex is reachable from the depot.
         * Unreachable vertices should not be considered when
         * building solutions.
         */
        bool reachable;

        /**
         * X-coordinate.
         */
        float x;

        /**
         * Y-coordinate.
         */
        float y;

        /**
         * Prize collected at the vertex.
         */
        float prize;
    };

    /**
     * Represents an edge of the graph.
     */
    struct Edge {
        /**
         * Unique, progressive, edge id.
         */
        std::size_t id;

        /**
         * Travel time along the edge.
         */
        float travel_time;
    };

    /**
     * Underlying boost graph type.
     */
    using BoostGraph = boost::adjacency_list<
        boost::vecS,         // Store vertices in a vector
        boost::vecS,         // Store out-edges in a vector
        boost::undirectedS,  // Undirected graph
        Vertex,              // Vertex type
        Edge>;               // Edge type

    /**
     * Convenient typedef for a boost vertex.
     */
    using BoostVertex = boost::graph_traits<BoostGraph>::vertex_descriptor;

    /**
     * Convenient typedef for a boost edge.
     */
    using BoostEdge = boost::graph_traits<BoostGraph>::edge_descriptor;

    /**
     * Point type used in the r-tree.
     */
    using BoostPoint = boost::geometry::model::d2::point_xy<float>;

    /**
     * Box type used to find values in the r-tree.
     */
    using BoostBox = boost::geometry::model::box<BoostPoint>;

    /**
     * Value to store in the r-tree. We store points, but the r-tree
     * also needs an index to access them in ~O(1). The index is going
     * to be a BoostVertex, so to make the conversion easier.
     */
    using BoostTreeValue = std::pair<BoostPoint, BoostVertex>;

    /**
     * R-tree used to store the vertex values. Points in this R-tree
     * can be queried, and most queries take O(log n) time, which is
     * a big speedup vs linear search.
     * TODO: benchmark alternatives to R-Star.
     */
    using BoostRTree = boost::geometry::index::rtree<
    BoostTreeValue, // Type of data to store in the tree.
    boost::geometry::index::rstar<16, 4>>; // For now, we use R-star.
}

#endif //OP_GRAPHTYPES_H
