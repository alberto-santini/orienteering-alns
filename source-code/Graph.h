//
// Created by alberto on 17/10/17.
//

#ifndef OP_GRAPH_H
#define OP_GRAPH_H

#include <experimental/filesystem>
#include <vector>
#include <map>
#include <as/oplib.h>

#include "GraphTypes.h"

#ifndef PROXIMITY_NEIGHBOURS
#define PROXIMITY_NEIGHBOURS 20u
#endif

namespace op {
    /**
     * This class represents the graph on which the OP is defined.
     */
    struct Graph {
        /**
         * File which contains the graph description.
         */
        std::experimental::filesystem::path graph_file;

        /**
         * Orienteering Problem Instance from as.
         */
        std::optional<as::oplib::OPInstance> opi;

        /**
         * Boost graph.
         */
        BoostGraph g;

        /**
         * Boost r-tree.
         */
        BoostRTree rtree;

        /**
         * Maximum allowed travel time.
         */
        float max_travel_time;

        /**
         * Number of vertices in the graph.
         */
        std::size_t n_vertices;

        /**
         * Smallest x coordinate for the vertices.
         */
        float min_x;

        /**
         * Largest x coordinate for the vertices.
         */
        float max_x;

        /**
         * Smallest y coordinate for the vertices.
         */
        float min_y;

        /**
         * Largest y coordinate for the vertices.
         */
        float max_y;

        /**
         * Smallest prize for the vertices.
         */
        float min_prize;

        /**
         * Largest prize for the vertices.
         */
        float max_prize;

        /**
         * Total prize among all vertices.
         */
        double total_prize;

        /**
         * Number of neighbours in proximity_map.
         */
        static constexpr std::size_t n_proximity_neighbours = PROXIMITY_NEIGHBOURS;

        struct ProximityMapEntry {
            BoostVertex vertex;
            float travel_time;
        };

        using ProximityMap = std::map<BoostVertex, std::vector<ProximityMapEntry>>;

        /**
         * Map which associate to each vertex its closest
         * neighbours, ordered by proximity.
         */
        ProximityMap proximity_map;

        /**
         * Default constructor.
         */
        Graph() = default;

        /**
         * Read graph from file.
         *
         * @param The graph file.
         */
        explicit Graph(std::experimental::filesystem::path graph_file);

        /**
         * Builds a graph given the list of vertices and a description
         * of the distance function to use to build the edges, together
         * with the maximum travel time.
         */
        Graph(std::vector<Vertex> vertices, float max_travel_time);

        /**
         * Returns the travel time between two vertices.
         *
         * @param v First vertex.
         * @param w Second vertex.
         * @return  The travel time.
         */
        float travel_time(const BoostVertex& v, const BoostVertex& w) const;

        /**
         * Instance name (i.e. the graph file without extension).
         *
         * @return  The name of this instance.
         */
        std::string instance_name() const { return graph_file.stem(); }

        /**
         * Method needed by PALNS.
         */
        std::size_t getInstanceSize() const { return n_vertices; }

    private:

        /**
         * Generate the rtree.
         */
        void generate_rtree();

        /**
         * Calculates and set the total prize among all vertices.
         */
        void set_total_prize();

        /**
         * Once vertices and edges are built, it generates the proximity map.
         */
        void generate_proximity_map();

        /**
         * Returns the minimum and maximum value of some vertex property
         * among all vertices of the graph.
         *
         * @param prop  The property to access.
         * @return      The min and max coordinates.
         */
        std::pair<float, float> min_max_vertex_property(float(Vertex::*prop)) const;

        /**
         * Gets the min and max x-coordinates among all vertices of the graph.
         *
         * @return  A pair containing min and max x-coordinates.
         */
        std::pair<float, float> min_max_x() const { return min_max_vertex_property(&Vertex::x); };

        /**
         * Gets the min and max y-coordinates among all vertices of the graph.
         *
         * @return A pair containing min and max y-coordinates.
         */
        std::pair<float, float> min_max_y() const { return min_max_vertex_property(&Vertex::y); };

        /**
         * Gets the min and max prizes among all vertices of the graph.
         *
         * @return A pair containing min and max prizes.
         */
        std::pair<float, float> min_max_prize() const { return min_max_vertex_property(&Vertex::prize); };
    };
}

#endif //OP_GRAPH_H
