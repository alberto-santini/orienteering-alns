//
// Created by alberto on 17/10/17.
//

#ifndef OP_TOUR_H
#define OP_TOUR_H

#include <experimental/filesystem>
#include <experimental/memory>
#include "GraphTypes.h"

namespace op {
    // Forward-definition.
    class Graph;

    struct VertexInsertionPrice {
        BoostVertex vertex;
        std::size_t position;
        float increase_in_travel_time;
        float increase_in_prize;
        float score;
    };

    struct VertexRemovalPrice {
        BoostVertex vertex;
        float decrease_in_travel_time;
        float decrease_in_prize;
        float score;
    };

    /**
     * Represents a simple closed tour in the graph.
     */
    struct Tour {
        /**
         * Non-owning pointer to the graph containing the path.
         */
        std::experimental::observer_ptr<const Graph> graph;

        /**
         * List of vertices composing the path.
         */
        std::vector<BoostVertex> vertices;

        /**
         * List of edges composing the path.
         */
        std::vector<BoostEdge> edges;

        /**
         * Total travel time along the tour.
         */
        float travel_time;

        /**
         * Total prize collected along the tour.
         */
        float total_prize;

        /**
         * Default constructor.
         */
        Tour() = default;

        /**
         * Construct by passing the graph and the edge list.
         *
         * @param graph The underlying graph.
         * @param edges The (ordered) edges of the tour.
         */
        Tour(const Graph *const graph, std::vector<BoostEdge> edges);

        /**
         * Construct by passing the graph and the vertex list.
         *
         * @param graph     The underlying graph.
         * @param vertices  The (ordered) vertices of the tour.
         */
        Tour(const Graph *const graph, std::vector<BoostVertex> vertices);

        /**
         * Builds a path reading a solution file.
         *
         * @param graph          The underlying graph.
         * @param solution_file  The file containing a solution.
         */
        Tour(const Graph *const graph, std::experimental::filesystem::path solution_file);

        /**
         * Tells whether the tour visits a vertex.
         *
         * @param v The vertex.
         * @return  True iff the tour visits the vertex.
         */
        bool visits_vertex(const BoostVertex& v) const;

        /**
         * Prints the tour to a png file.
         *
         * @param image_file     The output file.
         */
        void print_to_png(std::experimental::filesystem::path image_file) const;

        /**
         * Tries to reduce the travel time with a 2-opt heuristic.
         */
        void do_2opt();

        /**
         * Removes vertices to make the travel time feasible (heuristically).
         *
         * @return  The list of removed vertices.
         */
        std::vector<BoostVertex> make_travel_time_feasible_naive();

        /**
         * Removes vertices to make the travel time feasible (optimally).
         *
         * @return  The list of removed vertices.
         */
        std::vector<BoostVertex> make_travel_time_feasible_optimal();

        /**
         * Removes a vertex from the tour. The vertex must be visited
         * by the tour, and it cannot be the depot. If the vertex is
         * the only non-depot vertex visited by the tour (and, therefore,
         * the tour length is 2), then this method is a "no-op": we do
         * not remove the vertex.
         *
         * @param vertex    The vertex to remove.
         * @return          True iff the vertex was removed.
         */
        bool remove_vertex(const BoostVertex& vertex);

        /**
         * Removes a vertex from the tour. The vertex might or might not
         * be visited by the tour. If it is, it cannot be the depot.
         * If the vertex is the only non-depot vertex visited by the tour
         * (and, therefore, the tour length is 2), then this method is a
         * "no-op": we do not remove the vertex.
         *
         * @param vertex    The vertex to remove.
         * @return          True iff the vertex was removed.
         */
        bool remove_vertex_if_present(const BoostVertex& vertex);

        /**
         * Removes a vertex from the tour, by an iterator to the vertex
         * in the vertices list. The vertex must be visited
         * by the tour, and it cannot be the depot. If the vertex is
         * the only non-depot vertex visited by the tour (and, therefore,
         * the tour length is 2), then this method is a "no-op": we do
         * not remove the vertex.
         *
         * @param vertex_it     Iterator to the vertex to remove.
         * @return              True iff the vertex was removed.
         */
        bool remove_vertex(std::vector<BoostVertex>::iterator vertex_it);

        /**
         * Removes a vertex from the tour, by its position (index)
         * in the vertices list. The vertex must be visited
         * by the tour, and it cannot be the depot. If the vertex is
         * the only non-depot vertex visited by the tour (and, therefore,
         * the tour length is 2), then this method is a "no-op": we do
         * not remove the vertex.
         *
         * @param vertex_it     Iterator to the vertex to remove.
         * @return              True iff the vertex was removed.
         */
        bool remove_vertex_by_position(std::size_t position);

        /**
         * Adds a vertex to the tour. Position must be a valid position,
         * and corresponds to the index of the vertex immediately preceding
         * the new vertex. Position = 0, means that the vertex is added
         * just after the depot. Position = vertices.size() - 1 means that
         * the vertex is added immediately before going back to the depot.
         *
         * @param vertex    The vertex to add.
         * @param position  The position.
         */
        void add_vertex(const BoostVertex& vertex, std::size_t position);

        /**
         * Prices the removal of the vertex at position "position" in the
         * vertex list. Position must be valid, i.e. between 1 and
         * vertices.size() - 1.
         *
         * @param position  Vertex position in the vertex list.
         * @return          Vertex removal price.
         */
        VertexRemovalPrice price_vertex_removal(std::size_t position) const;

        /**
         * Prices the insertion of a vertex at a certain position.
         * Position must be a valid position, and corresponds to the index of
         * the vertex immediately preceding the new vertex. Position = 0,
         * means that the vertex is added just after the depot.
         * Position = vertices.size() - 1 means that the vertex is added
         * immediately before going back to the depot.
         *
         * @param vertex        The vertex.
         * @param position      The position.
         * @return              Vertex insertion price.
         */
        VertexInsertionPrice price_vertex_insertion(const BoostVertex& vertex, std::size_t position) const;

        /**
         * Checks that the tour is simple, i.e. does not contain repeated vertices.
         *
         * @return  True iff the tour is simple.
         */
        bool is_simple() const;

        /**
         * Recomputes the travel time from scratch and tells whether
         * the one which was saved in member travel_time was correct.
         *
         * @return True iff the travel time was correct.
         */
        bool is_travel_time_correct();

        /**
         * Checks that the vertices and edges vectors agree.
         * 
         * @return True iff they agree.
         */
        bool are_edges_correct() const;

        /**
         * Prints solution details to csv.
         */
        void to_csv(std::experimental::filesystem::path csv_file) const;

        /**
         * Prints solution details to json.
         */
        void to_json(std::experimental::filesystem::path json_file) const;

    private:

        /**
         * Calculate the travel time by summing the edges' travel time.
         */
        void calculate_travel_time();

        /**
         * Calculate the total prize by summing the vertices' prize.
         */
        void calculate_total_prize();

        /**
         * Calculate the vertex vector from the edge vector.
         */
        void calculate_vertices_from_edges();

        /**
         * Calculate the edge vector from the vertex vector.
         */
        void calculate_edges_from_vertices();

        /**
         * Reads a tour from an OPLIB .sol solution.
         */
        void read_from_oplib_solution(std::experimental::filesystem::path solution_file);

        /**
         * Reads a tour from a .json solution.
         */
        void read_from_json(std::experimental::filesystem::path solution_file);
    };

    std::ostream& operator<<(std::ostream& out, const Tour& tour);
}

#endif //OP_TOUR_H
