//
// Created by alberto on 31/10/17.
//

#ifndef OP_PALNSSOLUTION_H
#define OP_PALNSSOLUTION_H

#include <experimental/memory>
#include <as/containers.h>
#include <as/graph.h>
#include "PALNSProblemParams.h"
#include "../Graph.h"
#include "../Tour.h"

namespace op {
    struct PALNSSolution {
        /**
         * The underlying graph.
         */
        std::experimental::observer_ptr<const Graph> graph;

        /**
         * Palns Problem-specific Params.
         */
        std::experimental::observer_ptr<const PALNSProblemParams> params;

        /**
         * The tour corresponding to the solution.
         */
        Tour tour;

        /**
         * Free vertices: reachable vertices not included
         * in the tour.
         */
        std::vector<BoostVertex> free_vertices;

        /** Default constructor.
         */
        PALNSSolution() = default;

        /**
         * Constructs an empty solution, given a graph instance.
         * (Required by PALNS).
         *
         * @param graph The underlying graph.
         */
        PALNSSolution(const Graph& graph, const PALNSProblemParams *const params = nullptr);

        /**
         * Constructs a solution from a tour.
         *
         * @param tour The tour.
         */
        PALNSSolution(Tour tour, const PALNSProblemParams *const params = nullptr);

        /**
         * Gets the cost of a solution (the lower the better).
         * (Required by PALNS).
         *
         * @return The prizes *not* collected by the tour.
         */
        double getCost() const;

        /**
         * Removes a vertex from the solution tour.
         *
         * @param vertex    The vertex to remove.
         */
        void remove_vertex(BoostVertex vertex);

        /**
         * Removes a vertex from the solution tour, if it actually
         * is in the solution tour.
         *
         * @param vertex    The vertex to remove.
         * @return          True iff a vertex was removed.
         */
        bool remove_vertex_if_present(BoostVertex vertex);

        /**
         * Adds a vertex to the solution tour, in the specified position.
         *
         * @param vertex    The vertex to add.
         * @param position  The position where to add it (see Tour::add_vertex).
         */
        void add_vertex(BoostVertex vertex, std::size_t position);

        /**
         * Adds a vertex to the solution tour, in the best possible
         * position (i.e. the position which has the best insertion
         * score). It only adds the vertex if the insertion is feasible.
         *
         * @param vertex    The vertex to add.
         * @return          True iff a feasible insertion was possible.
         */
        bool add_vertex_in_best_pos_feasible(BoostVertex vertex);

        /**
         * Like add_vertex_in_best_pos_feasible but heuristically, i.e.
         * it is likely that the vertex is inserted in the best possible
         * position, but there is no guarantee. On the other hand,
         * the method is much faster.
         */
        bool heur_add_vertex_in_best_pos_feasible(BoostVertex vertex);

        /**
         * Adds a vertex to the solution tour, in the best possible
         * position (i.e. the position which has the best insertion
         * score). The insertion might be infeasible, i.e. the resulting
         * tour might violate the maximum travel time.
         *
         * @param vertex    The vertex to add.
         */
        void add_vertex_in_best_pos_any(BoostVertex vertex);

        /**
         * Like add_vertex_in_best_pos_any but heuristically, i.e.
         * it is likely that the vertex is inserted in the best possible
         * position, but there is no guarantee. On the other hand,
         * the method is much faster.
         */
        void heur_add_vertex_in_best_pos_any(BoostVertex vertex);

        /**
         * Returns a list of all possible insertions of all free
         * vertices.
         *
         * @return  The list of insertions.
         */
        std::vector<VertexInsertionPrice> all_insertions() const;

        /**
         * Returns a subset of all possible insertions, for all free
         * vertices. The subset is determined heuristically and is
         * supposed to contain "good" insertions.
         *
         * @return  The list of insertions.
         */
        std::vector<VertexInsertionPrice> heur_all_insertions() const;

        /**
        * Returns a list of all feasible insertions of all free
        * vertices.
        *
        * @return  The list of insertions.
        */
        std::vector<VertexInsertionPrice> feas_insertions() const;

        /**
         * Returns a subset of all feasible insertions, for all free
         * vertices. The subset is determined heuristically and is
         * supposed to contain "good" insertions.
         *
         * @return  The list of insertions.
         */
        std::vector<VertexInsertionPrice> heur_feas_insertions() const;

        /**
         * Removes enough vertices from the tour to make
         * the travel time feasible.
         */
        void make_travel_time_feasible();

    private:

        void find_positions_next_to_neighbour(BoostVertex vertex, BoostVertex neighbour, std::vector<VertexInsertionPrice>& insertions) const;
        void find_positions_next_to_neighbours(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const;
        void find_positions_next_to_nearby_vertices(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const;
        void find_feas_positions_next_to_neighbour(BoostVertex vertex, BoostVertex neighbour, std::vector<VertexInsertionPrice>& insertions) const;
        void find_feas_positions_next_to_neighbours(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const;
        void find_feas_positions_next_to_nearby_vertices(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const;
        void generic_find_positions_next_to_nearby_vertices(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions, bool feasible) const;
    };
}

#endif //OP_PALNSSOLUTION_H