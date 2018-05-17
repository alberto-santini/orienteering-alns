//
// Created by alberto on 28/10/17.
//

#ifndef OP_GREEDYHEURISTIC_H
#define OP_GREEDYHEURISTIC_H

#include "palns/PALNSProblemParams.h"
#include "Graph.h"
#include "ReducedGraph.h"

namespace op {
    /**
     * Greedy heuristic solver.
     */
    class GreedyHeuristic {
        /**
         * Underlying graph.
         */
        const Graph& graph;

        /**
         * Parameters (using clustering? using mip?).
         */
        const PALNSProblemParams& params;

    public:

        /**
         * Build a greedy heuristic solver.
         *
         * @param graph The underlying graph.
         */
        explicit GreedyHeuristic(const Graph& graph, const PALNSProblemParams& params) :
            graph{graph}, params{params} {}

        /**
         * Produce a greedy heuristic solution.
         *
         * @return  The greedy solution.
         */
        Tour solve() const;

    private:
        Tour solve_with_clustering_and_mip(ReducedGraph& red) const;
        Tour solve_with_clustering_constructive(ReducedGraph& red) const;
        Tour solve_without_clustering() const;
    };
}

#endif //OP_GREEDYHEURISTIC_H
