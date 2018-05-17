//
// Created by alberto on 24/10/17.
//

#ifndef OP_BCSOLVER_H
#define OP_BCSOLVER_H

#include "Graph.h"
#include "Tour.h"

namespace op {
    /**
     * Branch-and-cut solver for the orienteering problem.
     */
    class BCSolver {
        /**
         * The underlying graph.
         */
        const Graph& graph;

    public:

        /**
         * Builds a solver for the OP on a graph.
         *
         * @param graph The underlying graph.
         */
        BCSolver(const Graph& graph) : graph{graph} {}

        /**
         * Solves the OP with branch-and-cut.
         *
         * @return The optimal tour.
         */
        Tour solve() const;
    };
}

#endif //OP_BCSOLVER_H
