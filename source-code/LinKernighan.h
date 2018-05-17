//
// Created by alberto on 20/10/17.
//

#ifndef OP_LINKERNIGHAN_H
#define OP_LINKERNIGHAN_H

#include "Tour.h"
#include <vector>

namespace op {
    /**
     * Runs an external solver to provide a TSP solution using the Lin-Kernighan
     * heuristic. The result is a Hamiltonian tour of the vertices specified.
     *
     * @param g           The underlying graph.
     * @param vertices    The subset of vertices for which a tour is required.
     * @param unique_name A unique name for the set of vertices passed to the function.
     *                    This can be used to solve many tsps in parallel, and make sure
     *                    that LKH reads different files with different names.
     * @return            The best tour produced by LKH.
     */
    Tour run_lin_kernighan(const Graph& g, const std::vector<BoostVertex>& vertices, std::string unique_name = "");
}

#endif //OP_LINKERNIGHAN_H
