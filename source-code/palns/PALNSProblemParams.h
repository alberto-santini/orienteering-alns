//
// Created by alberto on 12/11/17.
//

#ifndef OP_PROBLEMPARAMS_H
#define OP_PROBLEMPARAMS_H

#include <cstddef>
#include <experimental/filesystem>

namespace op {
    // Problem-specific PALNS parameters.
    struct PALNSProblemParams {
        struct DestroyMethodsParams {
            // Which methods to enable:
            bool enable_random, enable_random_seq, enable_random_cluster;

            /**
             * How many vertices to remove from the tour, as a fraction of
             * the number of vertices in the tour.
             */
            float fraction_of_vertices_to_remove;

            /*
             * If the fraction_of_vertices_to_remove increases during the
             * solution process, up to what value can it increase?
             */
            float max_fraction_of_vertices_to_remove;

            /**
             * Absolute max number of vertices to remove, irrespective of
             * the number of vertices in the tour.
             */
            std::size_t max_n_of_vertices_to_remove;

            /**
             * Tells whether the destroy method should try to change during
             * the course of the solution. E.g. it might increase the number
             * of vertices it removes.
             */
            bool adaptive;

            DestroyMethodsParams();
        };

        struct RepairMethodsParams {
            // Which methods to enable:
            bool enable_greedy, enable_seq_random, enable_seq_by_prize, enable_cluster;

            /**
             * When repairing a solution, should we find the proven optimal
             * best insertions, or be content with a heuristic (good) one?
             */
            bool heuristic;

            /**
             * Tells whether, when repairing a solution,
             * intermediate time-infeasible solutions should be allowed.
             */
            bool intermediate_infeasible;

            /**
             * Tells wether 2-opt should be used on the solution before
             * restoring its feasibility. (Only applies if intermediate_infeasible
             * is true).
             */
            bool use_2opt_before_restoring_feasibility;

            /**
             * When restoring feasibility, should we select the optimal
             * subset of customers to remove? (As opposed to a heuristically
             * good subset.) This is the probability that the optimal
             * subset is sought.
             */
            float restore_feasibility_optimal;

            RepairMethodsParams();
        };

        struct InitialSolutionParams {
            /**
             * Try to use the clustering, if a proper one is available.
             */
            bool use_clustering;

            /**
             * When a clustering is available, should the OP on the reduced graph
             * be solved exactly, with a MIP?
             */
            bool use_mip;

            /**
             * Do local search on the initial solution?
             */
            bool local_search;

            /**
             * Order of the vertices for the greedy heuristic.
             */
            std::string vertex_order;

            InitialSolutionParams();
        };

        struct LocalSearchParams {
            /**
             * Use 2-opt as a local-search procedure.
             */
            bool use_2opt;

            /**
             * Solve the TSP over the tour vertices as a local-search procedure.
             */
            bool use_tsp;

            /**
             * Tells wether we should try to insert unassigned vertices to
             * a tour after shortening it with local search.
             */
            bool fill_tour;

            LocalSearchParams();
        };

        DestroyMethodsParams destroy;
        RepairMethodsParams repair;
        InitialSolutionParams initial_solution;
        LocalSearchParams local_search;

        /**
         * Empty constructor.
         */
        PALNSProblemParams() = default;

        /**
         * Read params from file.
         */
        PALNSProblemParams(std::experimental::filesystem::path params_file);
    };
}

#endif //OP_PROBLEMPARAMS_H
