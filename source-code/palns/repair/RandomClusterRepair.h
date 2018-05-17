//
// Created by alberto on 12/11/17.
//

#ifndef OP_RANDOMCLUSTERREPAIR_H
#define OP_RANDOMCLUSTERREPAIR_H

#include <palns/RepairMethod.h>
#include "../PALNSSolution.h"

namespace op {
    struct RandomClusterRepair : public mlpalns::RepairMethod<PALNSSolution> {
        /**
         * Problem-specific palns params.
         */
        std::experimental::observer_ptr<const PALNSProblemParams> params;

        /**
         * Graph clustering.
         */
        std::experimental::observer_ptr<const Clustering> clustering;

        /**
         * Distribution to get a cluster index.
         */
        std::uniform_int_distribution<std::size_t> ci_dist;

        /**
         * Construct with clustering.
         */
        RandomClusterRepair(const PALNSProblemParams *const params, const Clustering *const clustering) :
            params(std::experimental::make_observer(params)),
            clustering(std::experimental::make_observer(clustering)),
            ci_dist(0u, clustering->n_clusters - 1u) {}

        /**
         * Destructor (required by PALNS).
         */
        ~RandomClusterRepair() override = default;

        /**
         * Clone method (required by PALNS).
         */
        std::unique_ptr<mlpalns::RepairMethod<PALNSSolution>> clone() const override {
            return std::make_unique<RandomClusterRepair>(*this);
        }

        /**
         * Actual repair method.
         * (Required by PALNS).
         *
         * @param solution  The solution to repair.
         * @param mt        A random number generator.
         */
        void repair_solution(PALNSSolution& solution, std::mt19937& mt) override {
            using as::containers::contains;

            assert(params);

            const auto& cluster = clustering->clusters[ci_dist(mt)];

            for(const auto& vertex : cluster) {
                if(solution.tour.visits_vertex(vertex)) { continue; }
                if(!solution.graph->g[vertex].reachable) { continue; }

                assert(!solution.graph->g[vertex].depot);
                assert(contains(solution.free_vertices, vertex));

                if(params->repair.heuristic) {
                    if(params->repair.intermediate_infeasible) {
                        solution.heur_add_vertex_in_best_pos_any(vertex);
                    } else {
                        solution.heur_add_vertex_in_best_pos_feasible(vertex);
                    }
                } else {
                    if(params->repair.intermediate_infeasible) {
                        solution.add_vertex_in_best_pos_any(vertex);
                    } else {
                        solution.add_vertex_in_best_pos_feasible(vertex);
                    }
                }
            }

            if(params->repair.intermediate_infeasible && params->repair.use_2opt_before_restoring_feasibility) {
                solution.tour.do_2opt();
            }

            solution.make_travel_time_feasible();
        }
    };
}

#endif //OP_RANDOMCLUSTERREPAIRINFEAS_H
