//
// Created by alberto on 12/11/17.
//

#ifndef OP_SEQVERTEXREPAIR_H
#define OP_SEQVERTEXREPAIR_H

#include <palns/RepairMethod.h>
#include "../PALNSSolution.h"

namespace op {
    struct SeqVertexRepair : public mlpalns::RepairMethod<PALNSSolution> {
        using VertexSorter = std::function<void(std::vector<BoostVertex>&, std::mt19937&)>;

        /**
         * Problem-specific palns params.
         */
        std::experimental::observer_ptr<const PALNSProblemParams> params;

        /**
         * Function use to decide in which order vertices are
         * evaluated for insertion.
         */
        VertexSorter sort;

        /**
         * Normal constructor.
         */
        SeqVertexRepair(const PALNSProblemParams *const params, VertexSorter vertex_sorter) :
            params(std::experimental::make_observer(params)),
            sort(vertex_sorter) {}

        /**
         * Destructor (required by PALNS).
         */
        ~SeqVertexRepair() override = default;

        /**
         * Clone method (required by PALNS).
         */
        std::unique_ptr<mlpalns::RepairMethod<PALNSSolution>> clone() const override {
            return std::make_unique<SeqVertexRepair>(*this);
        }

        /**
         * Actual repair method.
         * (Required by PALNS).
         *
         * @param solution  The solution to repair.
         * @param mt        A random number generator.
         */
        void repair_solution(PALNSSolution& solution, std::mt19937& mt) override {
            assert(params);

            auto vertices = solution.free_vertices;
            sort(vertices, mt);

            auto zo_dist = std::uniform_real_distribution<float>(0.0, 1.0);
            auto n_v_insert = vertices.size() * zo_dist(mt);

            for(auto v_id = 0u; v_id < n_v_insert; ++v_id) {
                if(params->repair.heuristic) {
                    if(params->repair.intermediate_infeasible) {
                        solution.heur_add_vertex_in_best_pos_any(vertices[v_id]);
                    } else {
                        solution.heur_add_vertex_in_best_pos_feasible(vertices[v_id]);
                    }
                } else {
                    if(params->repair.intermediate_infeasible) {
                        solution.add_vertex_in_best_pos_any(vertices[v_id]);
                    } else {
                        solution.add_vertex_in_best_pos_feasible(vertices[v_id]);
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

#endif //OP_SEQVERTEXREPAIRINFEAS_H
