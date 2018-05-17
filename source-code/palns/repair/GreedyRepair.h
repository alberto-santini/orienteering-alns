//
// Created by alberto on 12/11/17.
//

#ifndef OP_GREEDYREPAIR_H
#define OP_GREEDYREPAIR_H

#include <palns/RepairMethod.h>
#include "../PALNSSolution.h"
#include "../PALNSProblemParams.h"

namespace op {
    struct GreedyInsertionTabuItem {
        BoostVertex v1;
        BoostVertex v2;
        std::uint32_t expire;

        GreedyInsertionTabuItem(BoostVertex v1, BoostVertex v2, std::uint32_t expire) :
                v1{v1}, v2{v2}, expire{expire} {}

        bool is_compatible(const VertexInsertionPrice& insertion, const PALNSSolution& solution) const {
            if(v1 == solution.tour.vertices[insertion.position] &&
               v2 == insertion.vertex) { return false; }

            return !(
                v1 == insertion.vertex &&
                v2 == solution.tour.vertices[(insertion.position + 1) % solution.tour.vertices.size()]
            );
        }
    };

    struct GreedyRepair : public mlpalns::RepairMethod<PALNSSolution> {
        /**
         * Problem-specific palns params.
         */
        std::experimental::observer_ptr<const PALNSProblemParams> params;

        /**
         * Use as::containers::swap_erase or the remove_if/erase idiom to delete insertions.
         */
        bool use_swap_erase;

        // Temporary: trying tabu moves.
        static std::uint32_t n_called;
        std::vector<GreedyInsertionTabuItem> tabu;

        /**
         * Construct with params.
         */
        GreedyRepair(const PALNSProblemParams *const p, bool use_swap_erase = false) :
                params(std::experimental::make_observer(p)),
                use_swap_erase(use_swap_erase) {}

        /**
         * Empty constructor.
         */
        GreedyRepair() = default;

        /**
         * Destructor (required by PALNS).
         */
        ~GreedyRepair() override = default;

        /**
         * Clone method (required by PALNS).
         */
        std::unique_ptr<mlpalns::RepairMethod<PALNSSolution>> clone() const override {
            return std::make_unique<GreedyRepair>(*this);
        }

        /**
         * Actual repair method.
         * (Required by PALNS).
         *
         * @param solution  The solution to repair.
         * @param mt        A random number generator.
         */
        void repair_solution(PALNSSolution& solution, std::mt19937&) override {
            assert(params);

            // --- Temporary: trying tabu moves ---
            ++n_called;
            tabu.erase(
                    std::remove_if(tabu.begin(), tabu.end(),
                                   [&] (const auto& t) { return t.expire <= n_called; }
                    ), tabu.end()
            );
            // --- End tabu part ---

            auto insertions = params->repair.heuristic ?
                solution.heur_feas_insertions() :
                solution.feas_insertions();

            if(insertions.empty()) { return; }

            while(true) {
                if(insertions.empty()) {
                    break;
                }

                std::sort(
                    insertions.begin(),
                    insertions.end(),
                    [] (const auto& ins1, const auto& ins2) -> bool {
                        return ins1.score < ins2.score;
                    }
                );

                // --- Temporary: trying tabu moves ---
                auto candidate_insertion_it = insertions.begin();
                while(  candidate_insertion_it != insertions.end() &&
                        !std::all_of(tabu.begin(), tabu.end(), [&] (const auto& t) { return t.is_compatible(*candidate_insertion_it, solution); }))
                {
                    ++candidate_insertion_it;
                }
                if(candidate_insertion_it == insertions.end()) { return; }
                // --- End tabu part ---

                // I need this copy because the insertion gets erased from the list
                // quite early in the following, but I keep referencing it until later.
                auto candidate_insertion = *candidate_insertion_it;

                // --- Temporary: trying tabu moves ---
                tabu.emplace_back(solution.tour.vertices[candidate_insertion.position], candidate_insertion.vertex, n_called + 10000);
                tabu.emplace_back(candidate_insertion.vertex, solution.tour.vertices[(candidate_insertion.position + 1) % (solution.tour.vertices.size())], n_called + 10000);
                // --- End tabu part

                assert(solution.tour.travel_time + candidate_insertion.increase_in_travel_time <= solution.graph->max_travel_time);
                solution.add_vertex(candidate_insertion.vertex, candidate_insertion.position);

                // Remove all other insertions for the inserted vertex.
                // Remove all other insertions for the same position.
                // Remove any insertion which is now travel-time-infeasible. (This works because of the triangle inequality)
                // Also, keep track of which vertices would not violate feasibility if added later.

                // Vertices which can potentially be added later in this position:
                std::unordered_set<BoostVertex> can_add;
                can_add.reserve(solution.free_vertices.size());

                auto insertion_visitor = [&candidate_insertion, &can_add, &solution] (auto& insertion) -> bool {
                    if(insertion.vertex == candidate_insertion.vertex ||
                        solution.tour.travel_time + insertion.increase_in_travel_time > solution.graph->max_travel_time ||
                        insertion.position == candidate_insertion.position
                    ) {
                        return true;
                    }

                    // At the same time, increase by one the position of all
                    // insertions following the current one.
                    if(insertion.position > candidate_insertion.position) {
                        ++insertion.position;
                    }

                    can_add.insert(insertion.vertex);
                    return false;
                };

                if(use_swap_erase) {
                    as::containers::swap_erase(insertions, insertion_visitor);
                } else {
                    insertions.erase(std::remove_if(insertions.begin(), insertions.end(), insertion_visitor), insertions.end());
                }

                // Recompute all insertions for the chosen position and the following one.
                for(const auto& vertex : can_add) {
                    const auto ins1 = solution.tour.price_vertex_insertion(vertex, candidate_insertion.position);
                    if(solution.tour.travel_time + ins1.increase_in_travel_time <= solution.graph->max_travel_time) {
                        insertions.push_back(ins1);
                    }
                    const auto ins2 = solution.tour.price_vertex_insertion(vertex, candidate_insertion.position + 1u);
                    if(solution.tour.travel_time + ins2.increase_in_travel_time <= solution.graph->max_travel_time) {
                        insertions.push_back(ins2);
                    }
                }
            }
        }
    };
}

#endif //OP_GREEDYREPAIR_H
