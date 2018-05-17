//
// Created by alberto on 12/11/17.
//

#ifndef OP_RANDOMSEQREMOVE_H
#define OP_RANDOMSEQREMOVE_H

#include <palns/DestroyMethod.h>
#include "../PALNSSolution.h"
#include "../PALNSProblemParams.h"

namespace op {
    struct RandomSeqRemove : public mlpalns::DestroyMethod<PALNSSolution> {
        /**
         * Problem-specific palns params.
         */
        std::experimental::observer_ptr<const PALNSProblemParams> params;

        /**
         * Empty constructor.
         */
        RandomSeqRemove() = default;

        /**
         * Construct with params.
         */
        RandomSeqRemove(const PALNSProblemParams *const par) :
            params(std::experimental::make_observer(par)) {}

        /**
         * Destructor (required by PALNS).
         */
        ~RandomSeqRemove() override = default;

        /**
         * Clone method (required by PALNS).
         */
        std::unique_ptr<mlpalns::DestroyMethod<PALNSSolution>> clone() const override {
            return std::make_unique<RandomSeqRemove>(*this);
        }

        /**
         * Actual destroy method.
         * (Required by PALNS).
         *
         * @param sol   The solution to destroy.
         * @param mt    A random number generator.
         */
        void destroy_solution(PALNSSolution& sol, std::mt19937& mt) override {
            auto& vertices = sol.tour.vertices;

            if(vertices.size() < 2u) { return; }

            assert(params);

            std::size_t n_vertices_to_remove = static_cast<std::size_t>(
                (vertices.size() - 1) * // - 1 to exclude the depot
                params->destroy.fraction_of_vertices_to_remove
            );

            n_vertices_to_remove = std::min(n_vertices_to_remove, params->destroy.max_n_of_vertices_to_remove);

            if(n_vertices_to_remove < 1u) { return; }

            std::uniform_int_distribution<std::size_t> pv_dist(1u, vertices.size() - 1);
            auto current_pivot = pv_dist(mt);

            std::vector<BoostVertex> vertices_to_remove;
            for(auto i = 0u; i < n_vertices_to_remove; ++i) {
                if(current_pivot == 0u) {
                    ++current_pivot; // Skip the depot
                }

                vertices_to_remove.push_back(vertices[current_pivot]);

                ++current_pivot;
                current_pivot = current_pivot % vertices.size();
            }

            for(const auto& vertex : vertices_to_remove) {
                sol.remove_vertex(vertex);
            }
        }
    };
}

#endif //OP_RANDOMSEQREMOVE_H
