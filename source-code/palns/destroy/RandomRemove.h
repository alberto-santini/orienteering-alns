//
// Created by alberto on 12/11/17.
//

#ifndef OP_RANDOMREMOVE_H
#define OP_RANDOMREMOVE_H

#include <palns/DestroyMethod.h>
#include <as/random.h>
#include "../PALNSProblemParams.h"
#include "../PALNSSolution.h"

namespace op {
    struct RandomRemove : public mlpalns::DestroyMethod<PALNSSolution> {
    private:
        /**
         * Problem-specific palns params.
         */
        std::experimental::observer_ptr<const PALNSProblemParams> params;

        /**
         * Fraction of vertices to remove.
         */
        float frac_v_remove;

        /**
         * Normal distribution from which to draw the fraction of vertices to remove.
         */
        std::normal_distribution<float> frac_v_remove_dist;

    public:
        void set_frac_v_remove(float fv) {
            frac_v_remove = fv;
            frac_v_remove_dist = std::normal_distribution<float>(fv, fv / 10);
        }

        float get_frac_v_remove() const {
            return frac_v_remove;
        }

        /**
         * Empty constructor.
         */
        RandomRemove() = default;

        /**
         * Construct with params.
         */
        RandomRemove(const PALNSProblemParams *const par) :
            params(std::experimental::make_observer(par)),
            frac_v_remove(par->destroy.fraction_of_vertices_to_remove),
            frac_v_remove_dist(frac_v_remove, frac_v_remove / 10)
        {}

        /**
         * Destructor (required by PALNS).
         */
        ~RandomRemove() override = default;

        /**
         * Clone method (required by PALNS).
         */
        std::unique_ptr<mlpalns::DestroyMethod<PALNSSolution>> clone() const override {
            return std::make_unique<RandomRemove>(*this);
        }

        /**
         * Actual destroy method.
         * (Required by PALNS).
         *
         * @param sol   The solution to destroy.
         * @param mt    A random number generator.
         */
        void destroy_solution(PALNSSolution& sol, std::mt19937& mt) override {
            using as::rnd::sample;

            if(sol.tour.vertices.empty()) { return; }

            assert(params);

            float fv = params->destroy.adaptive ?
                frac_v_remove_dist(mt) :
                frac_v_remove;

            std::size_t n_vertices_to_remove = static_cast<std::size_t>(
                (sol.tour.vertices.size() - 1) * // - 1 to exclude the depot
                fv
            );

            n_vertices_to_remove = std::min(n_vertices_to_remove, params->destroy.max_n_of_vertices_to_remove);

            // The removable vertices are all, but the depot.
            std::vector<BoostVertex> removable_vertices = sol.tour.vertices;
            assert(removable_vertices.front() == 0u);
            removable_vertices.erase(removable_vertices.begin());

            auto vertices_to_remove = sample(removable_vertices, n_vertices_to_remove, mt);

            for(const auto& vertex : vertices_to_remove) {
                sol.remove_vertex(vertex);
            }
        }
    };
}

#endif //OP_RANDOMREMOVE_H
