//
// Created by alberto on 12/11/17.
//

#ifndef OP_RANDOMCLUSTERREMOVE_H
#define OP_RANDOMCLUSTERREMOVE_H

#include <palns/DestroyMethod.h>
#include <as/random.h>
#include "../PALNSProblemParams.h"
#include "../PALNSSolution.h"

namespace op {
    struct RandomClusterRemove : public mlpalns::DestroyMethod<PALNSSolution> {
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
        RandomClusterRemove(const PALNSProblemParams *const params, const Clustering *const clustering) :
            params(std::experimental::make_observer(params)),
            clustering(std::experimental::make_observer(clustering)),
            ci_dist(0u, clustering->n_clusters - 1u) {}

        /**
         * Empty constructor.
         */
        RandomClusterRemove() = default;

        /**
         * Destructor (required by PALNS).
         */
        ~RandomClusterRemove() override = default;

        /**
         * Clone method (required by PALNS).
         */
        virtual std::unique_ptr<mlpalns::DestroyMethod<PALNSSolution>> clone() const override {
            return std::make_unique<RandomClusterRemove>(*this);
        }

        /**
         * Actual destroy method.
         * (Required by PALNS).
         *
         * @param sol   The solution to destroy.
         * @param mt    A random number generator.
         */
        void destroy_solution(PALNSSolution& sol, std::mt19937& mt) override {
            if(sol.tour.vertices.empty()) { return; }

            assert(params);
            assert(clustering);
            assert(clustering->is_proper());
            assert(clustering->n_clusters > 1u);

            auto cluster_id = ci_dist(mt);
            const auto& cluster = clustering->clusters[cluster_id];

            if(cluster.size() <= params->destroy.max_n_of_vertices_to_remove) {
                // Cluster small enough:
                for(const auto& vertex : cluster) {
                    // Check it we haven't already removed almost all
                    // vertices from the tour!
                    if(sol.tour.vertices.size() == 2u) { break; }

                    sol.remove_vertex_if_present(vertex);
                }
            } else {
                // Cluster potentially too big:
                auto cluster_cpy = cluster;
                std::shuffle(cluster_cpy.begin(), cluster_cpy.end(), mt);
                auto removed_v = 0u;

                do {
                    // Check it we haven't already removed almost all
                    // vertices from the tour!
                    if(sol.tour.vertices.size() == 2u) { break; }

                    auto vertex = cluster_cpy.back();
                    cluster_cpy.pop_back();

                    const bool removed = sol.remove_vertex_if_present(vertex);
                    
                    if(removed) {
                        ++removed_v;
                    }
                } while(
                    !cluster_cpy.empty() &&
                    removed_v <= params->destroy.max_n_of_vertices_to_remove
                );
            }
        }
    };
}

#endif //OP_RANDOMCLUSTERREMOVE_H
