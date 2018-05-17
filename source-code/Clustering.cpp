//
// Created by alberto on 21/10/17.
//

#include <thread>
#include <as/graph.h>
#include <as/containers.h>
#include "Clustering.h"
#include "RTreeUtils.h"
#include "LinKernighan.h"

namespace op {
    Clustering::Clustering(const Graph *const graph) :
        graph{std::experimental::make_observer(graph)}
    {
        clusters = dbscan(*graph);
        n_clusters = clusters.size();

        assert(std::all_of(
           clusters.begin(),
           clusters.end(),
           [] (const std::vector<BoostVertex>& cluster) -> bool {
               return cluster.size() > 1u;
           }
        ));

        calculate_noise();
        calculate_stats();
    }

    void Clustering::calculate_noise() {
        using as::containers::contains;

        for(const auto& vertex : as::graph::vertices(graph->g)) {
            if(graph->g[vertex].depot) { continue; }
            if(!graph->g[vertex].reachable) { continue; }

            if(std::none_of(
               clusters.begin(),
               clusters.end(),
               [&vertex] (const std::vector<BoostVertex>& cluster) -> bool {
                   return contains(cluster, vertex);
               }
            )) {
                noise.push_back(vertex);
            }
        }
    }

    void Clustering::calculate_stats() {
        centres = std::vector<BoostPoint>(clusters.size());
        prizes = std::vector<float>(clusters.size());

        std::vector<std::thread> threads(clusters.size());
        auto calc_cp = [this] (std::size_t id) -> void {
            float prize = 0.0f;
            float x_tot = 0.0f;
            float y_tot = 0.0f;

            for(const auto& vertex : clusters[id]) {
                auto vprize = graph->g[vertex].prize;

                prize += vprize;
                x_tot += graph->g[vertex].x * vprize;
                y_tot += graph->g[vertex].y * vprize;
            }

            centres[id] = BoostPoint(x_tot / prize, y_tot / prize);
            prizes[id] = prize;
        };

        for(auto i = 0u; i < clusters.size(); ++i) {
            threads[i] = std::thread(calc_cp, i);
        }

        for(auto& thread : threads) {
            thread.join();
        }
    }

    bool Clustering::is_proper() const {
        return (1 < n_clusters && n_clusters < graph->n_vertices);
    }
}