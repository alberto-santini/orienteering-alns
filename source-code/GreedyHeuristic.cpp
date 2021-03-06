//
// Created by alberto on 28/10/17.
//

#include <as/and_die.h>
#include <as/console.h>
#include <as/random.h>
#include "palns/PALNSSolution.h"
#include "palns/repair/GreedyRepair.h"
#include "GreedyHeuristic.h"
#include "BCSolver.h"

namespace op {
    Tour GreedyHeuristic::solve() const {
        Tour tour;

        if(params.initial_solution.use_clustering) {
            auto red = recursive_reduction(&graph);

            if(red) {
                // If the graph could be reduced, give a
                // cluster-based heuristic solution.

                if(params.initial_solution.use_mip) {
                    tour = solve_with_clustering_and_mip(*red);
                } else {
                    tour = solve_with_clustering_constructive(*red);
                }
            } else {
                tour = solve_without_clustering();
            }
        } else {
            tour = solve_without_clustering();
        }

        if(params.initial_solution.local_search) {
            tour.do_2opt();

            PALNSSolution sol(tour);
            GreedyRepair rep(&params);
            auto mt = as::rnd::get_seeded_mt();

            rep.repair_solution(sol, mt);
            tour = sol.tour;
        }

        if(params.repair.restore_feasibility_optimal > 0.0f) {
            tour.make_travel_time_feasible_optimal();
        } else {
            tour.make_travel_time_feasible_naive();
        }

        return tour;
    }

    Tour GreedyHeuristic::solve_with_clustering_and_mip(ReducedGraph& red) const {
        Tour tour;

        if(red.reduced_graph.n_vertices > 2u) {
            red.reduced_graph.max_travel_time *= 2.75f;

            auto bcs = BCSolver(red.reduced_graph);

            tour = bcs.solve();
            tour.do_2opt();

            red.reduced_graph.max_travel_time /= 2.75f;
        } else {
            std::vector<BoostVertex> vertices = { 0u, 1u };
            tour = Tour(&red.reduced_graph, vertices);
        }

        return project_back_tour(tour, red);
    }

    Tour GreedyHeuristic::solve_without_clustering() const {
        assert(graph.n_vertices >= 2u);

        std::vector<BoostVertex> vertices = { 0u };
        std::vector<BoostVertex> other_vertices(graph.n_vertices - 1u);
        std::iota(other_vertices.begin(), other_vertices.end(), 1u);

        other_vertices.erase(
            std::remove_if(other_vertices.begin(), other_vertices.end(),
                [&] (const auto& v) -> bool {
                    return !graph.g[v].reachable;
                }
            ),
            other_vertices.end()
        );

        if(params.initial_solution.vertex_order == "random") {
            std::shuffle(other_vertices.begin(), other_vertices.end(), as::rnd::get_seeded_mt());
        } else if(params.initial_solution.vertex_order == "prize") {
            std::sort(other_vertices.begin(), other_vertices.end(),
                [&] (const auto& v1, const auto& v2) -> bool {
                    return graph.g[v1].prize < graph.g[v2].prize;
                }
            );
        } else if(params.initial_solution.vertex_order == "distance") {
            std::sort(other_vertices.begin(), other_vertices.end(),
                [&] (const auto& v1, const auto& v2) -> bool {
                    return graph.travel_time(0u, v1) < graph.travel_time(0u, v2);
                }
            );
        } else {
            std::cerr << as::console::warning << "Unrecognised vertex order " << params.initial_solution.vertex_order << "\n";
        }

        Tour tour(&graph, vertices);

        // Use a PALNS Solution to take advantage of all
        // the nice methods it has to add vertices in good
        // positions.
        PALNSSolution sol(tour);

        for(const auto& v : other_vertices) {
            sol.add_vertex_in_best_pos_any(v);
        }

        return sol.tour;
    }

    Tour GreedyHeuristic::solve_with_clustering_constructive(ReducedGraph& red) const {
        assert(red.reduced_graph.n_vertices >= 2u);

        std::vector<BoostVertex> vertices = { 0u };
        Tour tour(&red.reduced_graph, vertices);

        PALNSSolution sol(tour);

        for(auto v = 1u; v < red.reduced_graph.n_vertices; ++v) {
            if(red.reduced_graph.g[v].reachable) {
                sol.add_vertex_in_best_pos_any(v);
            }
        }

        return project_back_tour(sol.tour, red);
    }
}