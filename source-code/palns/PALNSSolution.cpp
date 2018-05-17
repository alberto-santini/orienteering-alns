//
// Created by alberto on 02/11/17.
//

#include <as/containers.h>
#include <thread>
#include "../RTreeUtils.h"
#include "PALNSSolution.h"

namespace op {
    PALNSSolution::PALNSSolution(const Graph& graph, const PALNSProblemParams *const params) :
        graph{std::experimental::make_observer(&graph)},
        params{params},
        tour{Tour()}
    {
        for(const auto& vertex : as::graph::vertices(graph.g)) {
            if(!graph.g[vertex].depot &&
                graph.g[vertex].reachable)
            {
                free_vertices.push_back(vertex);
            }
        }
    }

    PALNSSolution::PALNSSolution(Tour tour, const PALNSProblemParams *const params) :
        graph{tour.graph},
        params{params},
        tour{tour}
    {
        using as::containers::contains;

        for(const auto& vertex : as::graph::vertices(graph->g)) {
            if(!graph->g[vertex].depot &&
                graph->g[vertex].reachable &&
               !contains(tour.vertices, vertex))
            {
                free_vertices.push_back(vertex);
            }
        }
    }

    double PALNSSolution::getCost() const {
        if(graph) {
            return graph->total_prize - tour.total_prize;
        } else {
            return std::numeric_limits<double>::max();
        }
    }

    void PALNSSolution::remove_vertex(BoostVertex vertex) {
        using as::containers::contains;

        assert(tour.visits_vertex(vertex));
        assert(!contains(free_vertices, vertex));

        // If the tour only contained one non-depot vertex, remove_vertex will not remove it.
        if(tour.remove_vertex(vertex)) {
            free_vertices.push_back(vertex);
            assert(!tour.visits_vertex(vertex));
        }
    }

    bool PALNSSolution::remove_vertex_if_present(BoostVertex vertex) {
        if(tour.remove_vertex_if_present(vertex)) {
            free_vertices.push_back(vertex);
            assert(!tour.visits_vertex(vertex));
            return true;
        }
        return false;
    }

    void PALNSSolution::add_vertex(BoostVertex vertex, std::size_t position) {
        using as::containers::contains;

        assert(!tour.visits_vertex(vertex));
        assert(position < tour.vertices.size());
        assert(contains(free_vertices, vertex));

        tour.add_vertex(vertex, position);

        free_vertices.erase(
            std::remove(
                free_vertices.begin(),
                free_vertices.end(),
                vertex
            ),
            free_vertices.end()
        );

        assert(tour.visits_vertex(vertex));
        assert(!contains(free_vertices, vertex));
    }

    namespace {
        float c_style_rand_01() {
            return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
        }
    }

    void PALNSSolution::make_travel_time_feasible() {
        assert(params);

        std::vector<BoostVertex> removed_vertices;

        if(c_style_rand_01() < params->repair.restore_feasibility_optimal) {
            tour.make_travel_time_feasible_optimal();
        } else {
            tour.make_travel_time_feasible_naive();
        }

        free_vertices.insert(
          free_vertices.end(),
          removed_vertices.begin(),
          removed_vertices.end()
        );

        assert(std::none_of(
            removed_vertices.begin(), removed_vertices.end(),
            [&] (const BoostVertex& v) -> bool { return tour.visits_vertex(v); }
        ));
    }

    bool PALNSSolution::add_vertex_in_best_pos_feasible(BoostVertex vertex) {
        std::vector<VertexInsertionPrice> insertions;

        for(auto position = 0u; position < tour.vertices.size(); ++position) {
            insertions.push_back(tour.price_vertex_insertion(vertex, position));
        }

        assert(!insertions.empty());

        std::sort(
            insertions.begin(),
            insertions.end(),
            [] (const auto& ins1, const auto& ins2) -> bool {
                return ins1.score < ins2.score;
            }
        );

        for(const auto& insertion : insertions) {
            if(tour.travel_time + insertion.increase_in_travel_time <= graph->max_travel_time) {
                add_vertex(insertion.vertex, insertion.position);
                return true;
            }
        }

        return false;
    }

    bool PALNSSolution::heur_add_vertex_in_best_pos_feasible(BoostVertex vertex) {
        std::vector<VertexInsertionPrice> insertions;

        find_positions_next_to_neighbours(vertex, insertions);

        if(insertions.empty()) {
            find_positions_next_to_nearby_vertices(vertex, insertions);
        }

        if(insertions.empty()) { return false; }

        std::sort(
            insertions.begin(),
            insertions.end(),
            [] (const auto& ins1, const auto& ins2) -> bool {
                return ins1.score < ins2.score;
            }
        );

        for(const auto& insertion : insertions) {
            if(tour.travel_time + insertion.increase_in_travel_time <= graph->max_travel_time) {
                add_vertex(insertion.vertex, insertion.position);
                return true;
            }
        }

        return false;
    }

    void PALNSSolution::add_vertex_in_best_pos_any(BoostVertex vertex) {
        VertexInsertionPrice best_insertion = tour.price_vertex_insertion(vertex, 0u);

        for(auto position = 1u; position < tour.vertices.size(); ++position) {
            auto i = tour.price_vertex_insertion(vertex, position);
            if(i.score < best_insertion.score) {
                best_insertion = i;
            }
        }

        add_vertex(best_insertion.vertex, best_insertion.position);
    }

    void PALNSSolution::heur_add_vertex_in_best_pos_any(BoostVertex vertex) {
        std::vector<VertexInsertionPrice> insertions;

        find_positions_next_to_neighbours(vertex, insertions);

        if(insertions.empty()) {
            find_positions_next_to_nearby_vertices(vertex, insertions);
        }

        if(insertions.empty()) {
            return add_vertex_in_best_pos_any(vertex);
        }

        VertexInsertionPrice best_insertion = *std::min_element(
            insertions.begin(),
            insertions.end(),
            [] (const auto& ins1, const auto& ins2) -> bool {
                return ins1.score < ins2.score;
            }
        );

        add_vertex(best_insertion.vertex, best_insertion.position);
    }

    std::vector<VertexInsertionPrice> PALNSSolution::all_insertions() const {
        std::vector<VertexInsertionPrice> insertions;

        for(const auto& vertex : free_vertices) {
            if(!graph->g[vertex].reachable) { continue; }
            for(auto position = 0u; position < tour.vertices.size(); ++position) {
                insertions.push_back(tour.price_vertex_insertion(vertex, position));
            }
        }

        return insertions;
    }

    std::vector<VertexInsertionPrice> PALNSSolution::feas_insertions() const {
        std::vector<VertexInsertionPrice> insertions;

        for(const auto& vertex : free_vertices) {
            if(!graph->g[vertex].reachable) { continue; }
            for(auto position = 0u; position < tour.vertices.size(); ++position) {
                const auto insertion = tour.price_vertex_insertion(vertex, position);
                if(insertion.increase_in_travel_time + tour.travel_time <= graph->max_travel_time) {
                    insertions.push_back(insertion);
                }
            }
        }

        return insertions;
    }

    std::vector<VertexInsertionPrice> PALNSSolution::heur_all_insertions() const {
        std::vector<VertexInsertionPrice> insertions;

        for(const auto& vertex : free_vertices) {
            if(!graph->g[vertex].reachable) { continue; }

            find_positions_next_to_neighbours(vertex, insertions);

            if(insertions.empty()) {
                find_positions_next_to_nearby_vertices(vertex, insertions);
            }
        }

        assert(!insertions.empty());

        return insertions;
    }

    std::vector<VertexInsertionPrice> PALNSSolution::heur_feas_insertions() const {
        std::vector<VertexInsertionPrice> insertions;

        for(const auto& vertex : free_vertices) {
            if(!graph->g[vertex].reachable) { continue; }

            find_feas_positions_next_to_neighbours(vertex, insertions);

            if(insertions.empty()) {
                find_feas_positions_next_to_nearby_vertices(vertex, insertions);
            }
        }

        return insertions;
    }

    void PALNSSolution::find_positions_next_to_neighbour(BoostVertex vertex, BoostVertex neighbour, std::vector<VertexInsertionPrice>& insertions) const {
        const auto nvertex_it = std::find(tour.vertices.begin(), tour.vertices.end(), neighbour);

        if(nvertex_it != tour.vertices.end()) {
            const auto position = nvertex_it - tour.vertices.begin();

            assert(position >= 1u);

            insertions.push_back(tour.price_vertex_insertion(vertex, position - 1));
            insertions.push_back(tour.price_vertex_insertion(vertex, position));
        }
    }

    void PALNSSolution::find_feas_positions_next_to_neighbour(BoostVertex vertex, BoostVertex neighbour, std::vector<VertexInsertionPrice>& insertions) const {
        const auto nvertex_it = std::find(tour.vertices.begin(), tour.vertices.end(), neighbour);

        if(nvertex_it != tour.vertices.end()) {
            const auto position = nvertex_it - tour.vertices.begin();

            assert(position >= 1u);

            const auto ins1 = tour.price_vertex_insertion(vertex, position - 1);
            const auto ins2 = tour.price_vertex_insertion(vertex, position);

            if(ins1.increase_in_travel_time + tour.travel_time <= graph->max_travel_time) {
                insertions.push_back(ins1);
            }

            if(ins2.increase_in_travel_time + tour.travel_time <= graph->max_travel_time) {
                insertions.push_back(ins2);
            }
        }
    }

    void PALNSSolution::find_positions_next_to_neighbours(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const {
        for(const auto& nvertex : graph->proximity_map.at(vertex)) {
            find_positions_next_to_neighbour(vertex, nvertex.vertex, insertions);
        }
    }

    void PALNSSolution::find_feas_positions_next_to_neighbours(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const {
        for(const auto& nvertex : graph->proximity_map.at(vertex)) {
            find_feas_positions_next_to_neighbour(vertex, nvertex.vertex, insertions);
        }
    }

    void PALNSSolution::generic_find_positions_next_to_nearby_vertices(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions, bool feasible) const {
        assert(!graph->proximity_map.at(vertex).empty());

        auto min_r = graph->proximity_map.at(vertex).back().travel_time;
        auto max_r = min_r * 1.1f;

        // Compensate for the imprecisions in not EUC_2D instances:
        // the minimum radius might be too large and exclude some
        // close vertex (i.e. a vertex with potential for a good
        // insertions). In this case, instead of starting with a
        // donut, in the first iteration we basically start with a
        // full circle.
        if(graph->opi->get_raw_specification<std::string>("EDGE_WEIGHT_TYPE") != "EUC_2D") {
            min_r = 0.1f;
        }

        const auto& vprop = graph->g[vertex];
        const auto centre = BoostPoint(vprop.x, vprop.y);

        const auto initial_insertions_n = insertions.size();
        std::size_t points_scanned = insertions.size();

        const auto max_n_times_can_fail = 2u * graph->n_vertices;
        std::size_t times_failed = 0u;

        while(true) {
            const auto points = within_radii(centre, min_r, max_r, graph->rtree);

            if(!points.empty()) {
                for(const auto& point : points) {
                    if(feasible) {
                        find_feas_positions_next_to_neighbour(vertex, point.second, insertions);
                    } else {
                        find_positions_next_to_neighbour(vertex, point.second, insertions);
                    }
                }

                points_scanned += points.size();

                if(insertions.size() > initial_insertions_n) {
                    // Found some insertion!
                    break;
                } else {
                    // No insertion found, but we scanned the whole graph!
                    if(points_scanned == graph->n_vertices - 2u) {
                        break;
                    }
                }
            } else {
                ++times_failed;
            }

            if(times_failed > max_n_times_can_fail) {
                break;
            }

            min_r = max_r;
            max_r *= 1.1f;
        }
    }

    void PALNSSolution::find_positions_next_to_nearby_vertices(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const {
        generic_find_positions_next_to_nearby_vertices(vertex, insertions, false);
    }

    void PALNSSolution::find_feas_positions_next_to_nearby_vertices(BoostVertex vertex, std::vector<VertexInsertionPrice>& insertions) const {
        generic_find_positions_next_to_nearby_vertices(vertex, insertions, true);
    }
}