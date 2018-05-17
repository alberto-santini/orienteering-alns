//
// Created by alberto on 22/10/17.
//

#include "ReducedGraph.h"
#include "LinKernighan.h"

#include <as/graph.h>
#include <as/containers.h>

namespace op {
    ReducedGraph::ReducedGraph(const Graph *const original_graph) :
        ReducedGraph(original_graph, Clustering(original_graph)) {}

    ReducedGraph::ReducedGraph(const Graph *const original_graph, const Clustering& c) :
        original_graph{std::experimental::make_observer(original_graph)}
    {
        std::vector<Vertex> vertices;

        // Add the depot.
        vertices.push_back(original_graph->g[0u]);
        vertices_mapping[0u] = {0u};

        // Add the vertices representing clusters.
        for(auto k = 0u; k < c.n_clusters; ++k) {
            vertices.push_back({
                                      k + 1, // Id
                                      false, // Depot?
                                      true, // Reachable?
                                      c.centres[k].x(), // X-coord
                                      c.centres[k].y(), // Y-coord
                                      c.prizes[k] // Prize
                                  });

            assert(c.clusters[k].size() > 1u);

            vertices_mapping[k + 1] = c.clusters[k];
            tsps[k + 1] = run_lin_kernighan(*original_graph, vertices_mapping[k + 1]);

            // Check that tsp and vertex mapping are consistent
            assert(
                std::set<BoostVertex>(vertices_mapping[k+1].begin(), vertices_mapping[k+1].end()) ==
                std::set<BoostVertex>(tsps[k+1].vertices.begin(), tsps[k+1].vertices.end())
            );
        }

        std::size_t vertex_id = c.n_clusters + 1;

        // Add the noise vertices.
        for(const auto& vertex : c.noise) {
            auto vprop = original_graph->g[vertex];

            vertices_mapping[vertex_id] = {vprop.id};

            vprop.id = vertex_id++;
            vertices.push_back(vprop);
        }

        // Creates the Graph object for the reduced graph.
        reduced_graph = Graph(vertices, original_graph->max_travel_time);

        // Add the travel time to the edges. Each edge incident to
        // a cluster gets half of that cluster's tsp travel time,
        // so that when a vertex is visited (by two incident edges)
        // the whole travel time is considered.
        for(const auto& edge : as::graph::edges(reduced_graph.g)) {
            auto& eprop = reduced_graph.g[edge];
            const auto& origin = boost::source(edge, reduced_graph.g);
            const auto& destination = boost::target(edge, reduced_graph.g);

            if(1u <= origin && origin <= c.n_clusters) {
                eprop.travel_time += tsps[origin].travel_time / 2.0f;
            }

            if(1u <= destination && destination <= c.n_clusters) {
                eprop.travel_time += tsps[destination].travel_time / 2.0f;
            }
        }
    }

    ReducedGraph reduce_again(const ReducedGraph& other) {
        Clustering c(&(other.reduced_graph));
        return reduce_again(other, c);
    }

    ReducedGraph reduce_again(const ReducedGraph& other, const Clustering& c) {
        ReducedGraph new_red;
        new_red.original_graph = other.original_graph;

        std::vector<Vertex> vertices;

        vertices.push_back(other.original_graph->g[0u]);
        new_red.vertices_mapping[0u] = {0u};

        // Add vertices representing the clustering. Make sure that
        // the mapping "propagates" to lower echelons.
        for(auto k = 0u; k < c.n_clusters; ++k) {
            vertices.push_back({
                                    k + 1, // Id
                                    false, // Depot?
                                    true, // Reachable?
                                    c.centres[k].x(), // X-coord
                                    c.centres[k].y(), // Y-coord
                                    c.prizes[k] // Prize
                               });

            for(const auto& red_v : c.clusters[k]) {
                for(const auto& mapped_v : other.vertices_mapping.at(red_v)) {
                    new_red.vertices_mapping[k + 1].push_back(mapped_v);
                }
            }

            new_red.tsps[k + 1] = run_lin_kernighan(*new_red.original_graph, new_red.vertices_mapping[k + 1]);

            // Check that tsp and vertex mapping are consistent
            assert(
                std::set<BoostVertex>(new_red.vertices_mapping[k+1].begin(), new_red.vertices_mapping[k+1].end()) ==
                std::set<BoostVertex>(new_red.tsps[k+1].vertices.begin(), new_red.tsps[k+1].vertices.end())
            );
        }

        std::size_t vertex_id = c.n_clusters + 1;

        for(const auto& vertex : c.noise) {
            using as::containers::contains;

            auto vprop = other.reduced_graph.g[vertex];

            new_red.vertices_mapping[vertex_id] = other.vertices_mapping.at(vertex);

            if(contains(other.tsps, vertex)) {
                new_red.tsps[vertex_id] = other.tsps.at(vertex);

                // Check that tsp and vertex mapping are consistent
                assert(
                    std::set<BoostVertex>(new_red.vertices_mapping[vertex_id].begin(), new_red.vertices_mapping[vertex_id].end()) ==
                    std::set<BoostVertex>(new_red.tsps[vertex_id].vertices.begin(), new_red.tsps[vertex_id].vertices.end())
                );
            } else {
                assert(other.vertices_mapping.at(vertex).size() == 1u);
            }

            vprop.id = vertex_id;
            vertices.push_back(vprop);

            ++vertex_id;
        }

        new_red.reduced_graph = Graph(vertices, new_red.original_graph->max_travel_time);

        // Add the travel time to the edges. Each edge incident to
        // a cluster gets half of that cluster's tsp travel time,
        // so that when a vertex is visited (by two incident edges)
        // the whole travel time is considered.
        for(const auto& edge : as::graph::edges(new_red.reduced_graph.g)) {
            auto& eprop = new_red.reduced_graph.g[edge];
            const auto& origin = boost::source(edge, new_red.reduced_graph.g);
            const auto& destination = boost::target(edge, new_red.reduced_graph.g);

            if(1u <= origin && origin <= c.n_clusters) {
                eprop.travel_time += new_red.tsps[origin].travel_time / 2.0f;
            }

            if(1u <= destination && destination <= c.n_clusters) {
                eprop.travel_time += new_red.tsps[destination].travel_time / 2.0f;
            }
        }

        return new_red;
    }

    std::optional<ReducedGraph> recursive_reduction(const Graph *const graph, float red_factor) {
        Clustering c(graph);

        if(!c.is_proper()) { return std::nullopt; }

        ReducedGraph red(graph, c);

        std::size_t limit_n_vertices = std::min(
            static_cast<std::size_t>(red.original_graph->n_vertices * red_factor),
            std::size_t(50)
        );

        while(true) {
            c = Clustering(&red.reduced_graph);

            // If we already reduced enough the number of vertices
            // compared to the original graph, stop.
            if(red.reduced_graph.n_vertices <= limit_n_vertices) {
                return std::make_optional(red);
            }

            // If the clustering is "degenerate", but there are still
            // too many vertices, abort.
            if(!c.is_proper()) {
                return std::nullopt;
            }

            // Else, reduce again.
            ReducedGraph red_again = reduce_again(red, c);

            red = red_again;
        }
    }

    // Determines which is the best vertex in a cluster to start the TSP.
    static std::size_t best_v(const Graph& g, const Tour& tsp, BoostPoint prev, BoostPoint next) {
        assert(tsp.vertices.size() > 1u);
        assert(tsp.is_simple());

        const auto eucl_dist = [] (float x1, float y1, float x2, float y2) -> float {
            return std::sqrt(
                std::pow(x1 - x2, 2.0f) +
                std::pow(y1 - y2, 2.0f)
            );
        };

        float best_dist = std::numeric_limits<float>::max();
        std::size_t best_id = tsp.vertices.size();
        const auto tsp_sz = tsp.vertices.size();

        for(auto v_id = 0u; v_id < tsp_sz; ++v_id) {
            // Entry point of the tsp if we start it at vertex # v_id
            const auto vertex = tsp.vertices[v_id];
            const auto& vprop = g.g[vertex];
            const auto entry_pt = BoostPoint(vprop.x, vprop.y);

            // Exit point of the tsp if we start it at vertex # v_id
            const auto v_id_pred = (v_id > 0 ? v_id - 1 : tsp_sz - 1);
            const auto vertex_pred = tsp.vertices[v_id_pred];
            const auto& vprop_pred = g.g[vertex_pred];
            const auto exit_pt = BoostPoint(vprop_pred.x, vprop_pred.y);

            // Estimate of the distance if we start the tsp at vertex # v_id
            const auto dist = eucl_dist(prev.x(), prev.y(), entry_pt.x(), entry_pt.y()) +
                              eucl_dist(exit_pt.x(), exit_pt.y(), next.x(), next.y()) -
                              eucl_dist(exit_pt.x(), exit_pt.y(), entry_pt.x(), entry_pt.y());

            if(dist < best_dist) {
                best_dist = dist;
                best_id = v_id;
            }
        }

        assert(best_id != tsp.vertices.size());

        return best_id;
    };

    Tour project_back_tour(const Tour& tour, const ReducedGraph& red) {
        using as::containers::contains;

        assert(tour.is_simple());

        std::vector<BoostVertex> vertices;
        const Graph& g = *(red.original_graph);
        const Graph& r = red.reduced_graph;

        for(auto k = 0u; k < tour.vertices.size(); ++k) {
            const auto& v = tour.vertices[k];

            if(!contains(red.tsps, v)) {
                // "Normal" vertices.
                vertices.push_back(red.vertices_mapping.at(v).front());
            } else {
                // "Cluster" vertices.
                assert(red.vertices_mapping.at(v).size() > 1u);

                const auto& prev_v = tour.vertices[k-1];
                const auto& next_v = tour.vertices[(k+1) % tour.vertices.size()];
                const auto prev = BoostPoint(r.g[prev_v].x, r.g[prev_v].y);
                const auto next = BoostPoint(r.g[next_v].x, r.g[next_v].y);
                const auto start_v_id = best_v(g, red.tsps.at(v), prev, next);
                auto curr_v_id = start_v_id;

                do {
                    vertices.push_back(red.tsps.at(v).vertices[curr_v_id]);
                    curr_v_id = (curr_v_id + 1) % red.tsps.at(v).vertices.size();
                } while (curr_v_id != start_v_id);
            }
        }

        Tour red_tour(&g, vertices);

        assert(red_tour.is_simple());

        return red_tour;
    }
}