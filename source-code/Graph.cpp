//
// Created by alberto on 17/10/17.
//

#include "Graph.h"

#include <as/oplib.h>
#include <as/and_die.h>
#include <as/console.h>
#include <as/graph.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <utility>
#include <numeric>

namespace op {
    namespace fs = std::experimental::filesystem;

    Graph::Graph(fs::path graph_file) : graph_file{graph_file}, opi{graph_file.string()} {
        n_vertices = opi->number_of_vertices();
        max_travel_time = opi->get_max_travel_time();

        for(auto i = 0u; i < n_vertices; ++i) {
            boost::add_vertex({
                i,                                             // Id
                i == 0u,                                       // Depot?
                opi->get_distance(0, i) <= max_travel_time / 2, // Reachable?
                opi->get_coordinates(i).x,                      // X coordinate
                opi->get_coordinates(i).y,                      // Y coordinate
                opi->get_prize(i)                               // Prize
            }, g);
        }

        std::cout << as::console::notice << "Generated " << n_vertices << " vertices." << std::endl;

        std::size_t edge_id = 0u;
        for(auto i = 0u; i < n_vertices; ++i) {
            if(!g[i].reachable) { continue; }

            for(auto j = i + 1; j < n_vertices; ++j) {
                if(!g[j].reachable) { continue; }

                const auto tt = opi->get_distance(i, j);

                // In theory we could skip these arcs, but in practices it will
                // give us more trouble than it saves, as then we cannot rely on
                // the assumption that the subgraph induced by reachable vertices
                // is complete:
                // if(opi->get_distance(0, i) + tt + opi->get_distance(j, 0) > max_travel_time) { continue; }

                boost::add_edge(
                    i, j,
                    {edge_id++, tt},
                    g
                );
            }
        }

        std::cout << as::console::notice << "Generated " << edge_id << " edges." << std::endl;

        generate_rtree();
        generate_proximity_map();
        set_total_prize();

        std::tie(min_x, max_x) = min_max_x();
        std::tie(min_y, max_y) = min_max_y();
        std::tie(min_prize, max_prize) = min_max_prize();
    }

    Graph::Graph(std::vector<Vertex> vertices, float max_travel_time) :
        opi{std::nullopt},
        max_travel_time{max_travel_time}
    {
        graph_file = "graph-" + std::to_string(std::rand());
        g = BoostGraph();

        const auto eucl_dist = [] (const Vertex& v, const Vertex& w) -> float {
            return std::sqrt(
                std::pow(v.x - w.x, 2.0f) +
                std::pow(v.y - w.y, 2.0f)
            );
        };

        std::cout << as::console::notice << "Received " << vertices.size() << " vertices." << std::endl;

        for(const auto& vertex : vertices) {
            boost::add_vertex(vertex, g);
        }
        n_vertices = boost::num_vertices(g);

        std::size_t edge_id = 0u;
        for(auto i = 0u; i < n_vertices; ++i) {
            if(!vertices[i].reachable) { continue; }

            for(auto j = i + 1; j < n_vertices; ++j) {
                if(!vertices[j].reachable) { continue; }

                const auto tt = eucl_dist(vertices[i], vertices[j]);
                boost::add_edge(i, j, {edge_id++, tt}, g);
            }
        }

        std::cout << as::console::notice << "Generated " << edge_id << " edges." << std::endl;

        generate_rtree();
        generate_proximity_map();
        set_total_prize();

        std::tie(min_x, max_x) = min_max_x();
        std::tie(min_y, max_y) = min_max_y();
        std::tie(min_prize, max_prize) = min_max_prize();
    }

    void Graph::generate_rtree() {
        for(const auto& vertex : as::graph::vertices(g)) {
            BoostPoint pt(g[vertex].x, g[vertex].y);
            rtree.insert(std::make_pair(pt, vertex));
        }
    }

    void Graph::generate_proximity_map() {
        for(const auto& v : as::graph::vertices(g)) {
            proximity_map[v] = std::vector<ProximityMapEntry>();
            proximity_map[v].reserve(n_proximity_neighbours);

            if(n_proximity_neighbours > 0u) {
                std::size_t vertex_n = 0u;

                for(const auto &w : as::graph::neighbours(v, g)) {
                    if(w == 0u) { continue; } // Skip the depot

                    auto tt = travel_time(v, w);

                    // Add the first n_proximity_neighbours neighbours.
                    if(vertex_n < n_proximity_neighbours) {
                        proximity_map[v].push_back({w, tt});
                        ++vertex_n;
                        continue;
                    }

                    for(auto &entry : proximity_map[v]) {
                        if(entry.travel_time > tt) {
                            entry = {w, tt};
                            break;
                        }
                    }
                }

                std::sort(
                        proximity_map[v].begin(),
                        proximity_map[v].end(),
                        [&] (const auto &w1, const auto &w2) -> bool {
                            return w1.travel_time < w2.travel_time;
                        }
                );
            }
        }

        std::cout << as::console::notice << "Generated the proximity map." << std::endl;
    }

    float Graph::travel_time(const BoostVertex& v, const BoostVertex& w) const {
        if(v == w) { return 0.0f; }
        auto e_found = boost::edge(v, w, g);

        if(!e_found.second) {
            std::cerr << as::console::error << "Requested travel time of " << v << ", " << w << " which are not adjacent." << std::endl;
            std::cerr << as::console::error << v << " reachable? " << g[v].reachable << std::endl;
            std::cerr << as::console::error << w << " reachable? " << g[w].reachable << as::and_die();
        }

        return g[e_found.first].travel_time;
    }

    std::pair<float, float> Graph::min_max_vertex_property(float(Vertex::*prop)) const {
        auto begin_end = boost::vertices(g);
        auto minmax = std::minmax_element(begin_end.first, begin_end.second,
                                          [&,this] (const BoostVertex& v1, const BoostVertex& v2) -> bool {
                                              return g[v1].*prop < g[v2].*prop;
                                          }
        );

        assert(g[*minmax.first].*prop <= g[*minmax.second].*prop);

        return std::make_pair(g[*minmax.first].*prop, g[*minmax.second].*prop);
    }

    void Graph::set_total_prize() {
        auto vertices_iter = as::graph::vertices(g);
        total_prize = std::accumulate(
            vertices_iter.begin(),
            vertices_iter.end(),
            0.0f,
            [&] (float acc, const BoostVertex& v) -> float {
                return acc + g[v].prize;
            }
        );
    }
}
