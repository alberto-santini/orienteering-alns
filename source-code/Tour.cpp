//
// Created by alberto on 17/10/17.
//

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>
#include <as/file_stream.h>
#include <as/containers.h>
#include <as/console.h>
#include <as/and_die.h>
#include <as/graph.h>
#include <numeric>
#include <optional>
#include <thread>
#include "Tour.h"
#include "Plotter.h"
#include "TourRemovalLabelling.h"

namespace op {
    namespace fs = std::experimental::filesystem;

    Tour::Tour(const Graph *const graph, std::vector<BoostEdge> edges) :
        graph{std::experimental::make_observer(graph)},
        edges{edges}
    {
        calculate_vertices_from_edges();
        calculate_travel_time();
        calculate_total_prize();
    }

    Tour::Tour(const Graph *const graph, std::vector<BoostVertex> vertices) :
        graph{std::experimental::make_observer(graph)},
        vertices{vertices}
    {
        calculate_edges_from_vertices();
        calculate_travel_time();
        calculate_total_prize();
    }

    Tour::Tour(const Graph *const graph, fs::path solution_file) :
        graph{std::experimental::make_observer(graph)}
    {
        using namespace as::console;
        using as::and_die;

        std::cout << notice << "Reading solution from file: " << solution_file << std::endl;

        if(!fs::exists(solution_file)) {
            std::cerr << error << "File not found: " << solution_file << and_die();
        }

        std::string ext = solution_file.extension();

        if(ext == ".sol") {
            read_from_oplib_solution(solution_file);
        } else if(ext == ".json") {
            read_from_json(solution_file);
        }
    }

    void Tour::read_from_json(fs::path solution_file) {
        using namespace boost::property_tree;
        using as::console::notice;

        ptree t;
        read_json(solution_file, t);

        for(const auto& vnode : t.get_child("tour")) {
            vertices.push_back(vnode.second.get_value<BoostVertex>());
        }
        travel_time = t.get<float>("travel_time");
        total_prize = t.get<float>("prize");

        std::cout << notice << "Read solution with " << vertices.size() << " vertices." << std::endl;

        calculate_edges_from_vertices();

        assert(is_travel_time_correct());
    }

    void Tour::read_from_oplib_solution(fs::path solution_file) {
        using namespace as::fstream;
        using namespace as::console;
        using as::and_die;

        std::ifstream ifs(solution_file);

        if(ifs.fail()) {
            std::cerr << error << "Cannot read from file: " << solution_file << and_die();
        }

        skip_lines(ifs, 9u);

        long int next_id = 1;
        std::size_t current_v = 0u; // Start from the depot.

        while(ifs >> next_id) {
            std::size_t next_v;

            if(next_id == -1) {
                // Tour is over, go back to the depot.
                next_v = 0u;
            } else {
                assert(next_id > 1u);
                next_v = static_cast<std::size_t>(next_id - 1);
            }

            const auto edge_found = boost::edge(current_v, next_v, graph->g);

            if(!edge_found.second) {
                std::cerr << error << "Edge not found from " << current_v << " to " << next_v << and_die();
            }

            edges.push_back(edge_found.first);
            current_v = next_v;
        }

        calculate_vertices_from_edges();
        calculate_travel_time();
        calculate_total_prize();

        std::cout << notice << "Read solution with " << edges.size() << " edges." << std::endl;
    }

    void Tour::print_to_png(fs::path image_file) const {
        assert(graph != nullptr);

        Plotter plotter{*graph};

        plotter.plot_tour_to_png(*this, image_file);
    }

    void Tour::calculate_vertices_from_edges() {
        using namespace as::containers;
        using namespace as::graph;
        using namespace as::console;
        using as::and_die;

        if(!incident_to_the_same_vertex(edges.front(), edges.back(), graph->g)) {
            std::cerr << error << "The tour is not closed!" << and_die();
        }

        vertices.clear();
        vertices.reserve(edges.size());

        for(const auto& edge : edges) {
            // Edges are undirected, so source and target can
            // really mean anything. For example, in a tour
            // A -> B -> C -> A, the edges could be:
            // (A,B), (B,C), (C,A) but also:
            // (A,B), (C,B), (A,C) and, in both cases, we
            // need to be able to find all vertices. In the
            // second case, if we only looked at what boost
            // calls source, we would not find B.

            if(!contains(vertices, boost::source(edge, graph->g))) {
                vertices.push_back(boost::source(edge, graph->g));
            }
            if(!contains(vertices, boost::target(edge, graph->g))) {
                vertices.push_back(boost::target(edge, graph->g));
            }
        }
    }

    void Tour::calculate_edges_from_vertices() {
        edges.clear();
        edges.reserve(vertices.size());

        for(auto i = 0u; i < vertices.size(); ++i) {
            const auto& curr_v = vertices[i];
            const auto& next_v = vertices[(i + 1) % vertices.size()];
            const auto& edge = boost::edge(curr_v, next_v, graph->g).first;
            edges.push_back(edge);
        }
    }

    void Tour::calculate_travel_time() {
        travel_time = std::accumulate(
          edges.begin(),
          edges.end(),
          0.0f,
          [this] (float acc, const BoostEdge& edge) -> float {
              return acc + graph->g[edge].travel_time;
          }
        );
    }

    void Tour::calculate_total_prize() {
        total_prize = std::accumulate(
            vertices.begin(),
            vertices.end(),
            0.0f,
            [this] (float acc, const BoostVertex& vertex) -> float {
                return acc + graph->g[vertex].prize;
            }
        );
    }

    bool Tour::visits_vertex(const BoostVertex& v) const {
        return as::containers::contains(vertices, v);
    }

    bool Tour::is_travel_time_correct() {
        const auto old_tt = travel_time;
        calculate_travel_time();
        return std::abs(old_tt - travel_time) < 0.5;
    }

    std::ostream& operator<<(std::ostream& out, const Tour& tour) {
        out << "(l: " << tour.vertices.size();
        out << ", tt: " << tour.travel_time;
        out << ", p: " << tour.total_prize << ") ";
        as::containers::join_and_print(tour.vertices, out);
        return out;
    }

    void Tour::do_2opt() {
        assert(is_simple());
        assert(is_travel_time_correct());
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);

        if(edges.size() < 4u) { return; }

        float best_gain = 0.0f;

        do {
            BoostVertex best_i = 0u, best_j = 0u;
            best_gain = 0.0f;

            for(auto i = 0u; i < vertices.size() - 2; ++i) {
                for(auto j = i + 2u; j < vertices.size(); ++j) {
                    const auto next_i = i + 1;
                    const auto next_j = (j + 1) % vertices.size();
                    const auto gain = graph->travel_time(vertices[i], vertices[next_i]) +
                                      graph->travel_time(vertices[j], vertices[next_j]) -
                                      graph->travel_time(vertices[i], vertices[j]) -
                                      graph->travel_time(vertices[next_i], vertices[next_j]);

                    if(gain > best_gain + 1) {
                        best_gain = gain;
                        best_i = i;
                        best_j = j;
                    }
                }
            }

            if(best_gain > 0.0f) {
                std::vector<BoostVertex> new_vertices;
                new_vertices.reserve(vertices.size());

                for(auto k = 0u; k <= best_i; ++k) {
                    new_vertices.push_back(vertices[k]);
                }
                for(auto k = best_j; k >= best_i + 1; --k) {
                    new_vertices.push_back(vertices[k]);
                }
                for(auto k = best_j + 1; k < vertices.size(); ++k) {
                    new_vertices.push_back(vertices[k]);
                }

                vertices = new_vertices;
                travel_time -= best_gain;
            }
        } while(best_gain > 0.0f);

        calculate_edges_from_vertices();

        assert(is_simple());
        assert(is_travel_time_correct());
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);
    }

    std::vector<BoostVertex> Tour::make_travel_time_feasible_optimal() {
        assert(is_simple());
        assert(is_travel_time_correct());
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);

        JGraph jgraph;

        for(auto i = 0u; i < vertices.size(); ++i) {
            boost::add_vertex(JVertexProperty{i, vertices[i], graph->g[vertices[i]].prize}, jgraph);
        }
        boost::add_vertex(JVertexProperty{vertices.size(), vertices[0], graph->g[vertices[0]].prize}, jgraph);

        std::size_t id = 0u;
        for(auto i = 0u; i <= vertices.size(); ++i) {
            for(auto j = i + 1; j <= vertices.size(); ++j) {
                if(i == 0u && j == vertices.size()) { continue; }
                boost::add_edge(i, j, JEdgeProperty{id++, graph->travel_time(jgraph[i].original_v, jgraph[j].original_v)}, jgraph);
            }
        }

        jgraph[boost::graph_bundle] = graph->max_travel_time;

        using JPath = std::vector<JEdge>;

        std::vector<JPath> opt_paths;
        std::vector<JLabel> opt_labels;
        JVFunctor jvf{jgraph};
        JEFunctor jef{jgraph};

        boost::r_c_shortest_paths(
            jgraph,
            make_property_map<JVertex>(jvf),
            make_property_map<JEdge>(jef),
            0u,
            vertices.size(),
            opt_paths,
            opt_labels,
            JLabel{graph->g[0u].prize, 0},
            JLabelExtender{},
            [] (const JLabel& l1, const JLabel& l2) { return l1 < l2; }
        );

        assert(!opt_paths.empty());

        std::vector<std::size_t> ids(opt_paths.size());
        std::iota(ids.begin(), ids.end(), 0u);
        std::sort(ids.begin(), ids.end(), [&] (const auto& i1, const auto& i2) { return opt_labels[i1].prize < opt_labels[i2].prize; });

        const auto& opt = opt_paths[ids.back()];

        std::vector<BoostVertex> opt_vertices;
        for(const auto& e : opt) {
            opt_vertices.push_back(jgraph[boost::source(e, jgraph)].original_v);
        }

        std::vector<BoostVertex> rem_vertices;
        for(const auto& v : vertices) {
            if(!as::containers::contains(opt_vertices, v)) {
                assert(v != 0u);
                rem_vertices.push_back(v);
            }
        }

        for(const auto& v : rem_vertices) {
            remove_vertex(v);
        }

        assert(is_simple());
        assert(is_travel_time_correct());
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);

        return rem_vertices;
    }

    std::vector<BoostVertex> Tour::make_travel_time_feasible_naive() {
        assert(is_travel_time_correct());
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);

        if(travel_time <= graph->max_travel_time) { return {}; }

        std::vector<BoostVertex> removed_vertices;
        std::vector<VertexRemovalPrice> removals(vertices.size());

        removals[0u] = VertexRemovalPrice(); // Depot
        for(auto i = 1u; i < vertices.size(); ++i) {
            removals[i] = price_vertex_removal(i);
        }

        while(travel_time > graph->max_travel_time) {
            std::size_t best_removal_pos = 1u;
            VertexRemovalPrice best_removal = removals[1u];

            for(auto i = 2u; i < removals.size(); ++i) {
                if(removals[i].score > best_removal.score) {
                    best_removal_pos = i;
                    best_removal = removals[i];
                }
            }

            removed_vertices.push_back(best_removal.vertex);
            remove_vertex_by_position(best_removal_pos);
            removals.erase(removals.begin() + best_removal_pos);

            assert(removals.size() == vertices.size());

            if(best_removal_pos > 1u) {
                removals[best_removal_pos - 1u] = price_vertex_removal(best_removal_pos - 1u);
            }

            if(best_removal_pos < removals.size()) {
                removals[best_removal_pos] = price_vertex_removal(best_removal_pos);
            }
        }

        assert(is_travel_time_correct());
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);

        return removed_vertices;
    }

    bool Tour::remove_vertex(std::vector<BoostVertex>::iterator vertex_it) {
        using as::console::warning;

        assert(vertex_it != vertices.begin()); // The depot.
        assert(vertex_it != vertices.end());
        assert(is_travel_time_correct());
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);

        // We don't remove the vertex, if it's the only
        // non-depot vertex... otherwise the tour would
        // become degenerate.
        if(vertices.size() <= 2u) {
            std::cerr << warning << "Trying to remove the only non-depot vertex of a tour. Ignoring the request." << std::endl;
            return false;
        }

        auto vertex = *vertex_it;
        auto vertex_pos = vertex_it - vertices.begin();

        assert(!graph->g[vertex].depot);
        assert(graph->g[vertex].reachable);

        const auto vertex_it_before = vertex_it - 1;
        auto vertex_it_after = vertex_it + 1;

        if(vertex_it_after == vertices.end()) { vertex_it_after = vertices.begin(); }

        auto new_edge = boost::edge(*vertex_it_before, *vertex_it_after, graph->g).first;

        const auto old_edge_pred_pos = vertex_pos - 1;
        const auto old_edge_succ_pos = vertex_pos;

        assert(old_edge_pred_pos >= 0);
        assert(static_cast<std::size_t>(old_edge_pred_pos) < edges.size() - 1u);
        assert(old_edge_succ_pos > 0);
        assert(static_cast<std::size_t>(old_edge_succ_pos) < edges.size());

        assert(boost::edge(*vertex_it_before, vertex, graph->g).first == edges[old_edge_pred_pos]);
        assert(boost::edge(vertex, *vertex_it_after, graph->g).first == edges[old_edge_succ_pos]);

        const auto travel_time_diff =
            graph->g[edges[old_edge_pred_pos]].travel_time +
            graph->g[edges[old_edge_succ_pos]].travel_time -
            graph->g[new_edge].travel_time;

        edges[old_edge_pred_pos] = new_edge;
        edges.erase(edges.begin() + old_edge_succ_pos);
        vertices.erase(vertex_it);
        travel_time -= travel_time_diff;
        total_prize -= graph->g[vertex].prize;

        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);
        assert(is_travel_time_correct());

        return true;
    }

    bool Tour::remove_vertex(const BoostVertex& vertex) {
        auto vertex_it = std::find(vertices.begin(), vertices.end(), vertex);

        assert(vertex_it != vertices.end());

        return remove_vertex(vertex_it);
    }

    bool Tour::remove_vertex_if_present(const BoostVertex& vertex) {
        auto vertex_it = std::find(vertices.begin(), vertices.end(), vertex);

        if(vertex_it != vertices.end()) {
            return remove_vertex(vertex_it);
        } else {
            return false;
        }
    }

    bool Tour::remove_vertex_by_position(std::size_t position) {
        assert(position > 0u); // No depot!
        assert(position < vertices.size());

        return remove_vertex(vertices.begin() + position);
    }

    void Tour::add_vertex(const BoostVertex& vertex, std::size_t position) {
        using namespace as::containers;

        assert(is_travel_time_correct());
        assert(position < vertices.size());
        assert(!contains(vertices, vertex));
        assert(!graph->g[vertex].depot);
        assert(graph->g[vertex].reachable);
        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);

        const auto vertex_before = vertices[position];
        const auto vertex_after = vertices[(position + 1u) % vertices.size()];

        const auto remove_edge = boost::edge(vertex_before, vertex_after, graph->g).first;
        auto remove_edge_it = std::find(edges.begin(), edges.end(), remove_edge);

        // In the case the tour is a loop, we would have:
        // (0, v) -- (v, 0)
        // The edge we want to remove is (v, 0) when we
        // insert the new vertex in position 1, i.e. after
        // v. However, since edges are not oriented, we are
        // finding (0, v). So, we need to manually increment
        // this iterator to point to the correct back-edge.
        if(edges.size() == 2u && position == 1u) {
            ++remove_edge_it;
        }

        assert(remove_edge_it != edges.end());

        const auto ne1 = boost::edge(vertex_before, vertex, graph->g);
        const auto ne2 = boost::edge(vertex, vertex_after, graph->g);

        assert(ne1.second && ne2.second);

        // Insert the new vertex:
        // (We use position as the index of the vertex after which
        // the new vertex is inserted; vector::insert, on the other
        // hand, wants an iterator to the item before which the new
        // item is inserted. Hence, we add 1 to position.)
        vertices.insert(vertices.begin() + position + 1, vertex);

        // If we are removing the first edge, make sure the one we are
        // replacing it with starts at the depot!
        assert(remove_edge_it != edges.begin() || vertex_before == 0u);

        // Subsitute the the old edge with the first of the two new edges:
        *remove_edge_it = ne1.first;

        // Add the second of the two new edges:
        edges.insert(remove_edge_it + 1, ne2.first);

        // Update travel time:
        travel_time -= graph->g[remove_edge].travel_time;
        travel_time += graph->g[ne1.first].travel_time;
        travel_time += graph->g[ne2.first].travel_time;

        // Update prize:
        total_prize += graph->g[vertex].prize;

        assert(boost::source(edges.front(), graph->g) == 0u);
        assert(boost::target(edges.back(), graph->g) == 0u);
        assert(is_travel_time_correct());
    }

    VertexRemovalPrice Tour::price_vertex_removal(std::size_t position) const {
        assert(position > 0u);
        assert(position < vertices.size());

        const auto& vertex_before = vertices[position - 1u];
        const auto& vertex = vertices[position];
        const auto& vertex_after = vertices[(position + 1u) % vertices.size()];

        const auto decrease_in_travel_time = graph->travel_time(vertex_before, vertex) +
                                             graph->travel_time(vertex, vertex_after) -
                                             graph->travel_time(vertex_before, vertex_after);

        const auto decrease_in_prize = graph->g[vertex].prize;

        return {vertex, decrease_in_travel_time, decrease_in_prize, decrease_in_travel_time / decrease_in_prize};
    }

    VertexInsertionPrice Tour::price_vertex_insertion(const BoostVertex& vertex, std::size_t position) const {
        using namespace as::containers;

        assert(position < vertices.size());
        assert(!contains(vertices, vertex));

        const auto vertex_before = vertices[position];
        const auto vertex_after = vertices[(position + 1u) % vertices.size()];

        const auto increase_in_travel_time = graph->travel_time(vertex_before, vertex) +
                                             graph->travel_time(vertex, vertex_after) -
                                             graph->travel_time(vertex_before, vertex_after);

        const auto increase_in_prize = graph->g[vertex].prize;

        return {vertex, position, increase_in_travel_time, increase_in_prize, increase_in_travel_time / increase_in_prize};
    }

    bool Tour::is_simple() const {
        std::set<BoostVertex> v(vertices.begin(), vertices.end());
        return v.size() == vertices.size();
    }

    void Tour::to_csv(std::experimental::filesystem::path csv_file) const {
        std::ofstream outs;
        bool print_header = false;

        if(fs::exists(csv_file)) {
            outs.open(csv_file, std::ios_base::app);
        } else {
            outs.open(csv_file);
            print_header = true;
        }

        if(outs.fail()) {
            std::cerr << as::console::error << "Cannot write solution csv to " << csv_file << as::and_die();
        }

        if(print_header) {
            outs << "instance,prize,traveltime\n";
        }

        outs << graph->instance_name() << "," << total_prize << "," << travel_time << "\n";
    }

    void Tour::to_json(std::experimental::filesystem::path json_file) const {
        using namespace boost::property_tree;

        ptree t;

        t.put("instance", graph->instance_name());
        t.put("prize", total_prize);
        t.put("travel_time", travel_time);

        ptree vertices_ary;

        for(const auto& vertex : vertices) {
            ptree vertex_t;
            vertex_t.put("", vertex);
            vertices_ary.push_back(std::make_pair("", vertex_t));
        }

        t.add_child("tour", vertices_ary);

        std::ofstream ofs(json_file);

        if(ofs.fail()) {
            std::cerr << as::console::error << "Cannot write solution json to " << json_file << as::and_die();
        }

        write_json(ofs, t);
    }
}