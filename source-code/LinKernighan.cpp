//
// Created by alberto on 20/10/17.
//

#include <as/file_stream.h>
#include <as/containers.h>
#include <as/console.h>
#include <as/and_die.h>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <utility>
#include <map>
#include "LinKernighan.h"
#include "Graph.h"

namespace op {
    namespace fs = std::experimental::filesystem;

    struct LKHInstance {
        fs::path params_file;
        fs::path instance_file;
        fs::path tour_file;
        std::map<std::size_t, std::size_t> vertex_id_mapping;
    };

    LKHInstance generate_lkh_params(const Graph& g, const std::vector<BoostVertex>& vertices, std::string unique_name) {
        using namespace as;

        const auto inst_name = g.instance_name();
        const auto tmp_dir = fs::temp_directory_path();
        const auto tsp_file = tmp_dir / (inst_name + unique_name + ".tmp.tsp");
        const auto params_file = tmp_dir / (inst_name + unique_name + ".tmp.par");
        const auto tour_file = tmp_dir / (inst_name + unique_name + ".tmp.sol");

        std::map<std::size_t, std::size_t> vertex_id_mapping;
        for(auto i = 0u; i < vertices.size(); ++i) {
            vertex_id_mapping[i + 1] = g.g[vertices[i]].id;
        }

        {
            std::ofstream tofs(tsp_file);

            if(tofs.fail()) {
                std::cerr << console::error << "Cannot open temporary file " << tsp_file << and_die();
            }

            tofs << "NAME: " << inst_name << "\n";
            tofs << "TYPE: TSP\n";
            tofs << "DIMENSION: " << vertices.size() << "\n";

            if(g.opi) {
                // If it has info from the original instance file, use them.

                std::string weight_type = g.opi->get_raw_specification<std::string>("EDGE_WEIGHT_TYPE");
                tofs << "EDGE_WEIGHT_TYPE: " << weight_type << "\n";

                std::string weight_format;
                try {
                    weight_format = g.opi->get_raw_specification<std::string>("EDGE_WEIGHT_FORMAT");
                    tofs << "EDGE_WEIGHT_FORMAT: " << weight_format << "\n";
                } catch(...) {
                    // No such specification.
                }

                if(weight_type == "EXPLICIT") {
                    // Print weight matrix
                    tofs << "EDGE_WEIGHT_SECTION\n";

                    if(weight_format == "UPPER_ROW") {
                        for(auto i = 0u; i < vertices.size(); ++i) {
                            for(auto j = i + 1; j < vertices.size(); ++j) {
                                tofs << g.opi->get_distance(i, j) << " ";
                            }
                            tofs << "\n";
                        }
                    } else {
                        assert(weight_format == "LOWER_DIAG_ROW");

                        for(auto i = 0u; i < vertices.size(); ++i) {
                            for(auto j = 0u; j < i; ++j) {
                                tofs << g.opi->get_distance(i, j) << " ";
                            }
                            tofs << "0 ";
                        }
                        tofs << "\n";
                    }
                } else {
                    // Print vertices coordinates
                    tofs << "NODE_COORD_SECTION\n";

                    for(auto i = 0u; i < vertices.size(); ++i) {
                        const auto coords = g.opi->get_original_coordinates(vertices[i]);
                        tofs << i + 1 << " " << coords.x << " " << coords.y << "\n";
                        vertex_id_mapping[i + 1] = g.g[vertices[i]].id;
                    }
                }
            } else {
                // Otherwise, use the computed info.

                tofs << "EDGE_WEIGHT_TYPE: EUC_2D\n";
                tofs << "NODE_COORD_SECTION\n";

                for(auto i = 0u; i < vertices.size(); ++i) {
                    const auto& v = g.g[vertices[i]];
                    tofs << i + 1 << " " << v.x << " " << v.y << "\n";
                }
            }

            tofs << "EOF\n";
        }

        {
            std::ofstream pofs(params_file);

            if(pofs.fail()) {
                std::cerr << console::error << "Cannot open temporary file " << params_file << and_die();
            }

            pofs << "PROBLEM_FILE = " << tsp_file.string() << "\n";
            pofs << "TOUR_FILE = " << tour_file.string() << "\n";
            pofs << "RUNS = 1\n";
        }

        return {params_file, tsp_file, tour_file, vertex_id_mapping};
    }

    Tour run_lin_kernighan(const Graph& g, const std::vector<BoostVertex>& vertices, std::string unique_name) {
        using namespace as::fstream;
        using namespace as::console;
        using as::and_die;

        // Special case, if |vertices| <= 3.
        if(vertices.size() == 1u) {
            return Tour();
        } else if(vertices.size() == 2u) {
            auto e = boost::edge(vertices[0u], vertices[1u], g.g).first;
            return Tour(&g, std::vector<BoostEdge>({e, e}));
        } else if(vertices.size() == 3u) {
            auto e1 = boost::edge(vertices[0u], vertices[1u], g.g).first;
            auto e2 = boost::edge(vertices[1u], vertices[2u], g.g).first;
            auto e3 = boost::edge(vertices[2u], vertices[0u], g.g).first;
            return Tour(&g, {e1, e2, e3});
        }

        auto lkhinst = generate_lkh_params(g, vertices, unique_name);

        std::string cmd = "~/local/bin/LKH " + lkhinst.params_file.string() + " >/dev/null 2>&1";
        int exit_code = std::system(cmd.c_str());

        if(exit_code != 0) {
            std::cerr << error << "LKH executable failed; exit code: " << std::strerror(exit_code) << and_die();
        }

        if(!fs::exists(lkhinst.tour_file)) {
            std::cerr << error << "Expected solution file not produced: " << lkhinst.tour_file << and_die();
        }

        std::ifstream ifs(lkhinst.tour_file);

        if(ifs.fail()) {
            std::cerr << error << "Solution file could not be read: " << lkhinst.tour_file << and_die();
        }

        skip_lines(ifs, 6u);

        BoostVertex start_node, current_node;

        if(!(ifs >> start_node)) {
            std::cerr << error << "Cannot read first node of tsp solution." << and_die();
        }

        start_node = lkhinst.vertex_id_mapping.at(start_node);

        std::vector<BoostEdge> edges;
        edges.reserve(vertices.size());

        current_node = start_node;

        while(true) {
            long int next_node;

            if(!(ifs >> next_node)) {
                std::cerr << error << "Cannot read next node; current one: " << current_node << and_die();
            }

            if(next_node == -1) {
                auto edge = boost::edge(current_node, start_node, g.g).first;
                edges.push_back(edge);
                break;
            } else {
                assert(next_node >= 0);
                next_node = lkhinst.vertex_id_mapping.at(static_cast<std::size_t>(next_node));

                auto edge = boost::edge(current_node, static_cast<BoostVertex>(next_node), g.g).first;
                edges.push_back(edge);
                current_node = static_cast<BoostVertex>(next_node);
            }
        }

        fs::remove(lkhinst.params_file);
        fs::remove(lkhinst.instance_file);
        fs::remove(lkhinst.tour_file);

        return Tour(&g, edges);
    }
}