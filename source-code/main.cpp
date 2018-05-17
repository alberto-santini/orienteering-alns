#include <experimental/filesystem>
#include <po/ProgramOptions.hxx>
#include <as/console.h>
#include <as/and_die.h>
#include <as/containers.h>
#include <palns/Parameters.h>

#include <random>
#include <as/random.h>
#include "palns/PALNSSolution.h"
#include "palns/repair/GreedyRepair.h"

#include "Graph.h"
#include "Plotter.h"
#include "ReducedGraph.h"
#include "GreedyHeuristic.h"
#include "PrintParamsCsv.h"
#include "GraphFeatures.h"
#include "palns/PALNSSolver.h"

namespace fs = std::experimental::filesystem;

using namespace as;
using namespace op;

std::uint32_t GreedyRepair::n_called = 0;

namespace {
    po::parser parser;
    fs::path instance_file;
    Graph inst_graph;

    void ensure_flag(std::string flag) {
        if(!parser[flag].was_set()) {
            std::cerr << console::error << "You need to set flag " << flag << and_die();
        }
    }

    void try_print_graph(const Graph& g) {
        for(const auto& of : parser["output-file"]) {
            const fs::path output_file(of.string);

            if(output_file.extension() == ".png") {
                Plotter plotter{g};
                plotter.plot_graph_to_png(output_file);
            } else {
                std::cerr << console::warning << "Unsupported file extension: " << output_file.extension() << "\n";
            }
        }
    }

    void try_print_features(const Graph& g) {
        for(const auto& of : parser["output-file"]) {
            const fs::path output_file(of.string);

            if(output_file.extension() == ".csv") {
                const bool print_header = !fs::exists(output_file);
                std::ofstream ofs(output_file, std::ios_base::app);

                if(print_header) {
                    ofs << "instance,diameter,max_dist_depot,dist_bary_depot,spread,largest_aligned,";
                    ofs << "num_clusters,avg_cluster_sz,avg_cluster_diameter,cluster_spread,";
                    ofs << "cluster_isolated_spread,isolated\n";
                }
                ofs << inst_graph.instance_name() << ",";

                auto clustering = Clustering(&g);

                if(clustering.is_proper()) {
                    features::print_features(ofs, g, std::experimental::make_observer<Clustering>(&clustering));
                } else {
                    features::print_features(ofs, g);
                }
            } else {
                std::cerr << console::warning << "Unsupported file extension: " << output_file.extension() << "\n";
            }
        }
    }

    void try_print_solution(const Tour& s) {
        for(const auto& of : parser["output-file"]) {
            const fs::path output_file(of.string);

            if(output_file.extension() == ".png") {
                s.print_to_png(output_file);
            } else if(output_file.extension() == ".csv") {
                s.to_csv(output_file);
            } else if(output_file.extension() == ".json") {
                s.to_json(output_file);
            } else {
                std::cerr << console::warning << "Unsupported file extension: " << output_file.extension() << "\n";
            }
        }
    }

    void try_print_palns_stats(const Tour& s, const PALNSSolver& solver) {
        if(!parser["log-file"].was_set()) {
            return;
        }

        const fs::path log_file(parser["log-file"].get().string);

        if(log_file.extension() == ".csv" || log_file.extension() == ".txt") {
            std::ofstream ofs(log_file, std::ios_base::app);

            ofs << s.graph->instance_name() << ",";
            ofs << s.total_prize << ",";
            ofs << s.travel_time << ",";
            ofs << solver.get_total_time_s() << ",";
            ofs << solver.get_time_to_best_s() << ",";
            ofs << solver.palns_problem_params << ",";
            ofs << solver.palns_framework_params << "\n";
        } else {
            std::cerr << console::warning << "Unsupported file extension: " << log_file.extension() << "\n";
        }
    }

    void print_graph() {
        ensure_flag("output-file");
        try_print_graph(inst_graph);
    }

    void print_clustered() {
        ensure_flag("output-file");

        auto red = recursive_reduction(&inst_graph);

        if(red) {
            try_print_graph(red->reduced_graph);
        } else {
            std::cerr << console::warning << "Could not reduce graph!" << and_die();
        }
    }

    void print_solution() {
        ensure_flag("solution-file");
        ensure_flag("output-file");

        Tour tour{&inst_graph, parser["solution-file"].get().string};

        std::cout << console::notice << "Solution travel time: " << tour.travel_time << std::endl;
        std::cout << console::notice << "Solution prize: " << tour.total_prize << std::endl;

        try_print_solution(tour);
    }

    void print_features() {
        ensure_flag("output-file");
        try_print_features(inst_graph);
    }

    void solve_greedy() {
        ensure_flag("alns-problem-params");

        PALNSProblemParams params{parser["alns-problem-params"].get().string};
        GreedyHeuristic gh{inst_graph, params};
        Tour greedy_sol = gh.solve();

        std::cout << console::notice << "Solution travel time: " << greedy_sol.travel_time << std::endl;
        std::cout << console::notice << "Solution prize: " << greedy_sol.total_prize << std::endl;

        try_print_solution(greedy_sol);
    }

    void test_greedy_repair(std::size_t n_tours, std::size_t max_tour_length) {
        using namespace std::chrono;

        ensure_flag("alns-problem-params");
        PALNSProblemParams params{parser["alns-problem-params"].get().string};

        std::random_device rd;
        std::mt19937 mt{rd()};
        std::uniform_int_distribution<std::size_t> dist(1u, max_tour_length);
        std::vector<BoostVertex> all_custs(inst_graph.n_vertices - 1u);
        std::iota(all_custs.begin(), all_custs.end(), 1u);

        std::vector<PALNSSolution> s1(n_tours), s2(n_tours);
        for(auto i = 0u; i < n_tours; ++i) {
            const auto tour_length = dist(mt);
            const auto customers = rnd::sample(all_custs, tour_length, mt);

            std::vector<BoostVertex> vertices = {0u};
            vertices.insert(vertices.end(), customers.begin(), customers.end());

            const Tour t{&inst_graph, vertices};
            s1[i] = PALNSSolution(t, &params);
            s2[i] = PALNSSolution(t, &params);
        }

        std::printf("%3zu, ", Graph::n_proximity_neighbours);

        GreedyRepair g1{&params, true};
        const auto t1 = high_resolution_clock::now();
        for(auto i = 0u; i < n_tours; ++i) {
            g1.repair_solution(s1[i], mt);
        }
        const auto e1 = high_resolution_clock::now();
        const auto d1 = duration_cast<duration<float>>(e1 - t1).count();
        std::printf("%10.2f, ", d1);

        GreedyRepair g2{&params, false};
        const auto t2 = high_resolution_clock::now();
        for(auto& sol : s2) {
            g2.repair_solution(sol, mt);
        }
        const auto e2 = high_resolution_clock::now();
        const auto d2 = duration_cast<duration<float>>(e2 - t2).count();
        std::printf("%10.2f\n", d2);
    }

    void solve_alns() {
        using namespace std::chrono;

        ensure_flag("alns-problem-params");
        ensure_flag("alns-framework-params");

        std::unique_ptr<Tour> initial_tour = nullptr;

        if(parser["solution-file"].was_set()) {
            initial_tour = std::make_unique<Tour>(&inst_graph, parser["solution-file"].get().string);
        }

        std::experimental::filesystem::path methods_stats_file("");

        if(parser["alns-methods-stats-file"].was_set()) {
            methods_stats_file = parser["alns-methods-stats-file"].get().string;
        }

        PALNSSolver palns_solver{inst_graph,
                                 parser["alns-problem-params"].get().string,
                                 parser["alns-framework-params"].get().string,
                                 methods_stats_file};

        Tour palns_solution = palns_solver.solve(initial_tour);

        std::cout << console::notice << "Solution travel time: " << palns_solution.travel_time << std::endl;
        std::cout << console::notice << "Solution prize: " << palns_solution.total_prize << std::endl;
        std::cout << console::notice << "Algorithm elapsed time: " << palns_solver.get_total_time_s() << std::endl;

        try_print_solution(palns_solution);
        try_print_palns_stats(palns_solution, palns_solver);
    }
}

int main(int argc, char** argv) {
    parser["action"]
        .abbreviation('a')
        .description("Action to perform: [print-graph|print-clustered|print-solution|print-features|greedy|test-repair|alns]. Mandatory.")
        .type(po::string);

    parser["output-file"]
        .abbreviation('o')
        .description("Adds an output file: [*.png] for graphs, [*.csv,*.json,*.png] for solutions, and [*.csv] for features. Mandatory.")
        .type(po::string)
        .multi();

    parser["instance-file"]
        .abbreviation('i')
        .description("Instance file. Mandatory.")
        .type(po::string);

    parser["solution-file"]
        .abbreviation('s')
        .description("Solution file. Mandatory for action print-solution, optional for action alns. Ignored otherwise.")
        .type(po::string);

    parser["log-file"]
        .abbreviation('l')
        .description("Log file where info about the solution process will be written. Optional for action alns. Ignored otherwise.")
        .type(po::string);

    parser["alns-problem-params"]
        .abbreviation('p')
        .description("ALNS problem-specific parameters. Mandatory for action alns. Ignored otherwise.")
        .type(po::string);

    parser["alns-framework-params"]
        .abbreviation('q')
        .description("ALNS framework paramters. Mandatory for action alns. Ignored otherwise.")
        .type(po::string);

    parser["alns-methods-stats-file"]
        .abbreviation('S')
        .description("File where ALNS can save statistics on the usage of destroy/repair methods. Optional for action alns. Ignored otherwise.")
        .type(po::string);

    parser["help"]
        .abbreviation('h')
        .description("Prints this help text.")
        .callback([&] { std::cout << parser << "\n"; });

    parser(argc, argv);

    ensure_flag("action");
    std::string action = parser["action"].get().string;

    ensure_flag("instance-file");
    instance_file = parser["instance-file"].get().string;
    inst_graph = Graph(instance_file);

    if(action == "print-graph") {
        print_graph();
    } else if(action == "print-clustered") {
        print_clustered();
    } else if(action == "print-solution") {
        print_solution();
    } else if(action == "print-features") {
        print_features();
    } else if(action == "greedy") {
        solve_greedy();
    } else if(action == "alns") {
        solve_alns();
    } else if(action == "test-repair") {
        test_greedy_repair(1000, 150    );
    } else {
        std::cerr << console::error << "Unrecognised action: " << action << and_die();
    }

    return 0;
}