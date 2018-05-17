//
// Created by alberto on 31/10/17.
//

#include "../GreedyHeuristic.h"

#include <palns/PALNS.h>
#include <as/console.h>
#include <as/and_die.h>
#include "PALNSSolver.h"
#include "PALNSSolution.h"
#include "destroy/RandomRemove.h"
#include "destroy/RandomSeqRemove.h"
#include "destroy/RandomClusterRemove.h"
#include "repair/GreedyRepair.h"
#include "repair/SeqVertexRepair.h"
#include "repair/RandomClusterRepair.h"
#include "PALNSVisitor.h"

namespace op {
    namespace fs = std::experimental::filesystem;

    PALNSSolver::PALNSSolver(const Graph& graph,
                             std::experimental::filesystem::path palns_problem_params_file,
                             std::experimental::filesystem::path palns_framework_params_file,
                             std::experimental::filesystem::path methods_stats_file) :
        graph{graph}, methods_stats_file{methods_stats_file}, total_time_s{0}, time_to_best_s{0}
    {
        if(!fs::exists(palns_problem_params_file)) {
            std::cerr << as::console::error << "Cannot find PALNS problem-specific params file: " << palns_problem_params_file << as::and_die();
        }

        palns_problem_params = PALNSProblemParams(palns_problem_params_file);

        if(!fs::exists(palns_framework_params_file)) {
            std::cerr << as::console::error << "Cannot find PALNS params file: " << palns_framework_params_file << as::and_die();
        }

        palns_framework_params = mlpalns::Parameters(palns_framework_params_file);
    }

    Tour PALNSSolver::solve(std::unique_ptr<Tour>& initial_sol) {
        using namespace std::chrono;

        const auto vertex_shuffle = [] (std::vector<BoostVertex>& vertices, std::mt19937& mt) -> void {
            std::shuffle(vertices.begin(), vertices.end(), mt);
        };

        const auto vertex_by_prize = [this] (std::vector<BoostVertex>& vertices, std::mt19937&) -> void {
            std::sort(
                vertices.begin(),
                vertices.end(),
                [this] (BoostVertex v1, BoostVertex v2) -> bool {
                    return graph.g[v1].prize > graph.g[v2].prize;
                }
            );
        };

        std::mt19937 mt(std::time(0u));

        Tour initial;

        if(initial_sol) {
            initial = *initial_sol;
        } else {
            const GreedyHeuristic gh(graph, palns_problem_params);
            initial = gh.solve();
        }

        const PALNSSolution palns_initial(initial, &palns_problem_params);

        mlpalns::PALNS<Graph, PALNSSolution> palns_solver(graph);

        const Clustering clustering(&graph);

        std::size_t n_destroy = 0u, n_repair = 0u;

        // --- Destroy methods --- //

        RandomRemove random_remove(&palns_problem_params);
        if(palns_problem_params.destroy.enable_random) {
            n_destroy = palns_solver.add_destroy_method(random_remove, "Random Remove");
        }

        RandomSeqRemove random_seq_remove(&palns_problem_params);
        if(palns_problem_params.destroy.enable_random_seq) {
            n_destroy = palns_solver.add_destroy_method(random_seq_remove, "Random Seq Remove");
        }

        RandomClusterRemove random_cl_remove(&palns_problem_params, &clustering);
        if(palns_problem_params.destroy.enable_random_cluster &&
           clustering.is_proper() && clustering.n_clusters > 1u)
        {
            n_destroy = palns_solver.add_destroy_method(random_cl_remove, "Random Cluster Remove");
        }

        // --- Repair methods --- //

        GreedyRepair greedy_repair(&palns_problem_params);
        if(palns_problem_params.repair.enable_greedy) {
            n_repair = palns_solver.add_repair_method(greedy_repair, "Greedy Repair");
        }

        SeqVertexRepair random_repair_i(&palns_problem_params, vertex_shuffle);
        if(palns_problem_params.repair.enable_seq_random) {
            n_repair = palns_solver.add_repair_method(random_repair_i, "Seq Repair (random)");
        }

        SeqVertexRepair insert_by_prize_i(&palns_problem_params, vertex_by_prize);
        if(palns_problem_params.repair.enable_seq_by_prize) {
            n_repair = palns_solver.add_repair_method(insert_by_prize_i, "Seq Repair (by prize)");
        }

        RandomClusterRepair random_cl_repair_i(&palns_problem_params, &clustering);
        if(palns_problem_params.repair.enable_cluster &&
           clustering.is_proper() && clustering.n_clusters > 1u)
        {
            n_repair = palns_solver.add_repair_method(random_cl_repair_i, "Random Cluster Repair");
        }

        // --- Algorithm visitor --- //

        time_point<high_resolution_clock> last_best_update;
        std::unique_ptr<mlpalns::AlgorithmVisitor<PALNSSolution>> visitor =
            std::make_unique<PALNSVisitor>(palns_problem_params, mt,
                                           n_destroy + 1, n_repair + 1,
                                           methods_stats_file, last_best_update);

        palns_solver.set_algorithm_visitor(visitor);

        // --- Algorithm start --- //

        const auto start_time = high_resolution_clock::now();
        PALNSSolution solution = palns_solver.go(palns_initial, 4u, palns_framework_params);
        const auto end_time = high_resolution_clock::now();
        total_time_s = duration_cast<duration<float>>(end_time - start_time).count();
        time_to_best_s = duration_cast<duration<float>>(last_best_update - start_time).count();

        // --- Algorithm end --- //

        // Make sure there is absolutely no vertex which can be added.
        palns_problem_params.repair.heuristic = false;
        greedy_repair.repair_solution(solution, mt);

        return solution.tour;
    }
}