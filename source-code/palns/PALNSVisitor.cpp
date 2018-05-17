//
// Created by alberto on 12/11/17.
//

#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <as/and_die.h>

#include "PALNSVisitor.h"
#include "../LinKernighan.h"
#include "repair/GreedyRepair.h"

namespace op {
    void PALNSVisitor::on_iteration_end(mlpalns::AlgorithmStatus<op::PALNSSolution>& alg_status) {
        if(alg_status.new_best) {
            last_best_update = std::chrono::high_resolution_clock::now();

            ++destroy_best[alg_status.destroy_method_id];
            ++repair_best[alg_status.repair_method_id];

            if(problem_params.local_search.use_2opt) {
                assert(!problem_params.local_search.use_tsp);
                alg_status.best_solution.tour.do_2opt();
            }

            if(problem_params.local_search.use_tsp) {
                assert(!problem_params.local_search.use_2opt);
                std::stringstream ss;
                ss << std::this_thread::get_id();

                Tour t = run_lin_kernighan(
                    *alg_status.best_solution.graph,
                    alg_status.best_solution.tour.vertices,
                    ss.str()
                );

                if(t.travel_time < alg_status.best_solution.tour.travel_time) {
                    std::cout << as::console::notice << "Solving the TSP saved ";
                    std::cout << alg_status.best_solution.tour.travel_time - t.travel_time;
                    std::cout << " in travel time.\n";
                    alg_status.best_solution = PALNSSolution(t, &problem_params);
                }
            }

            if(problem_params.local_search.fill_tour) {
                GreedyRepair gr(&problem_params);
                gr.repair_solution(alg_status.best_solution, mt);
            }

            if(problem_params.destroy.adaptive) {
                reset_fraction = true;
            }
        }
    }

    PALNSVisitor::~PALNSVisitor() {
        print_scores();
    }

    void PALNSVisitor::on_algorithm_start(
        std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy,
        std::vector<mlpalns::RepairMethod<PALNSSolution>*>&,
        const std::vector<std::string>& dnames,
        const std::vector<std::string>& rnames)
    {
        assert(dnames.size() == destroy_best.size());
        destroy_names = dnames;

        assert(rnames.size() == repair_best.size());
        repair_names = rnames;

        if(problem_params.destroy.adaptive) {
            max_random_remove_fraction(destroy);
        }
    }

    void PALNSVisitor::on_prerun_end(
        std::vector<mlpalns::DestroyMethod<PALNSSolution> *>& destroy,
        std::vector<mlpalns::RepairMethod<PALNSSolution> *>&)
    {
        if(problem_params.destroy.adaptive) {
            reset_random_remove_fraction(destroy);
        }
    }

    void PALNSVisitor::print_scores() const {
        if(scores_file == "") { return; }

        using namespace boost::property_tree;
        using namespace boost::interprocess;

        ptree t;

        try {
            file_lock flock(scores_file.string().c_str());
            scoped_lock<file_lock> slock(flock);

            read_json(scores_file, t);

            for(auto i = 0u; i < destroy_names.size(); ++i) {
                std::size_t score = 0u;
                try {
                    score = t.get<std::size_t>(destroy_names[i]);
                } catch(...) {
                    // Score does not exist.
                }
                t.put(destroy_names[i], score + destroy_best[i]);
            }

            for(auto i = 0u; i < repair_names.size(); ++i) {
                std::size_t score = 0u;
                try {
                    score = t.get<std::size_t>(repair_names[i]);
                } catch(...) {
                    // Score does not exist.
                }
                t.put(repair_names[i], score + repair_best[i]);
            }

            std::ofstream ofs(scores_file);

            if(ofs.fail()) {
                std::cerr << as::console::error << "Cannot write solution json to " << scores_file << as::and_die();
            }

            write_json(ofs, t);
            ofs.flush();
            ofs.close();
        } catch(interprocess_exception& ex) {
            std::cerr << as::console::error << "Interprocess exception: " << ex.what() << as::and_die();
        }
    }

    void PALNSVisitor::on_many_iters_without_improvement(std::vector<mlpalns::DestroyMethod<PALNSSolution> *>& destroy, std::vector<mlpalns::RepairMethod<PALNSSolution> *>&) {
        if(problem_params.destroy.adaptive) {
            if(reset_fraction) {
                reset_random_remove_fraction(destroy);
                reset_fraction = false;
            } else {
                increase_random_remove_fraction(destroy);
            }
        }
    }

    RandomRemove* PALNSVisitor::get_random_remove(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy) {
        auto random_remove_it = std::find(destroy_names.begin(), destroy_names.end(), "Random Remove");
        auto random_remove_id = random_remove_it - destroy_names.begin();
        auto destroy_method = dynamic_cast<RandomRemove*>(destroy[random_remove_id]);

        return destroy_method;
    }

    void PALNSVisitor::increase_random_remove_fraction(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy) {
        auto destroy_method = get_random_remove(destroy);
        destroy_method->set_frac_v_remove(
            std::min(
                destroy_method->get_frac_v_remove() * 1.1f,
                problem_params.destroy.max_fraction_of_vertices_to_remove
            )
        ); // Increase by 10%, up to a maximum
    }

    void PALNSVisitor::reset_random_remove_fraction(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy) {
        auto destroy_method = get_random_remove(destroy);
        destroy_method->set_frac_v_remove(problem_params.destroy.fraction_of_vertices_to_remove);
    }

    void PALNSVisitor::max_random_remove_fraction(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy) {
        auto destroy_method = get_random_remove(destroy);
        destroy_method->set_frac_v_remove(problem_params.destroy.max_fraction_of_vertices_to_remove);
    }
}