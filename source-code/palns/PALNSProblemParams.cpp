//
// Created by alberto on 12/11/17.
//

#include "PALNSProblemParams.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>

#define READPARAM(varname, vartype)                                                    \
        try{                                                                           \
            varname = t.get<vartype>(#varname);                                        \
        } catch(...) {                                                                 \
            std::cerr << "Cannot find key " << #varname << ": using default value.\n"; \
        }

namespace op {
    PALNSProblemParams::DestroyMethodsParams::DestroyMethodsParams() :
        enable_random{true},
        enable_random_seq{true},
        enable_random_cluster{true},
        fraction_of_vertices_to_remove{0.33f},
        max_fraction_of_vertices_to_remove{0.75f},
        max_n_of_vertices_to_remove{40u},
        adaptive{true}
    {}

    PALNSProblemParams::RepairMethodsParams::RepairMethodsParams() :
        enable_greedy{true},
        enable_seq_random{true},
        enable_seq_by_prize{true},
        enable_cluster{true},
        heuristic{false},
        intermediate_infeasible{true},
        use_2opt_before_restoring_feasibility{true},
        restore_feasibility_optimal{0.0f}
    {}

    PALNSProblemParams::InitialSolutionParams::InitialSolutionParams() :
        use_clustering{true},
        use_mip{false}
    {}

    PALNSProblemParams::LocalSearchParams::LocalSearchParams() :
        use_2opt{true},
        use_tsp{false},
        fill_tour{true}
    {}

    PALNSProblemParams::PALNSProblemParams(std::experimental::filesystem::path params_file) :
        destroy{}, repair{}, initial_solution{}, local_search{}
    {
        using namespace boost::property_tree;

        ptree t;
        read_json(params_file, t);

        READPARAM(destroy.enable_random, bool)
        READPARAM(destroy.enable_random_seq, bool)
        READPARAM(destroy.enable_random_cluster, bool)
        READPARAM(destroy.fraction_of_vertices_to_remove, float)
        READPARAM(destroy.max_fraction_of_vertices_to_remove, float)
        READPARAM(destroy.max_n_of_vertices_to_remove, std::size_t)
        READPARAM(destroy.adaptive, bool)

        READPARAM(repair.enable_greedy, bool)
        READPARAM(repair.enable_seq_random, bool)
        READPARAM(repair.enable_seq_by_prize, bool)
        READPARAM(repair.enable_cluster, bool)
        READPARAM(repair.heuristic, bool)
        READPARAM(repair.intermediate_infeasible, bool)
        READPARAM(repair.use_2opt_before_restoring_feasibility, bool)
        READPARAM(repair.restore_feasibility_optimal, float)

        READPARAM(initial_solution.use_clustering, bool)
        READPARAM(initial_solution.use_mip, bool)

        READPARAM(local_search.use_2opt, bool)
        READPARAM(local_search.use_tsp, bool)
        READPARAM(local_search.fill_tour, bool)
    }
}
