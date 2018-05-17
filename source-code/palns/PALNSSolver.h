//
// Created by alberto on 31/10/17.
//

#ifndef OP_PALNSSOLVER_H
#define OP_PALNSSOLVER_H

#include "../Tour.h"
#include "PALNSProblemParams.h"

namespace op {
    struct PALNSSolver {
        const Graph& graph;

        PALNSProblemParams palns_problem_params;
        mlpalns::Parameters palns_framework_params;
        std::experimental::filesystem::path methods_stats_file;

        PALNSSolver(const Graph& graph,
                    std::experimental::filesystem::path palns_problem_params_file,
                    std::experimental::filesystem::path palns_framework_params_file,
                    std::experimental::filesystem::path methods_stats_file);

        Tour solve(std::unique_ptr<Tour>& initial_sol);

        float get_total_time_s() const { return total_time_s; }
        float get_time_to_best_s() const { return time_to_best_s; }

    private:
        float total_time_s;
        float time_to_best_s;
    };
}

#endif //OP_PALNSSOLVER_H
