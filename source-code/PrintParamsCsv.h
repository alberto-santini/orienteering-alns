//
// Created by alberto on 24/01/18.
//

#ifndef OP_PRINTPARAMSCSV_H
#define OP_PRINTPARAMSCSV_H

#include "palns/PALNSProblemParams.h"
#include <iostream>
#include <palns/Parameters.h>

namespace op {
    inline std::ostream& operator<<(std::ostream& out, const PALNSProblemParams& params) {
        out << params.destroy.enable_random << ","
            << params.destroy.enable_random_seq << ","
            << params.destroy.enable_random_cluster << ","
            << params.destroy.fraction_of_vertices_to_remove << ","
            << params.destroy.max_fraction_of_vertices_to_remove << ","
            << params.destroy.max_n_of_vertices_to_remove << ","
            << params.destroy.adaptive << ","
            << params.repair.enable_greedy << ","
            << params.repair.enable_seq_random << ","
            << params.repair.enable_seq_by_prize << ","
            << params.repair.enable_cluster << ","
            << params.repair.heuristic << ","
            << params.repair.intermediate_infeasible << ","
            << params.repair.use_2opt_before_restoring_feasibility << ","
            << params.initial_solution.use_clustering << ","
            << params.local_search.use_2opt << ","
            << params.local_search.use_tsp << ","
            << params.local_search.fill_tour;
        return out;
    }

    inline std::ostream& operator<<(std::ostream& out, const mlpalns::Parameters& params) {
        out << params.score_decay << ",";
        out << params.score_mult_accepted << ",";
        out << params.score_mult_improved << ",";
        out << params.score_mult_global_best << ",";

        switch(params.acceptance_criterion_id) {
            case mlpalns::Parameters::AcceptanceCriterionId::RecordToRecordTravel:
                out << "rrt;";
                out << params.rrt_params.start_deviation << ";";
                out << params.rrt_params.end_deviation << ";";
                out << params.rrt_params.deviation_decrease_is_linear;
                break;
            case mlpalns::Parameters::AcceptanceCriterionId ::SimulatedAnnealing:
                out << "sa;";
                out << params.sa_params.init_accept_ratio_50p << ";";
                out << params.sa_params.end_accept_ratio_50p << ";";
                out << params.sa_params.temperature_decrease_is_linear << ";";
                out << params.sa_params.magic_number_exponent << ";";
                out << params.sa_params.reheating_is_enabled << ";";
                out << params.sa_params.reheating_times << ";";
                out << params.sa_params.reheating_coefficient;
                break;
            case mlpalns::Parameters::AcceptanceCriterionId::ThresholdAcceptance:
                out << "ta;";
                out << params.ta_params.start_threshold << ";";
                out << params.ta_params.end_threshold << ";";
                out << params.ta_params.threshold_decrease_is_linear;
                break;
            default:
                out << "other";
        }

        return out;
    }
}

#endif //OP_PRINTPARAMSCSV_H
