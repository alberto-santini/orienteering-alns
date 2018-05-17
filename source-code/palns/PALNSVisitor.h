//
// Created by alberto on 12/11/17.
//

#ifndef OP_PALNSVISITOR_H
#define OP_PALNSVISITOR_H

#include <random>
#include <chrono>
#include <palns/AlgorithmVisitor.h>
#include <palns/DestroyMethod.h>
#include <palns/RepairMethod.h>
#include "PALNSSolution.h"
#include "PALNSProblemParams.h"
#include "destroy/RandomRemove.h"

namespace op {
    struct PALNSVisitor : public mlpalns::AlgorithmVisitor<PALNSSolution> {
        const PALNSProblemParams& problem_params;
        std::mt19937& mt;

        std::vector<std::size_t> destroy_best;
        std::vector<std::size_t> repair_best;
        std::vector<std::string> destroy_names;
        std::vector<std::string> repair_names;

        std::experimental::filesystem::path scores_file;

        std::chrono::time_point<std::chrono::high_resolution_clock>& last_best_update;

        bool reset_fraction;

        PALNSVisitor(
            const PALNSProblemParams& problem_params,
            std::mt19937& mt,
            std::size_t n_destroy,
            std::size_t n_repair,
            std::experimental::filesystem::path scores_file,
            std::chrono::time_point<std::chrono::high_resolution_clock>& last_best_update
        ) :
            problem_params{problem_params},
            mt{mt},
            destroy_best(n_destroy, 0u),
            repair_best(n_repair, 0u),
            scores_file(scores_file),
            last_best_update(last_best_update),
            reset_fraction(false)
        {}

        void on_algorithm_start(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy,
                                std::vector<mlpalns::RepairMethod<PALNSSolution>*>& repair,
                                const std::vector<std::string>& dnames,
                                const std::vector<std::string>& rnames) override;

        void on_prerun_end(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy,
                           std::vector<mlpalns::RepairMethod<PALNSSolution>*>& repair) override;

        void on_iteration_end(mlpalns::AlgorithmStatus<PALNSSolution>& alg_status) override;

        void on_many_iters_without_improvement(
            std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy,
            std::vector<mlpalns::RepairMethod<PALNSSolution>*>& repair
        ) override;

        void print_scores() const;

        ~PALNSVisitor() override;

    private:
        RandomRemove* get_random_remove(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy);
        void increase_random_remove_fraction(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy);
        void reset_random_remove_fraction(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy);
        void max_random_remove_fraction(std::vector<mlpalns::DestroyMethod<PALNSSolution>*>& destroy);
    };
}

#endif //OP_PALNSVISITOR_H
