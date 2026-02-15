
#ifndef ISLAND_EVO_CORE__DE_ALGORITHM_HPP_
#define ISLAND_EVO_CORE__DE_ALGORITHM_HPP_

#include "island_evo_core/optimization_algorithm.hpp"
#include <random>
#include <vector>

class DEAlgorithm : public OptimizationAlgorithm {
public:
    // F: 缩放因子 (通常 0.5 ~ 0.9)
    // CR: 交叉概率 (通常 0.8 ~ 0.9)
    DEAlgorithm(int pop_size, int gene_dim, double F, double CR,
                FitnessFunction fitness_func);

    void step() override;
    Individual get_best_individual() const override;
    void add_migrant(const std::vector<double>& genes, double fitness) override;

private:
    int pop_size_;
    int gene_dim_;
    double F_;
    double CR_;

    FitnessFunction fitness_func_;
    std::vector<Individual> population_;
    std::mt19937 rng_;

    void initialize_population();
};

#endif
