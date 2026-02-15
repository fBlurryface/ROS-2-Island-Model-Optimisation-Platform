#ifndef ISLAND_EVO_CORE__PSO_ALGORITHM_HPP_
#define ISLAND_EVO_CORE__PSO_ALGORITHM_HPP_

#include "island_evo_core/optimization_algorithm.hpp"
#include <vector>
#include <random>

struct Particle {
    std::vector<double> position; // 当前位置 (genes)
    std::vector<double> velocity; // 当前速度
    double fitness;               // 当前适应度

    // 历史记忆
    std::vector<double> best_position; // pBest 位置
    double best_fitness;               // pBest 适应度
};

class PSOAlgorithm : public OptimizationAlgorithm {
public:
    // w: 惯性权重, c1: 认知系数(自我), c2: 社会系数(群体)
    PSOAlgorithm(int pop_size, int gene_dim, double w, double c1, double c2, 
                 FitnessFunction fitness_func);

    void step() override;
    Individual get_best_individual() const override;
    void add_migrant(const std::vector<double>& genes, double fitness) override;

private:
    int pop_size_;
    int gene_dim_;
    double w_;  // Inertia
    double c1_; // Cognitive (Personal)
    double c2_; // Social (Global)

    FitnessFunction fitness_func_;
    std::vector<Particle> swarm_;
    std::mt19937 rng_;

    // 全局最优 (gBest)
    std::vector<double> global_best_position_;
    double global_best_fitness_;

    void initialize_swarm();
    void update_particles();
};

#endif
