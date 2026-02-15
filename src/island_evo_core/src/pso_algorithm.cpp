#include "island_evo_core/pso_algorithm.hpp"
#include <limits>
#include <algorithm>
#include <iostream>

PSOAlgorithm::PSOAlgorithm(int pop_size, int gene_dim, double w, double c1, double c2, 
                           FitnessFunction fitness_func)
: pop_size_(pop_size), gene_dim_(gene_dim), w_(w), c1_(c1), c2_(c2),
  fitness_func_(fitness_func)
{
    std::random_device rd;
    rng_ = std::mt19937(rd());
    
    // 初始化 gBest 为无穷大
    global_best_fitness_ = std::numeric_limits<double>::max();
    
    initialize_swarm();
}

void PSOAlgorithm::initialize_swarm() {
    swarm_.resize(pop_size_);
    std::uniform_real_distribution<double> dist(-5.12, 5.12);
    std::uniform_real_distribution<double> vel_dist(-1.0, 1.0);

    for (auto& p : swarm_) {
        p.position.resize(gene_dim_);
        p.velocity.resize(gene_dim_);
        
        for (int i = 0; i < gene_dim_; ++i) {
            p.position[i] = dist(rng_);
            p.velocity[i] = vel_dist(rng_);
        }
        
        // 计算初始适应度
        p.fitness = fitness_func_(p.position);
        
        // 初始化 pBest
        p.best_position = p.position;
        p.best_fitness = p.fitness;

        // 更新 gBest
        if (p.fitness < global_best_fitness_) {
            global_best_fitness_ = p.fitness;
            global_best_position_ = p.position;
        }
    }
}

void PSOAlgorithm::step() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (auto& p : swarm_) {
        for (int i = 0; i < gene_dim_; ++i) {
            double r1 = dist(rng_);
            double r2 = dist(rng_);

            // 1. 更新速度
            // v = w*v + c1*r1*(pBest - x) + c2*r2*(gBest - x)
            double cognitive = c1_ * r1 * (p.best_position[i] - p.position[i]);
            double social = c2_ * r2 * (global_best_position_[i] - p.position[i]);
            
            p.velocity[i] = w_ * p.velocity[i] + cognitive + social;

            // 可选：速度限制 (Clamping)，防止飞太快
            // p.velocity[i] = std::clamp(p.velocity[i], -2.0, 2.0);

            // 2. 更新位置
            p.position[i] += p.velocity[i];
        }

        // 3. 评估适应度
        p.fitness = fitness_func_(p.position);

        // 4. 更新 pBest
        if (p.fitness < p.best_fitness) {
            p.best_fitness = p.fitness;
            p.best_position = p.position;
        }

        // 5. 更新 gBest
        if (p.fitness < global_best_fitness_) {
            global_best_fitness_ = p.fitness;
            global_best_position_ = p.position;
        }
    }
    
    current_generation++;
}

Individual PSOAlgorithm::get_best_individual() const {
    // PSO 的最优个体就是 gBest
    Individual ind;
    ind.genes = global_best_position_;
    ind.fitness = global_best_fitness_;
    return ind;
}

void PSOAlgorithm::add_migrant(const std::vector<double>& genes, double fitness) {
    // 简单策略：替换掉当前最差的粒子
    // 1. 找到最差粒子
    auto worst_it = std::max_element(swarm_.begin(), swarm_.end(), 
        [](const Particle& a, const Particle& b) {
            return a.fitness < b.fitness; // 找 fitness 最大的 (最差的)
        });

    // 2. 只有当移民比最差的强时才替换
    if (fitness < worst_it->fitness) {
        worst_it->position = genes;
        worst_it->fitness = fitness;
        
        // 重要：移民也是新的 pBest
        worst_it->best_position = genes;
        worst_it->best_fitness = fitness;
        
        // 速度归零 (让它重新开始加速)
        std::fill(worst_it->velocity.begin(), worst_it->velocity.end(), 0.0);

        // 检查移民是否打破了 gBest
        if (fitness < global_best_fitness_) {
            global_best_fitness_ = fitness;
            global_best_position_ = genes;
        }
    }
}
