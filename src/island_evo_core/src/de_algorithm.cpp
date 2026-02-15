#include "island_evo_core/de_algorithm.hpp"
#include <algorithm>
#include <iostream>

DEAlgorithm::DEAlgorithm(int pop_size, int gene_dim, double F, double CR, 
                         FitnessFunction fitness_func)
: pop_size_(pop_size), gene_dim_(gene_dim), F_(F), CR_(CR),
  fitness_func_(fitness_func)
{
    std::random_device rd;
    rng_ = std::mt19937(rd());
    initialize_population();
}

void DEAlgorithm::initialize_population() {
    population_.resize(pop_size_);
    std::uniform_real_distribution<double> dist(-5.12, 5.12);
    
    for (auto& ind : population_) {
        ind.genes.resize(gene_dim_);
        for (int i = 0; i < gene_dim_; ++i) {
            ind.genes[i] = dist(rng_);
        }
        ind.fitness = fitness_func_(ind.genes);
    }
}

void DEAlgorithm::step() {
    std::uniform_int_distribution<int> idx_dist(0, pop_size_ - 1);
    std::uniform_real_distribution<double> rand_01(0.0, 1.0);
    std::uniform_int_distribution<int> dim_dist(0, gene_dim_ - 1);

    // DE 需要一个临时的种群容器，或者直接原地更新（如果只看当前代）
    // 标准 DE 通常基于当前代生成下一代，不应该立即使用刚更新的个体
    // 所以我们需要一个 next_population
    std::vector<Individual> next_population = population_;

    for (int i = 0; i < pop_size_; ++i) {
        // 1. 选择三个互不相同且不等于 i 的随机索引 r1, r2, r3
        int r1, r2, r3;
        do { r1 = idx_dist(rng_); } while (r1 == i);
        do { r2 = idx_dist(rng_); } while (r2 == i || r2 == r1);
        do { r3 = idx_dist(rng_); } while (r3 == i || r3 == r1 || r3 == r2);

        // 2. 变异 & 交叉 (生成试验向量 Trial Vector)
        Individual trial = population_[i]; // 先复制父代作为基底
        int j_rand = dim_dist(rng_); // 确保至少有一个维度发生变异

        for (int j = 0; j < gene_dim_; ++j) {
            // Binomial Crossover
            if (rand_01(rng_) < CR_ || j == j_rand) {
                // Mutation: v = x_r1 + F * (x_r2 - x_r3)
                double diff = population_[r2].genes[j] - population_[r3].genes[j];
                trial.genes[j] = population_[r1].genes[j] + F_ * diff;
            }
            // else: 保持父代基因 (trial.genes[j] 已经是父代的了)
        }

        // 3. 评估 Trial Fitness
        trial.fitness = fitness_func_(trial.genes);

        // 4. 贪婪选择 (Greedy Selection)
        // 如果子代更强（或一样强），就取代父代
        // 注意：这是最小化问题
        if (trial.fitness <= population_[i].fitness) {
            next_population[i] = trial;
        } else {
            next_population[i] = population_[i]; // 保持原样
        }
    }

    // 更新种群
    population_ = next_population;
    current_generation++;
}

Individual DEAlgorithm::get_best_individual() const {
    // 遍历寻找最优
    auto it = std::min_element(population_.begin(), population_.end(),
        [](const Individual& a, const Individual& b) {
            return a.fitness < b.fitness;
        });
    return *it;
}

void DEAlgorithm::add_migrant(const std::vector<double>& genes, double fitness) {
    // 替换掉当前种群中最差的一个
    auto worst_it = std::max_element(population_.begin(), population_.end(),
        [](const Individual& a, const Individual& b) {
            return a.fitness < b.fitness;
        });

    // 只有当移民比最差的强时才替换
    if (fitness < worst_it->fitness) {
        worst_it->genes = genes;
        worst_it->fitness = fitness;
    }
}
