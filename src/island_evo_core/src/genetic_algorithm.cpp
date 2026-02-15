#include "island_evo_core/genetic_algorithm.hpp"
#include <iostream>
#include <numeric>

// 构造函数
GeneticAlgorithm::GeneticAlgorithm(int pop_size, int gene_dim, double crossover_rate, double mutation_rate, 
                                   FitnessFunction fitness_func)
: pop_size_(pop_size), gene_dim_(gene_dim), 
  crossover_rate_(crossover_rate), mutation_rate_(mutation_rate),
  fitness_func_(fitness_func)
{
    // 初始化随机数种子
    std::random_device rd;
    rng_ = std::mt19937(rd());
    
    initialize_population();
}

void GeneticAlgorithm::initialize_population() {
    population_.clear();
    population_.resize(pop_size_);

    // 假设搜索范围在 [-5.12, 5.12] 之间 (常见测试函数范围)
    std::uniform_real_distribution<double> dist(-5.12, 5.12);

    for (auto& ind : population_) {
        ind.genes.resize(gene_dim_);
        for (int i = 0; i < gene_dim_; ++i) {
            ind.genes[i] = dist(rng_);
        }
    }
    // 初始化完立刻计算一次适应度
    evaluate_fitness();
}

void GeneticAlgorithm::evaluate_fitness() {
    for (auto& ind : population_) {
        // 核心：调用外部注入的函数计算适应度
        ind.fitness = fitness_func_(ind.genes);
    }
    // 排序：fitness 越小越好，排在前面
    std::sort(population_.begin(), population_.end());
}

void GeneticAlgorithm::selection() {
    // 简单实现：轮盘赌选择 (针对最小化问题需要转换适应度)
    // 为了简化演示，这里暂时使用锦标赛选择 (Tournament Selection)，更适合最小化问题且更容易实现
    std::vector<Individual> new_population;
    new_population.reserve(pop_size_);
    
    std::uniform_int_distribution<int> dist(0, pop_size_ - 1);

    for (int i = 0; i < pop_size_; ++i) {
        // 随机选两个，留最好的
        int p1 = dist(rng_);
        int p2 = dist(rng_);
        if (population_[p1].fitness < population_[p2].fitness) {
            new_population.push_back(population_[p1]);
        } else {
            new_population.push_back(population_[p2]);
        }
    }
    population_ = new_population;
}

void GeneticAlgorithm::crossover() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::uniform_int_distribution<int> dim_dist(0, gene_dim_ - 1);

    for (int i = 0; i < pop_size_; i += 2) {
        if (i + 1 >= pop_size_) break;

        if (dist(rng_) < crossover_rate_) {
            // 二项式交叉
            for (int j = 0; j < gene_dim_; ++j) {
                if (dist(rng_) < 0.5) {
                    std::swap(population_[i].genes[j], population_[i+1].genes[j]);
                }
            }
        }
    }
}

void GeneticAlgorithm::mutation() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::normal_distribution<double> gauss(0.0, 0.5); // 均值0，标准差0.5

    for (auto& ind : population_) {
        if (dist(rng_) < mutation_rate_) {
            // 随机选择一个基因位进行变异
            std::uniform_int_distribution<int> dim_dist(0, gene_dim_ - 1);
            int mutation_point = dim_dist(rng_);
            ind.genes[mutation_point] += gauss(rng_);
        }
    }
}

void GeneticAlgorithm::step() {

    Individual elite = population_[0];
    // 1. 选择
    selection();
    // 2. 交叉
    crossover();
    // 3. 变异
    mutation();
    // 4. 评估 (必须重新评估，因为基因变了)
    evaluate_fitness();

    if (elite.fitness < population_[0].fitness) {
        population_.back() = elite;
        
        // 再次排序，把刚刚插进去的王者排到第一位去
        std::sort(population_.begin(), population_.end());
    }
    // 注意：如果新一代本来就产生了比 elite 更强的，那就不需要替换了，
    // 这样保证了种群最优解永远是“单调不增”的。
    current_generation++;
}

Individual GeneticAlgorithm::get_best_individual() const {
    return population_.front();
}

void GeneticAlgorithm::add_migrant(const std::vector<double>& genes, double fitness) {
    // 简单的策略：直接替换掉当前最差的个体
    // 因为 evaluate_fitness 后是排好序的，最差的在最后
    if (fitness < population_.back().fitness) {
        population_.back().genes = genes;
        population_.back().fitness = fitness;
        // 重新排序维持有序性
        std::sort(population_.begin(), population_.end());
    }
}
