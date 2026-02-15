#ifndef ISLAND_EVO_CORE__GENETIC_ALGORITHM_HPP_
#define ISLAND_EVO_CORE__GENETIC_ALGORITHM_HPP_

#include "island_evo_core/optimization_algorithm.hpp" // <--- 改引用这个
#include <random>
#include <algorithm>

// 让 GA 继承 OptimizationAlgorithm
class GeneticAlgorithm : public OptimizationAlgorithm {
public:
    GeneticAlgorithm(int pop_size, int gene_dim, double crossover_rate, double mutation_rate, 
                     FitnessFunction fitness_func);

    // 加上 override 关键字，确保我们实现了接口
    void step() override;
    Individual get_best_individual() const override;
    void add_migrant(const std::vector<double>& genes, double fitness) override;

private:
    int pop_size_;
    int gene_dim_;
    double crossover_rate_;
    double mutation_rate_;
    
    FitnessFunction fitness_func_; 
    std::vector<Individual> population_; // Individual 定义在基类头文件里了
    std::mt19937 rng_;

    void initialize_population();
    void evaluate_fitness();
    void selection();
    void crossover();
    void mutation();
};

#endif
