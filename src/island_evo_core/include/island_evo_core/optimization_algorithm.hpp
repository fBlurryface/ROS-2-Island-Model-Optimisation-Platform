#ifndef ISLAND_EVO_CORE__OPTIMIZATION_ALGORITHM_HPP_
#define ISLAND_EVO_CORE__OPTIMIZATION_ALGORITHM_HPP_

#include <vector>
#include <functional>

// 1. 把 Individual 结构体搬到这里，大家共用
struct Individual {
    std::vector<double> genes;
    double fitness;
    
    bool operator<(const Individual& other) const {
        return fitness < other.fitness; // 最小化问题
    }
};

// 2. 定义纯虚基类 (Interface)
class OptimizationAlgorithm {
public:
    virtual ~OptimizationAlgorithm() = default;

    // 通用函数指针定义
    using FitnessFunction = std::function<double(const std::vector<double>&)>;

    // 核心接口：每个算法必须实现这三个函数
    virtual void step() = 0;
    virtual Individual get_best_individual() const = 0;
    virtual void add_migrant(const std::vector<double>& genes, double fitness) = 0;

    // 公共变量
    int current_generation = 0;
};

#endif
