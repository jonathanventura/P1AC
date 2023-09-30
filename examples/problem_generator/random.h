#pragma once

#include <random>
#include <Eigen/Core>

namespace problem_generator
{
    static std::default_random_engine engine;

    class RandomGenerator
    {
        std::uniform_real_distribution<double> uniform;
        std::normal_distribution<double> normal;
    public:
        RandomGenerator() : uniform(-1,1) { }
        
        double rand() { return uniform(engine); }
        double randn() { return normal(engine); }
        Eigen::Vector2d rand2() { return Eigen::Vector2d(rand(),rand()); }
        Eigen::Vector3d rand3() { return Eigen::Vector3d(rand(),rand(),rand()); }

        Eigen::Vector2d randn2() { return Eigen::Vector2d(randn(),randn()); }
        Eigen::Vector3d randn3() { return Eigen::Vector3d(randn(),randn(),randn()); }

        Eigen::Vector3d rand_unit_vector() { const Eigen::Vector3d v = randn3(); return v/v.norm(); }
    };
}
