
#pragma once

#include <Eigen/Core>
#include <vector>

#include <problem_generator/random.h>

namespace problem_generator
{
    class ProblemGenerator
    {
        RandomGenerator generator;
    public:
        void make_random_problem(
            Eigen::Vector2d &x,
            double &d,
            Eigen::Vector3d &n,
            Eigen::Vector2d &y,
            Eigen::Matrix2d &A,
            Eigen::Matrix3d &R,
            Eigen::Vector3d &t
        );
    };

}
