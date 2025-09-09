#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <cstddef>  // for size_t and SIZE_MAX

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1Algorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        amp::Path2D plan(const amp::Problem2D& problem) override;
    
    private:
        static constexpr double EPSILON = 0.05;   // collision buffer
        static constexpr double STEP_SIZE = 0.2; // step size towards goal
        size_t hitObstacleIdx = SIZE_MAX; // "no obstacle yet"
};