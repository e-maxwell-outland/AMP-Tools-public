#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <cstddef>  // for size_t and SIZE_MAX

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1Algorithm : public amp::BugAlgorithm {
    public:
        explicit Bug1Algorithm(bool leftTurning);   // constructor declaration
        amp::Path2D plan(const amp::Problem2D& problem) override;
    
    private:
        static constexpr double EPSILON = 0.005;   // collision buffer
        static constexpr double STEP_SIZE = 0.005; // step size towards goal
        static constexpr double DELTA_THETA = 0.02; // amount for robot to turn while perimeter following (in rad)
        size_t hitObstacleIdx = SIZE_MAX; // "no obstacle yet"
        bool leftTurning;
};