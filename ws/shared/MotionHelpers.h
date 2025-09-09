#pragma once
#include <Eigen/Core>

namespace amp {
    Eigen::Vector2d moveToGoal(const Eigen::Vector2d& currentPos,
                               const Eigen::Vector2d& goalPos,
                               double stepSize);
}
