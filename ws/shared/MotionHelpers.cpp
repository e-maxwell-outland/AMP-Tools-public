#include "MotionHelpers.h"
#include <cmath>

namespace amp {

    // ------------------- Moves the robot one step size towards goal -------------------
    Eigen::Vector2d moveToGoal(const Eigen::Vector2d& currentPos, const Eigen::Vector2d& goalPos, double stepSize) {
        Eigen::Vector2d direction = goalPos - currentPos;
        double distance = direction.norm();
        if (distance <= stepSize) return goalPos;
        direction.normalize();
        return currentPos + direction * stepSize;
    }
}
