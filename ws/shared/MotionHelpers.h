#pragma once
#include <Eigen/Core>
#include <vector>
#include "CollisionHelpers.h"

namespace amp {

    // Moves robot one step in the direction of goal at the magnitude of stepSize
    Eigen::Vector2d moveToGoal(const Eigen::Vector2d& currentPos,
                               const Eigen::Vector2d& goalPos,
                               double stepSize);

    /*Computes the next point along the perimeter of an obstacle, assumes small epsilon and step size to handle
     * non-convex cases...
     *
     * current: current robot position
     * currentObstacleIdx: most recent obstacle collision
     * dir: current direction vector (updated by this function)
     * stepSize: forward step size
     * deltaTheta: small rotation increment (radians)
     * obstacles: list of all obstacles
     * leftHand: true = keep obstacle on left, false = right
     * epsilon: collision tolerance */
    Eigen::Vector2d nextPerimeterStep(
        const Eigen::Vector2d& current,
        size_t& currentObstacleIdx,
        Eigen::Vector2d& dir,
        double stepSize,
        double deltaTheta,
        const std::vector<Obstacle2D>& obstacles,
        std::unordered_set<size_t>& cluster,
        bool leftHand,
        double epsilon
    );

}
