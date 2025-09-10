#include "Bug1Algorithm.h"
#include "MotionHelpers.h"
#include "CollisionHelpers.h"
#include <iostream>

Bug1Algorithm::Bug1Algorithm(bool leftTurning)
    : leftTurning(leftTurning) {}

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    auto current = problem.q_init;
    auto end = problem.q_goal;
    auto obstacles = problem.obstacles;

    // Pre-run, build an adjacency graph to determine what obstacles intersect with each other
    auto adj = amp::buildAdjacencyList(obstacles);

    path.waypoints.push_back(current);

    //std::cout << "Starting navigation...\n";

    while ((current - end).norm() > Bug1Algorithm::STEP_SIZE * 0.5) {

        // Try to go forward
        auto next = amp::moveToGoal(current, end, Bug1Algorithm::STEP_SIZE);

        // Check if your next "step" forward will cause a collision. If so, find hit point on obstacle and enter
        // perimeter following mode.
        if (isInCollision(next, obstacles, Bug1Algorithm::EPSILON, this->hitObstacleIdx)) {

            // Do dfs to determine the entire "island" the robot has just hit (if more than 1 obstacle)
            std::unordered_set<size_t> cluster; // empty set for this “island”
            amp::dfs(adj, hitObstacleIdx, cluster);
            auto startObstacleIdx = hitObstacleIdx;

            // Just in case the goal is right on the obstacle's perimeter...
            if ((next - end).norm() < Bug1Algorithm::STEP_SIZE * 0.5) {
                path.waypoints.push_back(end);
                return path;
            }

            // Get the closest point on the obstacle to the robot
            next = amp::findHitPoint(current,
                obstacles[static_cast<std::vector<amp::Obstacle2D>::size_type>(this->hitObstacleIdx)],
                Bug1Algorithm::EPSILON);

            path.waypoints.push_back(next);

            // ------------------- Start perimeter-following -------------------
            auto perim_start = next;
            auto perim_best = perim_start;
            double bestDistToGoal = (perim_start - end).norm();

            // Initialize direction of robot to be perpendicular (right- or left-facing) to the path to goal
            Eigen::Vector2d toGoal = (end - perim_start).normalized();
            Eigen::Vector2d dir;

            if (this->leftTurning) {
                dir = Eigen::Vector2d(-toGoal.y(), toGoal.x());   // 90° CCW
            } else {
                dir = Eigen::Vector2d(toGoal.y(), -toGoal.x());   // 90° CW
            }

            auto perim_current = perim_start;

            // Safety variables for infinite loop, so it doesn't go forever
            int stepCount = 0;
            const int MAX_STEPS = 2000000;

            // Circumnavigate the obstacle to find the point closest to goal
            while (true) {
                //std::cerr << "Finding next perimeter step...\n";

                // Safety check on loop
                if (++stepCount > MAX_STEPS) {
                    std::cerr << "Loop 1: Perimeter following exceeded max steps.\n";
                    return path; // or break safely
                }

                auto perim_next = amp::nextPerimeterStep(perim_current, this->hitObstacleIdx, dir,
                    Bug1Algorithm::STEP_SIZE, Bug1Algorithm::DELTA_THETA, obstacles, cluster, this->leftTurning,
                    Bug1Algorithm::EPSILON);

                //std::cerr << "Perimeter step found.\n";

                // In case goal is on the perimeter of our object...
                if ((perim_next - end).norm() < Bug1Algorithm::STEP_SIZE * 1.5) {
                    path.waypoints.push_back(end);
                    return path;
                }

                auto distToGoal = (perim_next - end).norm();

                // If the next point is closer to goal than all others on the perimeter, update best point
                if (distToGoal <= bestDistToGoal) {
                    bestDistToGoal = distToGoal;
                    perim_best = perim_next;
                }

                perim_current = perim_next;

                // If we've made our way back to the beginning, then we're done circumnavigating
                if (((perim_current - perim_start).norm() < Bug1Algorithm::STEP_SIZE * 1.5 && stepCount > 1)) {
                    path.waypoints.push_back(perim_start);
                    perim_current = perim_start;
                    break;
                }

                // If not, add point to path and keep going
                path.waypoints.push_back(perim_current);
            }

            stepCount = 0;

            // Navigate the perimeter of the obstacle until the point closest to goal is found again
            while(true) {

                // Safety check on loop
                if (++stepCount > MAX_STEPS) {
                    std::cerr << "Loop 2: Perimeter following exceeded max steps.\n";
                    return path; // or break safely
                }

                // If we've made our way back to the best point, then we're done
                if (((perim_current - perim_best).norm() < Bug1Algorithm::STEP_SIZE * 1.5) && stepCount > 1) {
                    path.waypoints.push_back(perim_best);
                    break;
                }

                auto perim_next = amp::nextPerimeterStep(perim_current, this->hitObstacleIdx, dir,
                    Bug1Algorithm::STEP_SIZE, Bug1Algorithm::DELTA_THETA, obstacles, cluster, this->leftTurning,
                    Bug1Algorithm::EPSILON);

                perim_current = perim_next;
                path.waypoints.push_back(perim_current);
            }

            auto nextToGoal = amp::moveToGoal(perim_current, end, Bug1Algorithm::STEP_SIZE);

            // If move towards goal at this point collides with an obstacle, algorithm fails
            if (amp::isInCollision(nextToGoal, obstacles, Bug1Algorithm::EPSILON, this->hitObstacleIdx) &&
                (startObstacleIdx != this->hitObstacleIdx)) {
                std::cerr << "Algorithm failed.\n";
                return path;
            }

            current = perim_current;
        }

        // If there wasn't a collision...
        else {
            if ((next - end).norm() < Bug1Algorithm::STEP_SIZE * 0.5) {
                path.waypoints.push_back(next);
                path.waypoints.push_back(end);
            }

            current = next;
        }

    }

    return path;
}
