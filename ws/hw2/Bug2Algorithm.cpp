#include "Bug2Algorithm.h"
#include "MotionHelpers.h"
#include "CollisionHelpers.h"
#include <iostream>

Bug2Algorithm::Bug2Algorithm(bool leftTurning)
    : leftTurning(leftTurning) {}

amp::Path2D Bug2Algorithm::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    auto current = problem.q_init;
    auto end = problem.q_goal;
    auto obstacles = problem.obstacles;

    // Pre-run, build an adjacency graph to determine what obstacles intersect with each other
    auto adj = amp::buildAdjacencyList(obstacles);

    // Add start to the path
    path.waypoints.push_back(current);

    // Start of Bug 2 algorithm
    while ((current - end).norm() > Bug2Algorithm::STEP_SIZE * 0.5) {

        // ------------------- Forward moving -------------------
        auto next = amp::moveToGoal(current, end, Bug2Algorithm::STEP_SIZE);

        // Check if your next "step" forward will cause a collision. If so, find hit point on obstacle and enter
        // perimeter following mode.
        if (isInCollision(next, obstacles, Bug2Algorithm::EPSILON, this->hitObstacleIdx)) {

            // Do dfs to determine the entire "island" the robot has just hit (if more than 1 obstacle)
            std::unordered_set<size_t> cluster; // empty set for this “island”
            amp::dfs(adj, hitObstacleIdx, cluster);
            auto startObstacleIdx = hitObstacleIdx;

            // Just in case the goal is right on the obstacle's perimeter...
            if ((next - end).norm() < Bug2Algorithm::STEP_SIZE * 0.5) {
                path.waypoints.push_back(end);
                return path;
            }

            // Get the closest point on the obstacle to the robot
            next = amp::findHitPoint(current,
                obstacles[static_cast<std::vector<amp::Obstacle2D>::size_type>(this->hitObstacleIdx)],
                Bug2Algorithm::EPSILON);

            path.waypoints.push_back(next);

            // ------------------- Start perimeter-following -------------------
            auto perim_start = next;

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
            const int MAX_STEPS = 1000000;

            // Navigate the perimeter of the obstacle until the point on the m-line is found again
            while(true) {

                if (((perim_current - perim_start).norm() < Bug2Algorithm::STEP_SIZE * 0.5) && (stepCount > 1)) {
                    std::cerr << "Algorithm failed.\n";
                    return path;
                }

                // Safety check on loop
                if (++stepCount > MAX_STEPS) {
                    std::cerr << "Search for m-line Loop: Perimeter following exceeded max steps.\n";
                    return path; // or break safely
                }

                // If we've made our way back to the m-line...
                if (amp::isOnMLine(perim_current, problem.q_init, end, 0.05) &&
                    ((perim_current - end).norm() < (perim_start - end).norm())) {

                    path.waypoints.push_back(perim_current);
                    break;
                }

                auto perim_next = amp::nextPerimeterStep(perim_current, this->hitObstacleIdx, dir,
                    Bug2Algorithm::STEP_SIZE, Bug2Algorithm::DELTA_THETA, obstacles, cluster, this->leftTurning,
                    Bug2Algorithm::EPSILON);

                perim_current = perim_next;
                path.waypoints.push_back(perim_current);
            }

            current = perim_current;
        }

        // If there wasn't a collision...
        else {
            if ((next - end).norm() < Bug2Algorithm::STEP_SIZE * 0.5) {
                path.waypoints.push_back(next);
                path.waypoints.push_back(end);
            }

            current = next;
        }

    }

    return path;
}
