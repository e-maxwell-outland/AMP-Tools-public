#include "Bug1Algorithm.h"
#include "MotionHelpers.h"
#include "CollisionHelpers.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    auto current = problem.q_init;
    auto end = problem.q_goal;
    auto obstacles = problem.obstacles;

    path.waypoints.push_back(current);

    while (current != end) {
        auto next = amp::moveToGoal(current, end, Bug1Algorithm::STEP_SIZE);

        if (isInCollision(next, obstacles, Bug1Algorithm::EPSILON, Bug1Algorithm::hitObstacleIdx)) {

            // Get the closest point on the obstacle to the robot
            next = amp::findHitPoint(current,
                obstacles[static_cast<std::vector<amp::Obstacle2D>::size_type>(hitObstacleIdx)],
                Bug1Algorithm::EPSILON);

            path.waypoints.push_back(next);

            // Put code for perimeter following here. For each step size in here, update the path.
            double bestDistToGoal = (next - end).norm();
            auto perim_start = next;
            auto perim_best = perim_start;

            // need to implement a "next step along the perimeter" function which returns a point and pushes to path
            // check if dist from that point to goal is less than the current bestToGoal
            // if so, replace bestDistToGoal and perim_best
            // keep going until you reach perim_start again
            // once perim_start is reached, follow the perimeter until reaching perim_best (push points to path)
            // then perim_best to path and set perim_best as next
        }

        current = next;
    }

    path.waypoints.push_back(problem.q_goal);

    return path;
}
