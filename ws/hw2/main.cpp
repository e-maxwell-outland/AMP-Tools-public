// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "Bug1Algorithm.h"
#include "Bug2Algorithm.h"

using namespace amp;

int main(int argc, char** argv) {

    // Declare your algorithm object
    bool leftTurning = true;
    Bug2Algorithm algo(leftTurning);

    /* Different randomized environments every time you run your code */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /* Generate the problem */

    // Use W1 from Exercise 2
    Problem2D problem2a = HW2::getWorkspace1();

    // Use W2 from Exercise 2
    Problem2D problem2b = HW2::getWorkspace2();

    /* Run problem 2a */
    {
        // Call your algorithm on the problem
        amp::Path2D path = algo.plan(problem2a);

        // Check your path to make sure that it does not collide with the environment
        bool success = HW2::check(path, problem2a);

        LOG("Found valid solution to WS 1: " << (success ? "Yes!" : "No :("));
        LOG("WS 1 path length: " << path.length());

        // Visualize the path and environment
        Visualizer::makeFigure(problem2a, path);
    }

    /* Run problem 2b */
    {
        // Call your algorithm on the problem
        amp::Path2D path = algo.plan(problem2b);

        // Check your path to make sure that it does not collide with the environment
        bool success = HW2::check(path, problem2b);

        LOG("Found valid solution to WS 2: " << (success ? "Yes!" : "No :("));
        LOG("WS2 path length: " << path.length());

        // Visualize the path and environment
        Visualizer::makeFigure(problem2b, path);
    }

    Visualizer::saveFigures(true, "hw2_figs");

    /* Run numTests random tests */
    int i = 0;
    int fails = 0;
    int numTests = 100;

    while (i < numTests) {
        amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob;
        std::vector<Eigen::Vector2d> collision_points;
        HW2::generateAndCheck(algo, path, random_prob, collision_points);

        if ((!collision_points.empty()) || (path.waypoints.back() - random_prob.q_goal).norm() > 0.5) {
            fails += 1;
            Visualizer::makeFigure(random_prob, path, collision_points);
            Visualizer::saveFigures(true, "hw2_figs");
        }

        i++;
    }

    std::cout << "Fails = " << fails << "/" << numTests << "\n";

    HW2::grade(algo, "emily.maxwell@colorado.edu", argc, argv);

    return 0;
}