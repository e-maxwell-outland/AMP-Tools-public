// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

double computePathLength(const amp::Path2D& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.waypoints.size(); ++i) {
        length += (path.waypoints[i] - path.waypoints[i-1]).norm();
    }
    return length;
}

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Define constants
    double d_star = 0.8;    // ~8% of workspace, smooth quadratic near goal
    double zeta   = 12.0;    // moderate attractive pull
    double Q_star = 0.5;    // obstacle influence ~20% of workspace
    double eta    = 0.01;   // strong enough to avoid obstacles

    // Define problems
    Problem2D problem2a = HW5::getWorkspace1();
    Problem2D problem2b_1 = HW2::getWorkspace1();
    Problem2D problem2b_2 = HW2::getWorkspace2();

    MyGDAlgorithm algo(d_star, zeta, Q_star, eta);
    Path2D path;
    Problem2D prob;
    bool success;

//    /* Problem 2a */
//    path = algo.plan(problem2a);
//
//    // Check your path to make sure that it does not collide with the environment
//    success = HW5::check(path, problem2a);
//
//    LOG("Found valid solution to HW5 WS: " << (success ? "Yes!" : "No :("));
//    LOG("HW5 WS path length: " << computePathLength(path));
//
//    // Visualize the path and environment
//    Visualizer::makeFigure(problem2a, path);
//    Visualizer::makeFigure(algo.pf, problem2a, 50);

    /* Problem 2b_1 */
    path = algo.plan(problem2b_1);

    // Check your path to make sure that it does not collide with the environment
    success = HW5::check(path, problem2b_1);

    LOG("Found valid solution to HW2 WS 1: " << (success ? "Yes!" : "No :("));
    LOG("HW2 WS 1 path length: " << computePathLength(path));

    // Visualize the path and environment
    Visualizer::makeFigure(problem2b_1, path);
    Visualizer::makeFigure(algo.pf, problem2b_1, 50);

    /* Problem 2b_2 */
    path = algo.plan(problem2b_2);

    // Check your path to make sure that it does not collide with the environment
    success = HW5::check(path, problem2b_2);

    LOG("Found valid solution to HW2 WS 2: " << (success ? "Yes!" : "No :("));
    LOG("HW2 WS 2 path length: " << computePathLength(path));

    // Visualize the path and environment
    Visualizer::makeFigure(problem2b_2, path);
    Visualizer::makeFigure(algo.pf, problem2b_2, 50);

    Visualizer::saveFigures();

//    /* Find the optimal values for zeta and eta given Qstar and dstar above */
//    std::vector<double> zeta_vals = {8.0, 10.0, 12.0, 15.0};
//    std::vector<double> eta_vals  = {0.001, 0.005, 0.01, 0.03};
//
//    struct ParamResult {
//        double zeta;
//        double eta;
//        int successes;
//    };
//
//    std::vector<ParamResult> results;
//    int numTests = 100;
//
//    for (double z : zeta_vals) {
//        for (double e : eta_vals) {
//            int fails = 0;
//
//            MyGDAlgorithm test_algo(d_star, z, Q_star, e);
//
//            for (int i = 0; i < numTests; i++) {
//                amp::Path2D path;
//                amp::Problem2D random_prob;
//                std::vector<Eigen::Vector2d> collision_points;
//
//                // Use your existing helper
//                bool pass = HW5::generateAndCheck(test_algo, path, random_prob, collision_points);
//
//                // Mark as fail if collisions or not close enough to goal
//                if ((!pass) || (path.waypoints.back() - random_prob.q_goal).norm() > 0.5) {
//                    fails++;
//                }
//            }
//
//            results.push_back({ z, e, numTests - fails });
//            std::cout << "zeta: " << z << " eta: " << e
//                      << " Successes: " << numTests - fails << "/" << numTests << "\n";
//        }
//    }
//
//    // Find best combination
//    auto best = std::max_element(results.begin(), results.end(),
//        [](const ParamResult& a, const ParamResult& b){ return a.successes < b.successes; });
//
//    std::cout << "Best parameters => zeta: " << best->zeta << ", eta: " << best->eta
//              << ", Successes: " << best->successes << "/" << numTests << "\n";
//
//    zeta = best->zeta;
//    eta = best->eta;

    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("emily.maxwell@colorado.edu", argc, argv, d_star, zeta, Q_star, eta);

    return 0;
}