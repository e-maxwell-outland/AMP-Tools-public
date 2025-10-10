#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW6.h"

#include "MyAStar.h"
#include "CSpace2DConstructor.h"
#include "MyManipulator2D.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Get problems
    Problem2D point_problem1 = HW2::getWorkspace1();
    Problem2D point_problem2 = HW2::getWorkspace2();
    Problem2D manip_problem = HW6::getHW4Problem3();
    
    // Find the correct number of cells for Exercise 1 workspaces
    double cell_width = 0.25;
    double x_min = point_problem1.x_min;
    double x_max = point_problem1.x_max;
    double x_min_2 = point_problem2.x_min;
    double x_max_2 = point_problem2.x_max;

    int n_cells_problem1 = static_cast<int>(std::ceil((x_max - x_min) / cell_width));
    int n_cells_problem2 = static_cast<int>(std::ceil((x_max_2 - x_min_2) / cell_width));
    std::size_t n_cells = 60;

    // Construct point-agent and manipulator cspace instances
    MyManipulator2D manipulator;

    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor = std::make_shared<MyPointAgentCSConstructor>(n_cells_problem1);
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor_2 = std::make_shared<MyPointAgentCSConstructor>(n_cells_problem2);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(n_cells);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
    
    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algo(wf_algo, point_agent_ctor);
    PointWaveFrontAlgorithm point_algo2(wf_algo, point_agent_ctor_2);
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

    // -------- Exercise 1 --------

    // Workspace 1 from HW 2, point agent
    Path2D path = point_algo.plan(point_problem1);
    Visualizer::makeFigure(point_problem1, path); // Visualize path in workspace
    Visualizer::makeFigure(*point_algo.getCSpace(), path); // Visualize path in cspace

    bool pass = HW2::check(path, point_problem1);

    LOG("Found valid solution to WS 1: " << (pass ? "Yes!" : "No :("));
    LOG("WS1 path length: " << path.length() << "\n");

    // Workspace 2 from HW 2, point agent
    path = point_algo2.plan(point_problem2);
    Visualizer::makeFigure(point_problem2, path);
    Visualizer::makeFigure(*point_algo2.getCSpace(), path);

    pass = HW2::check(path, point_problem2);

    LOG("Found valid solution to WS 2: " << (pass ? "Yes!" : "No :("));
    LOG("WS2 path length: " << path.length() << "\n");

    // -------- Exercise 2 --------

    // Workspace 3 from HW 4, joint manipulator
    ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory);

    // -------- Exercise 3 --------

    // A* Algorithm on the given graph
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);

    Visualizer::saveFigures();

    amp::HW6::grade<PointWaveFrontAlgorithm, ManipulatorWaveFrontAlgorithm, MyAStarAlgo>("emily.maxwell@colorado.edu", argc, argv, std::make_tuple(wf_algo, point_agent_ctor), std::make_tuple(wf_algo, manipulator_ctor), std::make_tuple());
    return 0;
}