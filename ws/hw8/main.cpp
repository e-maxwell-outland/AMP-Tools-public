// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

//void timer_example() {
//    double startTime;
//    amp::Timer timer("timer");
//    for (int i=0; i < 5; ++i) {
//        startTime = timer.now(TimeUnit::ms);
//        std::cout << "Press any key to continue...\n";
//        std::cin.get();
//        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
//    }
//    timer.stop();
//    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
//}

int main(int argc, char** argv) {
    // Initializing workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentPath2D path;
    MultiAgentProblem2D problem = HW8::getWorkspace1(6);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

//    // Solve using a centralized approach
//    MyCentralPlanner central_planner;
//    path = central_planner.plan(problem);
//    bool isValid = HW8::check(path, problem, collision_states);
//    Visualizer::makeFigure(problem, path, collision_states);

//    // Solve using a decentralized approach
//    MyDecentralPlanner decentral_planner;
//    path = decentral_planner.plan(problem);
//    bool isValid = HW8::check(path, problem, collision_states);
//    Visualizer::makeFigure(problem, path, collision_states);

//     // Coupled Planner testbench
//     std::list<std::vector<double>> time_all_runtimes;
//     std::list<std::vector<double>> tree_size_all;
//     std::vector<std::string> labels;
//     MyCentralPlanner central_planner;
//
//    std::vector<double> time_avgs;
//    std::vector<double> tree_avgs;
//    std::vector<double> success_rates;
//
//     // Loop over all (n, r) parameter combinations
//     for (int i = 2; i < 7; i++) {
//         problem = HW8::getWorkspace1(i);
//
//         std::vector<double> time_runtimes;
//         time_runtimes.reserve(100);
//
//         std::vector<double> tree_runtimes;
//         tree_runtimes.reserve(100);
//
//         double time_sum = 0.0;
//         double tree_sum = 0.0;
//         int success_sum = 0;
//
//         for (size_t j = 0; j < 100; ++j) {
//
//             // Timing
//             auto start = std::chrono::high_resolution_clock::now();
//             path = central_planner.plan(problem);
//             auto end = std::chrono::high_resolution_clock::now();
//             double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
//             time_runtimes.push_back(elapsed_ms);
//             time_sum += elapsed_ms;
//
//             // Tree size
//             try
//             {
//                 tree_runtimes.push_back(central_planner.nodes_for_plot.size());
//                 tree_sum += central_planner.nodes_for_plot.size();
//             }
//             catch(const std::exception& e)
//             {
//                 tree_runtimes.push_back(0.0);
//             }
//
//             if(central_planner.did_it_work) {success_sum++;}
//
//         }
//
//         // Make the label
//         std::ostringstream label_stream;
//         label_stream << i << " agents";
//         std::string label = label_stream.str();
//         labels.push_back(label);
//
//         // Push the runtimes
//         time_all_runtimes.push_back(time_runtimes);
//         tree_size_all.push_back(tree_runtimes);
//
//         time_avgs.push_back(time_sum / 100.0);
//         tree_avgs.push_back(tree_sum / 100.0);
//         success_rates.push_back(double(success_sum) / 100.0);
//     }
//
//    // Print average compute times
//    std::cout << "Compute time averages: ";
//    for (double tm_avg : time_avgs) {
//        std::cout << tm_avg << " ";
//    }
//    std::cout << std::endl;
//
//    // Print average tree sizes
//    std::cout << "Tree size averages: ";
//    for (double tr_avg : tree_avgs) {
//        std::cout << tr_avg << " ";
//    }
//    std::cout << std::endl;
//
//    // Print success rates
//    std::cout << "Success Rates: ";
//    for (double s : success_rates) {
//        std::cout << s << " ";
//    }
//    std::cout << std::endl;
//
//     // Plot the box plots
//     amp::Visualizer::makeBoxPlot(
//         time_all_runtimes,
//         labels,
//         "Coupled Planner Computation Time Benchmark",
//         "Number of agents",
//         "Computation Time [ms]"
//     );
//
//     amp::Visualizer::makeBoxPlot(
//         tree_size_all,
//         labels,
//         "Coupled Planner Tree Size Benchmark",
//         "Number of agents",
//         "Tree size"
//     );

//    // De-coupled Planner testbench
//     std::list<std::vector<double>> time_all_runtimes;
//     std::vector<std::string> labels;
//     MyDecentralPlanner decentral_planner;
//
//     std::vector<double> time_avgs;
//
//     // Loop over all (n, r) parameter combinations
//     for (int i = 2; i < 7; i++) {
//         problem = HW8::getWorkspace1(i);
//
//         std::vector<double> time_runtimes;
//         time_runtimes.reserve(100);
//
//         std::vector<double> tree_runtimes;
//         tree_runtimes.reserve(100);
//
//         double time_sum = 0.0;
//         double tree_sum = 0.0;
//         int success_sum = 0;
//
//         for (size_t j = 0; j < 100; ++j) {
//
//             // Timing
//             auto start = std::chrono::high_resolution_clock::now();
//             path = decentral_planner.plan(problem);
//             auto end = std::chrono::high_resolution_clock::now();
//             double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
//             time_runtimes.push_back(elapsed_ms);
//             time_sum += elapsed_ms;
//
//         }
//
//         // Make the label
//         std::ostringstream label_stream;
//         label_stream << i << " agents";
//         std::string label = label_stream.str();
//         labels.push_back(label);
//
//         // Push the runtimes
//         time_all_runtimes.push_back(time_runtimes);
//
//         time_avgs.push_back(time_sum / 100.0);
//     }
//
//    // Print average compute times
//    std::cout << "Compute time averages: ";
//    for (double tm_avg : time_avgs) {
//        std::cout << tm_avg << " ";
//    }
//    std::cout << std::endl;
//
//     // Plot the box plots
//     amp::Visualizer::makeBoxPlot(
//         time_all_runtimes,
//         labels,
//         "Decoupled Planner Computation Time Benchmark",
//         "Number of agents",
//         "Computation Time [ms]"
//     );
//
//    // Visualize and grade methods
//    Visualizer::saveFigures();
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("emily.maxwell@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}