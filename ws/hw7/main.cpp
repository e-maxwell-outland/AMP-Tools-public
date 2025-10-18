#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono>
#include <string>

/* On this homework, I collaborated with Bennet Outland for the benchmarking code. */

using namespace amp;

int main(int argc, char** argv) {

    /* Problem 1 */
//    MyPRM prm5(200, 1.0);
//    Problem2D problem52a = HW5::getWorkspace1();
//    Problem2D problem21 = HW2::getWorkspace1();
//    Problem2D problem22 = HW2::getWorkspace2();

    // PRM on HW5, 2a
//    Visualizer::makeFigure(problem52a, prm5.plan(problem52a), *prm5.graph_for_plot, prm5.nodes_for_plot);

    // PRM on HW2, 2
//    MyPRM prm2(200, 2.0);
//    Visualizer::makeFigure(problem21, prm2.plan(problem21), *prm2.graph_for_plot, prm2.nodes_for_plot);

//    MyPRM prm22(300, 2.5);
//    Visualizer::makeFigure(problem22, prm22.plan(problem22), *prm22.graph_for_plot, prm22.nodes_for_plot);

//    /* Problem 2 */
//    MyRRT rrt;
//
//    // PRM on HW5, 2a
//    Visualizer::makeFigure(problem52a, rrt.plan(problem52a), *rrt.graph_for_plot, rrt.nodes_for_plot);
//
//    // PRM on HW2, 2
//    Visualizer::makeFigure(problem21, rrt.plan(problem21), *rrt.graph_for_plot, rrt.nodes_for_plot);
//    Visualizer::makeFigure(problem22, rrt.plan(problem22), *rrt.graph_for_plot, rrt.nodes_for_plot);
//
//    Visualizer::saveFigures();

     // // Define the (n, r) test pairs
//     std::vector<std::pair<int, double>> param_pairs = {
//         {200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0},
//         {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}
//     };

//    std::vector<std::pair<int, double>> param_pairs = {
//        {200, 1.0}, {200, 2.0}, {500, 1.0},
//        {500, 2.0}, {1000, 1.0}, {1000, 2.0}
//    };

//     std::list<std::vector<double>> time_all_runtimes;
//     std::list<std::vector<double>> valid_all_runtimes;
//     std::list<std::vector<double>> path_all_runtimes;
//     std::vector<std::string> labels;
//
//     // Loop over all (n, r) parameter combinations
//     for (const auto& [n, r] : param_pairs) {
//         std::vector<double> time_runtimes;
//         time_runtimes.reserve(100);
//
//         std::vector<double> valid_runtimes;
//         valid_runtimes.reserve(100);
//
//         std::vector<double> path_runtimes;
//         path_runtimes.reserve(100);
//
//         for (size_t i = 0; i < 100; ++i) {
//             Problem2D problem = HW2::getWorkspace2();
//             MyPRM prm(n, r);
//
//             // Timing
//             auto start = std::chrono::high_resolution_clock::now();
//             Path2D path = prm.plan(problem);
//             auto end = std::chrono::high_resolution_clock::now();
//             double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
//             time_runtimes.push_back(elapsed_ms);
//
//             // Valid
//             try
//             {
//                 valid_runtimes.push_back(static_cast<double>(prm.did_it_work));
//             }
//             catch(...)
//             {
//                 valid_runtimes.push_back(0.0);
//             }
//
//
//             // Path length
//             try
//             {
//                 path_runtimes.push_back(path.length());
//             }
//             catch(const std::exception& e)
//             {
//                 path_runtimes.push_back(0.0);
//             }
//
//
//         }
//
//         // Make the label
//         std::ostringstream label_stream;
//         label_stream << "(" << n << "," << std::fixed << std::setprecision(1) << r << ")";
//         std::string label = label_stream.str();
//         labels.push_back(label);
//
//         // Push the runtimes
//         time_all_runtimes.push_back(time_runtimes);
//         valid_all_runtimes.push_back(valid_runtimes);
//         path_all_runtimes.push_back(path_runtimes);
//     }
//
//     // Plot the box plots
//     amp::Visualizer::makeBoxPlot(
//         time_all_runtimes,
//         labels,
//         "PRM Benchmark Runtimes",
//         "(n, r) Parameters",
//         "Computation Time [ms]"
//     );
//
//     amp::Visualizer::makeBoxPlot(
//         path_all_runtimes,
//         labels,
//         "PRM Benchmark Runtimes",
//         "(n, r) Parameters",
//         "Path Length"
//     );
//
//     amp::Visualizer::makeBoxPlot(
//         valid_all_runtimes,
//         labels,
//         "PRM Benchmark Runtimes",
//         "(n, r) Parameters",
//         "Valid Solutions"
//     );

      // Problem vector
     std::vector<Problem2D> problem_vec = {HW5::getWorkspace1(), HW2::getWorkspace1(), HW2::getWorkspace2()};
     std::list<std::vector<double>> time_all_runtimes;
     std::list<std::vector<double>> valid_all_runtimes;
     std::list<std::vector<double>> path_all_runtimes;
     std::vector<std::string> labels = {"HW2W1", "HW2W2", "HW5W1"};

     // Loop over all (n, r) parameter combinations
     for (const auto& prob: problem_vec) {
         std::vector<double> time_runtimes;
         time_runtimes.reserve(100);

         std::vector<double> valid_runtimes;
         valid_runtimes.reserve(100);

         std::vector<double> path_runtimes;
         path_runtimes.reserve(100);

         for (size_t i = 0; i < 100; ++i) {
             MyRRT rrt;

             // Timing
             auto start = std::chrono::high_resolution_clock::now();
             Path2D path = rrt.plan(prob);
             auto end = std::chrono::high_resolution_clock::now();
             double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
             time_runtimes.push_back(elapsed_ms);

             // Valid
             try
             {
                 valid_runtimes.push_back(static_cast<double>(rrt.did_it_work));
             }
             catch(...)
             {
                 valid_runtimes.push_back(0.0);
             }


             // Path length
             try
             {
                 path_runtimes.push_back(path.length());
             }
             catch(const std::exception& e)
             {
                 path_runtimes.push_back(0.0);
             }


         }


         // Push the runtimes
         time_all_runtimes.push_back(time_runtimes);
         valid_all_runtimes.push_back(valid_runtimes);
         path_all_runtimes.push_back(path_runtimes);

     }


     // Plot the box plots
     amp::Visualizer::makeBoxPlot(
         time_all_runtimes,
         labels,
         "RRT Benchmark Runtimes",
         "Problems",
         "Computation Time [ms]"
     );

     amp::Visualizer::makeBoxPlot(
         path_all_runtimes,
         labels,
         "RRT Benchmark Runtimes",
         "Problems",
         "Path Length"
     );

     amp::Visualizer::makeBoxPlot(
         valid_all_runtimes,
         labels,
         "RRT Benchmark Runtimes",
         "Problems",
         "Valid Solutions"
     );

    Visualizer::saveFigures();

    // Grade method
    // HW7::grade<MyPRM, MyRRT>("emily.maxwell@colorado.edu", argc, argv, std::make_tuple(500, 2.0), std::make_tuple());
    return 0;
}