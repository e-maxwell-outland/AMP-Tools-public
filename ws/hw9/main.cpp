// This includes all the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>

using namespace amp;

void saveControlsToCSV(const std::string& filename, const KinoPath& path) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return;
    }

    // Write header
    file << "index,";
    int control_dim = path.controls.empty() ? 0 : path.controls[0].size();
    for (int i = 0; i < control_dim; ++i)
        file << "u" << i << ",";
    file << "dt\n";

    // Write data
    for (size_t i = 0; i < path.controls.size(); ++i) {
        file << i << ",";
        for (int j = 0; j < control_dim; ++j)
            file << path.controls[i][j] << ",";
        file << path.durations[i] << "\n";
    }

    file.close();
    std::cout << "Saved controls to " << filename << "\n";
}

double computeTrajectoryLength(const KinoPath& path) {
    if (path.waypoints.size() < 2) return 0.0;

    double length = 0.0;
    for (size_t i = 1; i < path.waypoints.size(); ++i) {
        Eigen::Vector2d p1 = path.waypoints[i-1].head<2>();
        Eigen::Vector2d p2 = path.waypoints[i].head<2>();
        length += (p2 - p1).norm();
    }
    return length;
}

double computeTotalDuration(const KinoPath& path) {
    double total = 0.0;
    for (double d : path.durations) total += d;
    return total;
}

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, [](){return std::make_shared<MySingleIntegrator>();} },
    {AgentType::FirstOrderUnicycle, [](){return std::make_shared<MyFirstOrderUnicycle>();} },
    {AgentType::SecondOrderUnicycle, [](){return std::make_shared<MySecondOrderUnicycle>();} },
    {AgentType::SimpleCar, [](){return std::make_shared<MySimpleCar>();} }
};

int main(int argc, char** argv) {
    // Exercise 1

    // Define the (num_u, num_iters) test pairs
    std::vector<std::pair<int, double>> param_pairs = {
        {1, 50000}, {5, 50000}, {10, 50000}, {15, 50000}
    };

    // for (int i = 0; i < 5; i += 2) {
    //
    //     // Parts (a) - (c)
    //     int select = i;
    //     KinodynamicProblem2D prob = problems[select];
    //     MyKinoRRT kino_planner(15, 50000);
    //
    //     auto start = std::chrono::high_resolution_clock::now();
    //     KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    //     auto end = std::chrono::high_resolution_clock::now();
    //     double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    //
    //     std::cout << "Trajectory length: " << computeTrajectoryLength(path) << "\n";
    //     std::cout << "Total path duration: " << computeTotalDuration(path) << " s\n";
    //     std::cout << "Computation time: " << elapsed_ms << " ms\n";
    //
    //     HW9::check(path, prob);
    //     if (path.valid)
    //         Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
    //
    //     std::cout << "Beginning Benchmark Testing for Problem" << i << "\n";
    //     std::list<std::vector<double>> time_all_runtimes;
    //     std::list<std::vector<double>> valid_all_runtimes;
    //     std::list<std::vector<double>> path_all_runtimes;
    //     std::vector<std::string> labels;
    //
    //     // Part (d) Benchmarking
    //     for (const auto& [u, n] : param_pairs) {
    //         std::cout << "Benchmarking number of samples, u = " << u << "\n";
    //
    //         std::vector<double> time_runtimes;
    //         time_runtimes.reserve(100);
    //
    //         std::vector<double> valid_runtimes;
    //         valid_runtimes.reserve(100);
    //
    //         std::vector<double> path_runtimes;
    //         path_runtimes.reserve(100);
    //
    //         for (size_t j = 0; j < 50; ++j) {
    //             std::cout << "Benchmark " << j + 1 << " of 50\n";
    //             Problem2D problem = HW2::getWorkspace2();
    //             MyKinoRRT kino_bench(u, n);
    //
    //             // Timing
    //             auto start = std::chrono::high_resolution_clock::now();
    //             KinoPath bench_path = kino_bench.plan(prob, *agentFactory[prob.agent_type]());
    //             auto end = std::chrono::high_resolution_clock::now();
    //             double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    //             time_runtimes.push_back(elapsed_ms);
    //
    //             auto path_length = computeTrajectoryLength(bench_path);
    //
    //             // Valid
    //             try
    //             {
    //                 double valid;
    //                 if (path_length < 1) valid = 0.0;
    //                 else valid = 1.0;
    //                 std::cout << "Valid? " << valid << "\n";
    //
    //                 valid_runtimes.push_back(static_cast<double>(valid));
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
    //                 path_runtimes.push_back(path_length);
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
    //         label_stream << "(" << u << "," << std::fixed << std::setprecision(1) << n << ")";
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
    //         "Benchmark Computation Runtimes",
    //         "Parameters",
    //         "Computation Time [ms]"
    //     );
    //
    //     amp::Visualizer::makeBoxPlot(
    //         path_all_runtimes,
    //         labels,
    //         "Benchmark Path Lengths",
    //         "Parameters",
    //         "Path Length"
    //     );
    //
    //     amp::Visualizer::makeBoxPlot(
    //         valid_all_runtimes,
    //         labels,
    //         "Benchmark Validity",
    //         "Parameters",
    //         "Valid Solutions"
    //     );
    // }
    //
    // Visualizer::saveFigures();

    // Exercise 2
    int select = 7;
    KinodynamicProblem2D prob = problems[select];
    MyKinoRRT kino_planner(15, 50000);

    auto start = std::chrono::high_resolution_clock::now();
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    auto end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

    std::cout << "Trajectory length: " << computeTrajectoryLength(path) << "\n";
    std::cout << "Total path duration: " << computeTotalDuration(path) << " s\n";
    std::cout << "Computation time: " << elapsed_ms << " ms\n";

    HW9::check(path, prob);
    if (path.valid)
        Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation

    // saveControlsToCSV("/Users/emaxwell/Desktop/parallel_park_control.csv", path);

    std::cout << "Beginning Benchmark Testing for Problem" << select << "\n";
    std::list<std::vector<double>> time_all_runtimes;
    std::list<std::vector<double>> valid_all_runtimes;
    std::list<std::vector<double>> path_all_runtimes;
    std::vector<std::string> labels;

    int num_tests = 20;

    // Part (d) Benchmarking
    for (const auto& [u, n] : param_pairs) {
        std::cout << "Benchmarking number of samples, u = " << u << "\n";

        std::vector<double> time_runtimes;
        time_runtimes.reserve(100);

        std::vector<double> valid_runtimes;
        valid_runtimes.reserve(100);

        std::vector<double> path_runtimes;
        path_runtimes.reserve(100);

        for (size_t j = 0; j < num_tests; ++j) {
            std::cout << "Benchmark " << j + 1 << " of " << num_tests << "\n";
            Problem2D problem = HW2::getWorkspace2();
            MyKinoRRT kino_bench(u, n);

            // Timing
            auto start = std::chrono::high_resolution_clock::now();
            KinoPath bench_path = kino_bench.plan(prob, *agentFactory[prob.agent_type]());
            auto end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
            time_runtimes.push_back(elapsed_ms);

            auto path_length = computeTrajectoryLength(bench_path);

            // Valid
            try
            {
                double valid;
                if (path_length < 1) valid = 0.0;
                else valid = 1.0;
                std::cout << "Valid? " << valid << "\n";

                valid_runtimes.push_back(static_cast<double>(valid));
            }
            catch(...)
            {
                valid_runtimes.push_back(0.0);
            }


            // Path length
            try
            {
                path_runtimes.push_back(path_length);
            }
            catch(const std::exception& e)
            {
                path_runtimes.push_back(0.0);
            }


        }

        // Make the label
        std::ostringstream label_stream;
        label_stream << "(" << u << "," << std::fixed << std::setprecision(1) << n << ")";
        std::string label = label_stream.str();
        labels.push_back(label);

        // Push the runtimes
        time_all_runtimes.push_back(time_runtimes);
        valid_all_runtimes.push_back(valid_runtimes);
        path_all_runtimes.push_back(path_runtimes);
    }

    // Plot the box plots
    amp::Visualizer::makeBoxPlot(
        time_all_runtimes,
        labels,
        "Benchmark Computation Runtimes",
        "Parameters",
        "Computation Time [ms]"
    );

    amp::Visualizer::makeBoxPlot(
        path_all_runtimes,
        labels,
        "Benchmark Path Lengths",
        "Parameters",
        "Path Length"
    );

    amp::Visualizer::makeBoxPlot(
        valid_all_runtimes,
        labels,
        "Benchmark Validity",
        "Parameters",
        "Valid Solutions"
    );

    Visualizer::saveFigures();

    // HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("emily.maxwell@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}