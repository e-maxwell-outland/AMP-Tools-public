#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        // Constructor
        MyPRM(int num_samples = 500, double neighbor_radius = 1.0);

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        std::shared_ptr<amp::Graph<double>> graph_for_plot;
        std::map<amp::Node, Eigen::Vector2d> nodes_for_plot;
        bool did_it_work;

    private:
        int N_in;
        int r_in;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        std::shared_ptr<amp::Graph<double>> graph_for_plot;
        std::map<amp::Node, Eigen::Vector2d> nodes_for_plot;
        bool did_it_work;

    private:

};

class MyRRTWithAgentPaths {
public:
    std::shared_ptr<amp::Graph<double>> graph_for_plot;
    std::map<amp::Node, Eigen::Vector2d> nodes_for_plot;  // copies, never references
    std::vector<amp::Path2D> other_agent_paths;
    double agent_radius;

    MyRRTWithAgentPaths(const std::vector<amp::Path2D>& other_agent_paths_, double agent_radius_ = 0.2)
        : other_agent_paths(other_agent_paths_), agent_radius(agent_radius_) {}

    amp::Path2D plan(const amp::Problem2D& problem);
};
