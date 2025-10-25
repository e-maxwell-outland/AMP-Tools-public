#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        std::shared_ptr<amp::Graph<double>> graph_for_plot;
        std::map<amp::Node, Eigen::VectorXd> nodes_for_plot;
        bool did_it_work;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};