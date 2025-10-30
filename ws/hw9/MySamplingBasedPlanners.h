#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        std::shared_ptr<amp::Graph<double>> graph_for_plot;
        std::map<amp::Node, Eigen::Vector2d> nodes_for_plot;
        bool did_it_work;

    private:

};

class KinoRRT {
    public:
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent);


};
