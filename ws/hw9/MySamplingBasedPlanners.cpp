# include "MySamplingBasedPlanners.h"
# include "CollisionHelpers.h"
# include "MyAStar.h"
# include <random>
# include <vector>
#include <queue>
#include <tuple>

// Struct for the A* heuristic
struct EuclideanHeuristic : public amp::SearchHeuristic {
    const std::map<amp::Node, Eigen::Vector2d>& nodes;
    Eigen::Vector2d goal;

    EuclideanHeuristic(const std::map<amp::Node, Eigen::Vector2d>& n, const Eigen::Vector2d& g)
        : nodes(n), goal(g) {}

    double operator()(amp::Node node) const override {
        return (nodes.at(node) - goal).norm();
    }
};

// RRT algorithm
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {

    // -------- Graph & node storage --------
    graph_for_plot = std::make_shared<amp::Graph<double>>();
    nodes_for_plot.clear();

    Eigen::Vector2d q_start = problem.q_init;
    Eigen::Vector2d q_goal  = problem.q_goal;

    // Start node: ID 0
    amp::Node start_id = 0;
    nodes_for_plot[start_id] = q_start;

    // -------- RRT parameters --------
    double step_size = 0.5;
    double goal_bias = 0.05;
    int max_iters = 5000;
    double epsilon = 0.25;     // distance to consider goal reached

    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    std::uniform_real_distribution<double> dist_x(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> dist_y(problem.y_min, problem.y_max);

    // -------- Main RRT loop --------
    for (int iter = 0; iter < max_iters; ++iter) {

        // --- Sample ---
        Eigen::Vector2d q_rand = (bias_dist(gen) < goal_bias) ? q_goal : Eigen::Vector2d(dist_x(gen), dist_y(gen));

        // --- Nearest neighbor (linear search) ---
        amp::Node nearest_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& [id, pt] : nodes_for_plot) {
            double d = (q_rand - pt).norm();
            if (d < min_dist) {
                min_dist = d;
                nearest_id = id;
            }
        }
        Eigen::Vector2d q_near = nodes_for_plot[nearest_id];

        // --- Steer toward q_rand along direction vector ---
        Eigen::Vector2d dir = q_rand - q_near;
        double dist_total = dir.norm();
        if (dist_total < 1e-6) continue; // sample too close
        Eigen::Vector2d dir_unit = dir / dist_total;

        // Clamp to step_size
        Eigen::Vector2d q_new = q_near + dir_unit * std::min(step_size, dist_total);

        // Collision check
        size_t hit;
        if (amp::isInCollision(q_new, problem.obstacles, 0.0, hit) ||
            edgeCollides(q_near, q_new, problem.obstacles, 0.005))
        {
            continue; // cannot move in this direction
        }

        // --- Add new node ---
        amp::Node new_id = nodes_for_plot.size(); // sequential integer ID
        nodes_for_plot[new_id] = q_new;
        graph_for_plot->connect(nearest_id, new_id, (q_new - q_near).norm());
        graph_for_plot->connect(new_id, nearest_id, (q_new - q_near).norm()); // undirected

        // --- Check if goal reached ---
        if ((q_new - q_goal).norm() <= epsilon &&
            !edgeCollides(q_new, q_goal, problem.obstacles, 0.05))
        {
            amp::Node goal_id = nodes_for_plot.size();
            nodes_for_plot[goal_id] = q_goal;
            graph_for_plot->connect(new_id, goal_id, (q_goal - q_new).norm());
            graph_for_plot->connect(goal_id, new_id, (q_goal - q_new).norm());

            // --- Use A* to find path in the tree ---
            amp::ShortestPathProblem spp;
            spp.graph = graph_for_plot;
            spp.init_node = start_id;
            spp.goal_node = goal_id;

            auto graph_nodes = graph_for_plot->nodes();
            bool start_exists = std::find(graph_nodes.begin(), graph_nodes.end(), start_id) != graph_nodes.end();
            bool goal_exists = std::find(graph_nodes.begin(), graph_nodes.end(), goal_id) != graph_nodes.end();

            if (!start_exists || !goal_exists) {
                amp::Path2D path;
                path.waypoints.push_back(problem.q_init);
                path.waypoints.push_back(problem.q_goal);
                return path;
            }

            EuclideanHeuristic heuristic(nodes_for_plot, q_goal);
            MyAStarAlgo astar;
            auto result = astar.search(spp, heuristic);

            // --- Convert to Path2D ---
            amp::Path2D path;
            for (amp::Node n : result.node_path)
                path.waypoints.push_back(nodes_for_plot[n]);

            did_it_work = result.success;

            if (!result.success) {
                amp::Path2D fail_path;
                fail_path.waypoints.push_back(problem.q_init);
                fail_path.waypoints.push_back(problem.q_goal);
                return fail_path;
            }

            return path;
        }
    }

    // Failed to reach goal
    return {};
}
