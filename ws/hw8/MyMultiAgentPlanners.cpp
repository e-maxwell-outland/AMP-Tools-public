# include "MyMultiAgentPlanners.h"
# include "CollisionHelpers.h"
# include "MySamplingBasedPlanners.h"
# include "MyAStar.h"
# include <random>
# include <vector>
# include <queue>
# include <tuple>

// Struct for the A* heuristic
struct EuclideanHeuristic : public amp::SearchHeuristic {
    const std::map<amp::Node, Eigen::VectorXd>& nodes;
    Eigen::VectorXd goal;

    EuclideanHeuristic(const std::map<amp::Node, Eigen::VectorXd>& n, const Eigen::VectorXd& g)
        : nodes(n), goal(g) {}

    double operator()(amp::Node node) const override {
        return (nodes.at(node) - goal).norm();
    }
};

// Centralized Coupled Planner
amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    // -------- Reset private variables --------
    graph_for_plot = std::make_shared<amp::Graph<double>>();
    nodes_for_plot.clear();
    did_it_work = false;

    // ---------- Multi-agent joint configuration setup ----------
    std::size_t m = problem.numAgents();              // number of agents
    int dim = static_cast<int>(2 * m);                // 2 DOF per agent (x,y)
    amp::MultiAgentPath2D path(m); // m = number of agents

    // Build joint start and goal vectors (size 2*m)
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd q_goal  = Eigen::VectorXd::Zero(dim);

    // Add each agent's start and goal to the state vector
    for (std::size_t i = 0; i < m; ++i) {
        // agent i's start and goal (Eigen::Vector2d)
        const Eigen::Vector2d& si = problem.agent_properties[i].q_init;
        const Eigen::Vector2d& gi = problem.agent_properties[i].q_goal;
        q_start.segment<2>(2 * static_cast<int>(i)) = si;
        q_goal.segment<2>(2 * static_cast<int>(i))  = gi;
    }

    // Start node: ID 0
    amp::Node start_id = 0;
    nodes_for_plot[start_id] = q_start;

    // -------- RRT parameters --------
    double step_size = 0.5;
    double goal_bias = 0.05;
    int max_iters = 30000;
    double epsilon = 0.25;     // distance to consider goal reached

    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    std::uniform_real_distribution<double> dist_x(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> dist_y(problem.y_min, problem.y_max);

    // -------- Main RRT loop --------
    for (int iter = 0; iter < max_iters; ++iter) {

        // --- Sample joint configuration ---
        Eigen::VectorXd q_rand(dim);
        if (bias_dist(gen) < goal_bias) {
            q_rand = q_goal;  // goal bias
        } else {
            for (int i = 0; i < m; ++i) {
                q_rand(2 * i)     = dist_x(gen);
                q_rand(2 * i + 1) = dist_y(gen);
            }
        }

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

        Eigen::VectorXd q_near = nodes_for_plot[nearest_id];

        // --- Steer toward q_rand along direction vector ---
        Eigen::VectorXd dir = q_rand - q_near;
        double dist_total = dir.norm();
        if (dist_total < 1e-6) continue; // keep your existing check
        Eigen::VectorXd dir_unit = dir / dist_total;

        // Clamp to step_size
        Eigen::VectorXd q_new = q_near + dir_unit * std::min(step_size, dist_total);

        // Collision check
        size_t hit;
        if (isMultiAgentInCollision(q_new, problem) || multiAgentEdgeCollides(q_near, q_new, problem, 0.01)) {
            continue; // cannot move in this direction
        }

        // --- Add new node ---
        amp::Node new_id = nodes_for_plot.size(); // sequential integer ID
        nodes_for_plot[new_id] = q_new;
        graph_for_plot->connect(nearest_id, new_id, (q_new - q_near).norm());
        graph_for_plot->connect(new_id, nearest_id, (q_new - q_near).norm()); // undirected

        // --- Check if goal reached ---
        if ((q_new - q_goal).norm() <= epsilon &&
            !multiAgentEdgeCollides(q_new, q_goal, problem, 0.01))
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
                amp::MultiAgentPath2D fail_path(m);
                for (int i = 0; i < m; ++i) {
                    fail_path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_init);
                    fail_path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
                }
                fail_path.valid = false;
                return fail_path;
            }

            EuclideanHeuristic heuristic(nodes_for_plot, q_goal);
            MyAStarAlgo astar;
            auto result = astar.search(spp, heuristic);

            // --- Convert to Path ---
            for (amp::Node n : result.node_path) {
                const Eigen::VectorXd& q_joint = nodes_for_plot[n];
                for (int i = 0; i < m; ++i) {
                    path.agent_paths[i].waypoints.push_back(q_joint.segment<2>(2*i));
                }
            }

            path.valid = true;
            did_it_work = result.success;

            if (!result.success) {
                amp::MultiAgentPath2D fail_path(m);
                for (int i = 0; i < m; ++i) {
                    fail_path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_init);
                    fail_path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
                }
                fail_path.valid = false;
                return fail_path;
            }

            return path;
        }
    }

    // Failed to reach goal
    return path;
}

// De-coupled Planner
amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    std::size_t m = problem.numAgents();

    double largest_radius = problem.agent_properties[0].radius;
		for (std::size_t l = 0; l < m; ++l) {
            double r = problem.agent_properties[l].radius;
    		if (r > largest_radius) {
        		largest_radius = r;
    		}
		}

    amp::MultiAgentPath2D all_paths(m);

    for (std::size_t i = 0; i < m; ++i) {

        // Build a Problem2D for this agent
        amp::Problem2D single_agent_problem;
        single_agent_problem.q_init = problem.agent_properties[i].q_init;
        single_agent_problem.q_goal = problem.agent_properties[i].q_goal;
        single_agent_problem.obstacles = problem.obstacles;
        single_agent_problem.x_min = problem.x_min;
        single_agent_problem.x_max = problem.x_max;
        single_agent_problem.y_min = problem.y_min;
        single_agent_problem.y_max = problem.y_max;

        // --- Collect already planned paths for collision checking ---
        std::vector<amp::Path2D> other_paths;
        for (std::size_t j = 0; j < i; ++j) {
            other_paths.push_back(all_paths.agent_paths[j]);
        }

        // --- Create a new RRT instance that considers other agent paths ---
        MyRRTWithAgentPaths rrt_with_agents(other_paths, largest_radius);

        // Plan path
        amp::Path2D path_i = rrt_with_agents.plan(single_agent_problem);

        // Store in the MultiAgentPath2D structure
        all_paths.agent_paths[i] = path_i;
    }

    return all_paths;
}
