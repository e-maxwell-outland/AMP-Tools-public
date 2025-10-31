#include "MyKinoRRT.h"
#include "CollisionHelpers.h"
#include <cmath>
#include <Eigen/Geometry>

template <typename funct>

// RK4 integrator for arbitrary dynamics
// x: current state (size n)
// u: control input (size m)
// dt: timestep
// f: dynamics function f(x, u) -> xdot
Eigen::VectorXd rk4Step(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    double dt,
    funct f) {

    Eigen::VectorXd k1 = f(x, u);
    Eigen::VectorXd k2 = f(x + (0.5 * dt * k1).eval(), u);
    Eigen::VectorXd k3 = f(x + (0.5 * dt * k2).eval(), u);
    Eigen::VectorXd k4 = f(x + (dt * k3).eval(), u);

    Eigen::VectorXd x_next = x + ((dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)).eval();
    return x_next;
}

bool isCollisionFree(const Eigen::VectorXd& x_start,
                     Eigen::VectorXd& u,
                     double dt,
                     int n_substeps,
                     amp::DynamicAgent& agent,
                     const amp::KinodynamicProblem2D& problem)
{

    auto type = problem.agent_type;
    auto obstacles = problem.obstacles;
    Eigen::VectorXd x = x_start;
    double sub_dt = dt / double(n_substeps);

    for (int i = 0; i < n_substeps; ++i) {
        agent.propagate(x, u, sub_dt);  // step forward

        // --- Check collision based on agent type ---
        if (type == amp::AgentType::SimpleCar) {
            // Reconstruct rectangle corners from rear axle
            double L = agent.agent_dim.length;
            double W = agent.agent_dim.width;
            double theta = x[2]; // state = [x_rear, y_rear, theta]

            // Local frame corners relative to rear axle
            Eigen::Vector2d rear_left(0.0,  W/2);
            Eigen::Vector2d rear_right(0.0, -W/2);
            Eigen::Vector2d front_left(L,  W/2);
            Eigen::Vector2d front_right(L, -W/2);

            std::vector<Eigen::Vector2d> corners = {rear_left, rear_right, front_left, front_right};

            // Rotate and translate to world frame
            Eigen::Rotation2D<double> R(theta);
            std::vector<Eigen::Vector2d> world_corners;
            for (const auto& c : corners)
                world_corners.push_back(R * c + x.head<2>());

            // --- Check if all corners are inside workspace bounds ---
            for (const auto& corner : world_corners) {
                if (corner[0] < problem.q_bounds[0].first || corner[0] > problem.q_bounds[0].second ||
                    corner[1] < problem.q_bounds[1].first || corner[1] > problem.q_bounds[1].second) {
                    return false; // corner outside workspace
                    }
            }

            // Build polygon
            amp::Obstacle2D car_poly(world_corners);

            // Check collision with obstacles
            for (const auto& obs : obstacles) {
                if (doPolygonsIntersect(car_poly, obs)) return false;
            }

        } else {
            // point agent check
            size_t hit;
            if (amp::isInCollision(x.head<2>(), obstacles, 0.1, hit)) {
                return false;
            }
        }
    }

    return true; // all substeps clear
}

// Kino RRT Constructor
MyKinoRRT::MyKinoRRT(int num_u, int iters) : u_samples(num_u), num_iters(iters) {
}

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {

    // Define radius of unicycle wheel
    double r = 0.25;

    // Define lambda of dynamics
    auto f = [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd xdot(x.size());

        xdot[0] = u[0] * r * cos(x[2]);
        xdot[1] = u[0] * r * sin(x[2]);
        xdot[2] = u[1];
        return xdot;
    };

    // Propagate with RK4
    state = rk4Step(state, control, dt, f);
};

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {

    // Define radius of unicycle wheel
    double r = 0.25;

    // Define lambda of dynamics
    auto f = [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd xdot(x.size());

        xdot[0] = x[3] * r * cos(x[2]);
        xdot[1] = x[3] * r * sin(x[2]);
        xdot[2] = x[4];
        xdot[3] = u[0];
        xdot[4] = u[1];
        return xdot;
    };

    // Propagate with RK4
    state = rk4Step(state, control, dt, f);
};

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {

    // Length of car
    double L = 5.0;

    // Define lambda of dynamics
    auto f = [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd xdot(x.size());

        double max_delta = M_PI/1.95; // Just short of 90 degrees max steering to prevent tan() from going crazy
        double delta = std::clamp(x[4], -max_delta, max_delta);

        xdot[0] = x[3] * cos(x[2]);
        xdot[1] = x[3] * sin(x[2]);
        xdot[2] = (x[3]/L) * tan(delta);
        xdot[3] = u[0];
        xdot[4] = u[1];

        return xdot;
    };


    // Propagate with RK4
    state = rk4Step(state, control, dt, f);
};

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    // Need to assign agent.dim here in plan method for grader to work
    agent.agent_dim = problem.agent_dim;

    amp::KinoPath path;
    path.valid = false;

    // ----- RRT storage -----
    std::map<amp::Node, Eigen::VectorXd> nodes;          // state per node
    std::map<amp::Node, amp::Node> parent;               // parent tracking
    std::map<amp::Node, Eigen::VectorXd> control_used;   // control that led to node
    std::map<amp::Node, double> duration_used;           // duration that led to node

    amp::Node start_id = 0;
    nodes[start_id] = problem.q_init;

    // ----- RRT parameters -----
    double goal_bias = 0.05;
    int max_iters = num_iters;

    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);


    // -------- Helper lambdas for sampling --------
    auto sampleState = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd s(problem.q_init.size());
        for (std::vector<std::pair<double, double>>::size_type i = 0; i < problem.q_bounds.size(); ++i) {
            std::uniform_real_distribution<double> d(problem.q_bounds[i].first,
                                                     problem.q_bounds[i].second);
            s[i] = d(gen);
        }
        return s;
    };

    auto sampleGoal = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd g(problem.q_init.size());
        for (std::vector<std::pair<double, double>>::size_type i = 0; i < problem.q_goal.size(); ++i) {
            std::uniform_real_distribution<double> d(problem.q_goal[i].first,
                                                     problem.q_goal[i].second);
            g[i] = d(gen);
        }
        return g;
    };

    auto sampleControl = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd u(problem.u_bounds.size());
        for (std::vector<std::pair<double, double>>::size_type i = 0; i < problem.u_bounds.size(); ++i) {
            std::uniform_real_distribution<double> d(problem.u_bounds[i].first, problem.u_bounds[i].second);
            u[i] = d(gen);
        }
        return u;
    };

    auto sampleDuration = [&]() -> double {
        std::uniform_real_distribution<double> dt(problem.dt_bounds.first, problem.dt_bounds.second);
        return dt(gen);
    };

    // -------- Helper lambdas for finding distance between states --------
    auto wrapAngleDiff = [](double a, double b) {
        double diff = std::fmod(a - b + M_PI, 2.0 * M_PI);
        if (diff < 0)
            diff += 2.0 * M_PI;
        return diff - M_PI;
    };

    auto stateDistance = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> double {
        double sum = 0.0;
        for (std::vector<bool>::size_type i = 0; i < a.size(); ++i) {
            double diff;
            bool isCartesian = (i < static_cast<Eigen::Index>(problem.isDimCartesian.size()))
                                   ? problem.isDimCartesian[i]
                                   : true; // default to Cartesian if unspecified
            if (isCartesian) diff = a[i] - b[i];
            else diff = wrapAngleDiff(a[i], b[i]);
            sum += diff * diff;
        }
        return std::sqrt(sum);
    };

    auto isGoal = [&](const Eigen::VectorXd& x) {
        // if (problem.agent_type == amp::AgentType::SimpleCar) {
        //     // Simple car agent (x = [x, y, theta])
        //     double L = agent.agent_dim.length;
        //     double W = agent.agent_dim.width;
        //     double theta = x[2];
        //
        //     // Corners in local frame (origin = rear axle)
        //     std::vector<Eigen::Vector2d> corners_local = {
        //         {0,  W/2},   // rear-left
        //         {0, -W/2},   // rear-right
        //         {L,  W/2},   // front-left
        //         {L, -W/2}    // front-right
        //     };
        //
        //     double c = std::cos(theta);
        //     double s = std::sin(theta);
        //
        //     for (const auto& c_local : corners_local) {
        //         // Rotate and translate to world frame
        //         Eigen::Vector2d c_world = Eigen::Vector2d(
        //             c * c_local.x() - s * c_local.y(),
        //             s * c_local.x() + c * c_local.y()
        //         ) + x.head<2>();
        //
        //         // Check if this corner is inside goal bounds
        //         if (c_world.x() < problem.q_goal[0].first || c_world.x() > problem.q_goal[0].second)
        //             return false;
        //         if (c_world.y() < problem.q_goal[1].first || c_world.y() > problem.q_goal[1].second)
        //             return false;
        //     }
        //     return true; // all corners in goal region
        // }

        // Simple point agent
        for (std::vector<std::pair<double, double>>::size_type i = 0; i < x.size(); ++i) {
            if (x[i] < problem.q_goal[i].first || x[i] > problem.q_goal[i].second)
                return false;
        }

        return true;
    };

    // -------- Helper lambdas for checking that the state and control are valid --------
    auto isStateValid = [&](const Eigen::VectorXd& x) -> bool {
        for (std::vector<std::pair<double,double>>::size_type i = 0; i < x.size(); ++i) {
            if (x[i] < problem.q_bounds[i].first || x[i] > problem.q_bounds[i].second)
                return false;
        }
        return true;
    };

    auto isControlValid = [&](const Eigen::VectorXd& u) -> bool {
        for (std::vector<std::pair<double,double>>::size_type i = 0; i < u.size(); ++i) {
            if (u[i] < problem.u_bounds[i].first || u[i] > problem.u_bounds[i].second)
                return false;
        }
        return true;
    };

    // ----- Main RRT loop -----
    for (int iter = 0; iter < max_iters; ++iter) {

        // Sample goal-biased target
        Eigen::VectorXd x_rand = (bias_dist(gen) < goal_bias)
                             ? sampleGoal()
                             : sampleState();

        // Find nearest node
        amp::Node nearest_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (auto& [id, s] : nodes) {
            double d = stateDistance(s, x_rand);
            if (d < min_dist) {
                min_dist = d;
                nearest_id = id;
            }
        }

        // Just in case...
        if (nearest_id < 0) continue;
        Eigen::VectorXd x_near = nodes[nearest_id];

        int m = u_samples; // number of candidate controls to try per expansion (tunable)

        double best_score = std::numeric_limits<double>::infinity();
        Eigen::VectorXd best_x;
        Eigen::VectorXd best_u;
        double best_dt = 0.0;
        bool found_valid = false;

        for (int k = 0; k < m; ++k) {
            Eigen::VectorXd u = sampleControl();      // your control sampler lambda
            if (!isControlValid(u)) continue;         // skip invalid control, just in case

            double dt = sampleDuration();             // your duration sampler lambda

            Eigen::VectorXd x_prop = x_near;          // start from x_near
            agent.propagate(x_prop, u, dt);           // in-place propagate

            // Check workspace collision and total state validity
            int n_substeps = 30; // tweak based on speed/dt

            if (!isCollisionFree(x_near, u, dt, n_substeps, agent, problem)) continue;
            if (!isStateValid(x_prop)) continue;  // skip if propagated state is out of bounds

            // score by distance to q_rand (use full-state-aware distance)
            double score = stateDistance(x_prop, x_rand);

            if (score < best_score) {
                best_score = score;
                best_x = x_prop;
                best_u = u;
                best_dt = dt;
                found_valid = true;
            }
        }

        // if none of the sampled controls produced a valid node, skip this iteration
        if (!found_valid) {
            continue;
        }

        // Add best node to tree
        amp::Node new_id = nodes.size();
        nodes[new_id] = best_x;
        parent[new_id] = nearest_id;
        control_used[new_id] = best_u;
        duration_used[new_id] = best_dt;

        // Goal check
        if (isGoal(best_x)) {
            // std::cout << "Found goal!\n";
            // std::cout << "Goal State:" << best_x << "\n";

            // Reconstruct path using parent pointers
            std::vector<amp::Node> node_path;
            amp::Node cur = new_id;
            while (cur != start_id) {
                node_path.push_back(cur);
                cur = parent[cur];
            }
            node_path.push_back(start_id);
            std::reverse(node_path.begin(), node_path.end());

            for (std::vector<unsigned>::size_type i = 0; i < node_path.size(); ++i) {
                path.waypoints.push_back(nodes[node_path[i]]);
                if (i > 0) {
                    path.controls.push_back(control_used[node_path[i]]);
                    path.durations.push_back(duration_used[node_path[i]]);
                }
            }
            path.valid = true;
            return path;
        }
    }

    // Failed
    std::cout << "Path finding failed!\n";
    path.valid = false;
    return path;
}
