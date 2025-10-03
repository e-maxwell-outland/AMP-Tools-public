#include "MyGDAlgorithm.h"
#include "CollisionHelpers.h"

/* Helper Function: Compute the centroid of an obstacle */
Eigen::Vector2d computeCentroid(const std::vector<Eigen::Vector2d>& vertices) {
    Eigen::Vector2d centroid(0.0, 0.0);
    double signedArea = 0.0;
    int n = vertices.size();

    for (int i = 0; i < n; i++) {
        Eigen::Vector2d current = vertices[i];
        Eigen::Vector2d next = vertices[(i + 1) % n];

        double a = current.x() * next.y() - next.x() * current.y();
        signedArea += a;
        centroid += (current + next) * a;
    }

    signedArea *= 0.5;
    centroid /= (6.0 * signedArea);

    return centroid;
}

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    // Start the path
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Current position
    Eigen::Vector2d q = problem.q_init;

    // Planning parameters
    const double epsilon = 0.25;       // distance to goal threshold
    const int max_iters = 10000;      // safety limit
    auto alpha_base = 0.03; // base step
    auto alpha_min  = 0.01; // minimum step
    auto alpha_max  = 0.05; // maximum step

    // Make sure the potential function has the current problem
    pf.setProblem(problem);

    for (int iter = 0; iter < max_iters; iter++) {
        Eigen::Vector2d grad = pf.getGradient(q);

        // Get step (variable step size)
        double grad_norm = grad.norm();
        double step_len = std::min(alpha_max, std::max(alpha_min, alpha_base * grad_norm));
        Eigen::Vector2d step = step_len * grad.normalized();

        // Small perturbation if stuck
        if (grad.norm() < 1e-3) {
            step += Eigen::Vector2d::Random() * 0.02; // 0.02 = small nudge
        }

        // Update position
        Eigen::Vector2d q_next = q + step;
        path.waypoints.push_back(q_next);

        // Check termination
        if ((q_next - problem.q_goal).norm() <= epsilon) {
            break;
        }

        q = q_next;
    }

    // Ensure final waypoint is exactly the goal
    path.waypoints.push_back(problem.q_goal);

    return path;
}

// Give the pf access to the potential function
void MyPotentialFunction::setProblem(const amp::Problem2D& prob) {
    problem = prob; // copy assignment
}

// Edit based on Lecture 7 slides
double MyPotentialFunction::operator()(const Eigen::Vector2d& q) const {
    // Set initial values
    double Uatt = 0;
    double Urep = 0;
    Eigen::Vector2d q_goal = problem.q_goal;
    auto obstacles = problem.obstacles;

    // Get the distance and squared distance
    double distance = (q_goal - q).norm();
    double distance_sq = (q_goal - q).squaredNorm();

    /* Get Uatt term */
    if (distance <= d_star) {
      Uatt = 0.5 * zeta * distance_sq;
    }
    else {
        Uatt = (d_star * zeta * distance) - (0.5 * zeta * (d_star * d_star));
    }

    /* Get Urep term */
    for (size_t i = 0; i < obstacles.size(); i++) {
        auto current_obstacle = obstacles[i];

        // Find closest point on obstacle to current position
        auto c = amp::findHitPoint(q, current_obstacle, 0);

        // Find the distance to that point
        double distance_i = (c - q).norm();

        if (distance_i <= Q_star) {
          auto sq_term = (1/distance_i) - (1/Q_star);
          Urep += 0.5 * eta * (sq_term * sq_term);
        }
    }

    return Uatt + Urep;
}

// Edit based on Lecture 7 slides
Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q) const{
    // Instantiate dUatt and dUrep
    Eigen::Vector2d dUatt = Eigen::Vector2d(0, 0);
    Eigen::Vector2d dUrep = Eigen::Vector2d(0, 0);

    // Get goal position and obstacles
    Eigen::Vector2d q_goal = problem.q_goal;
    auto obstacles = problem.obstacles;

    // Find unit vector from the current point to goal
    Eigen::Vector2d direction = q_goal - q;
    Eigen::Vector2d unit_dir = direction.normalized();

    // Get the distance and squared distance
    double distance = (q_goal - q).norm();

    /* Get dUatt term */
    if (distance <= d_star) {
        dUatt = zeta * unit_dir;
    }
    else {
        dUatt = (d_star * zeta * unit_dir) / distance;
    }

    /* Get dUrep term */
    for (size_t i = 0; i < obstacles.size(); i++) {
        auto current_obstacle = obstacles[i];
        const auto verts = current_obstacle.verticesCCW();

        // Find closest point on obstacle to current position
        auto c = amp::findHitPoint(q, current_obstacle, 0);

        // Find the distance to that point and its gradient
        double distance_i = (c - q).norm();

        if (distance_i <= Q_star) {
            auto centroid = computeCentroid(verts);

            const double eps = 1e-6;  // to avoid division by zero if to close to obstacle
            Eigen::Vector2d local_dir = (q - c).normalized();
            Eigen::Vector2d global_dir = (q - centroid).normalized();
            Eigen::Vector2d grad_dir = 0.3 * local_dir + 0.7 * global_dir;
            grad_dir.normalize();

            dUrep += eta * ( (1/distance_i) - (1/Q_star) ) * ( grad_dir/(distance_i * distance_i) );
        }
    }

    return dUatt + dUrep;
}
