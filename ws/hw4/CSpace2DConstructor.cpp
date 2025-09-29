#include "CSpace2DConstructor.h"
#include "MyManipulator2D.h"
#include "CollisionHelpers.h"

// -------- Helper: Determine if a link of the manipulator collides with an obstacle --------
bool linkCollides(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                  const std::vector<amp::Obstacle2D>& obstacles, double dt) {

    // Create empty/dummy values to pass in to isInCollision
    size_t hitObstacleIdx;
    double epsilon = 0.0;

    // Check all points (discretized) on the link
    for (double t = 0; t <= 1; t += dt) {
        Eigen::Vector2d p = (1-t)*start + t*end;
        if (amp::isInCollision(p, obstacles, epsilon, hitObstacleIdx)) return true;
    }

    return false;
}

// -------- Helper: Determine if manipulator collides with any obstacles for a given configuration --------
bool manipulatorCollides(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                         const std::vector<amp::Obstacle2D>& obstacles, double dt) {

    // For the first and second links, check if there is collision
    if (linkCollides(p0, p1, obstacles, dt)) return true;
    if (linkCollides(p1, p2, obstacles, dt)) return true;

    return false;
}

// Finds the cell in the C-Space grid does some configuration point (x0, x1) belong to
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Get all relevant values from private class variables
    auto x1_bounds = x1Bounds();
    auto x1_min = x1_bounds.first;
    auto x1_max = x1_bounds.second;

    auto x0_bounds = x0Bounds();
    auto x0_min = x0_bounds.first;
    auto x0_max = x0_bounds.second;

    auto num_cells = size();
    auto x0_cells = num_cells.first;
    auto x1_cells = num_cells.second;

    // Step 1: Compute cell widths using class variables
    double cellWidth0 = (x0_max - x0_min) / x0_cells;
    double cellWidth1 = (x1_max - x1_min) / x1_cells;

    // Step 2: Wrap coordinates into [min, max)
    x0 = std::fmod(x0 - x0_min, x0_max - x0_min);
    if (x0 < 0) x0 += (x0_max - x0_min);

    x1 = std::fmod(x1 - x1_min, x1_max - x1_min);
    if (x1 < 0) x1 += (x1_max - x1_min);

    // Step 3: Compute integer indices
    std::size_t i0 = static_cast<std::size_t>(std::floor(x0 / cellWidth0));
    std::size_t i1 = static_cast<std::size_t>(std::floor(x1 / cellWidth1));

    // Step 4: Clamp to [0, numCells-1] as safety
    if (i0 >= x0_cells) i0 = x0_cells - 1;
    if (i1 >= x1_cells) i1 = x1_cells - 1;

    return {i0, i1};
}

// Computes the boolean collision values for each configuration cell in C-Space (wrt a given manipulator and workspace)
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator,
                                                                         const amp::Environment2D& env) {

    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);

    // Regular GridCSpace2D object (de-reference)
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Get obstacles in the workspace
    auto obstacles = env.obstacles;

    // Manipulator start position fixed
    Eigen::Vector2d p0(0, 0);  // fixed at origin

    // Set the discretization step for checking collision on the links
    auto stepSize = 0.01;

    // Check for collision at the center of each grid
    const std::size_t N = m_cells_per_dim;
    const double delta = (2.0 * M_PI) / static_cast<double>(N);

    for (std::size_t i = 0; i < N; ++i) {
        double theta1 = (i + 0.5) * delta;
        for (std::size_t j = 0; j < N; ++j) {
            double theta2 = (j + 0.5) * delta;

            amp::ManipulatorState current_state = Eigen::Vector2d(theta1, theta2);

            // 1. Forward kinematics to get joint positions
            Eigen::Vector2d p1 = manipulator.getJointLocation(current_state, 1);
            Eigen::Vector2d p2 = manipulator.getJointLocation(current_state, 2);

            // 2. Collision check for this configuration
            bool collides = manipulatorCollides(p0, p1, p2, obstacles, stepSize);

            // 3. Store in occupancy grid
            auto cell_index = cspace.getCellFromPoint(current_state[0], current_state[1]);
            cspace(cell_index.first, cell_index.second) = collides;
        }
    }

    return cspace_ptr;
}
