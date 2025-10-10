#include "CSpace2DConstructor.h"
#include "MyManipulator2D.h"
#include "CollisionHelpers.h"

/* -------- Homework 4 -------- */

// -------- Helper: Determine if a link of the manipulator collides with an obstacle --------
bool linkCollides(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                  const std::vector<amp::Obstacle2D>& obstacles, double dt) {

    // Create empty/dummy values to pass in to isInCollision
    size_t hitObstacleIdx;
    double epsilon = 0.1;

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

// Finds the cell in the C-Space grid that some configuration point (x0, x1) belongs to
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

/* -------- Homework 6 -------- */

std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of the custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer.
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim,
                                                                                  env.x_min, env.x_max, env.y_min,
                                                                                  env.y_max);

    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;

    // Get obstacles in the workspace
    auto obstacles = env.obstacles;

    // Check for collision at the center of each grid
    const std::size_t N = m_cells_per_dim;
    const double delta_x = (env.x_max - env.x_min) / static_cast<double>(N);
    const double delta_y = (env.y_max - env.y_min) / static_cast<double>(N);

    for (std::size_t i = 0; i < N; ++i) {
        double x = env.x_min + (i + 0.5) * delta_x; // center of i-th row
        for (std::size_t j = 0; j < N; ++j) {
            double y = env.y_min + (j + 0.5) * delta_y; // center of j-th column

            // 1. Collision check for this configuration

            // Create empty/dummy values to pass in to isInCollision
            size_t hitObstacleIdx;
            double epsilon = 0.1;

            const Eigen::Vector2d p(x, y);
            bool collides = amp::isInCollision(p, obstacles, epsilon, hitObstacleIdx);

            // 2. Store in occupancy grid
            auto cell_index = cspace.getCellFromPoint(x, y);
            cspace(cell_index.first, cell_index.second) = collides;
        }
    }

    return cspace_ptr;
}

//amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal,
//                                               const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
//    // Implement your WaveFront algorithm here
//    amp::Path2D path;
//    path.waypoints.push_back(q_init);
//    path.waypoints.push_back(q_goal);
//
//
//    if (isManipulator) {
//        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
//        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
//        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
//    }
//    return path;
//}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal,
                                               const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    amp::Path2D path;

    // --- Step 1: Convert start/goal positions to grid indices ---
    auto start_idx = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    auto goal_idx  = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

    // Get bounds and grid information
    double x0_min = grid_cspace.x0Bounds().first;
    double x0_max = grid_cspace.x0Bounds().second;
    double x1_min = grid_cspace.x1Bounds().first;
    double x1_max = grid_cspace.x1Bounds().second;

    std::size_t N0 = grid_cspace.size().first;
    std::size_t N1 = grid_cspace.size().second;

    double cellWidthX = (x0_max - x0_min) / N0;
    double cellWidthY = (x1_max - x1_min) / N1;

    // --- Step 2: Create wave matrix and encode obstacles/goal ---
    std::vector<std::vector<int>> wave(N0, std::vector<int>(N1, 0));

    for (std::size_t i = 0; i < N0; ++i) {
        for (std::size_t j = 0; j < N1; ++j) {
            wave[i][j] = grid_cspace(i,j) ? 1 : 0; // 1 = obstacle, 0 = free
        }
    }

    // Make sure start and goal are not in obstacle
    if (wave[start_idx.first][start_idx.second] == 1 ||
        wave[goal_idx.first][goal_idx.second] == 1)
    {
        std::cout << "Start or goal is inside an obstacle!" << std::endl;
        return path;
    }

    wave[goal_idx.first][goal_idx.second] = 2; // 2 = goal

    // --- Step 3: Wave propagation using BFS ---
    std::queue<std::pair<std::size_t,std::size_t>> q;
    q.push(goal_idx);

    const int di[4] = {-1, 1, 0, 0}; // row offsets
    const int dj[4] = {0, 0, -1, 1}; // column offsets

    while (!q.empty()) {
        auto [i,j] = q.front(); q.pop();
        int current_val = wave[i][j];

        for (int k = 0; k < 4; ++k) {
            int ni = i + di[k];
            int nj = j + dj[k];

            if (isManipulator) {
                // wrap around periodic joints
                ni = (ni + N0) % N0;
                nj = (nj + N1) % N1;
            } else {
                // clamp for point robot
                if (ni < 0 || ni >= N0 || nj < 0 || nj >= N1) continue;
            }

            // propagate wave only to free/unvisited cells
            if (wave[ni][nj] == 0) {
                wave[ni][nj] = current_val + 1;
                q.push({ni, nj});
            }
        }
    }

    // --- Step 4: Path extraction from start to goal ---
    std::pair<size_t,size_t> current = start_idx;
    path.waypoints.push_back(q_init);

    while (wave[current.first][current.second] != 2) { // stop when reaching goal
        int min_val = INT_MAX;
        std::pair<size_t,size_t> next;

        for (int k = 0; k < 4; ++k) {
            int ni = current.first + di[k];
            int nj = current.second + dj[k];

            if (isManipulator) {
                ni = (ni + N0) % N0;
                nj = (nj + N1) % N1;
            } else {
                if (ni < 0 || ni >= N0 || nj < 0 || nj >= N1) continue;
            }

            int val = wave[ni][nj];

            // Skip obstacles and unreachable cells
            if (val > 1 && val < min_val) {
                min_val = val;
                next = {ni, nj};
            }
        }

        if (min_val == INT_MAX) {
            std::cout << "No path found! Cannot reach goal without hitting obstacles." << std::endl;
            return path;
        }

        // Convert grid cell back to C-space coordinates (center of cell)
        double x = 0.0;
        double y = 0.0;

        if (isManipulator) {
            // Convert grid index to angle in [0, 2π)
            x = ((next.first + 0.5) / N0) * 2*M_PI;
            y = ((next.second + 0.5) / N1) * 2*M_PI;

        } else {
            // Point robot / normal C-space
            x = x0_min + (next.first + 0.5) * cellWidthX;
            y = x1_min + (next.second + 0.5) * cellWidthY;
        }

        path.waypoints.push_back(Eigen::Vector2d(x, y));

        current = next;
    }

    path.waypoints.push_back(q_goal);

    // --- Step 6: Handle wraparound for manipulator joints ---
    if (isManipulator) {
        Eigen::Vector2d bounds0(0.0, 0.0);
        Eigen::Vector2d bounds1(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }

    return path;
}

