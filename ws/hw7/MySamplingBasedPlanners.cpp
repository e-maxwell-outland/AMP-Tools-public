# include "MySamplingBasedPlanners.h"
# include "CollisionHelpers.h"
# include "MyAStar.h"
# include <random>
# include <vector>
#include <queue>
#include <tuple>

// PRM Constructor
MyPRM::MyPRM(int num_samples, double neighbor_radius)
    : N_in(num_samples), r_in(neighbor_radius), did_it_work(false) {
}

// Define struct for nodes of KD tree
struct KDNode {
    Eigen::Vector2d point;  // The point in workspace
    amp::Node index;         // Global PRM node index
    KDNode* left = nullptr;  // Left subtree
    KDNode* right = nullptr; // Right subtree

    KDNode(const Eigen::Vector2d& pt, amp::Node idx)
        : point(pt), index(idx) {}
};

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

// -------- Definitions for knnSearch --------
using DistIndexPair = std::pair<double, amp::Node>;

// max-heap comparator: largest distance on top
struct MaxHeapCmp {
    bool operator()(const DistIndexPair& a, const DistIndexPair& b) const {
        return a.first < b.first; // max-heap
    }
};

// Define heap type
using HeapType = std::priority_queue<DistIndexPair, std::vector<DistIndexPair>, MaxHeapCmp>;

/* -------- Helper: Build KD Tree to determine nearest neighbors -------- */
KDNode* buildKDTree(std::vector<Eigen::Vector2d>& points, std::vector<amp::Node>& indices, int depth = 0) {
    if (points.empty()) return nullptr;

    int k = points[0].size();  // number of dimensions (2 for 2D)
    int axis = depth % k;  // choose splitting dimension

    // Find index of median
    size_t medianIdx = points.size() / 2;

    // Partially sort: elements to the left of the median are less than the median, and elements to the right are
    // greater. Partition around median, but each side is not sorted
    std::nth_element(points.begin(), points.begin() + medianIdx, points.end(),
        [axis](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            return a(axis) < b(axis);
        });

    // Create KDNode storing the median point and its global index
    KDNode* node = new KDNode(points[medianIdx], indices[medianIdx]);

    // Split points and indices into left and right subsets for recursion
    std::vector<Eigen::Vector2d> leftPoints(points.begin(), points.begin() + medianIdx);
    std::vector<Eigen::Vector2d> rightPoints(points.begin() + medianIdx + 1, points.end());
    std::vector<amp::Node> leftIndices(indices.begin(), indices.begin() + medianIdx);
    std::vector<amp::Node> rightIndices(indices.begin() + medianIdx + 1, indices.end());

    // Recursively build subtrees
    node->left  = buildKDTree(leftPoints, leftIndices, depth + 1);
    node->right = buildKDTree(rightPoints, rightIndices, depth + 1);

    return node;
}

/* -------- Recursive Helper: For a node, find its k nearest neighbors (Euclidean distance) -------- */
void knnSearch(KDNode* node, const Eigen::Vector2d& query, int k, int depth, HeapType& maxHeap) {
    if (!node) return;

    int axis = depth % query.size();
    double dist = (node->point - query).norm();

    if (maxHeap.size() < k) {
        maxHeap.push({dist, node->index});

    } else if (dist < maxHeap.top().first) {
        maxHeap.pop();
        maxHeap.push({dist, node->index});
    }

    KDNode* nearChild = (query(axis) < node->point(axis)) ? node->left : node->right;
    KDNode* farChild  = (query(axis) < node->point(axis)) ? node->right : node->left;

    // Search nearer side first
    knnSearch(nearChild, query, k, depth + 1, maxHeap);

    // Check if we need to explore the farther side
    if (maxHeap.size() < k || std::abs(query(axis) - node->point(axis)) < maxHeap.top().first) {
        knnSearch(farChild, query, k, depth + 1, maxHeap);
    }
}

/* -------- Helper: For a query, find its k nearest neighbors -------- */
std::vector<amp::Node> findKNearestNeighbors(KDNode* root, const Eigen::Vector2d& query, int k) {
    HeapType maxHeap;
    knnSearch(root, query, k, 0, maxHeap);

    std::vector<amp::Node> neighbors;
    while (!maxHeap.empty()) {
        neighbors.push_back(maxHeap.top().second);
        maxHeap.pop();
    }

    std::reverse(neighbors.begin(), neighbors.end()); // closest first
    return neighbors;
}

///* -------- Recursive helper: radius search on KD-tree -------- */
//void radiusSearch(KDNode* node,
//                  const Eigen::Vector2d& query,
//                  double radius,
//                  int depth,
//                  HeapType& maxHeap)
//{
//    if (!node) return;
//
//    // Compute squared distance to avoid sqrt
//    double distSquared = (node->point - query).squaredNorm();
//    double radiusSquared = radius * radius;
//
//    if (distSquared <= radiusSquared) {
//        maxHeap.push({std::sqrt(distSquared), node->index});  // store actual distance in heap
//    }
//
//    // Determine which child to search first based on splitting axis
//    int axis = depth % query.size();
//    double axisDiff = query(axis) - node->point(axis);
//
//    KDNode* nearChild = (axisDiff < 0) ? node->left : node->right;
//    KDNode* farChild  = (axisDiff < 0) ? node->right : node->left;
//
//    // Always search near child
//    radiusSearch(nearChild, query, radius, depth + 1, maxHeap);
//
//    // Only search far child if splitting plane intersects the radius sphere
//    if (axisDiff * axisDiff <= radiusSquared) {
//        radiusSearch(farChild, query, radius, depth + 1, maxHeap);
//    }
//}
//
///* -------- Helper: get all neighbors within radius -------- */
//std::vector<amp::Node> findRadiusNeighbors(KDNode* root, const Eigen::Vector2d& query, double radius,
//                                           int maxNeighbors = -1) {
//
//    // Create the heap and run the search
//    HeapType heap;
//    radiusSearch(root, query, radius, 0, heap);
//
//    std::vector<amp::Node> neighbors;
//    neighbors.reserve(heap.size());
//
//    // Pop all neighbors from the heap
//    while (!heap.empty()) {
//        neighbors.push_back(heap.top().second);
//        heap.pop();
//    }
//
//    // Heap gives farthest first, so reverse for closest first
//    std::reverse(neighbors.begin(), neighbors.end());
//
//    // If a max cap is set, trim the list
//    if (maxNeighbors > 0 && (int)neighbors.size() > maxNeighbors) {
//        neighbors.resize(maxNeighbors);
//    }
//
//    return neighbors;
//}

/* -------- Helper: Not using KDtree, finding neighbors by radius -------- */
std::vector<amp::Node> findRadiusNeighborsNaive(const std::vector<Eigen::Vector2d>& nodes, const Eigen::Vector2d& query,
                                                double radius) {
    std::vector<amp::Node> neighbors;
    double r2 = radius * radius;  // use squared distance for efficiency

    for (amp::Node i = 0; i < nodes.size(); ++i) {
        double dist2 = (nodes[i] - query).squaredNorm();
        if (dist2 <= r2) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

//
void greedySmoothPath(amp::Path2D &path, const std::vector<amp::Obstacle2D>& obstacles, double step = 0.01,
                      double epsilon = 0.1) {

    if (path.waypoints.size() < 3)
        return;

    size_t i = 0;
    while (i < path.waypoints.size() - 2) {
        size_t j = path.waypoints.size() - 1; // try connecting directly to the end
        while (j > i + 1) {
            if (!edgeCollides(path.waypoints[i], path.waypoints[j], obstacles, step, epsilon)) {
                // remove all points between i and j
                path.waypoints.erase(path.waypoints.begin() + i + 1,
                                     path.waypoints.begin() + j);
                break; // go to next i
            }
            --j;
        }
        ++i;
    }
}

/* -------- PRM algorithm (naive radius search, no KD-tree) -------- */
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    nodes_for_plot.clear();

    double xmin = problem.x_min;
    double xmax = problem.x_max;
    double ymin = problem.y_min;
    double ymax = problem.y_max;

    // Bounds for HW5, WS1
//    double xmin = -1.0;
//    double xmax = 11.0;
//    double ymin = -3.0;
//    double ymax = 3.0;

    // Bounds for HW2, WS2
//    double xmin = -5.0;
//    double xmax = 35.0;
//    double ymin = -5.0;
//    double ymax = 5.0;

      auto obstacles = problem.obstacles;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(xmin, xmax);
    std::uniform_real_distribution<double> distY(ymin, ymax);

    std::vector<Eigen::Vector2d> samples;
    int N = N_in;
    size_t hitObstacleIdx;
    double epsilon = 0.1;

    while (samples.size() < N) {
        Eigen::Vector2d q{distX(gen), distY(gen)};
        if (!amp::isInCollision(q, obstacles, epsilon, hitObstacleIdx)) {
            samples.push_back(q);
        }
    }

    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;

    for (amp::Node i = 0; i < samples.size(); ++i) nodes[i] = samples[i];

    double r = r_in;

    // Connect free-space samples
    for (amp::Node i = 0; i < N; ++i) {
        Eigen::Vector2d q = samples[i];
        std::vector<amp::Node> neighbors = findRadiusNeighborsNaive(samples, q, r);

        for (amp::Node j : neighbors) {
            if (j <= i) continue;
            Eigen::Vector2d neighborPoint = samples[j];
            if (!edgeCollides(q, neighborPoint, obstacles, 0.005, epsilon)) {
                double weight = (q - neighborPoint).norm();
                graphPtr->connect(i, j, weight);
                graphPtr->connect(j, i, weight);
            }
        }
    }

    // Add start and goal nodes
    amp::Node startIdx = samples.size();
    amp::Node goalIdx  = samples.size() + 1;
    samples.push_back(problem.q_init);
    samples.push_back(problem.q_goal);
    nodes[startIdx] = problem.q_init;
    nodes[goalIdx]  = problem.q_goal;

    // Connect start and goal to nearby samples
    for (auto [specialIdx, q] : std::vector<std::pair<amp::Node, Eigen::Vector2d>>{
            {startIdx, problem.q_init}, {goalIdx, problem.q_goal}}) {

        for (amp::Node j = 0; j < N; ++j) {
            Eigen::Vector2d neighborPoint = samples[j];
            if ((q - neighborPoint).norm() <= r &&
                !edgeCollides(q, neighborPoint, obstacles, 0.005, epsilon)) {
                double weight = (q - neighborPoint).norm();
                graphPtr->connect(specialIdx, j, weight);
                graphPtr->connect(j, specialIdx, weight);
            }
        }
    }

    // Shortest path search
    amp::ShortestPathProblem spp;
    spp.graph = graphPtr;
    spp.init_node = startIdx;
    spp.goal_node = goalIdx;

    auto graph_nodes = graphPtr->nodes();
    bool start_exists = std::find(graph_nodes.begin(), graph_nodes.end(), startIdx) != graph_nodes.end();
    bool goal_exists = std::find(graph_nodes.begin(), graph_nodes.end(), goalIdx) != graph_nodes.end();

    if (!start_exists || !goal_exists) {
        amp::Path2D path;
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
        return path;
    }

    amp::Path2D path;

    EuclideanHeuristic heuristic(nodes, problem.q_goal);
    MyAStarAlgo astar;
    auto result = astar.search(spp, heuristic);


    for (amp::Node n : result.node_path)
        path.waypoints.push_back(nodes[n]);

    // Optional path smoothing
    // greedySmoothPath(path, obstacles, 0.005, 0.1);

    nodes_for_plot = nodes;
    graph_for_plot = graphPtr;

    did_it_work = result.success;

    if (!result.success) {
        amp::Path2D fail_path;
        fail_path.waypoints.push_back(problem.q_init);
        fail_path.waypoints.push_back(problem.q_goal);
        return fail_path;
    }

    return path;
}

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
