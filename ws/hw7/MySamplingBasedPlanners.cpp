# include "MySamplingBasedPlanners.h"
# include "CollisionHelpers.h"
# include "MyAStar.h"
# include <random>
# include <vector>
#include <queue>
#include <tuple>

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

/* -------- PRM algorithm -------- */
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {

    // Get dimensions of the workspace
    double xmin = problem.x_min;
    double xmax = problem.x_max;
    double ymin = problem.y_min;
    double ymax = problem.y_max;

    // Get obstacles in WS
    auto obstacles = problem.obstacles;

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister engine

    // Make uniform distributions in the x and y dimensions
    std::uniform_real_distribution<double> distX(xmin, xmax);
    std::uniform_real_distribution<double> distY(ymin, ymax);

    // -------- Sample the workspace and find N points in the free space --------
    std::vector<Eigen::Vector2d> samples;
    int N = 500; // desired number of samples

    // Define dummy hit index and small epsilon distance to give clearance from obstacles
    size_t hitObstacleIdx;
    double epsilon = 0.1;

    while (samples.size() < N) {
        Eigen::Vector2d q;
        q << distX(gen), distY(gen);

        bool collides = amp::isInCollision(q, obstacles, epsilon, hitObstacleIdx);

        if (!collides) {
            samples.push_back(q);
        }
    }

    // -------- Build a KD tree to quickly determine the nearest neighbors for each sampled point --------
    // Build KD-tree
    std::vector<amp::Node> indices(samples.size());
    for (amp::Node i = 0; i < samples.size(); ++i)
        indices[i] = i;

    KDNode* kdTreeRoot = buildKDTree(samples, indices);

    // Add start and goal points to the PRM graph to be connected
    samples.push_back(problem.q_init);
    samples.push_back(problem.q_goal);

    // -------- Create the PRM graph --------
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;

    // Add sampled points as nodes
    for (amp::Node i = 0; i < samples.size(); ++i) {
        nodes[i] = samples[i];
    }

    // -------- Connect nodes to their nearest neighbors --------
    int k = N/2; // number of neighbors
    double r = 2;

    for (amp::Node i = 0; i < samples.size(); ++i) {
        Eigen::Vector2d q = samples[i];

        // Query KD-tree for k nearest neighbors
        std::vector<amp::Node> temp_neighbors = findKNearestNeighbors(kdTreeRoot, q, k);
        std::vector<amp::Node> neighbors;
//        std::vector<amp::Node> neighbors = temp_neighbors;

        for (amp::Node j = 0; j < temp_neighbors.size(); ++j) {
            if ((q - nodes[j]).norm() < r) {
                neighbors.push_back(temp_neighbors[j]);
            }
        }

        for (amp::Node j : neighbors) {
            Eigen::Vector2d neighborPoint = nodes[j];
            if (!edgeCollides(q, neighborPoint, obstacles, 0.005)) {
                double weight = (q - neighborPoint).norm();
                // Connect both ways to make the graph undirected
                graphPtr->connect(i, j, weight);
                graphPtr->connect(j, i, weight);
            }
        }
    }

    // graphPtr->print();

    /* -------- Search PRM from shortest path from start to goal -------- */
    // Wrap PRM graph into a shortest path problem
    amp::ShortestPathProblem spp;
    spp.graph = graphPtr;
    spp.init_node = samples.size() - 2;
    spp.goal_node = samples.size() - 1;

    // Euclidean heuristic heuristic
    EuclideanHeuristic heuristic(nodes, problem.q_goal);

    // Run A*
    MyAStarAlgo astar;
    auto result = astar.search(spp, heuristic);

    // Convert node path to waypoints
    amp::Path2D path;
    for (amp::Node n : result.node_path)
        path.waypoints.push_back(nodes[n]);

    return path;
}

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}