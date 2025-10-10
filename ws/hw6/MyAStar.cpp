#include "MyAStar.h"
#include <queue>
#include <unordered_map>

struct PQEntry {
    amp::Node node;  // Node ID
    double f;        // f = g + h
    bool operator>(const PQEntry& other) const {
        return f > other.f; // priority queue sorts smallest f first
    }
};

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem,
                                                   const amp::SearchHeuristic& heuristic){
    using Node = amp::Node;

    auto graph = problem.graph;
    Node start = problem.init_node;
    Node goal  = problem.goal_node;

    GraphSearchResult result;
    result.path_cost = 0.0;
    result.success = false;

    // 1. Initialize data
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> open_set;
    open_set.push({start, heuristic(start)});

    std::unordered_map<Node, Node> came_from;    // track parents
    std::unordered_map<Node, double> g_score;    // cost from start
    g_score[start] = 0.0;

    auto iter = 0;

    // 2. Main A* loop
    while (!open_set.empty()) {
        iter++;

        PQEntry current_entry = open_set.top();
        Node current = current_entry.node;
        open_set.pop();

        // Goal check
        if (current == goal) {
            result.success = true;
            break;
        }

        // Expand neighbors
        const auto& neighbors = graph->children(current);
        const auto& edges = graph->outgoingEdges(current);

        for (size_t i = 0; i < neighbors.size(); ++i) {
            Node neighbor = neighbors[i];
            double edge_cost = edges[i];
            double tentative_g = g_score[current] + edge_cost;

            // If neighbor not visited yet or we found a cheaper path
            if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                g_score[neighbor] = tentative_g;
                double f = tentative_g + 0 * heuristic(neighbor);
                open_set.push({neighbor, f});
                came_from[neighbor] = current;
            }
        }
    }

    // 3. Reconstruct path
    if (result.success) {
        Node current = goal;
        while (current != start) {
            result.node_path.push_front(current);
            current = came_from[current];
        }

        result.node_path.push_front(start);
        result.path_cost = g_score[goal];
    }

    result.print(); // optional debug print
    std::cout << "Algorithm completed in " << iter << " iterations." << std::endl;
    return result;
}


