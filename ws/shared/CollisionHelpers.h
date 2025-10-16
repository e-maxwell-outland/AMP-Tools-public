#pragma once
#include <vector>
#include <Eigen/Core>
#include <cstddef>  // for size_t
#include <unordered_set>

#include "AMPCore.h"  // for Polygon / Obstacle2D

namespace amp {

    // Check if a point is inside a polygon, including epsilon buffer
    bool isPointInsidePolygon(const Eigen::Vector2d& point,
                              const Obstacle2D& polygon);

    // Check if a point collides with any obstacle in a workspace
    bool isInCollision(const Eigen::Vector2d& point,
                       const std::vector<Obstacle2D>& obstacles,
                       double epsilon,
                       size_t& hitObstacleIdx);

    // Find hit point for a known obstacle
    Eigen::Vector2d findHitPoint(const Eigen::Vector2d& robotPos,
                                 const Obstacle2D& obs,
                                 double epsilon);

    // Builds an adjacency graph for the problem (which obstacles are connected to which obstacles)
    std::vector<std::vector<size_t>> buildAdjacencyList(std::vector<amp::Obstacle2D> obstacles);

    // DFS to mutate cluster
    void dfs(const std::vector<std::vector<size_t>>& adj,
             size_t current,
             std::unordered_set<size_t>& cluster);

    // For a given point, start, and goal (with some buffer epsilon), find if point is on the line from start to goal
    bool isOnMLine(const Eigen::Vector2d& point,
                    const Eigen::Vector2d& start,
                    const Eigen::Vector2d& goal,
                    double epsilon);

    bool edgeCollides(const Eigen::Vector2d& start,
                        const Eigen::Vector2d& end,
                        const std::vector<amp::Obstacle2D>& obstacles,
                        double stepSize,
                        double epsilon = 0.0);
}