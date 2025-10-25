#include "CollisionHelpers.h"
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

// Adapt Eigen::Vector2d to Boost.Geometry
BOOST_GEOMETRY_REGISTER_POINT_2D(Eigen::Vector2d, double,
    boost::geometry::cs::cartesian, x(), y())

namespace bg = boost::geometry;

namespace amp {

    // ------------------- Helper: distance from point to segment -----------------
    double distancePointToSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        Eigen::Vector2d ab = b - a;
        Eigen::Vector2d ap = p - a;

        /* Determine the closest point on the segment to the point by finding the projection of the vector made by the
        point and one end of the segment onto the vector made by the segment */
        double t = ap.dot(ab) / ab.squaredNorm();
        t = std::max(0.0, std::min(1.0, t));  // clamp t to [0,1], if nothing on the segment is closest, then an end

        Eigen::Vector2d closest = a + (t * ab); // "how much" of vector ab to add to a, which is the closest point to p
        return (p - closest).norm(); // finally, return the distance from p to the closest point on the segment
    }

    // ------------------- Helper: closest point from current pos to obstacle -----------------
    Eigen::Vector2d hitPointFromSegment(const Eigen::Vector2d& robotPos, const Eigen::Vector2d& a,
                                        const Eigen::Vector2d& b, double epsilon) {

        // Same code as above to find closest point
        Eigen::Vector2d ab = b - a;
        Eigen::Vector2d ap = robotPos - a;
        double t = std::clamp(ap.dot(ab) / ab.squaredNorm(), 0.0, 1.0);
        Eigen::Vector2d closestPoint = a + t * ab;

        // Find the vector from closest point to robot, then move epsilon away
        Eigen::Vector2d dir = (robotPos - closestPoint).normalized();
        return closestPoint + (dir * epsilon); // point that is "epsilon away"
    }

    // ----- Helper: Return if any two segments, including endpoints, intersect -----
    bool doSegmentsIntersect(const std::vector<Eigen::Vector2d>& vertsA, const std::vector<Eigen::Vector2d>& vertsB) {

        // Lambda for comparing two points
        auto pointsAreEqual = [](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
            return (p1 - p2).norm() < 1e-9;
        };

        // Compare all the segments formed by the vertices of A with those formed by the vertices of B
        for (size_t i = 0; i < vertsA.size(); ++i) {
            Eigen::Vector2d a1 = vertsA[i];
            Eigen::Vector2d a2 = vertsA[(i+1) % vertsA.size()];
            bg::model::segment<Eigen::Vector2d> segA(a1, a2);

            for (size_t j = 0; j < vertsB.size(); ++j) {
                Eigen::Vector2d b1 = vertsB[j];
                Eigen::Vector2d b2 = vertsB[(j+1) % vertsB.size()];
                bg::model::segment<Eigen::Vector2d> segB(b1, b2);

                if (pointsAreEqual(a1, b1) || pointsAreEqual(a1, b2) ||
                    pointsAreEqual(a2, b1) || pointsAreEqual(a2, b2)) {
                    return true;
                    }

                if (bg::intersects(segA, segB)) {
                    return true;
                }
            }
        }

        return false;
    }

    // ------------------- Simple ray-casting point-in-polygon -------------------
    bool isPointInsidePolygon(const Eigen::Vector2d& point, const Obstacle2D& polygon) {

        const auto& verts = polygon.verticesCCW(); // get vector of vertices counter clockwise
        int crossings = 0;  // counts how many times the ray crosses edges

        // This code uses ray-casting to determine if a given point is inside the obstacle or not
        for (size_t i = 0; i < verts.size(); ++i) {
            Eigen::Vector2d v1 = verts[i];
            Eigen::Vector2d v2 = verts[(i+1) % verts.size()];  // modulo handles wrap-around last->first
            if (((v1.y() > point.y()) != (v2.y() > point.y())) &&
                (point.x() < (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y()) + v1.x())) {

                crossings++;
            }
        }
        return (crossings % 2 == 1);  // odd → inside, even → outside
    }

    // ------------------- Collision check for a point in a 2D WS with epsilon buffer ----------------------
    bool isInCollision(const Eigen::Vector2d& point, const std::vector<Obstacle2D>& obstacles, double epsilon,
                       size_t& hitObstacleIdx) {
        for (size_t i = 0; i < obstacles.size(); ++i) {
            const auto& obs = obstacles[i];

            // Check if point is inside polygon
            if (isPointInsidePolygon(point, obs)) {
                hitObstacleIdx = i;
                return true;
            }

            // If not, check distance to each edge
            const auto& verts = obs.verticesCCW();
            size_t n = verts.size();
            for (size_t j = 0; j < n; ++j) {
                Eigen::Vector2d a = verts[j];
                Eigen::Vector2d b = verts[(j+1) % n];

                if (distancePointToSegment(point, a, b) < epsilon) {
                    hitObstacleIdx = i;
                    return true;
                }
            }
        }

        return false; // no collision detected
    }

    // ------------------- Find Hit Point ----------------------
    Eigen::Vector2d findHitPoint(const Eigen::Vector2d& robotPos, const Obstacle2D& obs, double epsilon) {

        // Find the vertices of the obstacle
        const auto& verts = obs.verticesCCW();
        size_t n = verts.size();

        // Instantiate variables for loop
        Eigen::Vector2d closestA, closestB;
        double minDist;
        bool firstSegment = true;

        // Check all segments of the polygon to determine which segment is closest to the point
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector2d a = verts[i];
            Eigen::Vector2d b = verts[(i + 1) % n];

            double d = distancePointToSegment(robotPos, a, b);

            if (firstSegment || d < minDist) {
                minDist = d;
                closestA = a;
                closestB = b;
                firstSegment = false;
            }
        }

        // Once we know the closest segment, compute hit point epsilon away
        return hitPointFromSegment(robotPos, closestA, closestB, epsilon);
    }

    // ------------------- Determine if two polygons intersect ----------------------
    bool doPolygonsIntersect(const Obstacle2D& A, const Obstacle2D& B) {
        const auto& vertsA = A.verticesCCW();
        const auto& vertsB = B.verticesCCW();

        // 1. Check if any vertex of A is inside B
        for (const auto& v : vertsA) {
            if (isPointInsidePolygon(v, B)) {
                return true;
            }
        }

        // 2. Check if any vertex of B is inside A
        for (const auto& v : vertsB) {
            if (isPointInsidePolygon(v, A)) {
                return true;
            }
        }

        // 3. Use doesSegmentIntersect for edge intersections
        if (doSegmentsIntersect(vertsA, vertsB)) {
            return true;
        }

        // No intersections found
        return false;
    }

    // ------------------- Build Adjacency List of All Obstacles ----------------------
    std::vector<std::vector<size_t>> buildAdjacencyList(std::vector<amp::Obstacle2D> obstacles) {
        size_t n = obstacles.size();
        std::vector<std::vector<size_t>> adj(n);

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                if (doPolygonsIntersect(obstacles[i], obstacles[j])) {
                    adj[i].push_back(j);
                    adj[j].push_back(i); // undirected graph
                }
            }
        }
        return adj;

    }

    // -------------- DF Search to determine the "island" the robot is allowed to navigate around -----------------
    void dfs(const std::vector<std::vector<size_t>>& adj, size_t current, std::unordered_set<size_t>& cluster) {
        cluster.insert(current);
        for (size_t neighbor : adj[current]) {
            if (cluster.count(neighbor) == 0) {
                dfs(adj, neighbor, cluster);
            }
        }
    }

    // -------------- Determine if a point is on the direct line from start to goal -----------------
    bool isOnMLine(const Eigen::Vector2d& point, const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                   double epsilon) {

        // Vector from start to goal
        Eigen::Vector2d sg = goal - start;

        // Vector from start to current point
        Eigen::Vector2d sp = point - start;

        // 1. Cross product (should be ~0 if on line)
        double cross = sg.x() * sp.y() - sg.y() * sp.x();

        // If |cross| is bigger than epsilon, the point is not on the line
        if (std::abs(cross) > epsilon) {
            return false;
        }

        // 2. Bounding box check (so point is between start and goal)
        double minX = std::min(start.x(), goal.x()) - epsilon;
        double maxX = std::max(start.x(), goal.x()) + epsilon;
        double minY = std::min(start.y(), goal.y()) - epsilon;
        double maxY = std::max(start.y(), goal.y()) + epsilon;

        if (point.x() < minX || point.x() > maxX) return false;
        if (point.y() < minY || point.y() > maxY) return false;

        return true;
    }

    // -------------- Determine if an edge/segment intersects with an obstacle -----------------
    bool edgeCollides(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                      const std::vector<amp::Obstacle2D>& obstacles, double stepSize, double epsilon) {

        // Dummy obstacle index for isInCollision function
        size_t hitObstacleIdx;

        // Check for all points stepSize apart along the segment for collision
        for (double t = 0; t <= 1.0; t += stepSize) {
            Eigen::Vector2d p = (1.0 - t) * start + t * end;
            if (amp::isInCollision(p, obstacles, epsilon, hitObstacleIdx)) return true;
        }

        return false;
    }

    // -------------- Determine if multi-agent meta-agent is in collision with obstacles or itself -----------------
    bool isMultiAgentInCollision(const Eigen::VectorXd& q, const amp::MultiAgentProblem2D& problem) {
        std::size_t m = problem.numAgents();

        // 1. Obstacle collision
        for (std::size_t i = 0; i < m; ++i) {
            const Eigen::Vector2d qi = q.segment<2>(2*i);
            size_t hit;
            if (isInCollision(qi, problem.obstacles, problem.agent_properties[i].radius, hit)) {
                return true; // robot i hits obstacle
            }
        }

        // 2. Robot–robot collision
        for (std::size_t i = 0; i < m; ++i) {
            for (std::size_t j = i + 1; j < m; ++j) {
                double dist = (q.segment<2>(2*i) - q.segment<2>(2*j)).norm();
                double min_dist = problem.agent_properties[i].radius + problem.agent_properties[j].radius;
                if (dist < min_dist) {
                    return true; // robots collide
                }
            }
        }

        return false; // safe
    }

    // -------------- Determine if an edge/segment intersects with an obstacle -----------------
    bool multiAgentEdgeCollides(const Eigen::VectorXd& start, const Eigen::VectorXd& end,
                      const amp::MultiAgentProblem2D& problem, double stepSize) {

        // Dummy obstacle index for isInCollision function
        size_t hitObstacleIdx;

        // Check for all points stepSize apart along the segment for collision
        for (double t = 0; t <= 1.0; t += stepSize) {
            Eigen::VectorXd p = ((1.0 - t) * start) + (t * end);
            if (amp::isMultiAgentInCollision(p, problem)) return true;
        }

        return false;
    }

}
