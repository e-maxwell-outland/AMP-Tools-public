#include "CollisionHelpers.h"
#include <cmath>

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

    // ------------------- Collision check with epsilon ----------------------
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
}
