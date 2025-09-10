#include "MotionHelpers.h"
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

// Adapt Eigen::Vector2d to Boost.Geometry
BOOST_GEOMETRY_REGISTER_POINT_2D(Eigen::Vector2d, double,
    boost::geometry::cs::cartesian, x(), y())

namespace bg = boost::geometry;

namespace amp {

//    // ------------------- Helper: Compute squared distance from point p to segment [a, b] -------------------
//    double pointToSegmentDistanceSq(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
//
//        Eigen::Vector2d ab = b - a;
//        Eigen::Vector2d closest;
//
//        double t = (p - a).dot(ab) / ab.squaredNorm();
//
//        if (t < 0.0) {
//            closest = a; // before a
//        } else if (t > 1.0) {
//            closest = b; // after b
//        } else {
//            closest = a + t * ab; // projection falls on segment
//        }
//
//        return (p - closest).squaredNorm();
//    }
//
//    // ------------------- Helper: Return index of closest segment of polygon -------------------
//    int closestSegmentIndex(const Eigen::Vector2d& p, const amp::Obstacle2D& poly) {
//
//        double bestDist = std::numeric_limits<double>::infinity();
//        int bestIdx = -1;
//
//        const auto& verts = poly.verticesCCW();
//        size_t n = verts.size();
//
//        for (int i = 0; i < n; ++i) {
//            const Eigen::Vector2d& a = verts[i];
//            const Eigen::Vector2d& b = verts[(i + 1) % n];
//
//            auto distSq = pointToSegmentDistanceSq(p, a, b);
//
//            if (distSq < bestDist) {
//                bestDist = distSq;
//                bestIdx = i;
//            }
//        }
//
//        return bestIdx; // index of segment [vertices[i], vertices[i+1]]
//    }

    // ------------------- Moves the robot one step size towards goal -------------------
    Eigen::Vector2d moveToGoal(const Eigen::Vector2d& currentPos, const Eigen::Vector2d& goalPos, double stepSize) {
        Eigen::Vector2d direction = goalPos - currentPos;
        double distance = direction.norm();
        if (distance <= stepSize) return goalPos;
        direction.normalize();
        return currentPos + direction * stepSize;
    }

    // ------------------- Moves the robot one step size around the perimeter -------------------
    Eigen::Vector2d nextPerimeterStep(const Eigen::Vector2d& current, size_t& currentObstacleIdx, Eigen::Vector2d& dir,
                                      double stepSize, double deltaTheta, const std::vector<Obstacle2D>& obstacles,
                                      std::unordered_set<size_t>& cluster, bool leftTurning, double epsilon) {

        //std::cerr << "Inside nextPerimeterStep...\n";

        Eigen::Vector2d candidate = current + stepSize * dir;
        size_t collisionIdx; // placeholder for collision check

        // Bool to determine if robot is falsely trying to hop to a nearby, non-intersecting obstacle
        bool obstacleHop = false;

        // Safety back-up vector
        Eigen::Vector2d fallbackDir = (-1 * dir).normalized();   // make sure dir is normalized!

        // ---------------- Step 1: Forward blocked? Rotate away ----------------
        if (isInCollision(candidate, obstacles, epsilon, collisionIdx)) {
            //std::cerr << "Collision stright ahead!\n";

            if ((collisionIdx != currentObstacleIdx) && (cluster.count(collisionIdx) == 0)) {
                obstacleHop = true;
                //std::cerr << "   Obstacle hop detected! collisionIdx not in cluster.\n";
            }

            // Safety limits on while loop
            const int maxDoWhileSteps = static_cast<int>(2 * M_PI / deltaTheta); // one full circle
            int count = 0;

            // If not trying to obstacle hop, then turn away
            if (!obstacleHop) {
                //std::cerr << "    No obstacle hop!\n";
                do {
                    // Determine values for rotation matrix
                    double sign = leftTurning ? 1.0 : -1.0; // left = CCW, right = CW
                    double cosT = cos(sign * deltaTheta);
                    double sinT = sin(sign * deltaTheta);

                    // Rotate dir vector
                    dir = Eigen::Vector2d(dir.x() * cosT - dir.y() * sinT,
                                          dir.x() * sinT + dir.y() * cosT).normalized();

                    candidate = current + stepSize * dir;

                    count++;
                    if (count > maxDoWhileSteps) {
                        std::cerr << "    Safety break: no free candidate found in Step 1!\n";
                        break;
                    }
                } while (isInCollision(candidate, obstacles, epsilon, collisionIdx));

                // Update global collision index to most recent collision
                currentObstacleIdx = collisionIdx;

                //std::cerr << "Found next candidate!\n";
                if (count > maxDoWhileSteps) {
                    // Step backwards along the direction you came from
                    return current + (0.5 * stepSize * fallbackDir);
                } else return candidate;
            }
        }

        // --------------- Step 2: Forward clear or obstacle hopping? Rotate towards current obstacle ---------------

        // Create variables to make sure robot doesn't 360 turn forever
        const int maxRotSteps = static_cast<int>(2 * M_PI / deltaTheta); // one full circle
        int rotCount = 0;

        while (rotCount < maxRotSteps) {
            //std::cerr << "Hugging rotation loop start.\n";
            obstacleHop = false;

            // Determine values for rotation matrix
            double sign = leftTurning ? -1.0 : 1.0; // rotate in opposite direction
            double cosT = cos(sign * deltaTheta);
            double sinT = sin(sign * deltaTheta);

            // Create new candidate rotated closer to the object
            auto dirCandidate = Eigen::Vector2d(dir.x() * cosT - dir.y() * sinT,
                                         dir.x() * sinT + dir.y() * cosT).normalized();

            candidate = current + stepSize * dirCandidate;

            //std::cerr << "Before isInCollision\n";
            // Check if new candidate collides with an obstacle
            if (isInCollision(candidate, obstacles, epsilon, collisionIdx)) {
            //std::cerr << "After isInCollision, collided. \n";

                if ((collisionIdx != currentObstacleIdx) && (cluster.count(collisionIdx) == 0)) {
                    obstacleHop = true;
                    //std::cerr << "      Obstacle hop detected! collisionIdx not in cluster.\n";
                }

                if (!obstacleHop) {
                    //std::cerr << "    No obstacle hop. \n";
                    // Update global collision index to most recent collision
                    currentObstacleIdx = collisionIdx;

                    // Too tight: keep original dir
                    return current + stepSize * dir;
                }

                else {
                    // Ignore intersection, keep turning towards obstacle
                    //std::cerr << "      Ignoring obstacle hop, rotating further.\n";
                    dir = dirCandidate;
                }

            } else {
                //std::cerr << "After isInCollision, not collided, hug closer. \n";
                // Safe to hug closer: update dir and loop again
                dir = dirCandidate;
            }

            //std::cerr << "Rotation count increased.\n";
            rotCount++;
        }
        // Safety fallback in case can't find a tighter hug after full rotation
        std::cerr << "Safety fallback.\n";

        // Step backwards along the direction you came from
        return current + (0.5 * stepSize * fallbackDir);
    }
}
