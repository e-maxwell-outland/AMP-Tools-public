#include "MyManipulator2D.h"
#include <cmath>
#include <iostream>

// “Default” two-link manipulator
MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Custom 2-link with specified lengths
MyManipulator2D::MyManipulator2D(double link1, double link2)
    : LinkManipulator2D({link1, link2})
{}

// variable number of links
MyManipulator2D::MyManipulator2D(const std::vector<double> links)
    : LinkManipulator2D(links)
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)

    auto link_lengths = LinkManipulator2D::getLinkLengths();

    auto x = 0.0;
    auto y = 0.0;
    auto angle_sum = 0.0;

    for (auto i = 0; i < joint_index; i++) {
        angle_sum += state[i];

        auto newx = link_lengths[i] * std::cos(angle_sum);
        x = x + newx;

        auto newy = link_lengths[i] * std::sin(angle_sum);
        y = y + newy;
    }

    Eigen::Vector2d joint_pos(x, y);

    return joint_pos;
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    auto x_e = end_effector_location[0];
    auto y_e = end_effector_location[1];

    auto link_lengths = LinkManipulator2D::getLinkLengths();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        amp::ManipulatorState joint_angles = Eigen::Vector2d(0, 0);
        auto a1 = link_lengths[0];
        auto a2 = link_lengths[1];

        // Find cosine theta2 and theta2 (law of cosines)
        auto cos_theta2 = ((x_e*x_e + y_e*y_e) - (a1*a1) - (a2*a2)) / (2 * a1 * a2);

        // Check if reachable
        if (cos_theta2 < -1.0 || cos_theta2 > 1.0) {
            return joint_angles;
        }

        auto theta2 = std::acos(cos_theta2);
        auto theta2_alt = -std::acos(cos_theta2);

        // Find theta1
        double k1 = a1 + (a2 * std::cos(theta2));
        double k2 = a2 * std::sin(theta2);
        double k1_alt = a1 + (a2 * std::cos(theta2_alt));
        double k2_alt = a2 * std::sin(theta2_alt);

        // theta2 for each configuration
        double theta1 = std::atan2(y_e, x_e) - std::atan2(k2, k1);
        double theta1_alt = std::atan2(y_e, x_e) - std::atan2(k2_alt, k1_alt);

        joint_angles = Eigen::Vector2d(theta1, theta2);

        return joint_angles;
    }

    else if (nLinks() == 3) {
        amp::ManipulatorState joint_angles = Eigen::Vector3d(0, 0, 0);
        auto a1 = link_lengths[0];
        auto a2 = link_lengths[1];
        auto a3 = link_lengths[2];

        // Set angle theta1 and re-define end effector point relative to that first link
        double theta1 = std::numeric_limits<double>::quiet_NaN();
        double x_prime = x_e;
        double y_prime = y_e;

        for (double t = 0; t <= 2*M_PI; t += 0.01) {
            x_prime = (x_e * std::cos(t)) + (y_e * std::sin(t)) - a1;
            y_prime = (-1 * x_e * std::sin(t)) + (y_e * std::cos(t));

            double d  = std::sqrt(x_prime*x_prime + y_prime*y_prime);
            if (d >= std::abs(a2 - a3) && d <= a2 + a3) {
                theta1 = t;
                break;          // stop at the first feasible angle
            }
        }

        // Find cosine theta3 and theta3 (law of cosines)
        auto cos_theta3 = ((x_prime*x_prime + y_prime*y_prime) - (a2*a2) - (a3*a3)) / (2 * a2 * a3);

        // Check if reachable
        if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
            return joint_angles;
        }

        auto theta3 = std::acos(cos_theta3);
        auto theta3_alt = -std::acos(cos_theta3);

        // Find theta2
        double k1 = a2 + a3 * std::cos(theta3);
        double k2 = a3 * std::sin(theta3);
        double k1_alt = a2 + a3 * std::cos(theta3_alt);
        double k2_alt = a3 * std::sin(theta3_alt);

        // theta2 for each configuration
        double theta2 = std::atan2(y_prime, x_prime) - std::atan2(k2, k1);
        double theta2_alt = std::atan2(y_prime, x_prime) - std::atan2(k2_alt, k1_alt);

        joint_angles = Eigen::Vector3d(theta1, theta2, theta3);

        return joint_angles;
    }

    else {
        // Get the number of links and create empty vector for joint angles
        const size_t n = nLinks();
        amp::ManipulatorState joint_angles = amp::ManipulatorState::Zero(n);

        // ---------- Step 1: aim the first n-3 links toward the target ----------
        // Angle from base to goal
        const double phi = std::atan2(y_e, x_e);

        double x_residual = x_e;
        double y_residual = y_e;

        for (size_t i = 0; i < n - 3; ++i) {
            joint_angles[i] = phi;

            // Remove its contribution to the remaining "virtual" end effector
            x_residual += (x_residual * std::cos(phi)) + (y_residual * std::sin(phi)) - link_lengths[i];;
            y_residual += (-1 * x_residual * std::sin(phi)) + (y_residual * std::cos(phi));
        }

        // ---------- Step 2: solve 3-link IK on the remainder ----------
        // Re-use your existing 3-link solver logic, but applied to the
        // *residual* target (x_residual, y_residual)
        const double a1 = link_lengths[n - 3];
        const double a2 = link_lengths[n - 2];
        const double a3 = link_lengths[n - 1];

        double theta1 = std::numeric_limits<double>::quiet_NaN();
        double x_prime = x_residual;
        double y_prime = y_residual;

        for (double t = 0; t <= 2*M_PI; t += 0.01) {
            x_prime = (x_e * std::cos(t)) + (y_e * std::sin(t)) - a1;
            y_prime = (-1 * x_e * std::sin(t)) + (y_e * std::cos(t));

            double d  = std::sqrt(x_prime*x_prime + y_prime*y_prime);
            if (d >= std::abs(a2 - a3) && d <= a2 + a3) {
                theta1 = t;
                break;          // stop at the first feasible angle
            }
        }

        // Find cosine theta3 and theta3 (law of cosines)
        auto cos_theta3 = ((x_prime*x_prime + y_prime*y_prime) - (a2*a2) - (a3*a3)) / (2 * a2 * a3);

        // Check if reachable
        if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
            return joint_angles;
        }

        auto theta3 = std::acos(cos_theta3);
        auto theta3_alt = -std::acos(cos_theta3);

        // Find theta2
        double k1 = a2 + a3 * std::cos(theta3);
        double k2 = a3 * std::sin(theta3);
        double k1_alt = a2 + a3 * std::cos(theta3_alt);
        double k2_alt = a3 * std::sin(theta3_alt);

        // theta2 for each configuration
        double theta2 = std::atan2(y_prime, x_prime) - std::atan2(k2, k1);
        double theta2_alt = std::atan2(y_prime, x_prime) - std::atan2(k2_alt, k1_alt);

        // Store the final three joint angles
        joint_angles[n - 3] = theta1;
        joint_angles[n - 2] = theta2;
        joint_angles[n - 1] = theta3;

        return joint_angles;
    }
}