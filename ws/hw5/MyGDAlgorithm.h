#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyPotentialFunction : public amp::PotentialFunction2D {
public:
	// Potential Function constructor
	MyPotentialFunction(double d_star, double zeta, double Q_star, double eta) :
		  d_star(d_star),
		  zeta(zeta),
		  Q_star(Q_star),
		  eta(eta) {}

	// Function to pass in the problem
    void setProblem(const amp::Problem2D& problem);

	// Returns the potential function value (height) for a given 2D point.
	// Suggestion: do this for just the attractor part first
	double operator()(const Eigen::Vector2d& q) const override;

	Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override;

private:
	double d_star, zeta, Q_star, eta;
    double alpha = 0.1;
	amp::Problem2D problem;
};

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Constructor that instantiates pf with constants
		MyGDAlgorithm(double d_star, double zeta, double Q_star, double eta) :
			d_star(d_star), zeta(zeta), Q_star(Q_star), eta(eta),
          	pf(MyPotentialFunction(d_star, zeta, Q_star, eta)) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& prob) override;

		MyPotentialFunction pf;

	private:
		double d_star, zeta, Q_star, eta;
};
