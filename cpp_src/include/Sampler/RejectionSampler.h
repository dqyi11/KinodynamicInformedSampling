#pragma once

// Standard Library Functions
#include <iostream>
#include <chrono>
using namespace std::chrono;
#include <tuple>
#include <math.h>

// Eigen namespace
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Include Sampler
#include <Sampler/Sampler.h>

// Example for how to inherit and create your own sampler
class RejectionSampler: public Sampler
{
public:
	RejectionSampler(const ProblemDefinition &problem)
		: Sampler(problem)
	{ }

	// Only function that you must implement
	virtual MatrixXd sample(const int& no_samples, const bool& time) const override;

	// Can implement as many private functions as you want to help do the sampling
	virtual VectorXd get_random_sample(const double& max, const double& min, const int& size) const;
};

///
/// Heirarchical Rejection Sampler
///
class HierarchicalRejectionSampler: public RejectionSampler
{
public:
	///
	/// Constructor
	///
	/// @param problem Problem Definition
	///
	HierarchicalRejectionSampler(const ProblemDefinition &problem)
		: RejectionSampler(problem)
	{ }

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, const bool& time) const override;

private:
	///
	/// Get one sample using a recursive algorithm of heirarchical rejection sampling
	///
	/// @param start_index Start index of the hierarchical sample
	/// @param end_index End index of the hierarchical sample
	/// @param sample Reference to a sample that gets changed in place
	/// @return (c_start, c_goal)
	///
	virtual std::tuple<double, double>
		HRS(const int &start_index, const int &end_index, VectorXd &sample) const;

	///
	/// Calculates the cost of a leaf node
	///
	/// @param x1 First state
	/// @param x2 Second state
	/// @param i Index of the degree of freedom
	/// @return Cost to go from x1 to x2
	///
	virtual double calculate_leaf(const VectorXd &x1, const VectorXd &x2, const int &i) const = 0;

    ///
    /// Combines the cost of two states
    ///
    /// @param x1 First state
    /// @param x2 Second state
    /// @param i Index of the degree of freedom for first state
    /// @param m Mid degree of freedom
    /// @param j Index of  the degree of freedom of the second state
    /// @param c1 Cost one
    /// @param c2 Cost two
    /// @return Combination of the costs
    ///
    virtual double combine_costs(const VectorXd &x1,
                                 const VectorXd &x2,
                                 const int i,
                                 const int m,
                                 const int j,
                                 const double &c1,
                                 const double &c2) const = 0;

	///
	/// How to sample a leaf (ex: geometric is one dimension and kino is 2)
	///
	/// @param sample A vector to the sample
	/// @param dof An index to the degree of freedom to sample
	/// @return A random vector in the space
	///
	virtual void sample_leaf(VectorXd &sample, const int dof) const = 0;

	///
	/// Function to get the final cost from all of the dimensions
	///
	/// @param cost Cost to get final form of
	/// @return Final cost of the aggregation
	///
	virtual double get_cost(const double &cost) const { return cost; }
};

///
/// Geometric Heirarchical Rejection Sampler
///
class GeometricHierarchicalRejectionSampler: public HierarchicalRejectionSampler
{
public:
	///
	/// Constructor
	///
	/// @param problem Problem Definition
	///
	GeometricHierarchicalRejectionSampler(const ProblemDefinition &problem)
		: HierarchicalRejectionSampler(problem)
	{ }

private:
	///
	/// Calculates the cost of a leaf node
	///
	/// @param x1 First state
	/// @param x2 Second state
	/// @param i Index of the degree of freedom
	/// @return Cost to go from x1 to x2
	///
	virtual double calculate_leaf(const VectorXd &x1, const VectorXd &x2, const int &i) const override;

    ///
    /// Combines the cost of two states
    ///
    /// @param x1 First state
    /// @param x2 Second state
    /// @param i Index of the degree of freedom for first state
    /// @param m Mid degree of freedom
    /// @param j Index of  the degree of freedom of the second state
    /// @param c1 Cost one
    /// @param c2 Cost two
    /// @return Combination of the costs
    ///
    virtual double combine_costs(const VectorXd &x1,
                                 const VectorXd &x2,
                                 const int i,
                                 const int m,
                                 const int j,
                                 const double &c1,
                                 const double &c2) const override;

	///
	/// How to sample a leaf (ex: geometric is one dimension and kino is 2)
	///
	/// @param sample A vector to the sample
	/// @param dof An index to the degree of freedom to sample
	/// @return A random vector in the space
	///
	virtual void sample_leaf(VectorXd &sample, const int dof) const override;

	///
	/// Function to get the final cost from all of the dimensions
	///
	/// @param cost Cost to get final form of
	/// @return Final cost of the aggregation
	///
	virtual double get_cost(const double &cost) const override { return std::sqrt(cost); }
};
